// muse_athena_full.cpp  -  Muse S Athena full signal dashboard + dual blink detector
//
// Build (Visual Studio x64 Release):
//   Include dir : sdk/libmuse_windows_8.0.0/include
//   Lib dir     : sdk/libmuse_windows_8.0.0/lib/release/x64
//   Link        : libmuse-wrt.lib  windowsapp.lib
//   Post-build  : xcopy /Y sdk\libmuse_windows_8.0.0\lib\release\x64\libmuse.dll $(OutDir)
//
// Signals shown
//   EEG raw + notch-filtered (TP9 AF7 AF8 TP10)
//   Band powers absolute: alpha beta delta theta gamma
//   Accelerometer (g), gyro (deg/s), magnetometer (uT)
//   PPG (Ambient/Green IR Red) + Optics fNIRS 730/850nm (uA)
//   Pressure (mBar), ambient temp (C), body temp via AVG_BODY_TEMPERATURE (C)
//   DRL/REF contact (uV), battery %, HSI headband fit quality
//   SDK artifact blink/jaw-clench + spike-based blink detector (AF7 + AF8)

#include <iostream>
#include <iomanip>
#include <sstream>
#include <memory>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <csignal>
#include <chrono>
#include <thread>
#include <cmath>
#include <algorithm>
#include <string>

#define NOMINMAX
#include <windows.h>
#include <winrt/Windows.Foundation.h>

#pragma comment(lib, "libmuse-wrt")
#pragma comment(lib, "windowsapp")

#include "muse.h"

using namespace interaxon::bridge;
using clk = std::chrono::steady_clock;

// ── time helper ───────────────────────────────────────────────────────────────
static double now_sec() {
    return std::chrono::duration<double>(clk::now().time_since_epoch()).count();
}

// ── spike-based blink detector (RISE -> PEAK -> FALL state machine) ───────────
//   AF7/AF8 signal must rise above baseline, hit a peak, then fall back.
//   A sustained high signal (no fall) is ignored — avoids false positives.
static constexpr double ALPHA_EMA    = 0.997;  // baseline EMA decay at ~220 Hz
static constexpr double RISE_THRESH  = 120.0;  // uV above baseline to enter spike
static constexpr double MIN_PEAK     = 180.0;  // uV minimum to count as blink
static constexpr double FALL_FRAC    = 0.40;   // spike must fall to 40% of rise range
static constexpr double COOLDOWN_SEC = 0.35;   // min seconds between consecutive blinks
static constexpr double LIT_DUR_SEC  = 0.50;   // how long the BLINK! indicator stays on

struct BlinkDetector {
    double baseline   = -1.0;
    bool   in_spike   = false;
    double peak_val   = 0.0;
    int    count      = 0;
    double last_blink = 0.0;
    double lit_until  = 0.0;
    double disp_val   = 0.0;

    bool process(double raw) {
        double val = std::abs(raw);
        disp_val = val;
        if (baseline < 0.0) { baseline = val; return false; }

        if (!in_spike) {
            baseline = ALPHA_EMA * baseline + (1.0 - ALPHA_EMA) * val;
            if ((val - baseline) > RISE_THRESH && val > MIN_PEAK) {
                in_spike = true;
                peak_val = val;
            }
        } else {
            if (val > peak_val) peak_val = val;
            double fall_target = baseline + (peak_val - baseline) * FALL_FRAC;
            if (val < fall_target) {
                in_spike = false;
                baseline = ALPHA_EMA * baseline + (1.0 - ALPHA_EMA) * val;
                double t = now_sec();
                if (peak_val > MIN_PEAK && (t - last_blink) > COOLDOWN_SEC) {
                    ++count;
                    last_blink = t;
                    lit_until  = t + LIT_DUR_SEC;
                    return true;
                }
            }
        }
        return false;
    }

    bool is_lit()     const { return now_sec() < lit_until; }
    bool is_spiking() const { return in_spike; }
};

// ── all live sensor data (mutex-protected) ────────────────────────────────────
struct LiveData {
    std::mutex mtx;

    // EEG: [0]=TP9(EEG1) [1]=AF7(EEG2) [2]=AF8(EEG3) [3]=TP10(EEG4)
    double eeg[4]   = {};
    double notch[4] = {};

    // band powers (Bels absolute)
    double band_alpha[4] = {};
    double band_beta[4]  = {};
    double band_delta[4] = {};
    double band_theta[4] = {};
    double band_gamma[4] = {};

    // motion
    double accel[3]  = {};  // g
    double gyro_v[3] = {};  // deg/s
    double mag[3]    = {};  // uT

    // physio
    double ppg[3]       = {};  // [0]=Ambient/Green  [1]=IR  [2]=Red
    double optics[8]    = {};  // OPTICS1-8 in uA  (ch1-4=outer, ch5-8=inner)
    double pressure_avg = 0.0; // mBar
    double temperature  = 0.0; // ambient deg C
    double body_temp    = 0.0; // skin contact deg C (AVG_BODY_TEMPERATURE)
    double drl_uv       = 0.0; // uV
    double ref_uv       = 0.0; // uV

    // battery
    double batt_pct = 0.0; // %
    double batt_mv  = 0.0; // mV

    // headband fit (HSI precision: 1=good 2=mediocre 4=poor)
    double hsi[4] = { 4, 4, 4, 4 };

    // SDK artifacts
    bool headband_on     = false;
    bool sdk_blink       = false;
    bool jaw_clench      = false;
    int  sdk_blink_count = 0;

    // spike-based blink detectors
    BlinkDetector spike_l;  // AF7 = left eye
    BlinkDetector spike_r;  // AF8 = right eye
};
static LiveData g_data;

// ── control state ─────────────────────────────────────────────────────────────
static std::atomic<bool>       g_running{true};
static std::shared_ptr<Muse>   g_muse;
static std::mutex              g_ctrl_mtx;
static std::condition_variable g_cv;
static std::atomic<bool>       g_muse_found{false};
static std::atomic<bool>       g_disconnected{false};

static void on_signal(int) { g_running = false; g_cv.notify_all(); }

// ── silent SDK logger (suppresses console spam) ───────────────────────────────
class SilentLogger : public LogListener {
public:
    void receive_log(const LogPacket&) override {}
};

// ── device scanner ────────────────────────────────────────────────────────────
class MuseScanner : public MuseListener {
    std::shared_ptr<MuseManagerWindows> mgr_;
public:
    explicit MuseScanner(std::shared_ptr<MuseManagerWindows> m) : mgr_(m) {}
    void muse_list_changed() override {
        auto muses = mgr_->get_muses();
        if (!muses.empty() && !g_muse_found.load()) {
            std::lock_guard<std::mutex> lk(g_ctrl_mtx);
            g_muse       = muses[0];
            g_muse_found = true;
            g_cv.notify_all();
        }
    }
};

// ── connection state listener ─────────────────────────────────────────────────
class ConnectionHandler : public MuseConnectionListener {
public:
    std::string status = "Connecting...";
    void receive_muse_connection_packet(const MuseConnectionPacket& pkt,
                                        const std::shared_ptr<Muse>&) override {
        switch (pkt.current_connection_state) {
        case ConnectionState::CONNECTING:   status = "Connecting...";          break;
        case ConnectionState::CONNECTED:    status = "Connected";               break;
        case ConnectionState::DISCONNECTED: status = "Disconnected";
            g_disconnected = true; g_cv.notify_all();                          break;
        case ConnectionState::NEEDS_UPDATE: status = "Needs firmware update";  break;
        default: break;
        }
    }
};

// ── full data + artifact handler ──────────────────────────────────────────────
class FullDataHandler : public MuseDataListener {
public:
    void receive_muse_data_packet(const std::shared_ptr<MuseDataPacket>& pkt,
                                  const std::shared_ptr<Muse>&) override {
        auto type = pkt->packet_type();
        std::lock_guard<std::mutex> lk(g_data.mtx);

        switch (type) {
        case MuseDataPacketType::EEG:
            for (int i = 0; i < 4; ++i)
                g_data.eeg[i] = pkt->get_eeg_channel_value(static_cast<Eeg>(i));
            g_data.spike_l.process(g_data.eeg[1]);  // AF7 = left eye
            g_data.spike_r.process(g_data.eeg[2]);  // AF8 = right eye
            break;

        case MuseDataPacketType::NOTCH_FILTERED_EEG:
            for (int i = 0; i < 4; ++i)
                g_data.notch[i] = pkt->get_eeg_channel_value(static_cast<Eeg>(i));
            break;

        case MuseDataPacketType::ALPHA_ABSOLUTE:
            for (int i = 0; i < 4; ++i)
                g_data.band_alpha[i] = pkt->get_eeg_channel_value(static_cast<Eeg>(i));
            break;
        case MuseDataPacketType::BETA_ABSOLUTE:
            for (int i = 0; i < 4; ++i)
                g_data.band_beta[i] = pkt->get_eeg_channel_value(static_cast<Eeg>(i));
            break;
        case MuseDataPacketType::DELTA_ABSOLUTE:
            for (int i = 0; i < 4; ++i)
                g_data.band_delta[i] = pkt->get_eeg_channel_value(static_cast<Eeg>(i));
            break;
        case MuseDataPacketType::THETA_ABSOLUTE:
            for (int i = 0; i < 4; ++i)
                g_data.band_theta[i] = pkt->get_eeg_channel_value(static_cast<Eeg>(i));
            break;
        case MuseDataPacketType::GAMMA_ABSOLUTE:
            for (int i = 0; i < 4; ++i)
                g_data.band_gamma[i] = pkt->get_eeg_channel_value(static_cast<Eeg>(i));
            break;

        case MuseDataPacketType::ACCELEROMETER:
            g_data.accel[0] = pkt->get_accelerometer_value(Accelerometer::X);
            g_data.accel[1] = pkt->get_accelerometer_value(Accelerometer::Y);
            g_data.accel[2] = pkt->get_accelerometer_value(Accelerometer::Z);
            break;

        case MuseDataPacketType::GYRO:
            g_data.gyro_v[0] = pkt->get_gyro_value(Gyro::X);
            g_data.gyro_v[1] = pkt->get_gyro_value(Gyro::Y);
            g_data.gyro_v[2] = pkt->get_gyro_value(Gyro::Z);
            break;

        case MuseDataPacketType::MAGNETOMETER:
            g_data.mag[0] = pkt->get_magnetometer_value(Magnetometer::X);
            g_data.mag[1] = pkt->get_magnetometer_value(Magnetometer::Y);
            g_data.mag[2] = pkt->get_magnetometer_value(Magnetometer::Z);
            break;

        case MuseDataPacketType::PPG:
            g_data.ppg[0] = pkt->get_ppg_channel_value(Ppg::AMBIENT);
            g_data.ppg[1] = pkt->get_ppg_channel_value(Ppg::IR);
            g_data.ppg[2] = pkt->get_ppg_channel_value(Ppg::RED);
            break;

        case MuseDataPacketType::OPTICS:
            // Muse S Athena (2025) routes PPG through Optics; also provides fNIRS
            for (int i = 0; i < 8; ++i)
                g_data.optics[i] = pkt->get_optics_channel_value(static_cast<Optics>(i));
            break;

        case MuseDataPacketType::BATTERY:
            g_data.batt_pct = pkt->get_battery_value(Battery::CHARGE_PERCENTAGE_REMAINING);
            g_data.batt_mv  = pkt->get_battery_value(Battery::MILLIVOLTS);
            break;

        case MuseDataPacketType::DRL_REF:
            g_data.drl_uv = pkt->get_drl_ref_value(DrlRef::DRL);
            g_data.ref_uv = pkt->get_drl_ref_value(DrlRef::REF);
            break;

        case MuseDataPacketType::HSI_PRECISION:
            for (int i = 0; i < 4; ++i)
                g_data.hsi[i] = pkt->get_eeg_channel_value(static_cast<Eeg>(i));
            break;

        case MuseDataPacketType::PRESSURE:
            g_data.pressure_avg = pkt->get_pressure_value(Pressure::AVERAGED);
            break;

        case MuseDataPacketType::TEMPERATURE:
            g_data.temperature = pkt->get_temperature_value();
            break;

        case MuseDataPacketType::AVG_BODY_TEMPERATURE: {
            auto vals = pkt->values();
            if (!vals.empty()) g_data.body_temp = vals[0];
            break;
        }

        default: break;
        }
    }

    void receive_muse_artifact_packet(const MuseArtifactPacket& pkt,
                                      const std::shared_ptr<Muse>&) override {
        std::lock_guard<std::mutex> lk(g_data.mtx);
        g_data.headband_on = pkt.headband_on;
        if (pkt.blink && !g_data.sdk_blink) ++g_data.sdk_blink_count;
        g_data.sdk_blink  = pkt.blink;
        g_data.jaw_clench = pkt.jaw_clench;
    }
};

// ── ANSI + display helpers ────────────────────────────────────────────────────
static void enable_ansi() {
    HANDLE h = GetStdHandle(STD_OUTPUT_HANDLE);
    DWORD m = 0;
    GetConsoleMode(h, &m);
    SetConsoleMode(h, m | ENABLE_VIRTUAL_TERMINAL_PROCESSING);
}

// Win32-based cursor home — reliable in both conhost and Windows Terminal.
// Records cursor position on first call (after the scan/connect messages),
// then jumps back there on every subsequent call. Also hides the cursor.
static COORD g_display_origin = {0, 0};
static void frame_home() {
    static bool first = true;
    HANDLE h = GetStdHandle(STD_OUTPUT_HANDLE);
    if (first) {
        CONSOLE_CURSOR_INFO ci;
        GetConsoleCursorInfo(h, &ci);
        ci.bVisible = false;
        SetConsoleCursorInfo(h, &ci);
        CONSOLE_SCREEN_BUFFER_INFO csbi;
        GetConsoleScreenBufferInfo(h, &csbi);
        g_display_origin = csbi.dwCursorPosition;
        first = false;
    }
    SetConsoleCursorPosition(h, g_display_origin);
}

static std::string hsi_label(double h) {
    if (h <= 1.0) return "\033[92mGood    \033[0m";
    if (h <= 2.0) return "\033[93mMediocre\033[0m";
    return               "\033[91mPoor    \033[0m";
}

static std::string blink_tag(bool lit) {
    return lit ? "\033[1;97;44m BLINK! \033[0m"
               : "\033[90m  ---   \033[0m";
}

static std::string jaw_tag(bool jaw) {
    return jaw ? "\033[1;93mCLENCH\033[0m"
               : "\033[90m ---  \033[0m";
}

static std::string hband_tag(bool on) {
    return on ? "\033[92mON \033[0m" : "\033[91mOFF\033[0m";
}

static std::string spike_state(bool lit, bool spiking) {
    if (lit)     return "\033[1;97;44m BLINK! \033[0m";
    if (spiking) return "\033[93m spike  \033[0m";
    return              "\033[90m  ---   \033[0m";
}

// ── display loop (10 Hz, 33 fixed rows) ───────────────────────────────────────
static void display_loop(std::shared_ptr<ConnectionHandler> conn) {
    using namespace std::chrono_literals;

    while (g_running) {
        std::this_thread::sleep_for(250ms);

        // ── snapshot under lock ───────────────────────────────────────────────
        double eeg[4], notch[4];
        double alpha[4], beta[4], delta[4], theta[4], gamma[4];
        double accel[3], gyro_v[3], mag[3];
        double ppg[3], optics[8];
        double pressure_avg, temperature, body_temp, drl_uv, ref_uv;
        double batt_pct, batt_mv, hsi[4];
        bool   headband_on, sdk_blink, jaw_clench;
        int    sdk_blink_count;
        double sl_val, sl_base; int sl_count; bool sl_lit, sl_spike;
        double sr_val, sr_base; int sr_count; bool sr_lit, sr_spike;
        {
            std::lock_guard<std::mutex> lk(g_data.mtx);
            std::copy(g_data.eeg,        g_data.eeg        + 4, eeg);
            std::copy(g_data.notch,      g_data.notch      + 4, notch);
            std::copy(g_data.band_alpha, g_data.band_alpha + 4, alpha);
            std::copy(g_data.band_beta,  g_data.band_beta  + 4, beta);
            std::copy(g_data.band_delta, g_data.band_delta + 4, delta);
            std::copy(g_data.band_theta, g_data.band_theta + 4, theta);
            std::copy(g_data.band_gamma, g_data.band_gamma + 4, gamma);
            std::copy(g_data.accel,      g_data.accel      + 3, accel);
            std::copy(g_data.gyro_v,     g_data.gyro_v     + 3, gyro_v);
            std::copy(g_data.mag,        g_data.mag        + 3, mag);
            std::copy(g_data.ppg,        g_data.ppg        + 3, ppg);
            std::copy(g_data.optics,     g_data.optics     + 8, optics);
            pressure_avg    = g_data.pressure_avg;
            temperature     = g_data.temperature;
            body_temp       = g_data.body_temp;
            drl_uv          = g_data.drl_uv;
            ref_uv          = g_data.ref_uv;
            batt_pct        = g_data.batt_pct;
            batt_mv         = g_data.batt_mv;
            std::copy(g_data.hsi, g_data.hsi + 4, hsi);
            headband_on     = g_data.headband_on;
            sdk_blink       = g_data.sdk_blink;
            jaw_clench      = g_data.jaw_clench;
            sdk_blink_count = g_data.sdk_blink_count;
            sl_val   = g_data.spike_l.disp_val;
            sl_base  = std::max(0.0, g_data.spike_l.baseline);
            sl_count = g_data.spike_l.count;
            sl_lit   = g_data.spike_l.is_lit();
            sl_spike = g_data.spike_l.is_spiking();
            sr_val   = g_data.spike_r.disp_val;
            sr_base  = std::max(0.0, g_data.spike_r.baseline);
            sr_count = g_data.spike_r.count;
            sr_lit   = g_data.spike_r.is_lit();
            sr_spike = g_data.spike_r.is_spiking();
        }

        SYSTEMTIME st;
        GetLocalTime(&st);
        char tbuf[12];
        snprintf(tbuf, sizeof(tbuf), "%02d:%02d:%02d", st.wHour, st.wMinute, st.wSecond);

        frame_home();
        auto& o = std::cout;
        o << std::fixed << std::right;

        // ── line 1: title ─────────────────────────────────────────────────────
        o << "  === Muse S Athena  Full Signal Dashboard ===   [" << tbuf << "]         \n";
        // ── line 2: blank ─────────────────────────────────────────────────────
        o << "\n";
        // ── line 3: status + battery ──────────────────────────────────────────
        o << "  [STATUS]  " << std::setw(14) << std::left << conn->status << std::right
          << "  Battery: " << std::setprecision(1) << std::setw(5) << batt_pct << "%"
          << "  (" << std::setprecision(0) << std::setw(4) << batt_mv << " mV)"
          << "  Headband: " << hband_tag(headband_on) << "         \n";
        // ── line 4: headband fit (HSI) ────────────────────────────────────────
        o << "  [FIT]  TP9: " << hsi_label(hsi[0])
          << "  AF7: "        << hsi_label(hsi[1])
          << "  AF8: "        << hsi_label(hsi[2])
          << "  TP10: "       << hsi_label(hsi[3]) << "         \n";
        // ── line 5: blank ─────────────────────────────────────────────────────
        o << "\n";
        // ── line 6: EEG header ────────────────────────────────────────────────
        o << "  [EEG uV]             TP9        AF7        AF8       TP10        \n";
        // ── line 7: raw EEG ───────────────────────────────────────────────────
        o << "  Raw               "
          << std::setprecision(1)
          << std::setw(9) << eeg[0] << "  "
          << std::setw(9) << eeg[1] << "  "
          << std::setw(9) << eeg[2] << "  "
          << std::setw(9) << eeg[3] << "         \n";
        // ── line 8: notch-filtered EEG ────────────────────────────────────────
        o << "  Notch filt.       "
          << std::setw(9) << notch[0] << "  "
          << std::setw(9) << notch[1] << "  "
          << std::setw(9) << notch[2] << "  "
          << std::setw(9) << notch[3] << "         \n";
        // ── line 9: blank ─────────────────────────────────────────────────────
        o << "\n";
        // ── line 10: band powers header ───────────────────────────────────────
        o << "  [BAND POWERS Bels]   TP9     AF7     AF8    TP10          \n";

        auto band_row = [&](const char* label, const double* b) {
            char buf[64];
            snprintf(buf, sizeof(buf), "  %-16s %6.2f  %6.2f  %6.2f  %6.2f         \n",
                     label, b[0], b[1], b[2], b[3]);
            o << buf;
        };
        // ── lines 11-15: bands ────────────────────────────────────────────────
        band_row("Alpha 7.5-13 Hz", alpha);
        band_row("Beta  13-30  Hz", beta);
        band_row("Delta 1-3    Hz", delta);
        band_row("Theta 3-7.5  Hz", theta);
        band_row("Gamma 30-44  Hz", gamma);
        // ── line 16: blank ────────────────────────────────────────────────────
        o << "\n";
        // ── line 17: motion header ────────────────────────────────────────────
        o << "  [MOTION]               X           Y           Z          \n";
        // ── lines 18-20: accel / gyro / mag ──────────────────────────────────
        o << "  Accel    (g)     "
          << std::setprecision(3) << std::setw(9) << accel[0] << "   "
          << std::setw(9) << accel[1] << "   "
          << std::setw(9) << accel[2] << "         \n";
        o << "  Gyro  (deg/s)    "
          << std::setprecision(1) << std::setw(9) << gyro_v[0] << "   "
          << std::setw(9) << gyro_v[1] << "   "
          << std::setw(9) << gyro_v[2] << "         \n";
        o << "  Mag      (uT)    "
          << std::setprecision(2) << std::setw(9) << mag[0] << "   "
          << std::setw(9) << mag[1] << "   "
          << std::setw(9) << mag[2] << "         \n";
        // ── line 21: blank ────────────────────────────────────────────────────
        o << "\n";
        // ── line 22: physio header ────────────────────────────────────────────
        o << "  [PHYSIO]                                                              \n";
        // ── line 23: PPG ──────────────────────────────────────────────────────
        o << "  PPG   Amb/Green: " << std::setprecision(0) << std::setw(7) << ppg[0]
          << "     IR: " << std::setw(7) << ppg[1]
          << "    Red: " << std::setw(7) << ppg[2] << "         \n";
        // ── line 24: Optics 730nm (HbR) ──────────────────────────────────────
        {
            char buf[80];
            snprintf(buf, sizeof(buf),
                "  730nm L-out:%7.1f R-out:%7.1f L-in:%7.1f R-in:%7.1f uA    \n",
                optics[0], optics[1], optics[4], optics[5]);
            o << buf;
        }
        // ── line 25: Optics 850nm (HbO) ──────────────────────────────────────
        {
            char buf[80];
            snprintf(buf, sizeof(buf),
                "  850nm L-out:%7.1f R-out:%7.1f L-in:%7.1f R-in:%7.1f uA    \n",
                optics[2], optics[3], optics[6], optics[7]);
            o << buf;
        }
        // ── line 26: pressure / temp / DRL/REF ───────────────────────────────
        {
            char buf[80];
            snprintf(buf, sizeof(buf),
                "  Pres:%7.1f mBar  Amb:%5.1fC  Body:%5.1fC  DRL:%5.0fuV REF:%5.0fuV\n",
                pressure_avg, temperature, body_temp, drl_uv, ref_uv);
            o << buf;
        }
        // ── line 27: blank ────────────────────────────────────────────────────
        o << "\n";
        // ── line 28: blinks header + SDK artifact ─────────────────────────────
        o << "  [BLINKS]  SDK artifact: " << blink_tag(sdk_blink)
          << " total: " << std::setw(4) << sdk_blink_count
          << "    Jaw clench: " << jaw_tag(jaw_clench) << "         \n";
        // ── line 29: spike detector AF7 (left) ───────────────────────────────
        o << "  Spike AF7 L: " << spike_state(sl_lit, sl_spike)
          << "  val: " << std::setprecision(1) << std::setw(7) << sl_val
          << " uV  base: " << std::setw(7) << sl_base
          << " uV  count: " << std::setw(4) << sl_count << "     \n";
        // ── line 30: spike detector AF8 (right) ──────────────────────────────
        o << "  Spike AF8 R: " << spike_state(sr_lit, sr_spike)
          << "  val: " << std::setw(7) << sr_val
          << " uV  base: " << std::setw(7) << sr_base
          << " uV  count: " << std::setw(4) << sr_count << "     \n";
        // ── line 31: blank ────────────────────────────────────────────────────
        o << "\n";
        // ── line 32: optics legend ────────────────────────────────────────────
        o << "  Optics ch1-4=outer ch5-8=inner | 730nm->HbR(deoxy) 850nm->HbO(oxy)  \n";
        // ── line 33: status / quit ────────────────────────────────────────────
        o << "  Status: " << std::left << std::setw(20) << conn->status
          << "  Ctrl+C to quit                         \n" << std::right;

        o << std::flush;
    }
}

// ── main ──────────────────────────────────────────────────────────────────────
int main() {
    enable_ansi();
    winrt::init_apartment();
    std::signal(SIGINT,  on_signal);
    std::signal(SIGTERM, on_signal);

    std::cout << "=== Muse S Athena Full Signal Dashboard ===\n";
    std::cout << "Ensure Bluetooth is on and the headband is paired.\n\n";

    auto mgr = MuseManagerWindows::get_instance();
    mgr->remove_from_list_after(0);
    LogManager::instance()->set_log_listener(std::make_shared<SilentLogger>());

    auto scanner   = std::make_shared<MuseScanner>(mgr);
    auto conn_hdlr = std::make_shared<ConnectionHandler>();
    auto data_hdlr = std::make_shared<FullDataHandler>();

    mgr->set_muse_listener(scanner);
    mgr->start_listening();
    std::cout << "[SCAN] Scanning for Muse headbands...\n";

    {
        std::unique_lock<std::mutex> lk(g_ctrl_mtx);
        g_cv.wait(lk, [] { return g_muse_found.load() || !g_running; });
    }
    if (!g_running) { mgr->stop_listening(); return 0; }

    mgr->stop_listening();
    std::cout << "[SCAN] Found: " << g_muse->get_name() << "\n";
    std::cout << "[CONN] Connecting...\n";

    g_muse->register_connection_listener(conn_hdlr);

    // register one handler for every useful packet type
    const MuseDataPacketType REGISTER_TYPES[] = {
        MuseDataPacketType::EEG,
        MuseDataPacketType::NOTCH_FILTERED_EEG,
        MuseDataPacketType::ALPHA_ABSOLUTE,
        MuseDataPacketType::BETA_ABSOLUTE,
        MuseDataPacketType::DELTA_ABSOLUTE,
        MuseDataPacketType::THETA_ABSOLUTE,
        MuseDataPacketType::GAMMA_ABSOLUTE,
        MuseDataPacketType::ACCELEROMETER,
        MuseDataPacketType::GYRO,
        MuseDataPacketType::MAGNETOMETER,
        MuseDataPacketType::PPG,
        MuseDataPacketType::OPTICS,
        MuseDataPacketType::BATTERY,
        MuseDataPacketType::DRL_REF,
        MuseDataPacketType::HSI_PRECISION,
        MuseDataPacketType::PRESSURE,
        MuseDataPacketType::TEMPERATURE,
        MuseDataPacketType::AVG_BODY_TEMPERATURE,
        MuseDataPacketType::ARTIFACTS,
    };
    for (auto t : REGISTER_TYPES)
        g_muse->register_data_listener(data_hdlr, t);

    // PRESET_1031: muse2025 only — 4CH EEG 14-bit 256Hz + accel/gyro 52Hz +
    // battery 1Hz + DRL/REF 32Hz + 16CH Optics 64Hz (fNIRS/PPG)
    // PRESET_21 is muse2016-2024 only and causes immediate disconnect on Muse S Athena.
    g_muse->set_preset(MusePreset::PRESET_1031);
    g_muse->run_asynchronously();

    std::thread display_thread(display_loop, conn_hdlr);

    {
        std::unique_lock<std::mutex> lk(g_ctrl_mtx);
        g_cv.wait(lk, [] { return !g_running || g_disconnected.load(); });
    }

    g_running = false;
    if (display_thread.joinable()) display_thread.join();

    if (!g_disconnected.load()) {
        std::cout << "\n[MAIN] Disconnecting...\n";
        g_muse->disconnect();
        std::unique_lock<std::mutex> lk(g_ctrl_mtx);
        g_cv.wait_for(lk, std::chrono::seconds(3),
                      [] { return g_disconnected.load(); });
    }

    std::cout << "[MAIN] Goodbye.\n";
    return 0;
}
