// muse_blink.cpp - Muse 2 spike-based blink detector
// Port of https://github.com/urish/muse-blink
//
// Detection: signal must RISE above baseline, hit a PEAK, then FALL back.
// A signal that is simply sustained high is ignored.
//
// Build (Visual Studio x64 Release) - same project settings as muse_viewer:
//   Include dir : sdk/libmuse_windows_8.0.0/include
//   Lib dir     : sdk/libmuse_windows_8.0.0/lib/release/x64
//   Link        : libmuse-wrt.lib  windowsapp.lib
//   Copy to exe : sdk/libmuse_windows_8.0.0/lib/release/x64/libmuse.dll

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

// ── Time helper ───────────────────────────────────────────────────────────────
static double now_sec() {
    return std::chrono::duration<double>(clk::now().time_since_epoch()).count();
}

// ── Tuning ────────────────────────────────────────────────────────────────────
static constexpr double ALPHA        = 0.997;  // baseline EMA per sample (~220 Hz)
static constexpr double RISE_THRESH  = 120.0;  // µV above baseline to start spike
static constexpr double MIN_PEAK     = 180.0;  // µV minimum peak to count as blink
static constexpr double FALL_FRAC    = 0.40;   // spike must fall to 40% of rise range
static constexpr double COOLDOWN     = 0.35;   // seconds minimum between two blinks
static constexpr double SHOW_DUR     = 0.50;   // seconds blink indicator stays lit

// ── Spike-based blink detector ────────────────────────────────────────────────
// State machine per electrode:
//   IDLE    → track baseline EMA, watch for rise above baseline + RISE_THRESH
//   SPIKING → track peak, wait for fall back below baseline + FALL_FRAC*(peak-baseline)
//   → on confirmed fall: count blink (if peak > MIN_PEAK and cooldown passed)
struct BlinkDetector {
    double baseline    = -1.0;   // -1 = not seeded yet
    bool   in_spike    = false;
    double peak        = 0.0;
    int    count       = 0;      // confirmed blinks (rise + fall)
    int    spike_count = 0;      // every time a spike starts (rising edge only)
    double last_blink  = 0.0;    // timestamp of last confirmed blink
    double lit_until   = 0.0;    // timestamp until indicator stays lit
    double disp_val    = 0.0;    // latest absolute value for display

    bool process(double raw) {
        double val = std::abs(raw);
        disp_val = val;

        if (baseline < 0.0) {
            baseline = val;
            return false;
        }

        if (!in_spike) {
            // Calm — slowly update baseline
            baseline = ALPHA * baseline + (1.0 - ALPHA) * val;

            double deviation = val - baseline;
            if (deviation > RISE_THRESH && val > MIN_PEAK) {
                in_spike = true;
                peak     = val;
                ++spike_count;   // count every rising edge
            }
        } else {
            // Spiking — track the peak
            if (val > peak) peak = val;

            // Confirmed fall: dropped back toward baseline
            double fall_target = baseline + (peak - baseline) * FALL_FRAC;
            if (val < fall_target) {
                in_spike = false;
                baseline = ALPHA * baseline + (1.0 - ALPHA) * val;

                double t = now_sec();
                if (peak > MIN_PEAK && (t - last_blink) > COOLDOWN) {
                    ++count;
                    last_blink = t;
                    lit_until  = t + SHOW_DUR;
                    return true;
                }
            }
        }
        return false;
    }

    bool is_lit() const { return now_sec() < lit_until; }
};

// ── Shared state ──────────────────────────────────────────────────────────────
static std::atomic<bool>       g_running{true};
static std::shared_ptr<Muse>   g_muse;
static std::mutex              g_mtx;
static std::condition_variable g_cv;
static std::atomic<bool>       g_muse_found{false};
static std::atomic<bool>       g_disconnected{false};

static void on_signal(int) { g_running = false; g_cv.notify_all(); }

// ── EEG state (protected by g_eeg_mtx) ───────────────────────────────────────
static std::mutex    g_eeg_mtx;
static BlinkDetector g_left;    // AF7 = EEG2 = left eye
static BlinkDetector g_right;   // AF8 = EEG3 = right eye

// ── Silent logger ─────────────────────────────────────────────────────────────
class SilentLogger : public LogListener {
public:
    void receive_log(const LogPacket&) override {}
};

// ── Muse scanner ──────────────────────────────────────────────────────────────
class MuseScanner : public MuseListener {
    std::shared_ptr<MuseManagerWindows> mgr_;
public:
    explicit MuseScanner(std::shared_ptr<MuseManagerWindows> m) : mgr_(m) {}
    void muse_list_changed() override {
        auto muses = mgr_->get_muses();
        if (!muses.empty() && !g_muse_found.load()) {
            std::lock_guard<std::mutex> lk(g_mtx);
            g_muse = muses[0];
            g_muse_found = true;
            g_cv.notify_all();
        }
    }
};

// ── Connection handler ────────────────────────────────────────────────────────
class ConnectionHandler : public MuseConnectionListener {
public:
    std::string status = "Connecting...";
    void receive_muse_connection_packet(const MuseConnectionPacket& pkt,
                                        const std::shared_ptr<Muse>&) override {
        switch (pkt.current_connection_state) {
        case ConnectionState::CONNECTING:   status = "Connecting...";         break;
        case ConnectionState::CONNECTED:    status = "Connected";              break;
        case ConnectionState::DISCONNECTED: status = "Disconnected";
            g_disconnected = true; g_cv.notify_all();                         break;
        case ConnectionState::NEEDS_UPDATE: status = "Needs firmware update"; break;
        default: break;
        }
    }
};

// ── EEG handler — runs on Muse callback thread ────────────────────────────────
// Each EEG packet = one sample per channel at ~220 Hz
// AF7 = EEG2 (left eye), AF8 = EEG3 (right eye)
class EEGHandler : public MuseDataListener {
public:
    void receive_muse_data_packet(const std::shared_ptr<MuseDataPacket>& pkt,
                                  const std::shared_ptr<Muse>&) override {
        if (pkt->packet_type() != MuseDataPacketType::EEG) return;
        double af7 = pkt->get_eeg_channel_value(Eeg::EEG2);
        double af8 = pkt->get_eeg_channel_value(Eeg::EEG3);
        std::lock_guard<std::mutex> lk(g_eeg_mtx);
        g_left.process(af7);
        g_right.process(af8);
    }
    void receive_muse_artifact_packet(const MuseArtifactPacket&,
                                      const std::shared_ptr<Muse>&) override {}
};

// ── ANSI helpers ──────────────────────────────────────────────────────────────
static void enable_ansi() {
    HANDLE h = GetStdHandle(STD_OUTPUT_HANDLE);
    DWORD  m = 0;
    GetConsoleMode(h, &m);
    SetConsoleMode(h, m | ENABLE_VIRTUAL_TERMINAL_PROCESSING);
}

static void cursor_up(int n) {
    std::cout << "\033[" << n << "A\033[J";
}

// ── Display thread ─────────────────────────────────────────────────────────────
static std::string peak_bar(double val, int width = 22) {
    double scale  = MIN_PEAK * 2.5;
    int    filled = (int)(std::min(val / scale, 1.0) * width);
    if (filled < 0) filled = 0;
    if (filled > width) filled = width;
    const char* col = (val > MIN_PEAK)           ? "\033[91m" :   // red
                      (val > MIN_PEAK * 0.5)     ? "\033[93m" :   // yellow
                                                   "\033[92m";    // green
    std::string bar;
    bar += col;
    bar += std::string(filled,        '#');
    bar += "\033[90m";
    bar += std::string(width - filled, '-');
    bar += "\033[0m";
    return bar;
}

static std::string blink_tag(bool lit) {
    return lit ? "\033[1;97;44m  BLINK!  \033[0m"
               : "\033[90m  ------  \033[0m";
}

static void display_loop(std::shared_ptr<ConnectionHandler> conn) {
    using namespace std::chrono_literals;
    const int ROWS = 11;

    // Print blank rows once so first cursor_up has lines to erase
    for (int i = 0; i < ROWS; ++i) std::cout << '\n';

    while (g_running && !g_disconnected) {
        std::this_thread::sleep_for(100ms);   // 10 Hz display

        double lv, rv, lb, rb;
        int    lc, rc, lsc, rsc;
        bool   l_lit, r_lit, l_spike, r_spike;
        {
            std::lock_guard<std::mutex> lk(g_eeg_mtx);
            lv      = g_left.disp_val;
            rv      = g_right.disp_val;
            lb      = g_left.baseline  < 0 ? 0 : g_left.baseline;
            rb      = g_right.baseline < 0 ? 0 : g_right.baseline;
            lc      = g_left.count;
            rc      = g_right.count;
            lsc     = g_left.spike_count;
            rsc     = g_right.spike_count;
            l_lit   = g_left.is_lit();
            r_lit   = g_right.is_lit();
            l_spike = g_left.in_spike;
            r_spike = g_right.in_spike;
        }

        // Get current time string
        SYSTEMTIME st;
        GetLocalTime(&st);
        char tbuf[16];
        snprintf(tbuf, sizeof(tbuf), "%02d:%02d:%02d", st.wHour, st.wMinute, st.wSecond);

        auto state_tag = [](bool in_spike, bool lit) -> std::string {
            if (lit)       return "\033[1;97;44m  BLINK!  \033[0m";
            if (in_spike)  return "\033[93m  spiking \033[0m";
            return               "\033[90m  ------  \033[0m";
        };

        cursor_up(ROWS);

        std::cout
            << "  Muse 2 Blink Detector   [" << tbuf << "]              \n"
            << "\n"
            << "  Channel  Peak (uV)  Base (uV)  Signal                        State\n"
            << "  -------  ---------  ---------  ----------------------------  ----------\n"
            << "  LEFT AF7 "
                << std::fixed << std::setprecision(1)
                << std::setw(9) << lv << "  "
                << std::setw(9) << lb << "  ["
                << peak_bar(lv) << "]  "
                << state_tag(l_spike, l_lit) << "\n"
            << "  RIGHT AF8 "
                << std::setw(8) << rv << "  "
                << std::setw(9) << rb << "  ["
                << peak_bar(rv) << "]  "
                << state_tag(r_spike, r_lit) << "\n"
            << "\n"
            << "  Confirmed blinks:  Left (AF7) = " << std::setw(5) << lc
            << "    Right (AF8) = " << std::setw(5) << rc << "     \n"
            << "  Spikes detected:   Left (AF7) = " << std::setw(5) << lsc
            << "    Right (AF8) = " << std::setw(5) << rsc << "     \n"
            << "\n"
            << "  Detection: RISE >" << (int)RISE_THRESH
            << " uV above baseline, then FALL back. Sustained high = ignored.\n"
            << "  Status: " << conn->status << "   Ctrl+C to quit             \n";

        std::cout << std::flush;
    }
}

// ── Entry point ───────────────────────────────────────────────────────────────
int main() {
    enable_ansi();
    winrt::init_apartment();
    std::signal(SIGINT,  on_signal);
    std::signal(SIGTERM, on_signal);

    std::cout << "=== Muse 2 Blink Detector ===\n";
    std::cout << "Make sure Bluetooth is on and the headband is paired.\n\n";

    auto mgr = MuseManagerWindows::get_instance();
    mgr->remove_from_list_after(0);

    LogManager::instance()->set_log_listener(std::make_shared<SilentLogger>());

    auto scanner   = std::make_shared<MuseScanner>(mgr);
    auto conn_hdlr = std::make_shared<ConnectionHandler>();
    auto data_hdlr = std::make_shared<EEGHandler>();

    mgr->set_muse_listener(scanner);
    mgr->start_listening();
    std::cout << "[SCAN] Scanning for Muse headbands...\n";

    {
        std::unique_lock<std::mutex> lk(g_mtx);
        g_cv.wait(lk, [] { return g_muse_found.load() || !g_running; });
    }
    if (!g_running) { mgr->stop_listening(); return 0; }

    mgr->stop_listening();
    std::cout << "[SCAN] Found: " << g_muse->get_name() << "\n";

    g_muse->register_connection_listener(conn_hdlr);
    g_muse->register_data_listener(data_hdlr, MuseDataPacketType::EEG);
    g_muse->set_preset(MusePreset::PRESET_21);
    g_muse->run_asynchronously();

    std::thread display_thread(display_loop, conn_hdlr);

    {
        std::unique_lock<std::mutex> lk(g_mtx);
        g_cv.wait(lk, [] { return !g_running || g_disconnected.load(); });
    }

    g_running = false;
    if (display_thread.joinable()) display_thread.join();

    if (!g_disconnected.load()) {
        std::cout << "\n[MAIN] Disconnecting...\n";
        g_muse->disconnect();
        std::unique_lock<std::mutex> lk(g_mtx);
        g_cv.wait_for(lk, std::chrono::seconds(3),
                      [] { return g_disconnected.load(); });
    }

    std::cout << "[MAIN] Goodbye.\n";
    return 0;
}
