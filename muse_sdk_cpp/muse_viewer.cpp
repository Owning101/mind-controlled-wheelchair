// muse_viewer.cpp - Muse 2 EEG live console viewer
// Build: Visual Studio x64 Release
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
#include <string>

#define NOMINMAX
#include <windows.h>
#include <winrt/Windows.Foundation.h>

#pragma comment(lib, "libmuse-wrt")
#pragma comment(lib, "windowsapp")

#include "muse.h"

using namespace interaxon::bridge;

// --- ANSI cursor helpers (ASCII-safe) ----------------------------------------
static void enable_ansi() {
    HANDLE h = GetStdHandle(STD_OUTPUT_HANDLE);
    DWORD mode = 0;
    GetConsoleMode(h, &mode);
    SetConsoleMode(h, mode | ENABLE_VIRTUAL_TERMINAL_PROCESSING);
}

static void cursor_up(int n) {
    for (int i = 0; i < n; ++i)
        std::cout << "\033[A\033[2K";
}

// --- Shared live data --------------------------------------------------------
struct LiveData {
    std::mutex mtx;
    double eeg[4]   = {};
    double alpha[4] = {};
    bool   blink    = false;
    bool   jaw      = false;
    bool   headband = true;
};
static LiveData g_data;

// --- Control state -----------------------------------------------------------
static std::atomic<bool>       g_running{true};
static std::shared_ptr<Muse>   g_muse;
static std::mutex              g_mtx;
static std::condition_variable g_cv;
static std::atomic<bool>       g_muse_found{false};
static std::atomic<bool>       g_disconnected{false};

static void on_signal(int) { g_running = false; g_cv.notify_all(); }

// --- Silent log listener (suppresses SDK console spam) -----------------------
class SilentLogger : public LogListener {
public:
    void receive_log(const LogPacket&) override {}
};

// --- Muse scanner ------------------------------------------------------------
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

// --- Connection handler ------------------------------------------------------
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

// --- EEG + artifact handler --------------------------------------------------
class EEGHandler : public MuseDataListener {
public:
    void receive_muse_data_packet(const std::shared_ptr<MuseDataPacket>& pkt,
                                  const std::shared_ptr<Muse>&) override {
        std::lock_guard<std::mutex> lk(g_data.mtx);
        switch (pkt->packet_type()) {
        case MuseDataPacketType::EEG:
            g_data.eeg[0] = pkt->get_eeg_channel_value(Eeg::EEG1);
            g_data.eeg[1] = pkt->get_eeg_channel_value(Eeg::EEG2);
            g_data.eeg[2] = pkt->get_eeg_channel_value(Eeg::EEG3);
            g_data.eeg[3] = pkt->get_eeg_channel_value(Eeg::EEG4);
            break;
        case MuseDataPacketType::ALPHA_ABSOLUTE:
            g_data.alpha[0] = pkt->get_eeg_channel_value(Eeg::EEG1);
            g_data.alpha[1] = pkt->get_eeg_channel_value(Eeg::EEG2);
            g_data.alpha[2] = pkt->get_eeg_channel_value(Eeg::EEG3);
            g_data.alpha[3] = pkt->get_eeg_channel_value(Eeg::EEG4);
            break;
        default: break;
        }
    }
    void receive_muse_artifact_packet(const MuseArtifactPacket& pkt,
                                      const std::shared_ptr<Muse>&) override {
        std::lock_guard<std::mutex> lk(g_data.mtx);
        g_data.blink    = pkt.blink;
        g_data.jaw      = pkt.jaw_clench;
        g_data.headband = pkt.headband_on;
    }
};

// --- Display loop (4 Hz, fixed position, pure ASCII) -------------------------
static void display_loop(std::shared_ptr<ConnectionHandler> conn) {
    using namespace std::chrono_literals;

    // Print static header once — pure ASCII, no box-drawing chars
    std::cout << "\n";
    std::cout << "+----------+----------+----------+----------+---------+\n";
    std::cout << "|          |   TP9    |   AF7    |   AF8    |  TP10   |\n";
    std::cout << "+----------+----------+----------+----------+---------+\n";
    std::cout << "| EEG (uV) |          |          |          |         |\n";
    std::cout << "| Alpha    |          |          |          |         |\n";
    std::cout << "+----------+----------+----------+----------+---------+\n";
    std::cout << "| Status   :                                           |\n";
    std::cout << "| Artifact :                                           |\n";
    std::cout << "+-----------------------------------------------------+\n";
    std::cout << "  Ctrl+C to quit\n";

    // Lines we rewrite each tick (everything from EEG row down)
    const int REDRAW_LINES = 6;

    auto col = [](double v) -> std::string {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(1) << std::setw(8) << v;
        return ss.str();
    };
    auto pad = [](std::string s, int w) -> std::string {
        if ((int)s.size() < w) s += std::string(w - s.size(), ' ');
        return s.substr(0, w);
    };

    while (g_running && !g_disconnected) {
        std::this_thread::sleep_for(250ms);

        double eeg[4], alpha[4];
        bool   blink, jaw, headband;
        {
            std::lock_guard<std::mutex> lk(g_data.mtx);
            std::copy(g_data.eeg,   g_data.eeg   + 4, eeg);
            std::copy(g_data.alpha, g_data.alpha  + 4, alpha);
            blink    = g_data.blink;
            jaw      = g_data.jaw;
            headband = g_data.headband;
        }

        std::string art = headband ? "none" : "HEADBAND OFF";
        if (blink) art = "** BLINK **";
        if (jaw)   art = "** JAW CLENCH **";

        cursor_up(REDRAW_LINES);

        std::cout << "| EEG (uV) |" << col(eeg[0])   << " |" << col(eeg[1])   << " |"
                                     << col(eeg[2])   << " |" << col(eeg[3])   << " |\n";
        std::cout << "| Alpha    |" << col(alpha[0]) << " |" << col(alpha[1]) << " |"
                                     << col(alpha[2]) << " |" << col(alpha[3]) << " |\n";
        std::cout << "+----------+----------+----------+----------+---------+\n";
        std::cout << "| Status   : " << pad(conn->status, 41) << "|\n";
        std::cout << "| Artifact : " << pad(art,          41) << "|\n";
        std::cout << "+-----------------------------------------------------+\n";
        std::cout << "  Ctrl+C to quit\n";
        std::cout << std::flush;
    }
}

// --- Entry point -------------------------------------------------------------
int main() {
    enable_ansi();
    winrt::init_apartment();
    std::signal(SIGINT,  on_signal);
    std::signal(SIGTERM, on_signal);

    std::cout << "=== Muse 2 EEG Viewer ===\n";
    std::cout << "Make sure Bluetooth is on and the headband is paired.\n\n";

    auto mgr = MuseManagerWindows::get_instance();
    mgr->remove_from_list_after(0);

    // Silence SDK log messages so they don't corrupt the display
    auto silent_log = std::make_shared<SilentLogger>();
    LogManager::instance()->set_log_listener(silent_log);

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
    std::cout << "[CONN] Connecting...\n";

    g_muse->register_connection_listener(conn_hdlr);
    g_muse->register_data_listener(data_hdlr, MuseDataPacketType::EEG);
    g_muse->register_data_listener(data_hdlr, MuseDataPacketType::ALPHA_ABSOLUTE);
    g_muse->register_data_listener(data_hdlr, MuseDataPacketType::ARTIFACTS);
    g_muse->set_preset(MusePreset::PRESET_1031); // muse2025 (Muse S Athena)
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
