// =============================================================================
//  sync_engine.hpp
//  LI-KF Clock Synchronization Engine (Zhang et al. 2024) — ROS2 build.
//
//  CHANGES vs offline C++ version:
//    1. NO CSV writers (the Python recorder_node handles logging at the
//       message layer).
//    2. NO print/cout in hot path (replaced with std::function callbacks
//       so the node owner can route to RCLCPP_INFO/WARN/DEBUG as needed).
//    3. GLITCH PROTECTION + ROLLBACK added in run_step1() — rejects CCP
//       frames with delta_k outside [1-DELTA_K_MAX_DEV, 1+DELTA_K_MAX_DEV]
//       and rolls back curr_sync so subsequent tag blinks aren't corrupted.
//       (This was missing from offline C++; present in Python sync.py.)
//    4. process_pending_tags(slave_id) instead of iterate-all-slaves.
//       Reduces per-call work by 4× in the typical 4-slave deployment.
//    5. NO internal std::mutex — relies on caller (rclcpp executor) to
//       serialize. SingleThreadedExecutor already guarantees this; for
//       MultiThreadedExecutor the node-level mutex is sufficient.
//    6. Fixed-size Eigen matrices throughout → zero heap alloc in hot path.
// =============================================================================
#pragma once

#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>

#include "ips_nodes_cpp/dw1000_constants.hpp"
#include "ips_nodes_cpp/kalman_3state.hpp"
#include "ips_nodes_cpp/timestamp_unwrapper.hpp"

namespace ips {

// -----------------------------------------------------------------------------
// Output record: emitted to node for every (slave, blink) that completes
// synchronization through Step 2 + (optional) Step 3 + Step 4.
// -----------------------------------------------------------------------------
struct CorrectedToAResult {
    int     tag_id;
    int     slave_id;
    int     tag_seq;
    int     sync_seq;
    int64_t toa_final_dtu;
    int64_t toa_li_dtu;
    int64_t toa_kf_dtu;
    int64_t prop_delay_dtu;
    double  delta_k;
    int64_t theta_k_dtu;
    bool    kf_used;
};

using ToACallback = std::function<void(const CorrectedToAResult&)>;

// -----------------------------------------------------------------------------
// Diagnostic event callback (low-rate, for RCLCPP_INFO/WARN routing).
// Severity: 0=DEBUG, 1=INFO, 2=WARN, 3=ERROR
// -----------------------------------------------------------------------------
using LogCallback = std::function<void(int severity, const std::string& msg)>;

// -----------------------------------------------------------------------------
// Internal record types
// -----------------------------------------------------------------------------
struct MasterClockRecord {
    int     master_id;
    int     seq;
    int64_t Tk;        // unwrapped
};

struct SlaveSyncRecord {
    int     slave_id;
    int     source_id;
    int     seq;
    int64_t Rk;        // unwrapped
};

struct TagBlinkRecord {
    int     slave_id;
    int     tag_id;
    int     tag_seq;
    int64_t ToA;       // unwrapped
};

struct SyncPair {
    int     seq;
    int64_t Tk;
    int64_t Rk;
    int     master_id;
    int     slave_id;
};

// -----------------------------------------------------------------------------
// Per-slave context
// -----------------------------------------------------------------------------
struct SlaveAnchorContext {
    explicit SlaveAnchorContext(int slave_id_)
        : slave_id(slave_id_),
          clock_unwrapper("slave" + std::to_string(slave_id_) + "_clock") {}

    int                       slave_id;
    TimestampUnwrapper40      clock_unwrapper;

    std::optional<SyncPair>   prev_sync;
    std::optional<SyncPair>   curr_sync;

    double  delta_k             = 1.0;
    int64_t theta_k             = 0;
    bool    li_ready            = false;
    bool    delta_k_set         = false;
    bool    theta_k_set         = false;

    KalmanFilter3State kf;
    double             last_ccp_interval_s = 0.150;

    int64_t prop_delay_dtu      = 0;
    int     sync_count          = 0;
    int     glitch_count        = 0;   // # of CCPs rejected by glitch protection
};

// -----------------------------------------------------------------------------
// Main engine
// -----------------------------------------------------------------------------
class SyncEngine {
public:
    struct Config {
        // Per-slave propagation delays in microseconds.
        // Empty → no compensation (canonical TDoA mode).
        std::map<int, double> propagation_delay_us;

        // Enable Step 3 KF refinement (default ON, matches Zhang 2024).
        bool kf_enabled = true;
    };

    SyncEngine() : SyncEngine(Config{}) {}
    explicit SyncEngine(const Config& cfg) : cfg_(cfg) {}

    // ---- Callback registration (set once at construction time) -----------
    void set_toa_callback(ToACallback cb) { toa_cb_ = std::move(cb); }
    void set_log_callback(LogCallback cb) { log_cb_ = std::move(cb); }

    // ---- Input handlers --------------------------------------------------
    void add_master_clock(int master_id, int session_id, int seq,
                          const std::string& tk_hex);
    void add_slave_master_rx(int slave_id, int source_id, int seq,
                             const std::string& rx_hex);
    void add_slave_tag_rx(int slave_id, int tag_id, int seq,
                          const std::string& toa_hex);
    void handle_ma_reset(int master_id, int session_id);
    void handle_ma_hello(int master_id, int session_id);

    // ---- Status -----------------------------------------------------------
    struct SyncStatus {
        int                   sync_count    = 0;
        bool                  li_ready      = false;
        std::optional<double> delta_k;
        std::optional<double> theta_k_us;
        bool                  kf_converged  = false;
        std::optional<double> kf_drift;
        std::optional<double> kf_drift_rate;
        int                   kf_updates    = 0;
        int                   glitch_count  = 0;
    };
    std::map<int, SyncStatus> get_sync_status() const;

    int reset_count()              const { return reset_count_; }
    int dropped_packets_on_reset() const { return dropped_packets_on_reset_; }

private:
    SlaveAnchorContext& get_or_create_slave(int slave_id);
    void try_build_sync_pair(int slave_id, int seq);
    bool run_step1(SlaveAnchorContext& ctx);            // returns false on glitch
    void run_step3_kf(SlaveAnchorContext& ctx);
    void process_pending_tags_for(int slave_id);        // OPTIMIZED: one slave only
    void reset_all_state(const std::string& reason);

    inline void log(int sev, const std::string& msg) const {
        if (log_cb_) log_cb_(sev, msg);
    }

    Config       cfg_;
    ToACallback  toa_cb_;
    LogCallback  log_cb_;

    std::unordered_map<int, MasterClockRecord>         master_clock_by_seq_;
    std::map<std::pair<int, int>, SlaveSyncRecord>     slave_sync_by_seq_;
    std::map<int, SlaveAnchorContext>                  slave_contexts_;
    std::map<int, std::deque<TagBlinkRecord>>          pending_tags_by_slave_;

    TimestampUnwrapper40 master_tk_unwrapper_{"master_tk"};

    std::optional<int> current_ma_session_;
    int                current_ma_seq_max_       = -1;
    int                reset_count_              = 0;
    int                dropped_packets_on_reset_ = 0;
};

} // namespace ips
