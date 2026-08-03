// =============================================================================
//  sync_engine.cpp
//  Implementation of LI-KF SyncEngine for ROS2.
// =============================================================================
#include "ips_nodes_cpp/sync_engine.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace ips {

// =============================================================================
// Slave context management
// =============================================================================

SlaveAnchorContext& SyncEngine::get_or_create_slave(int slave_id) {
    auto it = slave_contexts_.find(slave_id);
    if (it == slave_contexts_.end()) {
        auto ins = slave_contexts_.emplace(std::piecewise_construct,
                                           std::forward_as_tuple(slave_id),
                                           std::forward_as_tuple(slave_id));
        auto delay_it = cfg_.propagation_delay_us.find(slave_id);
        if (delay_it != cfg_.propagation_delay_us.end()) {
            ins.first->second.prop_delay_dtu =
                microseconds_to_dtu(delay_it->second);
        }
        return ins.first->second;
    }
    return it->second;
}

// =============================================================================
// Reset handling
// =============================================================================

void SyncEngine::reset_all_state(const std::string& reason) {
    reset_count_ += 1;
    int pending_count = 0;
    for (auto& kv : pending_tags_by_slave_)
        pending_count += static_cast<int>(kv.second.size());
    dropped_packets_on_reset_ += pending_count;

    std::ostringstream oss;
    oss << "[RESET #" << reset_count_ << "] " << reason
        << " | cleared " << master_clock_by_seq_.size() << " CCPs, "
        << slave_sync_by_seq_.size() << " sync pairs, "
        << pending_count << " pending tags";
    log(2 /*WARN*/, oss.str());

    master_clock_by_seq_.clear();
    slave_sync_by_seq_.clear();
    pending_tags_by_slave_.clear();
    master_tk_unwrapper_.reset();
    for (auto& kv : slave_contexts_) {
        auto& ctx = kv.second;
        ctx.clock_unwrapper.reset();
        ctx.li_ready    = false;
        ctx.curr_sync.reset();
        ctx.prev_sync.reset();
        ctx.delta_k     = 1.0;
        ctx.theta_k     = 0;
        ctx.delta_k_set = false;
        ctx.theta_k_set = false;
        ctx.kf.reset();
    }
    current_ma_seq_max_ = -1;
}

void SyncEngine::handle_ma_reset(int master_id, int session_id) {
    if (!current_ma_session_.has_value()) {
        current_ma_session_ = session_id;
        std::ostringstream oss;
        oss << "MA" << master_id << " initial session: " << session_id;
        log(1, oss.str());
    } else if (*current_ma_session_ != session_id) {
        std::ostringstream oss;
        oss << "MA" << master_id << " session change "
            << *current_ma_session_ << " -> " << session_id;
        reset_all_state(oss.str());
        current_ma_session_ = session_id;
    }
}

void SyncEngine::handle_ma_hello(int master_id, int session_id) {
    if (!current_ma_session_.has_value()) {
        current_ma_session_ = session_id;
    } else if (*current_ma_session_ != session_id) {
        std::ostringstream oss;
        oss << "MA" << master_id << " silent session change "
            << *current_ma_session_ << " -> " << session_id;
        reset_all_state(oss.str());
        current_ma_session_ = session_id;
    }
}

// =============================================================================
// Input handlers
// =============================================================================

void SyncEngine::add_master_clock(int master_id, int session_id, int seq,
                                  const std::string& tk_hex) {
    // Session change detection
    if (session_id != 0) {
        if (!current_ma_session_.has_value()) {
            current_ma_session_ = session_id;
        } else if (*current_ma_session_ != session_id) {
            std::ostringstream oss;
            oss << "MA" << master_id << " session change via CCP "
                << *current_ma_session_ << " -> " << session_id;
            reset_all_state(oss.str());
            current_ma_session_ = session_id;
        }
    }
    // Seq reset detection
    if (current_ma_seq_max_ > 0 &&
        seq < current_ma_seq_max_ - SEQ_RESET_THRESHOLD) {
        std::ostringstream oss;
        oss << "MA seq reset detected: " << current_ma_seq_max_
            << " -> " << seq;
        reset_all_state(oss.str());
    }
    current_ma_seq_max_ = std::max(current_ma_seq_max_, seq);

    const int64_t tk_raw40  = hex40_to_int(tk_hex);
    const int64_t tk_unwrap = master_tk_unwrapper_.unwrap(tk_raw40);

    MasterClockRecord rec;
    rec.master_id = master_id;
    rec.seq       = seq;
    rec.Tk        = tk_unwrap;
    master_clock_by_seq_[seq] = rec;

    // Try to build sync pair for every known slave at this seq
    for (auto& kv : slave_contexts_) {
        try_build_sync_pair(kv.first, seq);
    }
}

void SyncEngine::add_slave_master_rx(int slave_id, int source_id, int seq,
                                     const std::string& rx_hex) {
    SlaveAnchorContext& ctx = get_or_create_slave(slave_id);
    const int64_t rk_raw40  = hex40_to_int(rx_hex);
    const int64_t rk_unwrap = ctx.clock_unwrapper.unwrap(rk_raw40);

    SlaveSyncRecord rec;
    rec.slave_id  = slave_id;
    rec.source_id = source_id;
    rec.seq       = seq;
    rec.Rk        = rk_unwrap;
    slave_sync_by_seq_[{slave_id, seq}] = rec;

    try_build_sync_pair(slave_id, seq);
}

void SyncEngine::add_slave_tag_rx(int slave_id, int tag_id, int seq,
                                  const std::string& toa_hex) {
    SlaveAnchorContext& ctx = get_or_create_slave(slave_id);
    const int64_t toa_raw40  = hex40_to_int(toa_hex);
    const int64_t toa_unwrap = ctx.clock_unwrapper.unwrap(toa_raw40);

    TagBlinkRecord tag;
    tag.slave_id = slave_id;
    tag.tag_id   = tag_id;
    tag.tag_seq  = seq;
    tag.ToA      = toa_unwrap;
    pending_tags_by_slave_[slave_id].push_back(tag);

    // OPTIMIZATION: only process this slave's queue, not all slaves
    process_pending_tags_for(slave_id);
}

// =============================================================================
// Sync-pair building
// =============================================================================

void SyncEngine::try_build_sync_pair(int slave_id, int seq) {
    auto m_it = master_clock_by_seq_.find(seq);
    if (m_it == master_clock_by_seq_.end()) return;
    auto s_it = slave_sync_by_seq_.find({slave_id, seq});
    if (s_it == slave_sync_by_seq_.end()) return;

    const MasterClockRecord& master_rec = m_it->second;
    const SlaveSyncRecord&   slave_rec  = s_it->second;

    SlaveAnchorContext& ctx = get_or_create_slave(slave_id);

    SyncPair new_sync;
    new_sync.seq       = seq;
    new_sync.Tk        = master_rec.Tk;
    new_sync.Rk        = slave_rec.Rk;
    new_sync.master_id = master_rec.master_id;
    new_sync.slave_id  = slave_id;

    if (!ctx.curr_sync.has_value()) {
        ctx.curr_sync = new_sync;
        return;
    }
    if (ctx.curr_sync->seq == seq) return;
    if (seq <= ctx.curr_sync->seq) return;   // out-of-order: ignore

    // -------- Critical section: try to advance, rollback on glitch --------
    auto saved_prev = ctx.prev_sync;
    auto saved_curr = ctx.curr_sync;
    ctx.prev_sync   = ctx.curr_sync;
    ctx.curr_sync   = new_sync;

    if (!run_step1(ctx)) {
        // GLITCH PROTECTION: rollback so tag corrections keep using last
        // good sync pair. This was the bug missing from the offline C++.
        ctx.prev_sync = saved_prev;
        ctx.curr_sync = saved_curr;
        return;
    }

    if (cfg_.kf_enabled) run_step3_kf(ctx);
    process_pending_tags_for(slave_id);
}

// =============================================================================
// Step 1: Linear Interpolation (with glitch protection)
// =============================================================================

bool SyncEngine::run_step1(SlaveAnchorContext& ctx) {
    if (!ctx.prev_sync.has_value() || !ctx.curr_sync.has_value()) return false;

    const int64_t Tk_1 = ctx.prev_sync->Tk;
    const int64_t Rk_1 = ctx.prev_sync->Rk;
    const int64_t Tk   = ctx.curr_sync->Tk;
    const int64_t Rk   = ctx.curr_sync->Rk;

    const int64_t denom = Rk - Rk_1;
    if (denom == 0) {
        ctx.li_ready = false;
        return false;
    }

    // Compute delta_k via exact int subtraction then float division.
    const double new_delta_k =
        static_cast<double>(Tk - Tk_1) / static_cast<double>(denom);

    // ---- GLITCH PROTECTION ----------------------------------------------
    // Crystal drift physical limit: ±50 ppm = |delta_k - 1| < 5e-5.
    // Threshold 1e-3 (1000 ppm) is very generous; anything beyond = DW1000
    // RX timestamp glitch. Reject this CCP frame.
    if (std::fabs(new_delta_k - 1.0) > DELTA_K_MAX_DEV) {
        ctx.glitch_count += 1;
        std::ostringstream oss;
        oss << "[GLITCH] SA" << ctx.slave_id
            << " CCP seq=" << ctx.curr_sync->seq
            << ": delta_k=" << new_delta_k << " rejected (count="
            << ctx.glitch_count << ")";
        log(2 /*WARN*/, oss.str());
        return false;  // caller rolls back curr_sync
    }

    ctx.delta_k             = new_delta_k;
    ctx.theta_k             = Tk - Rk;
    ctx.delta_k_set         = true;
    ctx.theta_k_set         = true;
    ctx.li_ready            = true;
    ctx.sync_count         += 1;
    ctx.last_ccp_interval_s = dtu_to_seconds(Tk - Tk_1);
    return true;
}

// =============================================================================
// Step 3: Kalman filter update
// =============================================================================

void SyncEngine::run_step3_kf(SlaveAnchorContext& ctx) {
    if (!ctx.li_ready || !ctx.curr_sync.has_value()) return;
    auto& kf = ctx.kf;
    const double T_ccp_s = ctx.last_ccp_interval_s;
    const double Tk_s    = dtu_to_seconds(ctx.curr_sync->Tk);

    if (!kf.initialized()) {
        kf.initialize(Tk_s);
    } else {
        kf.predict_and_update(T_ccp_s, Tk_s);
    }
}

// =============================================================================
// Step 2 + 3 + 4: Tag blink correction (OPTIMIZED: one slave at a time)
// =============================================================================

void SyncEngine::process_pending_tags_for(int slave_id) {
    auto q_it = pending_tags_by_slave_.find(slave_id);
    if (q_it == pending_tags_by_slave_.end()) return;
    auto& queue = q_it->second;

    auto ctx_it = slave_contexts_.find(slave_id);
    if (ctx_it == slave_contexts_.end()) return;
    SlaveAnchorContext& ctx = ctx_it->second;

    if (!ctx.li_ready || !ctx.curr_sync.has_value()) return;

    const int64_t Tk       = ctx.curr_sync->Tk;
    const int64_t Rk       = ctx.curr_sync->Rk;
    const int     sync_seq = ctx.curr_sync->seq;

    while (!queue.empty()) {
        TagBlinkRecord tag = queue.front();

        // Skip tags whose ToA is before current sync's Rk
        if (tag.ToA <= Rk) {
            queue.pop_front();
            continue;
        }
        queue.pop_front();

        // ---- Step 2: Linear Interpolation correction --------------------
        const int64_t Delta = tag.ToA - Rk;
        const double  li_corrected_toa =
            static_cast<double>(Tk) + static_cast<double>(Delta) * ctx.delta_k;
        const int64_t li_corrected_int =
            static_cast<int64_t>(std::llround(li_corrected_toa));

        // ---- Step 3: Kalman Filter refinement ---------------------------
        int64_t kf_corrected_int = li_corrected_int;
        bool    kf_used          = false;
        if (cfg_.kf_enabled && ctx.kf.converged()) {
            const double dt_master_s = dtu_to_seconds(Delta) * ctx.delta_k;
            const double kf_pred_s   = ctx.kf.predict_toa(dt_master_s);
            kf_corrected_int = seconds_to_dtu(kf_pred_s);
            kf_used = true;
        }

        // ---- Step 4: Propagation delay compensation ---------------------
        const int64_t final_corrected_int =
            kf_corrected_int + ctx.prop_delay_dtu;

        // ---- Emit to callback (node publishes ROS message) -------------
        if (toa_cb_) {
            CorrectedToAResult r;
            r.tag_id         = tag.tag_id;
            r.slave_id       = slave_id;
            r.tag_seq        = tag.tag_seq;
            r.sync_seq       = sync_seq;
            r.toa_final_dtu  = final_corrected_int;
            r.toa_li_dtu     = li_corrected_int;
            r.toa_kf_dtu     = kf_corrected_int;
            r.prop_delay_dtu = ctx.prop_delay_dtu;
            r.delta_k        = ctx.delta_k;
            r.theta_k_dtu    = ctx.theta_k;
            r.kf_used        = kf_used;
            toa_cb_(r);
        }
    }
}

// =============================================================================
// Status reporting
// =============================================================================

std::map<int, SyncEngine::SyncStatus> SyncEngine::get_sync_status() const {
    std::map<int, SyncStatus> status;
    for (const auto& kv : slave_contexts_) {
        const auto& ctx = kv.second;
        SyncStatus s;
        s.sync_count   = ctx.sync_count;
        s.li_ready     = ctx.li_ready;
        s.glitch_count = ctx.glitch_count;
        if (ctx.delta_k_set) s.delta_k    = ctx.delta_k;
        if (ctx.theta_k_set) s.theta_k_us = dtu_to_microseconds(ctx.theta_k);
        s.kf_converged = cfg_.kf_enabled ? ctx.kf.converged() : false;
        if (ctx.kf.initialized()) {
            s.kf_drift      = ctx.kf.drift();
            s.kf_drift_rate = ctx.kf.drift_rate();
        }
        s.kf_updates = ctx.kf.update_count();
        status[kv.first] = s;
    }
    return status;
}

} // namespace ips
