// =============================================================================
//  predictive_tdoa_gate.cpp
// =============================================================================
#include "ips_nodes_cpp/predictive_tdoa_gate.hpp"

#include <cmath>
#include <sstream>

namespace ips {

Eigen::Vector3d PredictiveTdoaGate::jacobian_tdoa(
        const Eigen::Vector3d& x_pred, int slave_idx) const {
    const Eigen::Vector3d& a_i = cfg_.anchors[slave_idx];
    const Eigen::Vector3d& a_r = cfg_.anchors[0];   // reference at index 0
    const Eigen::Vector3d diff_i = x_pred - a_i;
    const Eigen::Vector3d diff_r = x_pred - a_r;
    const double norm_i = diff_i.norm();
    const double norm_r = diff_r.norm();
    if (norm_i < 1e-9 || norm_r < 1e-9) {
        return Eigen::Vector3d::Zero();
    }
    // ∂(d_i − d_r)/∂x = u_i − u_r
    return diff_i / norm_i - diff_r / norm_r;
}

double PredictiveTdoaGate::predicted_tdoa(
        const Eigen::Vector3d& x_pred, int slave_idx) const {
    const double d_i = (x_pred - cfg_.anchors[slave_idx]).norm();
    const double d_r = (x_pred - cfg_.anchors[0]).norm();
    return d_i - d_r;
}

int PredictiveTdoaGate::filter(std::array<double, 4>& toa_ordered,
                                const Eigen::Vector3d& x_pred,
                                const Eigen::Matrix3d& P_pos) {
    // Warmup: pass-through
    if (n_seen_ < cfg_.warmup) {
        ++n_seen_;
        return 0;
    }
    const double toa_ref = toa_ordered[0];
    int clipped_count = 0;

    // For each non-ref anchor (indices 1..3)
    for (int idx = 1; idx < 4; ++idx) {
        const int slot = idx - 1;          // stats indices 0..2 for SA3..SA5
        stats_.n_total[slot] += 1;

        // Actual TDoA in meters
        const double tdoa_meas = C_MS * (toa_ordered[idx] - toa_ref);
        const double tdoa_pred = predicted_tdoa(x_pred, idx);
        const double residual  = tdoa_meas - tdoa_pred;

        // Adaptive threshold: τ = k × sqrt(J · P · Jᵀ + σ_tdoa²)
        const Eigen::Vector3d J = jacobian_tdoa(x_pred, idx);
        const double var_geom = J.transpose() * P_pos * J;
        const double var_total = std::max(
            var_geom + cfg_.sigma_tdoa * cfg_.sigma_tdoa, 1e-6);
        const double sigma_pred = std::sqrt(var_total);
        const double threshold  = cfg_.huber_k * sigma_pred;

        // Track residual stats (Welford-style would be more accurate, but
        // simple accumulator is fine for diagnostics)
        stats_.res_sum[slot]    += residual;
        stats_.res_sum_sq[slot] += residual * residual;

        // Huber clip
        if (std::fabs(residual) > threshold) {
            const double sign = (residual > 0.0) ? 1.0 : -1.0;
            const double tdoa_clipped = tdoa_pred + sign * threshold;
            // Convert back to ToA: t_sa = t_ref + tdoa / c
            toa_ordered[idx] = toa_ref + tdoa_clipped / C_MS;
            stats_.n_clipped[slot] += 1;
            ++clipped_count;
        }
    }
    ++n_seen_;
    return clipped_count;
}

std::string PredictiveTdoaGate::stats_summary() const {
    std::ostringstream oss;
    oss << "[Layer 1] clip stats: ";
    for (int idx = 1; idx < 4; ++idx) {
        const int slot = idx - 1;
        const long tot = stats_.n_total[slot];
        const long clp = stats_.n_clipped[slot];
        const double pct = tot ? 100.0 * static_cast<double>(clp) / tot : 0.0;
        double mean_res_cm = 0.0;
        if (tot > 0) {
            mean_res_cm = (stats_.res_sum[slot] / tot) * 100.0;
        }
        oss << "SA" << cfg_.rx_ids_ordered[idx]
            << ": " << clp << "/" << tot
            << " (" << static_cast<int>(pct * 10) / 10.0 << "%)"
            << " res_mean=" << static_cast<int>(mean_res_cm * 10) / 10.0 << "cm  ";
    }
    return oss.str();
}

} // namespace ips
