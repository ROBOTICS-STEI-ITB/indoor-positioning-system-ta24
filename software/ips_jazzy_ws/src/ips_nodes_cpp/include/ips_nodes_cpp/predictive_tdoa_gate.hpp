// =============================================================================
//  predictive_tdoa_gate.hpp
//  Layer 1 — Prediction-based TDoA gating with Huber clipping.
//
//  Concept (port of two_layer_tdoa_estimator.py PredictiveTDoAGate):
//    For each blink, use the KF position prediction x_pred + covariance P_pos
//    to compute what each TDoA "should" be from anchor geometry. Compare to
//    the actual TDoA measurement; if residual exceeds adaptive threshold,
//    Huber-clip it back to threshold (not reject — clip).
//
//  Adaptive threshold τ_i = k_huber × sqrt(J_i · P_pos · J_iᵀ + σ_tdoa²)
//    where J_i = (x_pred − a_i)/||x_pred − a_i|| − (x_pred − a_ref)/||x_pred − a_ref||
//
//  Motion-compatible: prediction comes from KF (state already includes velocity),
//  so threshold tracks moving target as long as KF tracks it.
// =============================================================================
#pragma once

#include <Eigen/Dense>
#include <array>
#include <cstddef>
#include <map>

namespace ips {

struct PredictiveTdoaStats {
    long n_total[4]   = {0, 0, 0, 0};   // per non-ref anchor (in rx_ids order, skipping ref)
    long n_clipped[4] = {0, 0, 0, 0};
    // Rolling running mean/std of residuals for diagnostics (optional)
    double res_sum[4]    = {0, 0, 0, 0};
    double res_sum_sq[4] = {0, 0, 0, 0};
};

class PredictiveTdoaGate {
public:
    struct Config {
        std::array<int, 4>   rx_ids_ordered;   // index 0 = reference (MA)
        std::array<Eigen::Vector3d, 4> anchors;// position per rx_ids order
        double               sigma_tdoa = 0.10;  // baseline TDoA noise (m)
        double               huber_k    = 2.5;   // threshold in sigma
        std::size_t          warmup     = 20;
    };

    explicit PredictiveTdoaGate(const Config& cfg) : cfg_(cfg) {}

    // Filter one blink. toa_ordered is index-aligned with cfg_.rx_ids_ordered
    // (index 0 = ToA at reference anchor). x_pred is the KF position prediction
    // BEFORE measurement update, and P_pos is the 3×3 position covariance
    // (upper-left block of the 6×6 KF covariance).
    //
    // Modifies toa_ordered in-place: any TDoA whose residual exceeds threshold
    // is clipped (the corresponding ToA is re-computed from the clipped TDoA).
    //
    // Returns number of anchors clipped this blink.
    int filter(std::array<double, 4>&         toa_ordered,
               const Eigen::Vector3d&         x_pred,
               const Eigen::Matrix3d&         P_pos);

    const PredictiveTdoaStats& stats() const { return stats_; }
    std::size_t n_seen() const                { return n_seen_; }

    // For debugging: print average clip rate per anchor
    std::string stats_summary() const;

private:
    Eigen::Vector3d jacobian_tdoa(const Eigen::Vector3d& x_pred,
                                  int slave_idx) const;
    double          predicted_tdoa(const Eigen::Vector3d& x_pred,
                                   int slave_idx) const;

    static constexpr double C_MS = 299792458.0;

    Config              cfg_;
    PredictiveTdoaStats stats_;
    std::size_t         n_seen_ = 0;
};

} // namespace ips
