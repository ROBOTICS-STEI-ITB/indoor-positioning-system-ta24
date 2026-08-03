// =============================================================================
//  dzs_filter.hpp
//  Asymmetric Dead-Zone Statistical (DZS) outlier filter for TDoA distances.
//
//  For each anchor pair (ref, sa_i), maintains a sliding window of recent
//  TDoA*c values in meters, computes median + asymmetric MAD-based thresholds:
//    upper = median + k_pos * 1.4826 * MAD
//    lower = median - k_neg * 1.4826 * MAD
//  Reject any sample whose TDoA falls outside the band for ANY pair.
//
//  Also includes a simple G1 triangular-inequality pre-filter:
//    |tdoa_i| > g1_margin * d(SA_i, SA_ref)  →  impossible, reject.
// =============================================================================
#pragma once

#include <cstddef>
#include <deque>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace ips {

struct DzsThresholds {
    double pos_cm = 0.0;   // upper margin from median in cm
    double neg_cm = 0.0;   // lower margin from median in cm
    bool   ready  = false;
};

struct DzsStats {
    long n_total      = 0;
    long n_inliers    = 0;
    long n_outliers   = 0;
    long n_g1_reject  = 0;
    double reject_rate() const {
        return (n_total == 0) ? 0.0
            : static_cast<double>(n_outliers + n_g1_reject) / n_total;
    }
};

class AsymmetricDzsFilter {
public:
    struct Config {
        std::size_t window    = 100;
        double      k_pos     = 2.5;
        double      k_neg     = 3.5;
        double      mad_min_m = 0.03;    // floor to avoid tight thresholds
        std::size_t warmup    = 20;       // accept everything until window full enough
        bool        g1_enabled = true;
        double      g1_margin  = 1.5;
        // Per-anchor inter-anchor distance for G1 (anchor_id -> distance in m)
        std::map<int, double> g1_distance_m;
    };

    explicit AsymmetricDzsFilter(const Config& cfg) : cfg_(cfg) {}

    // Check this blink. tdoa_m[sa_id] = TDoA(sa_id - ref) * c in meters.
    // Returns true if inlier (publish), false if outlier (reject).
    bool check_and_update(const std::map<int, double>& tdoa_m,
                          std::string* reject_reason = nullptr);

    std::map<int, DzsThresholds> thresholds_cm() const;
    const DzsStats& stats() const { return stats_; }

private:
    Config                            cfg_;
    DzsStats                          stats_;
    std::map<int, std::deque<double>> windows_;   // per-anchor sliding window
};

} // namespace ips
