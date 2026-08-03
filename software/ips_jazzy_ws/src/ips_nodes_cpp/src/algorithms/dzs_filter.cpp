// =============================================================================
//  dzs_filter.cpp
// =============================================================================
#include "ips_nodes_cpp/dzs_filter.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace ips {

namespace {

// Median of a deque (copies into vector for nth_element; deque size is ≤ window
// which is small, so this is cheap).
double dq_median(const std::deque<double>& dq) {
    if (dq.empty()) return 0.0;
    std::vector<double> v(dq.begin(), dq.end());
    const std::size_t mid = v.size() / 2;
    std::nth_element(v.begin(), v.begin() + mid, v.end());
    return v[mid];
}

// MAD = median of |x_i - median(x)|
double dq_mad(const std::deque<double>& dq, double med) {
    if (dq.empty()) return 0.0;
    std::vector<double> dev;
    dev.reserve(dq.size());
    for (double v : dq) dev.push_back(std::fabs(v - med));
    const std::size_t mid = dev.size() / 2;
    std::nth_element(dev.begin(), dev.begin() + mid, dev.end());
    return dev[mid];
}

} // namespace

bool AsymmetricDzsFilter::check_and_update(
        const std::map<int, double>& tdoa_m,
        std::string* reject_reason) {
    stats_.n_total += 1;

    // ---- G1 pre-filter: triangular inequality ---------------------------
    if (cfg_.g1_enabled) {
        for (const auto& [sid, td] : tdoa_m) {
            auto dit = cfg_.g1_distance_m.find(sid);
            if (dit == cfg_.g1_distance_m.end()) continue;
            const double limit = cfg_.g1_margin * dit->second;
            if (std::fabs(td) > limit) {
                stats_.n_g1_reject += 1;
                if (reject_reason) {
                    std::ostringstream oss;
                    oss << "G1: SA" << sid << " |td|=" << std::fabs(td)
                        << "m > " << limit << "m";
                    *reject_reason = oss.str();
                }
                return false;
            }
        }
    }

    // ---- DZS check ------------------------------------------------------
    bool inlier = true;
    for (const auto& [sid, td] : tdoa_m) {
        auto& win = windows_[sid];

        // Warm-up: just accept and learn
        if (win.size() < cfg_.warmup) {
            continue;
        }
        const double med = dq_median(win);
        const double mad = std::max(dq_mad(win, med), cfg_.mad_min_m / 1.4826);
        const double sigma = 1.4826 * mad;
        const double upper = med + cfg_.k_pos * sigma;
        const double lower = med - cfg_.k_neg * sigma;
        if (td > upper || td < lower) {
            inlier = false;
            if (reject_reason) {
                std::ostringstream oss;
                oss << "DZS: SA" << sid << " td=" << td
                    << "m outside [" << lower << ", " << upper << "]";
                *reject_reason = oss.str();
            }
            break;
        }
    }

    // ---- Update windows: only if inlier (don't poison the model) -------
    if (inlier) {
        stats_.n_inliers += 1;
        for (const auto& [sid, td] : tdoa_m) {
            auto& win = windows_[sid];
            win.push_back(td);
            while (win.size() > cfg_.window) win.pop_front();
        }
    } else {
        stats_.n_outliers += 1;
    }
    return inlier;
}

std::map<int, DzsThresholds> AsymmetricDzsFilter::thresholds_cm() const {
    std::map<int, DzsThresholds> out;
    for (const auto& [sid, win] : windows_) {
        DzsThresholds t;
        if (win.size() >= cfg_.warmup) {
            const double med   = dq_median(win);
            const double mad   = std::max(dq_mad(win, med),
                                          cfg_.mad_min_m / 1.4826);
            const double sigma = 1.4826 * mad;
            t.pos_cm = cfg_.k_pos * sigma * 100.0;
            t.neg_cm = cfg_.k_neg * sigma * 100.0;
            t.ready  = true;
            // suppress unused warning
            (void)med;
        }
        out[sid] = t;
    }
    return out;
}

} // namespace ips
