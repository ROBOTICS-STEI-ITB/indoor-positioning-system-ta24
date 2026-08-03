// =============================================================================
//  dw1000_constants.hpp
//  DW1000 hardware constants + KF tuning parameters.
//  These are *algorithm parameters* — keep them numerically identical to the
//  Python/offline-C++ reference so behaviour stays comparable.
// =============================================================================
#pragma once

#include <cmath>
#include <cstdint>
#include <string>

namespace ips {

// --- DW1000 clock --------------------------------------------------------
constexpr double  DW1000_CLOCK_HZ = 499.2e6 * 128.0;        // 63,897,600,000 Hz
constexpr double  DTU_SECONDS     = 1.0 / DW1000_CLOCK_HZ;  // ~15.65 ps
constexpr int64_t WRAP_40         = static_cast<int64_t>(1) << 40;
constexpr int64_t HALF_WRAP_40    = static_cast<int64_t>(1) << 39;

// --- 3-state Kalman filter (Zhang et al., 2024) --------------------------
constexpr double KF_MEAS_NOISE_R_S2     = 1.5e-20;
constexpr double KF_PROC_NOISE_Q_TOA    = 2.0e-21;
constexpr double KF_PROC_NOISE_Q_DRIFT  = 4.0e-26;
constexpr double KF_PROC_NOISE_Q_FDRIFT = 4.0e-30;
constexpr double KF_INIT_P_TOA          = 1.0e-18;
constexpr double KF_INIT_P_DRIFT        = 1.0e-20;
constexpr double KF_INIT_P_FDRIFT       = 1.0e-24;
constexpr int    KF_MIN_CONVERGENCE     = 5;

// --- Engine policy --------------------------------------------------------
constexpr int    SEQ_RESET_THRESHOLD    = 100;

// Glitch protection: reject CCP frames whose computed delta_k deviates from
// 1.0 by more than this (1e-3 = 1000 ppm, far above crystal drift of ±50 ppm).
constexpr double DELTA_K_MAX_DEV        = 1e-3;

// --- Conversion helpers ---------------------------------------------------
inline int64_t hex40_to_int(const std::string& hex) {
    return static_cast<int64_t>(std::stoull(hex, nullptr, 16));
}

inline double dtu_to_seconds(int64_t x)      { return static_cast<double>(x) * DTU_SECONDS; }
inline double dtu_to_seconds(double  x)      { return x * DTU_SECONDS; }
inline double dtu_to_microseconds(int64_t x) { return dtu_to_seconds(x) * 1e6; }
inline double dtu_to_microseconds(double  x) { return dtu_to_seconds(x) * 1e6; }
inline double dtu_to_nanoseconds(int64_t x)  { return dtu_to_seconds(x) * 1e9; }
inline double dtu_to_nanoseconds(double  x)  { return dtu_to_seconds(x) * 1e9; }

inline int64_t seconds_to_dtu(double s) {
    return static_cast<int64_t>(std::llround(s / DTU_SECONDS));
}
inline int64_t microseconds_to_dtu(double us) {
    return static_cast<int64_t>(std::llround((us * 1e-6) / DTU_SECONDS));
}

} // namespace ips
