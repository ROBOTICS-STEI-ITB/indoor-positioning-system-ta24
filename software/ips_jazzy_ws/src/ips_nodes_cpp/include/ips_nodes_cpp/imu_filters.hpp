// =============================================================================
//  imu_filters.hpp
//  Filter IMU reusable (header-only) — port dari read_UDP.py ke C++.
//
//  Tiga filter, semuanya per-sumbu skalar, streaming (online):
//    1. LowPassEma        — low-pass EMA (out = a·baru + (1-a)·lama)
//    2. KalmanFilter1D    — Kalman skalar (q proses, r ukur)
//    3. SavGolDerivative  — turunan Savitzky-Golay causal (fit poly, ambil dy/dt)
//
//  Catatan dt: read_UDP.py mengasumsikan FIXED_DT=0.02 (50Hz). Di sini SG
//  memakai dt AKTUAL yang dilewatkan (IMU ~20Hz, dt~0.05). Koefisien poly-fit
//  dihitung ulang tiap evaluasi memakai dt nyata, jadi benar di rate berapa pun.
// =============================================================================
#pragma once

#include <Eigen/Dense>
#include <array>
#include <cstddef>
#include <deque>

namespace ips {

// -----------------------------------------------------------------------------
// 1. Low-pass EMA (read_UDP.py: low_pass_filter, alpha)
//    out = alpha*new + (1-alpha)*prev ; alpha kecil = smoothing kuat.
// -----------------------------------------------------------------------------
class LowPassEma {
public:
    explicit LowPassEma(double alpha = 1.0) : alpha_(alpha) {}
    void set_alpha(double a) { alpha_ = a; }

    double update(double x) {
        if (!init_) { y_ = x; init_ = true; return y_; }
        y_ = alpha_ * x + (1.0 - alpha_) * y_;
        return y_;
    }
    void reset() { init_ = false; y_ = 0.0; }
    double value() const { return y_; }

private:
    double alpha_;
    double y_    = 0.0;
    bool   init_ = false;
};

// -----------------------------------------------------------------------------
// 2. Kalman 1D skalar (read_UDP.py: KalmanFilter1D, q proses, r ukur)
//    Random-walk model: x_pred=x; P+=q; K=P/(P+r); x+=K(z-x); P=(1-K)P.
// -----------------------------------------------------------------------------
class KalmanFilter1D {
public:
    KalmanFilter1D(double q = 0.0005, double r = 0.05) : q_(q), r_(r) {}
    void set_params(double q, double r) { q_ = q; r_ = r; }

    double update(double z) {
        if (!init_) { x_ = z; init_ = true; return x_; }
        p_ += q_;
        const double k = p_ / (p_ + r_);
        x_ += k * (z - x_);
        p_  = (1.0 - k) * p_;
        return x_;
    }
    void reset() { init_ = false; x_ = 0.0; p_ = 1.0; }
    double value() const { return x_; }

private:
    double q_, r_;
    double x_    = 0.0;
    double p_    = 1.0;
    bool   init_ = false;
};

// -----------------------------------------------------------------------------
// 3. Savitzky-Golay causal derivative (read_UDP.py: savitzky_golay_derivative)
//    Simpan window sampel terakhir, fit polinomial orde-p, ambil turunan di
//    titik sekarang (t=0, newest). Causal: hanya sampel lampau.
//    dt aktual dipakai → benar di rate berapa pun (bukan FIXED_DT 50Hz).
//
//    Implementasi: bangun matriks Vandermonde T (window × (poly+1)) dengan
//    t = (-(N-1)..0)*dt, solve least-squares c = pinv(T)·y, lalu turunan di
//    t=0 = c[poly-1] (koefisien orde-1, karena d/dt(sum c_k t^k)|_{t=0}=c_1).
//    Pakai Eigen untuk least-squares (robust, sama hasil dgn numpy polyfit).
// -----------------------------------------------------------------------------
class SavGolDerivative {
public:
    SavGolDerivative(std::size_t window = 15, int poly = 3,
                     double clip_abs = 0.0)
        : window_(window), poly_(poly), clip_abs_(clip_abs) {}

    // Tambah sampel baru, kembalikan turunan saat ini (0 bila window belum penuh).
    double update(double y, double dt) {
        hist_.push_back(y);
        if (hist_.size() > window_) hist_.pop_front();
        if (hist_.size() < window_ || dt <= 0.0) return 0.0;

        const int N = static_cast<int>(window_);
        const int P = poly_;
        // t = [-(N-1), ..., 0] * dt  (newest at t=0)
        Eigen::MatrixXd T(N, P + 1);
        Eigen::VectorXd Y(N);
        for (int i = 0; i < N; ++i) {
            const double t = static_cast<double>(-(N - 1) + i) * dt;
            double tp = 1.0;
            for (int j = 0; j <= P; ++j) { T(i, j) = tp; tp *= t; }
            Y(i) = hist_[static_cast<std::size_t>(i)];
        }
        // Least-squares solve T·c = Y  → c (koefisien c0 + c1 t + c2 t^2 + ...)
        Eigen::VectorXd c =
            T.colPivHouseholderQr().solve(Y);
        // Turunan di t=0 = c1 (koefisien orde-1)
        double deriv = (P >= 1) ? c(1) : 0.0;
        if (clip_abs_ > 0.0) {
            if (deriv >  clip_abs_) deriv =  clip_abs_;
            if (deriv < -clip_abs_) deriv = -clip_abs_;
        }
        return deriv;
    }

    bool ready() const { return hist_.size() >= window_; }
    void reset() { hist_.clear(); }

private:
    std::size_t        window_;
    int                poly_;
    double             clip_abs_;
    std::deque<double> hist_;
};

}  // namespace ips
