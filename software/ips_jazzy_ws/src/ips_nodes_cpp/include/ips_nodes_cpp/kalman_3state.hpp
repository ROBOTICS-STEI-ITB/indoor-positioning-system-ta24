// =============================================================================
//  kalman_3state.hpp
//  Three-state Kalman filter for clock synchronization (Zhang et al. 2024).
//
//  State X = [ToA, rate (~1.0), accel (~0)]
//  F     = | 1   dt   dt^2/2 |
//          | 0   1    dt     |
//          | 0   0    1      |
//  H     = [1, 0, 0]   (observation = master timestamp Tk at CCP)
//
//  All matrices fixed-size Eigen — zero heap allocation in hot path.
// =============================================================================
#pragma once

#include <Eigen/Dense>

#include "ips_nodes_cpp/dw1000_constants.hpp"

namespace ips {

class KalmanFilter3State {
public:
    KalmanFilter3State() { reset_state(); }

    void initialize(double Tk_s) {
        X_ << Tk_s, 1.0, 0.0;
        last_update_Tk_s_ = Tk_s;
        initialized_      = true;
        update_count_     = 0;
    }

    // Predict + update with new CCP observation.
    //   dt_s   : time since last CCP in master-domain seconds
    //   z_Tk_s : master timestamp Tk at this CCP (observation)
    void predict_and_update(double dt_s, double z_Tk_s) {
        if (!initialized_) return;

        const double dt  = dt_s;
        const double dt2 = dt * dt;

        // ---- Predict X = F * X (analytic, faster than matrix mul) ----
        Eigen::Vector3d X_pred;
        X_pred(0) = X_(0) + X_(1) * dt + 0.5 * X_(2) * dt2;
        X_pred(1) = X_(1) + X_(2) * dt;
        X_pred(2) = X_(2);

        // ---- Predict P = F P F^T + Q ----
        Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
        F(0, 1) = dt;
        F(0, 2) = 0.5 * dt2;
        F(1, 2) = dt;
        const Eigen::Matrix3d P_pred = F * P_ * F.transpose() + Q_;

        // ---- Update with H = [1, 0, 0] (so H*x = x[0], H*P = P.row(0)) ----
        const double y = z_Tk_s - X_pred(0);
        const double S = P_pred(0, 0) + R_;
        if (std::fabs(S) < 1e-50) return;

        // K = P_pred * H^T / S → column 0 of P_pred divided by S
        Eigen::Vector3d K;
        K(0) = P_pred(0, 0) / S;
        K(1) = P_pred(1, 0) / S;
        K(2) = P_pred(2, 0) / S;

        X_ = X_pred + K * y;

        // P = (I - K H) P_pred → P_pred - K * P_pred.row(0)
        Eigen::Matrix3d P_new;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                P_new(i, j) = P_pred(i, j) - K(i) * P_pred(0, j);

        // Force symmetry to suppress accumulated round-off drift
        P_ = 0.5 * (P_new + P_new.transpose());

        last_update_Tk_s_ = z_Tk_s;
        update_count_    += 1;
    }

    // Predict corrected ToA at dt_master_s seconds after last update.
    double predict_toa(double dt_master_s) const {
        const double dt = dt_master_s;
        return X_(0) + X_(1) * dt + 0.5 * X_(2) * dt * dt;
    }

    bool   converged()   const { return initialized_ && update_count_ >= KF_MIN_CONVERGENCE; }
    bool   initialized() const { return initialized_; }
    int    update_count()const { return update_count_; }
    double drift()       const { return X_(1); }
    double drift_rate()  const { return X_(2); }
    double state_toa()   const { return X_(0); }
    const Eigen::Matrix3d& P() const { return P_; }
    const Eigen::Vector3d& X() const { return X_; }

    void reset() {
        reset_state();
        initialized_  = false;
        update_count_ = 0;
    }

private:
    void reset_state() {
        X_ << 0.0, 1.0, 0.0;
        P_.setZero();
        P_(0, 0) = KF_INIT_P_TOA;
        P_(1, 1) = KF_INIT_P_DRIFT;
        P_(2, 2) = KF_INIT_P_FDRIFT;
        Q_.setZero();
        Q_(0, 0) = KF_PROC_NOISE_Q_TOA;
        Q_(1, 1) = KF_PROC_NOISE_Q_DRIFT;
        Q_(2, 2) = KF_PROC_NOISE_Q_FDRIFT;
        R_       = KF_MEAS_NOISE_R_S2;
        last_update_Tk_s_ = 0.0;
    }

    Eigen::Vector3d X_;
    Eigen::Matrix3d P_;
    Eigen::Matrix3d Q_;
    double R_ = KF_MEAS_NOISE_R_S2;

    bool   initialized_      = false;
    int    update_count_     = 0;
    double last_update_Tk_s_ = 0.0;
};

} // namespace ips
