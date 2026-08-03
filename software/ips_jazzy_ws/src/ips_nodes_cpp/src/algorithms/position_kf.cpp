// =============================================================================
//  position_kf.cpp
//  CHANGES:
//    - predict_only()                    (Layer 1)
//    - StudentTFilter innovation gate    (Layer 2)
// =============================================================================
#include "ips_nodes_cpp/position_kf.hpp"

#include <algorithm>
#include <cmath>

namespace ips {

// =============================================================================
// PositionKF (standard Gaussian)
// =============================================================================

PositionKF::PositionKF(const Eigen::Matrix3d& R,
                       const Eigen::Vector3d& x0,
                       double sigma_a)
    : R_(R), sigma_a_(sigma_a) {
    x_.setZero();
    x_.head<3>() = x0;
    P_.setIdentity();
    H_.setZero();
    H_(0, 0) = H_(1, 1) = H_(2, 2) = 1.0;
}

Eigen::Matrix<double, 6, 6> PositionKF::Q_from_dt(double dt) const {
    Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Zero();
    const double s2  = sigma_a_ * sigma_a_;
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    const double dt4 = dt2 * dt2;
    const double q00 = 0.25 * dt4 * s2;
    const double q01 = 0.5  * dt3 * s2;
    const double q11 =        dt2 * s2;
    for (int i = 0; i < 3; ++i) {
        Q(i,     i)     = q00;
        Q(i,     i + 3) = q01;
        Q(i + 3, i)     = q01;
        Q(i + 3, i + 3) = q11;
    }
    return Q;
}

void PositionKF::do_predict(double dt,
                            Eigen::Matrix<double, 6, 1>& xp,
                            Eigen::Matrix<double, 6, 6>& Pp) const {
    xp = x_;
    xp(0) += x_(3) * dt;
    xp(1) += x_(4) * dt;
    xp(2) += x_(5) * dt;

    Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();
    F(0, 3) = F(1, 4) = F(2, 5) = dt;
    Pp = F * P_ * F.transpose() + Q_from_dt(dt);
}

void PositionKF::predict_only(double dt,
                              Eigen::Vector3d& x_pred_out,
                              Eigen::Matrix3d& P_pos_out) const {
    Eigen::Matrix<double, 6, 1> xp;
    Eigen::Matrix<double, 6, 6> Pp;
    do_predict(dt, xp, Pp);
    x_pred_out = xp.head<3>();
    P_pos_out  = Pp.topLeftCorner<3, 3>();
}

Eigen::Vector3d PositionKF::step(const Eigen::Vector3d& z, double dt) {
    Eigen::Matrix<double, 6, 1> xp;
    Eigen::Matrix<double, 6, 6> Pp;
    do_predict(dt, xp, Pp);

    const Eigen::Matrix3d             S  = H_ * Pp * H_.transpose() + R_;
    const Eigen::Matrix3d             Si = S.inverse();
    const Eigen::Matrix<double, 6, 3> K  = Pp * H_.transpose() * Si;

    x_ = xp + K * (z - H_ * xp);

    const Eigen::Matrix<double, 6, 6> I_KH =
        Eigen::Matrix<double, 6, 6>::Identity() - K * H_;
    P_ = I_KH * Pp * I_KH.transpose() + K * R_ * K.transpose();
    P_ = 0.5 * (P_ + P_.transpose());

    return x_.head<3>();
}

// =============================================================================
// StudentTFilter
//   - Innovation gate (Layer 2): if NIS > gate_threshold, skip update.
//   - Else: Student-t α-scaled update.
// =============================================================================

StudentTFilter::StudentTFilter(const Eigen::Matrix3d& R,
                               const Eigen::Vector3d& x0,
                               double sigma_a,
                               double eta0,
                               double alpha_max,
                               double gate_threshold)
    : PositionKF(R, x0, sigma_a),
      eta0_(eta0),
      alpha_max_(alpha_max),
      gate_threshold_(gate_threshold) {
    P_.setIdentity();
    P_ *= 0.1;
}

Eigen::Vector3d StudentTFilter::step(const Eigen::Vector3d& z, double dt) {
    Eigen::Matrix<double, 6, 1> xp;
    Eigen::Matrix<double, 6, 6> Pp;
    do_predict(dt, xp, Pp);

    const Eigen::Vector3d  innov = z - H_ * xp;
    const Eigen::Matrix3d  S     = H_ * Pp * H_.transpose() + R_;
    const Eigen::Matrix3d  Si    = S.inverse();

    // Mahalanobis-squared distance of innovation
    const double nis = innov.transpose() * Si * innov;
    last_nis_ = nis;
    ++total_count_;

    // ===== Layer 2 innovation gate =====
    // If NIS exceeds chi-squared threshold, the measurement is geometrically
    // inconsistent with the predicted trajectory — REJECT, fall back to
    // prediction-only (no measurement update applied).
    if (gate_threshold_ > 0.0 && nis > gate_threshold_) {
        x_ = xp;
        P_ = 0.5 * (Pp + Pp.transpose());
        ++gate_count_;
        return x_.head<3>();
    }

    // ===== Student-t measurement update with α-scaling =====
    const Eigen::Matrix<double, 6, 3>  K = Pp * H_.transpose() * Si;
    x_ = xp + K * innov;

    const Eigen::Matrix<double, 6, 6> I_KH =
        Eigen::Matrix<double, 6, 6>::Identity() - K * H_;
    const Eigen::Matrix<double, 6, 6> Pkf =
        I_KH * Pp * I_KH.transpose() + K * R_ * K.transpose();

    // α-scaling (Roth eq. 45c): degrade trust when NIS large
    double alpha = (eta0_ + nis) / (eta0_ + M_DOF);
    alpha = std::clamp(alpha, 0.5, alpha_max_);

    P_ = alpha * Pkf;
    P_ = 0.5 * (P_ + P_.transpose());

    return x_.head<3>();
}

} // namespace ips
