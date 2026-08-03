// =============================================================================
//  position_kf.hpp
//  6-state constant-velocity Kalman filter for position smoothing.
//
//  CHANGES:
//    - predict_only()        (Layer 1 patch)
//    - StudentTFilter gate   (Layer 2 patch) — innovation gate via Mahalanobis
// =============================================================================
#pragma once

#include <Eigen/Dense>

namespace ips {

class PositionKF {
public:
    PositionKF(const Eigen::Matrix3d& R,
               const Eigen::Vector3d& x0,
               double sigma_a);

    // Virtual destructor — needed so dynamic_cast works on subclasses.
    // Also makes the destructor virtual for safe polymorphic delete via base ptr.
    virtual ~PositionKF() = default;

    Eigen::Vector3d step(const Eigen::Vector3d& z, double dt);

    void predict_only(double dt,
                      Eigen::Vector3d& x_pred_out,
                      Eigen::Matrix3d& P_pos_out) const;

    const Eigen::Matrix<double, 6, 1>& state() const { return x_; }
    const Eigen::Matrix<double, 6, 6>& cov()   const { return P_; }
    Eigen::Vector3d position() const             { return x_.head<3>(); }
    Eigen::Vector3d velocity() const             { return x_.tail<3>(); }

    void set_R(const Eigen::Matrix3d& R) { R_ = R; }
    const Eigen::Matrix3d& R() const     { return R_; }

protected:
    Eigen::Matrix<double, 6, 1>  x_;
    Eigen::Matrix<double, 6, 6>  P_;
    Eigen::Matrix3d              R_;
    Eigen::Matrix<double, 3, 6>  H_;
    double                       sigma_a_;

    Eigen::Matrix<double, 6, 6> Q_from_dt(double dt) const;
    void                        do_predict(double dt,
                                           Eigen::Matrix<double, 6, 1>& xp,
                                           Eigen::Matrix<double, 6, 6>& Pp) const;
};

class StudentTFilter : public PositionKF {
public:
    // gate_threshold: chi-squared(dim=3) inverse-CDF for desired gate probability.
    //   gate_prob=0.95 → 7.815
    //   gate_prob=0.99 → 11.345    (default)
    //   gate_prob=0.999→ 16.266
    //   Pass <= 0 to disable gate (pure Student-t, no Layer 2).
    StudentTFilter(const Eigen::Matrix3d& R,
                   const Eigen::Vector3d& x0,
                   double sigma_a,
                   double eta0,
                   double alpha_max,
                   double gate_threshold = 11.345);

    Eigen::Vector3d step(const Eigen::Vector3d& z, double dt);

    // Layer 2 stats
    long n_gated()    const { return gate_count_; }
    long n_accepted() const { return total_count_ - gate_count_; }
    long n_total()    const { return total_count_; }
    double last_nis() const { return last_nis_; }
    double gate_threshold() const { return gate_threshold_; }

private:
    double                  eta0_;
    double                  alpha_max_;
    double                  gate_threshold_;
    long                    gate_count_  = 0;
    long                    total_count_ = 0;
    double                  last_nis_    = 0.0;
    static constexpr double M_DOF = 3.0;
};

} // namespace ips
