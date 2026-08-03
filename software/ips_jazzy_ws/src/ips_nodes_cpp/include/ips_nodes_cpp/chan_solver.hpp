// =============================================================================
//  chan_solver.hpp
//  Chan & Ho (1994) closed-form 3D TDoA solver — 4 anchors, fixed-size Eigen.
// =============================================================================
#pragma once

#include <Eigen/Dense>
#include <array>
#include <utility>

namespace ips {

constexpr double C_MS_S      = 299792458.0;
constexpr double EPSILON_NUM = 1e-8;

// Pre-computed geometry from anchor positions.
//   A: 4x3 with anchor i in row i (index 0 = reference)
//   K: 4-vector with K(i) = A.row(i).dot(A.row(i))
//   G: 3x3 with G(j,k) = A(j+1,k) - A(0,k)
//   iG: inverse of G (precomputed once, reused)
struct ChanGeometry {
    Eigen::Matrix<double, 4, 3> A;
    Eigen::Vector4d             K;
    Eigen::Matrix3d             G;
    Eigen::Matrix3d             iG;
    bool                        valid = false;
};

// Build geometry from 4 anchor positions in Chan order (index 0 = reference).
ChanGeometry build_chan_geometry(const std::array<Eigen::Vector3d, 4>& anchors);

// Solve Chan TDoA. `toa_ordered[i]` is the ToA at anchor i (in Chan order,
// in seconds). Returns (position, valid). On failure returns prev_pos.
//
// `lower_b` and `upper_b` are the in-room bounds for plausibility check.
std::pair<Eigen::Vector3d, bool>
chan_solve(const std::array<double, 4>& toa_ordered,
           const ChanGeometry&          geom,
           const Eigen::Vector3d&       lower_b,
           const Eigen::Vector3d&       upper_b,
           const Eigen::Vector3d&       prev_pos);

} // namespace ips
