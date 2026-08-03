// =============================================================================
//  chan_solver.cpp
// =============================================================================
#include "ips_nodes_cpp/chan_solver.hpp"

#include <cmath>
#include <limits>

namespace ips {

ChanGeometry build_chan_geometry(const std::array<Eigen::Vector3d, 4>& anchors) {
    ChanGeometry geom;
    for (int i = 0; i < 4; ++i) {
        geom.A.row(i) = anchors[i].transpose();
        geom.K(i)     = anchors[i].dot(anchors[i]);
    }
    for (int j = 0; j < 3; ++j)
        for (int k = 0; k < 3; ++k)
            geom.G(j, k) = geom.A(j + 1, k) - geom.A(0, k);

    Eigen::FullPivLU<Eigen::Matrix3d> lu(geom.G);
    if (lu.isInvertible()) {
        geom.iG    = lu.inverse();
        geom.valid = true;
    }
    return geom;
}

std::pair<Eigen::Vector3d, bool>
chan_solve(const std::array<double, 4>& toa_ordered,
           const ChanGeometry&          geom,
           const Eigen::Vector3d&       lower_b,
           const Eigen::Vector3d&       upper_b,
           const Eigen::Vector3d&       prev_pos) {
    if (!geom.valid) return {prev_pos, false};

    // d[i] = c * (t[i+1] - t[0])
    Eigen::Vector3d d;
    for (int i = 0; i < 3; ++i)
        d(i) = C_MS_S * (toa_ordered[i + 1] - toa_ordered[0]);

    // h[i] = 0.5 * (d[i]^2 - K[i+1] + K[0])
    Eigen::Vector3d h;
    for (int i = 0; i < 3; ++i)
        h(i) = 0.5 * (d(i) * d(i) - geom.K(i + 1) + geom.K(0));

    // Am = -iG @ d ; bm = -iG @ h
    const Eigen::Vector3d Am = -geom.iG * d;
    const Eigen::Vector3d bm = -geom.iG * h;
    const Eigen::Vector3d bt = geom.A.row(0).transpose() - bm;

    const double ak = Am.dot(Am) - 1.0;
    const double bk = -2.0 * bt.dot(Am);
    const double ck = bt.dot(bt);
    const double D  = bk * bk - 4.0 * ak * ck;

    if (D < 0.0 || std::fabs(ak) < EPSILON_NUM) {
        return {prev_pos, false};
    }
    const double sd = std::sqrt(D);

    Eigen::Vector3d best     = prev_pos;
    double          best_res = std::numeric_limits<double>::infinity();
    bool            any_ok   = false;

    const double d1_arr[2] = { (-bk + sd) / (2.0 * ak),
                                (-bk - sd) / (2.0 * ak) };
    for (int s = 0; s < 2; ++s) {
        const double d1 = d1_arr[s];
        if (d1 <= 0.0) continue;
        Eigen::Vector3d p = Am * d1 + bm;
        bool in_room = true;
        for (int k = 0; k < 3; ++k) {
            if (p(k) < lower_b(k) || p(k) > upper_b(k)) { in_room = false; break; }
        }
        if (!in_room) continue;

        // residual = sum |r[i+1] - (r[0] + d[i])|
        Eigen::Vector4d r;
        for (int i = 0; i < 4; ++i)
            r(i) = (geom.A.row(i).transpose() - p).norm();
        double res = 0.0;
        for (int i = 0; i < 3; ++i)
            res += std::fabs(r(i + 1) - (r(0) + d(i)));

        if (res < best_res) {
            best_res = res;
            best     = p;
            any_ok   = true;
        }
    }
    return {best, any_ok};
}

} // namespace ips
