"""
Position estimation algorithms for UWB TDoA — Chan closed-form + KF.
v7: Added StudentTFilter (robust KF for dynamic tag tracking).
"""

from __future__ import annotations
import math
from typing import Sequence
import numpy as np

C_MS = 299_792_458.0
EPSILON = 1e-8

# =============================================================================
# GEOMETRY HELPERS
# =============================================================================

def prepare_geometry(anchor_positions_ordered: Sequence[Sequence[float]]):
    A = np.asarray(anchor_positions_ordered, dtype=float)
    if A.shape != (4, 3):
        raise ValueError(f"need 4 anchors of shape (4,3), got {A.shape}")
    K = np.array([float(np.dot(A[i], A[i])) for i in range(4)])
    G = np.array([
        [A[1,0]-A[0,0], A[1,1]-A[0,1], A[1,2]-A[0,2]],
        [A[2,0]-A[0,0], A[2,1]-A[0,1], A[2,2]-A[0,2]],
        [A[3,0]-A[0,0], A[3,1]-A[0,1], A[3,2]-A[0,2]],
    ])
    return A, K, G


def compute_R_kf(A, pos_nominal, sigma_t_tdoa):
    r_lin = np.array([np.linalg.norm(pos_nominal - A[i]) for i in range(4)])
    J = np.zeros((3, 3))
    for i in range(1, 4):
        J[i-1, :] = (pos_nominal - A[i]) / r_lin[i] - (pos_nominal - A[0]) / r_lin[0]
    sigma_r = sigma_t_tdoa * C_MS
    C_TDoA = sigma_r**2 * np.array([[2,1,1],[1,2,1],[1,1,2]])
    return np.linalg.inv(J.T @ np.linalg.inv(C_TDoA) @ J)


def compute_R_from_data(errors, fallback_R, min_samples=100):
    """Compute R from actual Chan errors vs ground truth (full 3x3 covariance)."""
    if len(errors) < min_samples:
        return fallback_R
    E = np.asarray(errors, dtype=float)
    return np.cov(E.T)


def compute_Q_kf(dt, sigma_a):
    """Process noise Q for 6-state constant-velocity model."""
    G = np.array([[0.5*dt**2], [dt]])
    Qb = sigma_a**2 * (G @ G.T)
    Q = np.zeros((6, 6))
    for i in range(3):
        Q[i, i] = Qb[0, 0]
        Q[i, i+3] = Qb[0, 1]
        Q[i+3, i] = Qb[1, 0]
        Q[i+3, i+3] = Qb[1, 1]
    return Q

# =============================================================================
# CHAN CLOSED-FORM TDoA
# =============================================================================

def chan_estimate(toa_ordered, A, K, G, room_dim, prev_pos):
    t = np.asarray(toa_ordered, dtype=float)
    if t.shape != (4,):
        return prev_pos, False
    d21 = C_MS * (t[1] - t[0])
    d31 = C_MS * (t[2] - t[0])
    d41 = C_MS * (t[3] - t[0])
    G_del = np.array([d21, d31, d41])
    h_chan = 0.5 * np.array([
        d21**2 - K[1] + K[0],
        d31**2 - K[2] + K[0],
        d41**2 - K[3] + K[0],
    ])
    if 1.0 / np.linalg.cond(G) < 1e-10:
        return prev_pos, False
    inv_G = np.linalg.inv(G)
    A_mat = -inv_G @ G_del
    b_mat = -inv_G @ h_chan
    beta = A[0] - b_mat
    a_k = float(A_mat @ A_mat) - 1.0
    b_k = -2.0 * float(beta @ A_mat)
    c_k = float(beta @ beta)
    D = b_k**2 - 4.0 * a_k * c_k
    if D < 0 or abs(a_k) < EPSILON:
        return prev_pos, False
    sd = math.sqrt(D)
    sols = [(-b_k + sd) / (2*a_k), (-b_k - sd) / (2*a_k)]
    bb_lo = -0.5
    bb_hi = np.asarray(room_dim) + 0.5
    pos_chosen = prev_pos
    res_min = float("inf")
    valid = False
    for d1 in sols:
        if d1 <= 0:
            continue
        pos = A_mat * d1 + b_mat
        if not (np.all(pos >= bb_lo) and np.all(pos <= bb_hi)):
            continue
        r = np.linalg.norm(A - pos, axis=1)
        residu = float(np.sum(np.abs(r[1:] - (r[0] + G_del))))
        if residu < res_min:
            res_min, pos_chosen, valid = residu, pos, True
    return pos_chosen, valid

# =============================================================================
# 6-STATE POSITION KALMAN FILTER (standard)
# =============================================================================

class PositionKalmanFilter:
    """6-state KF: [x, y, z, vx, vy, vz]. Standard Gaussian assumption."""

    def __init__(self, R_kf, x0, sigma_a):
        self.x = np.concatenate([np.asarray(x0, dtype=float), np.zeros(3)])
        self.P = np.eye(6)
        self.R = R_kf.copy()
        self.sigma_a = sigma_a
        self.H = np.zeros((3, 6))
        self.H[0,0] = self.H[1,1] = self.H[2,2] = 1.0

    def step(self, z, dt):
        F = np.eye(6)
        F[0,3] = F[1,4] = F[2,5] = dt
        Q = compute_Q_kf(dt, self.sigma_a)
        xp = F @ self.x
        Pp = F @ self.P @ F.T + Q
        S = self.H @ Pp @ self.H.T + self.R
        K = Pp @ self.H.T @ np.linalg.inv(S)
        self.x = xp + K @ (np.asarray(z) - self.H @ xp)
        self.P = (np.eye(6) - K @ self.H) @ Pp
        return self.x[:3].copy()


# =============================================================================
# STUDENT-T FILTER (robust KF for heavy-tailed / dynamic tracking)
# =============================================================================

# Default STF parameters
STF_ETA0_DEFAULT      = 5      # degrees of freedom (lower = heavier tails = more robust)
STF_ALPHA_MAX_DEFAULT = 3.0    # max P inflation factor


class StudentTFilter:
    """
    Student's t Filter — robust against heavy-tailed measurement noise.

    Key difference vs standard KF: after each update, the posterior
    covariance P is scaled by alpha = (eta + NIS) / (eta + m), where
    NIS = normalized innovation squared. When an outlier arrives (large
    NIS), alpha > 1 inflates P, reducing the outlier's influence on the
    state. Normal measurements produce alpha ~ 1 (standard KF behavior).

    This makes the filter naturally robust to:
    - Multipath-induced position jumps
    - DZS warm-up misses
    - Dynamic motion that DZS would incorrectly reject

    Ref: Roth, Ardeshiri, Ozkan, Gustafsson (2017), arXiv:1703.02428

    Interface matches PositionKalmanFilter: same __init__ signature + step().
    """

    def __init__(self, R_kf, x0, sigma_a,
                 eta0=STF_ETA0_DEFAULT, alpha_max=STF_ALPHA_MAX_DEFAULT):
        self.x = np.concatenate([np.asarray(x0, dtype=float), np.zeros(3)])
        self.P = np.eye(6) * 0.1
        self.R = R_kf.copy()
        self.sigma_a = sigma_a
        self.eta0 = float(eta0)
        self.alpha_max = float(alpha_max)
        self.m = 3  # measurement dimension
        self.H = np.zeros((3, 6))
        self.H[0,0] = self.H[1,1] = self.H[2,2] = 1.0
        # Diagnostic counters
        self._n_updates = 0
        self._last_nis = 0.0
        self._last_alpha = 1.0

    def step(self, z, dt):
        """One predict-update cycle. Returns 3-D position estimate."""
        F = np.eye(6)
        F[0,3] = F[1,4] = F[2,5] = dt
        Q = compute_Q_kf(dt, self.sigma_a)

        # Predict
        xp = F @ self.x
        Pp = F @ self.P @ F.T + Q

        # Innovation
        innov = np.asarray(z) - self.H @ xp
        S = self.H @ Pp @ self.H.T + self.R
        Si = np.linalg.inv(S)
        K = Pp @ self.H.T @ Si

        # Update state
        self.x = xp + K @ innov

        # Standard posterior covariance
        Pkf = Pp - K @ S @ K.T

        # Student-t scaling: inflate P when innovation is large (outlier)
        nis = float(innov.T @ Si @ innov)
        alpha = max(0.5, min(
            (self.eta0 + nis) / (self.eta0 + self.m),
            self.alpha_max
        ))
        self.P = alpha * Pkf
        self.P = 0.5 * (self.P + self.P.T)  # enforce symmetry

        # Diagnostics
        self._n_updates += 1
        self._last_nis = nis
        self._last_alpha = alpha

        return self.x[:3].copy()

    @property
    def diagnostics(self):
        return {
            'n_updates': self._n_updates,
            'last_nis': self._last_nis,
            'last_alpha': self._last_alpha,
        }


# =============================================================================
# ASYMMETRIC DZS OUTLIER FILTER + G=1 PRE-FILTER
# =============================================================================

DZS_WINDOW_DEFAULT    = 100
DZS_K_POS_DEFAULT     = 2.5
DZS_K_NEG_DEFAULT     = 3.5
DZS_MAD_MIN_M_DEFAULT = 0.03
DZS_WARMUP_DEFAULT    = 20
G1_MARGIN_DEFAULT     = 1.5


class AsymmetricDZSFilter:
    """DZS with asymmetric thresholds + G=1 triangular pre-filter."""

    def __init__(self, anchor_ids, reference_id, anchor_positions=None,
                 window=DZS_WINDOW_DEFAULT, k_pos=DZS_K_POS_DEFAULT,
                 k_neg=DZS_K_NEG_DEFAULT, mad_min=DZS_MAD_MIN_M_DEFAULT,
                 warmup=DZS_WARMUP_DEFAULT, g1_margin=G1_MARGIN_DEFAULT,
                 g1_enabled=True):
        self.reference_id = int(reference_id)
        self.sids = [s for s in anchor_ids if s != self.reference_id]
        self.window = int(window)
        self.k_pos = float(k_pos)
        self.k_neg = float(k_neg)
        self.mad_min = float(mad_min)
        self.warmup = int(warmup)
        self.g1_enabled = bool(g1_enabled) and anchor_positions is not None
        self.g1_margin = float(g1_margin)
        self._d_anchor = {}
        if self.g1_enabled and anchor_positions is not None:
            ref_pos = anchor_positions.get(self.reference_id)
            if ref_pos is not None:
                for sid in self.sids:
                    pos = anchor_positions.get(sid)
                    if pos is not None:
                        self._d_anchor[sid] = float(
                            np.linalg.norm(np.asarray(pos) - np.asarray(ref_pos)))
        self._buf = {sid: [] for sid in self.sids}
        self._n = 0
        self._n_inliers = 0
        self._n_outliers = 0
        self._n_g1_reject = 0
        self._med = {sid: None for sid in self.sids}
        self._mad = {sid: None for sid in self.sids}

    def _update_stats(self, sid):
        buf = self._buf[sid]
        if not buf:
            return
        arr = np.array(buf, dtype=np.float64)
        self._med[sid] = float(np.median(arr))
        self._mad[sid] = max(float(np.median(np.abs(arr - self._med[sid]))),
                             self.mad_min)

    def check_and_update(self, tdoa_dict):
        """Returns (is_inlier, z_scores, reject_reason)."""
        self._n += 1
        if self.g1_enabled:
            for sid in self.sids:
                if sid not in tdoa_dict or sid not in self._d_anchor:
                    continue
                if abs(tdoa_dict[sid]) > self.g1_margin * self._d_anchor[sid]:
                    self._n_g1_reject += 1
                    self._n_outliers += 1
                    return False, {sid: float('inf')}, 'g1'
        if self._n <= self.warmup:
            for sid in self.sids:
                if sid not in tdoa_dict:
                    continue
                self._buf[sid].append(tdoa_dict[sid])
                if len(self._buf[sid]) > self.window:
                    self._buf[sid].pop(0)
                self._update_stats(sid)
            self._n_inliers += 1
            return True, {sid: 0.0 for sid in self.sids}, 'warmup'
        z_scores = {}
        is_outlier = False
        for sid in self.sids:
            if sid not in tdoa_dict:
                continue
            med = self._med[sid]
            mad = self._mad[sid]
            if med is None or mad is None:
                z_scores[sid] = 0.0
                continue
            deviation = tdoa_dict[sid] - med
            mz = 0.6745 * abs(deviation) / mad
            z_scores[sid] = mz
            k = self.k_pos if deviation > 0 else self.k_neg
            if mz > k:
                is_outlier = True
        if not is_outlier:
            for sid in self.sids:
                if sid not in tdoa_dict:
                    continue
                self._buf[sid].append(tdoa_dict[sid])
                if len(self._buf[sid]) > self.window:
                    self._buf[sid].pop(0)
                self._update_stats(sid)
            self._n_inliers += 1
        else:
            self._n_outliers += 1
        return (not is_outlier), z_scores, 'dzs'

    def stats(self):
        return {
            'n_total': self._n, 'n_inliers': self._n_inliers,
            'n_outliers': self._n_outliers, 'n_g1_reject': self._n_g1_reject,
            'reject_rate': (self._n_outliers / self._n) if self._n > 0 else 0.0,
            'window_fill': {sid: len(self._buf[sid]) for sid in self.sids},
            'median_m': {sid: self._med[sid] for sid in self.sids},
            'mad_m': {sid: self._mad[sid] for sid in self.sids},
        }

    def threshold_cm(self):
        out = {}
        for sid in self.sids:
            mad = self._mad[sid]
            if mad is None:
                out[sid] = None
            else:
                out[sid] = {
                    'pos': (self.k_pos / 0.6745) * mad * 100.0,
                    'neg': (self.k_neg / 0.6745) * mad * 100.0,
                }
        return out


# =============================================================================
# REALTIME HYBRID CALIBRATOR
# =============================================================================

class RealtimeHybridCalibrator:
    """CCP-based antenna bias calibration (realtime, no tag GT needed)."""

    def __init__(self, mc_pos, anchor_positions, reference_id,
                 warmup_skip=100, outlier_ns=5.0):
        self.mc_pos = np.asarray(mc_pos, dtype=float)
        self.anchor_positions = {int(k): np.asarray(v, dtype=float)
                                 for k, v in anchor_positions.items()}
        self.ref_id = int(reference_id)
        self.warmup_skip = int(warmup_skip)
        self.outlier_ns = float(outlier_ns)
        self.tau_geom = {
            sid: np.linalg.norm(self.mc_pos - pos) / C_MS
            for sid, pos in self.anchor_positions.items()
        }
        self._ma_tx = {}
        self._sa_rx = {sid: {} for sid in self.anchor_positions}
        self._ma_unwrap_offset = 0
        self._ma_last_raw = None
        self._sa_unwrap_offset = {sid: 0 for sid in self.anchor_positions}
        self._sa_last_raw = {sid: None for sid in self.anchor_positions}
        self.MAX_40 = 1 << 40

    def push_master_tx(self, seq, raw_dtu):
        if self._ma_last_raw is not None and raw_dtu < self._ma_last_raw - self.MAX_40 // 2:
            self._ma_unwrap_offset += self.MAX_40
        self._ma_last_raw = raw_dtu
        self._ma_tx[seq] = raw_dtu + self._ma_unwrap_offset

    def push_slave_rx(self, sid, seq, raw_dtu):
        sid = int(sid)
        if sid not in self._sa_rx:
            return
        last = self._sa_last_raw[sid]
        if last is not None and raw_dtu < last - self.MAX_40 // 2:
            self._sa_unwrap_offset[sid] += self.MAX_40
        self._sa_last_raw[sid] = raw_dtu
        self._sa_rx[sid][seq] = raw_dtu + self._sa_unwrap_offset[sid]

    def compute_bias(self):
        DTU_S = 1.0 / (499.2e6 * 128.0)
        bias_s = {self.ref_id: 0.0}
        for sid in self.anchor_positions:
            if sid == self.ref_id:
                continue
            common = sorted(
                set(self._ma_tx) &
                set(self._sa_rx.get(sid, {})) &
                set(self._sa_rx.get(self.ref_id, {}))
            )[self.warmup_skip:]
            if len(common) < 100:
                continue
            biases_ns = []
            for i in range(2, len(common)):
                s0, s1, sc = common[i-2], common[i-1], common[i]
                Tk0, Tk1 = self._ma_tx[s0], self._ma_tx[s1]
                Rs0, Rs1, Rsc = self._sa_rx[sid][s0], self._sa_rx[sid][s1], self._sa_rx[sid][sc]
                Rr0, Rr1, Rrc = self._sa_rx[self.ref_id][s0], self._sa_rx[self.ref_id][s1], self._sa_rx[self.ref_id][sc]
                if Rs1 - Rs0 == 0 or Rr1 - Rr0 == 0:
                    continue
                dk_sa = (Tk1 - Tk0) / (Rs1 - Rs0)
                dk_ref = (Tk1 - Tk0) / (Rr1 - Rr0)
                if abs(dk_sa - 1.0) > 0.001 or abs(dk_ref - 1.0) > 0.001:
                    continue
                sa_ma = Tk1 + (Rsc - Rs1) * dk_sa
                ref_ma = Tk1 + (Rrc - Rr1) * dk_ref
                tdoa_ccp_s = (sa_ma - ref_ma) * DTU_S
                biases_ns.append((tdoa_ccp_s - (self.tau_geom[sid]-self.tau_geom[self.ref_id])) * 1e9)

            if biases_ns:
                arr = np.array(biases_ns)
                med = np.median(arr)
                inliers = arr[np.abs(arr - med) < 5.0]
                if len(inliers) > 0:
                    bias_s[sid] = float(np.median(inliers)) * 1e-9
        return bias_s


def compute_hybrid_residual(blink_buffer, anchor_positions, reference_id, gt_pos):
    d_id = {sid: np.linalg.norm(gt_pos - anchor_positions[sid]) for sid in anchor_positions}
    tdoa_id = {sid: d_id[sid] - d_id[reference_id] for sid in anchor_positions if sid != reference_id}
    res_s = {reference_id: 0.0}
    for sid in anchor_positions:
        if sid == reference_id:
            continue
        tdoa_m = [C_MS*(b[sid]-b[reference_id]) for b in blink_buffer if sid in b and reference_id in b]
        res_s[sid] = (np.median(tdoa_m) - tdoa_id[sid])/C_MS if tdoa_m else 0.0
    return res_s
