"""latency_monitor_node — pengukur latensi pipeline IPS.

Mengukur latensi pemrosesan dari saat paket blink tag TIBA di komputer
(ingest udp_gateway) sampai posisi terkalibrasi TERSEDIA di hilir
(/state/position_compensated, yaitu data yang dibaca state_aggregator).

Cara kerja (zero-touch — tidak mengubah node pipeline mana pun):
  1. /uwb/anchor_reports (SLAVE_TAG)  : header.stamp diisi udp_gateway saat
     ingest. Untuk tiap blink seq, simpan stamp TERAWAL (laporan anchor
     pertama yang tiba) → t_ingest[seq].
  2. /uwb/corrected_toa               : membawa tag_seq; header.stamp diisi
     clock_sync saat publish. Simpan stamp_ns → tag_seq. (wolf_position
     menyalin stamp corrected_toa yang melengkapi blink ke pesan posisi,
     dan bias_compensator meneruskan header — stamp ini menjadi kunci
     korelasi di hilir.)
  3. /state/position_compensated      : saat tiba, t_out = now().
     stamp pesan → seq → t_ingest → latensi.

Komponen yang dilaporkan per sampel:
  total_ms  = t_out − t_ingest          (ingest → output tersedia)
  hulu_ms   = stamp_corrected − t_ingest (ingest → keluar clock_sync)
  hilir_ms  = t_out − stamp_corrected    (clock_sync → output tersedia)

KETERBATASAN (jujur):
  * t0 adalah saat paket TIBA di laptop, BUKAN saat tag memancar blink.
    Komponen radio UWB + WiFi tag→laptop tidak terukur dari sini karena
    jam tag tidak tersinkron dengan jam laptop. (Orde ~beberapa ms WiFi.)
  * state_aggregator menerbitkan snapshot @rate tetap (mis. 10 Hz); jeda
    snapshot menambah rata-rata 1/(2·rate) — mis. ~50 ms @10 Hz — di atas
    angka yang diukur di sini. Itu artefak desain snapshot, dilaporkan
    analitik di log, bukan diukur per-sampel.
  * Sampel yang stamp-nya tidak ter-korelasi (blink hilang / buffer penuh)
    dilewati, dihitung sebagai unmatched — fail-safe, tidak salah hitung.

Output:
  * Log ringkasan berkala (default tiap 5 s): n, match-rate,
    mean/median/p95/max untuk total + breakdown hulu/hilir.
  * /diag/pipeline_latency_ms (Vector3Stamped): x=total, y=hulu, z=hilir
    (milidetik), stamp = stamp pesan posisi. Mudah direkam/diplot.
  * CSV opsional (param csv_path): per-sampel untuk analisis offline.
"""

import csv
import statistics
from collections import OrderedDict, deque
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Vector3Stamped

from ips_msgs.msg import UwbAnchorReport, CorrectedToA
from ips_nodes.common import (
    QOS_SENSOR_BEST_EFFORT_DEEP,
    QOS_STATE_RELIABLE,
    REPORT_TYPE_SLAVE_TAG,
)


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class LatencyMonitorNode(Node):
    """Mengukur latensi ingest→output pipeline posisi."""

    MAX_INGEST_ENTRIES = 2000   # ~80 s blink @25 Hz
    MAX_STAMP_ENTRIES = 8000    # 4 corrected_toa per blink
    STATS_WINDOW = 1000         # sampel untuk statistik berjalan

    def __init__(self) -> None:
        super().__init__('latency_monitor')

        self.declare_parameter('log_every_s', 5.0)
        self.declare_parameter('csv_path', '')
        self.declare_parameter('aggregator_rate_hz', 10.0)

        self._log_every = float(self.get_parameter('log_every_s').value)
        self._csv_path = str(self.get_parameter('csv_path').value)
        self._agg_rate = float(self.get_parameter('aggregator_rate_hz').value)

        # seq -> stamp_ns ingest TERAWAL (laporan anchor pertama utk blink itu)
        self._ingest: 'OrderedDict[int, int]' = OrderedDict()
        # stamp_ns corrected_toa -> seq
        self._stamp2seq: 'OrderedDict[int, int]' = OrderedDict()

        self._total = deque(maxlen=self.STATS_WINDOW)
        self._hulu = deque(maxlen=self.STATS_WINDOW)
        self._hilir = deque(maxlen=self.STATS_WINDOW)
        self._n_matched = 0
        self._n_unmatched = 0

        self._csv_file = None
        self._csv = None
        if self._csv_path:
            Path(self._csv_path).parent.mkdir(parents=True, exist_ok=True)
            self._csv_file = open(self._csv_path, 'w', newline='')
            self._csv = csv.writer(self._csv_file)
            self._csv.writerow(
                ['t_out_s', 'seq', 'total_ms', 'hulu_ms', 'hilir_ms'])
            self.get_logger().info(f'menulis CSV latensi ke {self._csv_path}')

        self.create_subscription(
            UwbAnchorReport, '/uwb/anchor_reports',
            self._on_report, QOS_SENSOR_BEST_EFFORT_DEEP)
        self.create_subscription(
            CorrectedToA, '/uwb/corrected_toa',
            self._on_corrected, QOS_STATE_RELIABLE)
        self.create_subscription(
            PoseWithCovarianceStamped, '/state/position_compensated',
            self._on_position, QOS_STATE_RELIABLE)

        self._pub = self.create_publisher(
            Vector3Stamped, '/diag/pipeline_latency_ms', QOS_STATE_RELIABLE)

        self._timer = self.create_timer(self._log_every, self._log_stats)

        self.get_logger().info(
            'latency_monitor aktif — ukur ingest→output '
            '(t0 = paket tiba di laptop; komponen tag→laptop tidak terukur, '
            'lihat keterbatasan di header file)')

    # ------------------------------------------------------------------
    def _on_report(self, msg: UwbAnchorReport) -> None:
        if msg.report_type != REPORT_TYPE_SLAVE_TAG:
            return
        seq = int(msg.seq)
        t_ns = _stamp_ns(msg.header.stamp)
        # simpan TERAWAL: laporan pertama yang tiba untuk blink ini
        prev = self._ingest.get(seq)
        if prev is None or t_ns < prev:
            self._ingest[seq] = t_ns
            self._ingest.move_to_end(seq)
        while len(self._ingest) > self.MAX_INGEST_ENTRIES:
            self._ingest.popitem(last=False)

    def _on_corrected(self, msg: CorrectedToA) -> None:
        self._stamp2seq[_stamp_ns(msg.header.stamp)] = int(msg.tag_seq)
        while len(self._stamp2seq) > self.MAX_STAMP_ENTRIES:
            self._stamp2seq.popitem(last=False)

    def _on_position(self, msg: PoseWithCovarianceStamped) -> None:
        t_out_ns = self.get_clock().now().nanoseconds
        stamp_ns = _stamp_ns(msg.header.stamp)

        seq = self._stamp2seq.get(stamp_ns)
        t0_ns = self._ingest.get(seq) if seq is not None else None
        if t0_ns is None:
            self._n_unmatched += 1
            return

        total_ms = (t_out_ns - t0_ns) * 1e-6
        hulu_ms = (stamp_ns - t0_ns) * 1e-6
        hilir_ms = (t_out_ns - stamp_ns) * 1e-6

        # buang sampel non-fisik (jam mundur / korelasi salah) — fail-safe
        if total_ms < 0.0 or total_ms > 5000.0:
            self._n_unmatched += 1
            return

        self._n_matched += 1
        self._total.append(total_ms)
        self._hulu.append(hulu_ms)
        self._hilir.append(hilir_ms)

        out = Vector3Stamped()
        out.header = msg.header
        out.vector.x = total_ms
        out.vector.y = hulu_ms
        out.vector.z = hilir_ms
        self._pub.publish(out)

        if self._csv is not None:
            self._csv.writerow([
                f'{t_out_ns * 1e-9:.6f}', seq,
                f'{total_ms:.3f}', f'{hulu_ms:.3f}', f'{hilir_ms:.3f}'])

    # ------------------------------------------------------------------
    @staticmethod
    def _summ(d: deque) -> str:
        if not d:
            return '—'
        vals = sorted(d)
        p95 = vals[min(len(vals) - 1, int(0.95 * len(vals)))]
        return (f'mean={statistics.fmean(vals):.1f} '
                f'med={statistics.median(vals):.1f} '
                f'p95={p95:.1f} max={vals[-1]:.1f}')

    def _log_stats(self) -> None:
        if self._n_matched == 0 and self._n_unmatched == 0:
            return
        denom = self._n_matched + self._n_unmatched
        match_pct = 100.0 * self._n_matched / denom if denom else 0.0
        snap_ms = 500.0 / self._agg_rate if self._agg_rate > 0 else 0.0
        self.get_logger().info(
            f'latensi ms (jendela {len(self._total)}): '
            f'TOTAL[{self._summ(self._total)}]  '
            f'hulu[{self._summ(self._hulu)}]  '
            f'hilir[{self._summ(self._hilir)}]  '
            f'match={match_pct:.0f}% (n={self._n_matched}) | '
            f'+jeda snapshot aggregator rata-rata ~{snap_ms:.0f} ms @'
            f'{self._agg_rate:.0f} Hz (analitik, tidak diukur)')

    def destroy_node(self) -> bool:
        if self._csv_file is not None:
            try:
                self._csv_file.close()
            except OSError:
                pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LatencyMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
