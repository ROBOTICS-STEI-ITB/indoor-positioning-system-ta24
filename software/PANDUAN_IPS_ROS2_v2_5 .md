# Panduan Lengkap — IPS ROS2 Indoor Positioning System
**TA24 | ITB Teknik Elektro | UWB TDoA 3D**
**Versi dokumen: v2.5.0 — + optitrack_bridge GT marker tag (param source/gt_marker_idx/input_units) + fix entry point setup.py ips_nodes + setelan NatNet Motive + flag verifikasi (indeks marker, konstanta transform). [Basis v2.4.0: state_aggregator v3.3 /state/json + orientasi opsi A + demo offline + bias_compensator v2 affine + analyze_latency.py]**

---

## Changelog

| Versi | Perubahan |
|---|---|
| v0.1.0 | Pipeline awal: 6 node Python, JSON output |
| v0.2.0 | Recorder node + service `RecordControl` |
| v0.3.0 | Glitch protection delta_k, DZS simetris |
| v0.4.0 | AsymmetricDZSFilter + G1 pre-filter |
| v0.5.0 | StudentTFilter (robust KF) |
| v1.0.0 | Migrasi Jazzy + C++ compute + Two-Layer (L1+L2) + bias compensator |
| **v2.0.0** | **Estimator WoLF-EKF-CA (alternatif Chan) + ZUPT + arena OptiTrack + validasi GT presisi** |
| **v2.1.0** | **Integrasi IMU bertahap (T1 ingest → T1.5 filter → T2 rekam → T2.1 semua besaran) + WoLF sebagai sumber kecepatan output** |
| **v2.1.1** | **WoLF v0.3.3 anti-divergensi (clamp σ_a_max + divergence guard) — fix ledakan saat manuver di arena besar; arena baru 5.19×5.03** |
| **v2.2.0** | **Produk WoLF-only (Chan = seed/recovery internal); kerangka tangan-kanan (swap x↔y); bias affine v4 (M·p+b, 34 titik); temuan validasi vs GT (affine global mentok ~150mm, z terlemah, bias spasial); bias_values_ns frame baru** |
| **v2.3.0** | **+ latency_monitor (ukur latensi ingest→output); state_aggregator v2 (format `lines` rapi, urutan tetap, kecepatan dari wolf_velocity); optitrack_bridge (GT NatNet live → /gt/pose, /gt/position); demo pameran HTML (visualisasi 3D via rosbridge, smoothing di display)** |
| **v2.4.0** | **state_aggregator v3.3 (format JSON terstruktur {tag_id, position, velocity:{linear,angular}, acceleration:{linear,angular}, orientation:{roll,pitch,yaw}}, publikasi ke topik `/state/json`); koreksi orientasi opsi A (flip Euler manual untuk BNO055); demo_pameran_offline (bundle lib lokal, tanpa internet); bias_compensator v2 (affine + orientation passthrough); analyze_latency.py (analisis offline CSV)** |
| **v2.5.0** | **optitrack_bridge mode marker tag (GT posisi dari marker, bukan rigid body) + param `source`/`gt_marker_idx`/`input_units`; fix entry point `optitrack_bridge` di setup.py ips_nodes; tabel setelan NatNet Motive (Local Interface≠loopback, Up Axis Y, Scale=1, Labeled Markers); flag inkonsistensi indeks marker (0/1/2) & konstanta transform (0.646 vs 0.637) untuk verifikasi** |

**Perubahan utama v2.5.0 (paling baru — sesi GT live marker tag + setelan NatNet):**

1. **optitrack_bridge: sumber GT = marker tag** (bukan rigid body). Parameter
   baru `source` (`marker`|`rigid_body`), `marker_set`, `gt_marker_idx`
   (indeks marker tag — **berubah tiap sesi**, set via param), `input_units`
   (`m`|`mm`). Output `/gt/position` (utama) + `/gt/pose` (orientasi =
   identitas di mode marker). Lihat §6.4.
2. **Fix entry point** `optitrack_bridge` di `setup.py` paket `ips_nodes`
   (sebelumnya TIDAK terdaftar di `console_scripts` → `ros2 run` gagal
   `executable not found`). Lihat §6.4 prasyarat build.
3. **Tabel setelan NatNet Motive** lengkap (Local Interface ≠ loopback,
   Up Axis = Y-Axis, Scale = 1, Labeled Markers ON, port 1510/1511,
   multicast 239.255.42.99). Lihat §6.4.
4. **Flag verifikasi (⚠ belum tuntas — butuh konfirmasi lapangan):**
   - **Indeks marker tag tidak konsisten** antar file referensi:
     `export_gt_synced.py` = `GT_MARKER_IDX=1`; `ips_analysis_common.py` =
     `GT_MARKER_IDX=0` (komentar di file klaim idx=2 = tag terverifikasi).
     Tiga nilai beredar (0/1/2) → set via `gt_marker_idx` + verifikasi
     `raw_opti` di log node.
   - **Konstanta transform beda antar sesi kalibrasi:** `export_gt_synced`
     X=0.646/Z=3.425 vs `ips_analysis_common` X=0.637/Z=3.426 (~9 mm).
     Lihat §9.2.
5. **Update laju data:** blink kini **~25 Hz tanpa loss** (drop ~0%, match
   ~100%; sebelumnya ~16% drop) dan **IMU/BNO055 kini ~25 Hz** (sebelumnya
   ~20 Hz). Laju topik IMU di §2 & spec §11.3 disesuaikan.

---

**Perubahan utama v2.4.0 (paling baru — sesi orientasi + format JSON + bundling offline):**

1. **state_aggregator v3.3**: format JSON DIUBAH ke struktur bersarang
   `{tag_id, timestamp, position, velocity:{linear,angular},
    acceleration:{linear,angular}, orientation:{roll,pitch,yaw}}`.
   **Publikasi ke topik ROS `/state/json`** (`std_msgs/String`) — konsumsi
   via `ros2 topic echo /state/json` di terminal terpisah. Lihat §9.1
   parameter `publish_topic` + §6.6 konsumsi JSON.
2. **Koreksi orientasi opsi A**: untuk BNO055 dengan remap fisik
   (REMAP_CONFIG=0x18, SIGN=0x01), aggregator menerapkan
   `roll = -roll_BNO`, `pitch = pitch_BNO`, `yaw = 360 - yaw_BNO` ke output
   JSON. Topik `/state/orientation` TIDAK diubah — RViz tetap pakai
   quaternion mentah. Lihat §6.6 + §10c orientation correction.
3. **demo_pameran_offline**: versi self-contained dengan Three.js + roslibjs
   ter-bundle di folder `lib/`, font sistem (tidak Google Fonts). Aman untuk
   venue tanpa internet. Lihat §6.5.
4. **bias_compensator v2** (orient_affine_patch): gabungan AFFINE penuh
   (M·p+b) + orientation passthrough (lampirkan quaternion IMU ke pose).
   Backward-compatible: baca format YAML lama (`bias:{x,y,z}`) maupun
   affine baru (`bias_model: affine, bias_matrix, bias_offset`).
5. **analyze_latency.py**: skrip analisis offline untuk CSV `latency_monitor`
   dengan 5 plot (time series, histogram, CDF, breakdown, heatmap).
   Lihat §6.3.

1. **latency_monitor** (node baru, ips_nodes): ukur latensi pemrosesan
   ingest→output via korelasi blink seq + stamp. Zero-touch (tidak ubah node
   pipeline). Lihat §6.3.
2. **state_aggregator v2** (ganti): format keluaran `lines` — satu state per
   baris, urutan kunci TETAP, dibulatkan, tetap JSON valid. Kecepatan dari
   `/state/wolf_velocity` (sebelumnya dari differentiator). Lihat §9.1 param
   `output_style` + §6 contoh output.
3. **optitrack_bridge** (node baru, ips_nodes): terima OptiTrack NatNet
   streaming → transform ke koordinat sistem → publish `/gt/pose` + `/gt/position`.
   Self-contained (tanpa SDK eksternal). Untuk **demo teknis** overlay GT vs
   estimasi di RViz live. Lihat §6.4.
4. **Demo pameran HTML** (file `index.html` + rosbridge): visualisasi 3D untuk
   audiens umum via WebSocket. Smoothing **di display** (EMA), bukan di
   sistem — data sistem tetap asli untuk metrik. Lihat §6.5.

⚠ **Verifikasi WAJIB sebelum eksperimen** (silent no-op risk): pastikan
`bias_compensator` di workspace = versi **AFFINE** (baca `bias_matrix`), bukan
versi offset-only lama. Cek: `grep -l "bias_matrix" ~/ips_jazzy_ws/src/ips_nodes_cpp/src/bias_compensator_node.cpp`.

**Perubahan utama v2.2.0:**

1. **Estimator produk = WoLF-EKF-CA tunggal.** Chan TDoA hanya dipakai internal
   untuk seed inisialisasi + pemulihan divergensi — **tidak** dilaporkan atau
   dibandingkan sebagai estimator keluaran. (position_solver masih ada di kode
   untuk uji, tapi bukan jalur produk.)
2. **Kerangka koordinat tangan-kanan.** anchors.yaml diperbaiki dari left-handed
   via **tukar x↔y**. Arena = 5.026 × 5.191 × 3.5 m. Data sudah direkam ulang di
   kerangka baru. Flag `SWAP_XY` di toolkit lama sudah dihapus (frame baru
   di-hardcode). Lihat §9.2.
3. **Bias model = affine** `p_true = M·p_meas + b` (bukan offset-only). bias.yaml
   v4: 34 titik, M≈I, rmse 222→178mm. Lihat §5.4.
4. **Temuan validasi vs OptiTrack (metode bersih: nearest-neighbor tanpa
   interpolasi GT):** x,y sehat (~50–70mm de-biased); **z terlemah** (~120–140mm
   saat bergerak; VDOP buruk + cakupan-z kalibrasi tipis). Residual 98–99%
   sistematik (bias spasial fisik), acak hanya ~10mm. **Affine global sudah
   mentok ~130–170mm** — bottleneck = bias bergantung-posisi, butuh peta bias
   spasial (GP/RBFN), bukan tuning WoLF. Lihat §11.
5. **bias_values_ns kerangka baru.** Nilai live (WoLF): `[0.0, -8.366, 1.641,
   -8.663]`. ⚠ SA3/SA4 tertukar urutannya vs position_solver (stale) — lihat
   catatan §9.1.

---

**Perubahan utama v2.0.0:**

1. **Estimator kedua: WoLF-EKF-CA** (Duran-Martin et al. 2024) — node
   `wolf_position_node` paralel dengan `position_solver` (Chan). Toggle lewat
   launch arg `algorithm:=chan|wolf`. WoLF = EKF 9-state constant-acceleration
   dengan IMQ weighting (anti-outlier) + adaptive σ_a. Menggantikan
   L1+Chan+L2+KF dengan satu blok terpadu.
2. **ZUPT (Zero-Velocity Hold)** — post-filter pada output WoLF; tekan jitter
   hover. Gate `‖v̂‖` state EKF + spread jendela + histeresis.
3. **Arena baru terukur OptiTrack** — anchor & ruang dikalibrasi via motion
   capture (lihat §9.2). Konfig lama (SA3 x=3, ruang 3.43×2.74) USANG.
4. **OptiTrack sebagai ground truth presisi sub-mm** — validasi pakai
   `eval_harness_v2.py` (sync cross-correlation + gap-mask + metrik adil).
5. **LI-KF clock sync terverifikasi setia paper Zhang 2024** (sampai matriks
   F 3×3, H=[1 0 0], dan nilai R=1.5e-20).

**Status estimator:** pipeline lama (Chan + L1/L2) DIPERTAHANKAN — masih dipakai
untuk uji diferensial & A/B. WoLF adalah jalur pengembangan utama untuk dinamis.

**Perubahan utama v2.1.0:**

1. **Integrasi IMU bertahap (BNO055 di tag).** Empat tingkat terpisah, masing-
   masing divalidasi sendiri:
   - **Tingkat 1** — ingest: paket IMU 17-field di-parse udp_gateway → `/imu/raw`
     (ImuTelemetry). Node compute C++: `imu_processor` (orientasi/gyro/accel) +
     `differentiator` dipindah Python→C++.
   - **Tingkat 1.5** — filter pipeline C++ (port dari read_UDP.py): LPF EMA +
     Kalman 1D (gyro/accel, default OFF) + Savitzky-Golay untuk turunan
     (menggantikan backward-difference).
   - **Tingkat 2** — rekam IMU ke CSV dengan jangkar sinkron (ros_time/tag_ms/blink).
   - **Tingkat 2.1** — rekam SEMUA besaran (14 CSV) + WoLF v0.3.2 publish
     `/state/wolf_velocity` (kecepatan state EKF).
2. **WoLF sebagai sumber kecepatan output sistem.** state_aggregator kini ambil
   kecepatan dari `/state/wolf_velocity` (EKF state), bukan differentiator.
   Dasar: analisis vs OptiTrack — wolf 6.3× lebih akurat (RMSE 0.36 vs 2.29 m/s),
   kebal spike (max 2.4 vs 89 m/s). Lihat §10c.
3. **Asal data IMU diperjelas**: gyro = sensor langsung; orientasi (quat/Euler) &
   linear-accel = hasil fusi internal BNO055 (di chip). Tidak ada velocity
   translasi & angular-accel di `/imu/raw` (itu besaran turunan, dihitung node).

---

## 1. Pipeline Data Flow

**Produk akhir = estimator WoLF.** Chan hanya seed/recovery internal WoLF
(bukan estimator keluaran, tidak dilaporkan di paper). Toggle
`algorithm:=chan|wolf` masih ada di kode untuk uji diferensial, tetapi jalur
produk dan semua hasil di dokumen ini memakai **WoLF**.

```
[MC+SA2-SA5]──UDP──> udp_gateway ──> clock_sync (C++, LI-KF) ──> /uwb/corrected_toa
 192.168.10.x                            │ glitch filter              │
                          calibration_service                         │
                          (latched anchor pos)──────────┐             │
                                                        ▼             ▼
            ┌──────────────────── algorithm:=chan ──────────────────────────┐
            │  position_solver (C++)                                         │
            │    KF.predict_only → [Layer 1 Huber] → Chan → [Layer 2 gate]  │
            │    → Student-t α-scaled KF → /state/position                  │
            └────────────────────────────────────────────────────────────────┘
            ┌──────────────────── algorithm:=wolf ──────────────────────────┐
            │  wolf_position (C++)                                           │
            │    WoLF-EKF-CA 9-state: predict (adaptive σ_a) → IMQ update    │
            │    → [ZUPT post-filter saat hover] → /state/position          │
            └────────────────────────────────────────────────────────────────┘
                                                        │
                                              /state/position (raw)
                                              /state/position_chan (diagnostic)
                                                        │
                                              bias_compensator (C++, affine M·p+b)
                                                        │
                                              /state/position_compensated  (UTAMA)
                                                        │
                                              differentiator (C++) → state_aggregator
                                                                      │
                                                               recorder (CSV)

[Tag IMU BNO055]──UDP──> udp_gateway ──> /imu/raw (ImuTelemetry, ~25 Hz)
                                              │
                          imu_processor (C++): quat→/state/orientation,
                          gyro→[LPF/Kalman]→/state/angular_velocity,
                          accel→[LPF/Kalman]→/state/translation_acceleration
                                              │
                          differentiator (C++, SG): /state/angular_acceleration
                          (cabang IMU paralel — independen dari estimator posisi)

  Sumber kecepatan output (v2.1): state_aggregator ambil /state/wolf_velocity
  (EKF state, hanya saat algorithm:=wolf) — bukan /state/translation_velocity.
```

**Catatan:** kedua estimator subscribe `/uwb/corrected_toa` dan publish ke
`/state/position` yang sama, jadi `bias_compensator` di hilir tidak peduli mana
yang aktif. WoLF tetap menghitung Chan per-blink hanya untuk diagnostic
`/state/position_chan` (tidak masuk filter).

---

## 2. Daftar Semua Topic

| Topic | Type | Rate | Publisher | Keterangan |
|---|---|---|---|---|
| `/uwb/anchor_reports` | `UwbAnchorReport` | ~233 Hz | udp_gateway | Semua paket UDP |
| `/uwb/session_events` | `SessionEvent` | event | udp_gateway | HELLO / RESET / HB |
| `/uwb/corrected_toa` | `CorrectedToA` | ~25 Hz × 4 | clock_sync | ToA setelah LI-KF |
| `/uwb/sync_status` | `SyncStatus` | 1 Hz | clock_sync | Diagnostik sync |
| `/uwb/anchor_config` | `PoseArray` | latched | calibration_service | Posisi anchor |
| `/state/position` | `PoseWithCovarianceStamped` | ~25 Hz | position_solver | KF raw output |
| `/state/position_chan` | `PointStamped` | ~25 Hz | position_solver | Raw Chan |
| `/state/position_compensated` | `PoseWithCovarianceStamped` | ~25 Hz | bias_compensator | Bias-corrected **(UTAMA)** |
| `/state/translation_velocity` | `Vector3Stamped` | ~25 Hz | differentiator | Kecepatan (diff turunan posisi) |
| `/state/wolf_velocity` | `Vector3Stamped` | ~25 Hz | wolf_position | Kecepatan state EKF (**sumber kecepatan output v2.1**) |
| `/imu/raw` | `ImuTelemetry` | ~25 Hz | udp_gateway | IMU mentah (Euler/quat/gyro/accel + blink) |
| `/state/orientation` | `QuaternionStamped` | ~25 Hz | imu_processor | Quaternion (fusi BNO055) |
| `/state/angular_velocity` | `Vector3Stamped` | ~25 Hz | imu_processor | Gyro (terfilter bila LPF/Kalman ON) |
| `/state/translation_acceleration` | `Vector3Stamped` | ~25 Hz | imu_processor | Accel linear (terfilter bila ON) |
| `/state/angular_acceleration` | `Vector3Stamped` | ~25 Hz | differentiator | Percepatan sudut SG (perlu enable_angular) |
| `/diag/pipeline_latency_ms` | `Vector3Stamped` | ~25 Hz | latency_monitor | x=total, y=hulu, z=hilir (ms) — **v2.3** |
| `/gt/pose` | `PoseStamped` | ~120 Hz | optitrack_bridge | GT OptiTrack (frame `world`); **orientasi = identitas di mode marker** (v2.5) — **v2.3** |
| `/gt/position` | `PointStamped` | ~120 Hz | optitrack_bridge | GT posisi saja (mudah plot) — **v2.3** |
| `/state/json` | `std_msgs/String` | output_rate_hz | state_aggregator | Snapshot JSON terstruktur (v2.4) — `ros2 topic echo /state/json` |

**Output posisi utama**: `/state/position_compensated` (sudah Layer 1 + Layer 2 + bias compensated).

---

## 3. Daftar Semua Node

| Node | Package | Executable | Bahasa | Tugas |
|---|---|---|---|---|
| `udp_gateway` | ips_nodes | `udp_gateway` | Python | Terima UDP, publish ke ROS |
| `clock_sync` | ips_nodes_cpp | `clock_sync_node` | **C++** | LI-KF clock sync + glitch protection |
| `position_solver` | ips_nodes_cpp | `position_solver_node` | **C++** | (algorithm=chan) L1 → Chan → L2 → StudentTFilter |
| `wolf_position` | ips_nodes_cpp | `wolf_position_node` | **C++** | (algorithm=wolf) WoLF-EKF-CA + IMQ + ZUPT |
| `bias_compensator` | ips_nodes_cpp | `bias_compensator_node` | **C++** | Koreksi bias affine (M·p+b) |
| `imu_processor` | ips_nodes_cpp | `imu_processor_node` | **C++** | IMU: orientasi + gyro/accel terfilter (selalu jalan) |
| `differentiator` | ips_nodes_cpp | `differentiator_node` | **C++** | Kecepatan (SG dari posisi) + percepatan sudut |
| `state_aggregator` | ips_nodes | `state_aggregator` | Python | JSON stdout/file |
| `calibration_service` | ips_nodes | `calibration_service` | Python | Load posisi anchor dari YAML |
| `recorder` | ips_nodes | `recorder` | Python | CSV recording on-demand (14 layer) |
| `latency_monitor` | ips_nodes | `latency_monitor` | Python | **v2.3** — ukur latensi ingest→output (opsional, diagnostik) |
| `optitrack_bridge` | ips_nodes | `optitrack_bridge` | Python | **v2.3/v2.5** — NatNet → GT **posisi marker tag** (`source:=marker`); ⚠ butuh entry point di setup.py (§6.4) |

**Catatan:** `position_solver` dan `wolf_position` saling eksklusif — hanya satu
yang di-launch tergantung `algorithm`. Default `chan`. `imu_processor` dan
`differentiator` (cabang IMU) **selalu jalan**, independen dari pilihan estimator.

---

## 4. Cara Run

### 4.1 Setup awal

```bash
cd ~/ips_jazzy_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Tambahkan ke `~/.bashrc`:
```bash
echo "source ~/ips_jazzy_ws/install/setup.bash" >> ~/.bashrc
```

### 4.2 Launch semua (all-C++ compute)

```bash
# PRODUK: estimator WoLF-EKF-CA (dengan ZUPT) — INI yang dipakai
ros2 launch ips_bringup ips_system_cpp.launch.py algorithm:=wolf

# (uji diferensial saja) estimator Chan — bukan jalur produk
ros2 launch ips_bringup ips_system_cpp.launch.py algorithm:=chan

# Dengan opsi
ros2 launch ips_bringup ips_system_cpp.launch.py \
    algorithm:=wolf \
    bias_yaml:=~/ips_jazzy_ws/bias.yaml \
    record_dir:=~/ips_logs \
    output_rate_hz:=10.0
```

### 4.3 Tanda sistem siap

**Estimator Chan (default):**
```
[calibration_service] published anchor_config for ids=[1, 2, 3, 4, 5]
[position_solver] position_solver (C++) ready  KF=on  kf_type=student_t  L1=on  L2=on  DZS=off
[bias_compensator] bias_compensator (C++) ready  state=OPERATIONAL  mode=AFFINE
[Layer 1] clip stats: SA3: 0/20 (0.00%)  ...
[Layer 2] innovation gate: rejected 0/20 (0.00%)  last_NIS=2.35  threshold=11.35
```

**Estimator WoLF:**
```
[wolf_position] wolf_position (C++) v0.3.0 ready — WoLF-EKF-CA 9-state, IMQ c=0.30, σ_a_min=0.005, σ_a_gain=1.00, ZUPT=ON
[wolf_position] anchor_config loaded: 5 anchors, WoLF initialized from pos_nominal=(2.51, 2.60, 1.75) with P0_pos_std=1.50 m
[bias_compensator] bias_compensator (C++) ready  state=OPERATIONAL  mode=AFFINE
[wolf_position] WoLF: pos=(1.05,1.04,1.73) |v|=0.03m/s  w_mean=0.98  w<0.5: 0.5%  ZUPT=HOLD  (n=210)
```
Status `ZUPT=HOLD` muncul saat drone diam (post-filter aktif); `live` saat
bergerak. Posisi langsung keluar setelah blink pertama (tidak ada delay init).

### 4.4 Service tambahan (opsional — v2.3)

Tiga komponen diagnostik/visualisasi yang **terpisah** dari launch utama; jalankan
sesuai kebutuhan di terminal terpisah:

```bash
# Ringkasan satu-baris: kapan jalankan apa
# ─────────────────────────────────────────────────────────────────
# Selalu (utama):       ros2 launch ips_bringup ips_system_cpp.launch.py algorithm:=wolf
# Diagnostik latensi:   ros2 run ips_nodes latency_monitor          → §6.3
# GT live (RViz demo):  ros2 run ips_nodes optitrack_bridge         → §6.4
# Demo pameran HTML:    ros2 launch rosbridge_server rosbridge_websocket_launch.xml  → §6.5
```

**Tabel keputusan — kapan butuh apa:**

| Skenario | Launch utama | latency_monitor | optitrack_bridge | rosbridge |
|---|---|---|---|---|
| Eksperimen / rekam data | ✓ | (saat ukur latensi) | — | — |
| Validasi vs GT (offline) | ✓ | — | — *(pakai CSV)* | — |
| Demo teknis RViz + GT live | ✓ | — | ✓ | — |
| Demo pameran HTML (umum) | ✓ | — | — | ✓ |
| Validasi + demo pameran | ✓ | (opsional) | — | ✓ |

**Catatan:**
- Semua node v2.3 (`latency_monitor`, `optitrack_bridge`) ada di paket `ips_nodes`.
- `rosbridge_websocket` bukan kode kita — paket standar ROS, tinggal launch.
- Tiap subbagian (§6.3–§6.5) berisi setup detail + troubleshooting.

---

## 5. Kalibrasi Bias Compensator

### 5.0 Dua jenis kalibrasi (jangan tertukar)

Sistem punya **dua** koreksi bias terpisah, di tahap berbeda:
1. **`bias_values_ns`** (domain nanodetik, per-anchor, **pra-Chan**) — delay
   antena tiap anchor, dari `self_calibrate.py`. Ada di system.yaml (§9.1).
2. **bias affine** (domain meter, **pasca-KF**) — `p_true = M·p_meas + b`,
   koreksi distorsi geometris sistematis. Di bias.yaml (§5.4).

Bagian ini membahas yang **kedua** (affine). Karena affine = transformasi 3D
(12 parameter), ia di-fit dari **banyak titik** dengan ground truth, **bukan**
satu titik. Service call satu-titik lama hanya mengisi offset; untuk affine
penuh gunakan workflow fit offline di bawah.

### 5.1 Prosedur kalibrasi affine (workflow deploy)

**Prinsip penting (dari analisis):** kalibrasi dengan **tag DIAM (hover) di
koordinat terketahui**, BUKAN sambil manuver. Affine mengoreksi distorsi
geometris **statis**; getaran/gerak adalah noise dinamis (domain filter, bukan
bias). Tiap sumbu (x, y, z) harus **bervariasi independen** antar-titik — kalau
satu sumbu diam saat rekam, skalanya tak teramati dan M bisa liar (uji nyata:
data x-diam → diag(M) rusak ke 0.04). Sebar titik D-optimal (ekstrem ruang + z
penuh).

```bash
# 1. Generate ~12 titik kalibrasi D-optimal (pojok + z penuh + space-filling)
python3 calib_targets.py        # -> koordinat hover + peta 3D "terbang ke sini"

# 2. Terbangkan drone ke tiap titik, HOVER 10-30 s, satu sesi kontinu (~4 menit)
#    (warm-up 30 menit dulu; jangan restart antara warm-up & rekam)

# 3. Export GT tersinkron (COMP_FILE = position.csv MENTAH, bukan compensated,
#    agar affine = raw->GT penuh). Edit path di blok atas skrip lalu:
python3 export_gt_synced.py     # -> gt_synced.csv [ros_time, sys_xyz, gt_xyz, valid]

# 4. Fit affine (ridge-toward-identity, kekuatan via cross-validation) + opsi GP
python3 fit_bias.py             # -> bias.yaml (skema affine v4)
```

Kunci efisiensi-titik: D-optimal (ekstrem + z penuh) + ridge-toward-identity (M
ditarik ke I, bukan ke 0) + cross-validation (pilih ridge & ukur generalisasi
jujur). Target ≤10–15 titik agar praktis di lapangan.

> **Tanpa OptiTrack di lapangan:** meteran laser (~mm) sudah cukup presisi
> sebagai GT — error GT (mm) jauh di bawah RMSE sistem (puluhan mm), jadi bukan
> bottleneck. Yang penting penempatan tag presisi (jig/tripod), bukan alat ukur
> GT-nya. "Self-calibration tanpa GT sama sekali" tidak bisa memberi akurasi
> absolut (gauge freedom) — minimal butuh posisi anchor + beberapa jarak/titik
> terketahui untuk mengunci kerangka.

### 5.1b Kalibrasi cepat satu-titik (offset saja — bukan affine penuh)

Untuk koreksi offset cepat (bukan affine penuh), service call masih tersedia:

```bash
ros2 launch ips_bringup ips_system_cpp.launch.py    # warm-up dulu
# tag diam di GT terukur, lalu:
ros2 service call /bias_compensator/calibrate ips_msgs/srv/Calibrate \
    '{gt_x: 2.5, gt_y: 2.6, gt_z: 1.15, n_samples: 500, skip_warmup: 100}'
# bias auto-saved; launch berikutnya auto-load
```

### 5.2 Response service

```
success: True
message: "Calibration done with 400 samples. Bias: (-11.22, -1.27, -48.30) cm"
bias_x: -0.1122
bias_y: -0.0127
bias_z: -0.4830
std_x: 0.0077
std_y: 0.0119
std_z: 0.0148
samples_used: 400
```

### 5.3 Re-kalibrasi

Kalibrasi ulang **wajib dilakukan** ketika:

- Setelah re-install Layer 1 atau Layer 2 (bias raw berubah setelah filter baru aktif)
- Setelah posisi anchor diubah atau `anchors.yaml` diupdate
- Setelah antenna delay di-kalibrasi ulang

Cukup panggil service lagi di titik GT yang sama. File bias.yaml akan dioverwrite.

### 5.4 Format bias.yaml (affine)

Model bias adalah **affine 3D**: `p_true = M·p_meas + b` (matriks M 3×3 + offset
b). Ini mengoreksi distorsi geometris sistematis (skala + rotasi kecil + offset),
bukan sekadar offset konstan. Skema yaml:

```yaml
# bias.yaml v4 -- affine, fit_bias_ftmd.py (arena baru, GT campuran)
bias_model: affine
bias_matrix:                          # M (3x3)
  - [0.998494, -0.005812, 0.014481]
  - [-0.023187, 1.013112, -0.010001]
  - [0.001406, -0.041357, 0.965379]
bias_offset:                          # b
  x: -0.100123
  y: 0.046609
  z: 0.152811
calibration_points: 34
rmse_before_m: 0.221904
rmse_after_m: 0.178305
timestamp: 2026-06-09T21:25:17
```

**Catatan model (dari analisis vs GT):**
- M ≈ I (diagonal ~0.998/1.013/0.965, suku silang kecil) → bias dominan adalah
  **offset b**; M memperbaiki bentuk ~20% (terutama skala z).
- **bias_compensator HARUS versi affine** (baca `bias_matrix`). Versi offset-only
  lama membaca key `bias:{x,y,z}` — kalau terpasang, bias.yaml affine ini tidak
  terbaca → bias 0 **tanpa error** (silent no-op). Verifikasi versi node
  (lihat §8.5).
- `hybrid_calib_enabled` HARUS `false` saat bias affine aktif (mencegah
  double-correction).

⚠ **Batas affine global (temuan penting):** affine satu-matriks sudah **mentok
~130–170mm** karena bias **bergantung-posisi** (offset berbeda antar-wilayah
ruangan). Sumbu z terlemah (~120–140mm saat bergerak; VDOP buruk + cakupan-z
kalibrasi tipis). Langkah akurasi berikutnya = kalibrasi z-coverage penuh + peta
bias spasial (GP/RBFN), bukan tuning WoLF. Lihat §11.

---

## 6. Melihat Posisi Realtime

### 6.1 Topic echo

```bash
# Posisi terkoreksi bias (UTAMA)
ros2 topic echo /state/position_compensated --field pose.pose.position

# Posisi raw KF (untuk debug)
ros2 topic echo /state/position --field pose.pose.position

# Raw Chan (untuk debug)
ros2 topic echo /state/position_chan
```

### 6.2 PlotJuggler

```bash
sudo apt install ros-jazzy-plotjuggler-ros
ros2 run plotjuggler plotjuggler
# Streaming -> ROS2 Topic Subscriber -> Start
# Drag /state/position_compensated/pose/pose/position/x, y, z ke panel
```

### 6.3 Latency monitor (v2.3 — ukur latensi pipeline)

Ukur latensi pemrosesan **ingest→output**: dari paket blink tiba di laptop
sampai posisi terkalibrasi tersedia (yang dibaca state_aggregator). Zero-touch
— tidak mengubah node pipeline apa pun; korelasi via blink seq + stamp.

```bash
# Terminal terpisah saat sistem jalan:
ros2 run ips_nodes latency_monitor

# Atau dengan CSV per-sampel untuk analisis offline:
ros2 run ips_nodes latency_monitor --ros-args -p csv_path:=latency_sesi1.csv
```

**Output log tiap 5 detik:**
```
latensi ms (jendela 104): TOTAL[mean=12.3 med=11.8 p95=18.2 max=24.1]
  hulu[mean=8.1 ...] hilir[mean=4.2 ...] match=96% (n=104)
  | +jeda snapshot aggregator rata-rata ~50 ms @10 Hz (analitik, tidak diukur)
```

- **TOTAL** = latensi pemrosesan ingest→output (angka utama)
- **hulu** = ingest → keluar clock_sync
- **hilir** = clock_sync → output tersedia (`TOTAL = hulu + hilir`)
- **match** = persentase posisi yang berhasil dikorelasikan ke blink-nya
  (komplemennya ≈ drop rate). **Update v2.5: blink kini ~25 Hz tanpa loss →
  match ~100% (drop ~0%).** (Sebelumnya tercatat ~16% drop / match ~84%.)

**Topik diagnostik:** `/diag/pipeline_latency_ms` (Vector3: x=total, y=hulu,
z=hilir, milidetik). Bisa diplot di PlotJuggler.

**Batas yang harus disebut jujur di paper** (lihat §11.3):
- t0 = paket **tiba di laptop**, BUKAN saat tag memancar (jam tag tak
  sinkron). Komponen radio + WiFi tag→laptop tidak terukur.
- Jeda snapshot aggregator (~50ms @10Hz) dilaporkan analitik terpisah.

**Analisis offline CSV (v2.4 — `analyze_latency.py`):**

Untuk menganalisis CSV yang dihasilkan `latency_monitor`, jalankan
`analyze_latency.py` (Python; lihat `/mnt/user-data/outputs/`):

```bash
python3 analyze_latency.py    # set CSV_FILE di blok konfigurasi
```

Menghasilkan 5 plot di folder `latency_plots/`:
- **time_series.png** — TOTAL/hulu/hilir vs waktu + deteksi anomali
- **histogram.png** — distribusi tiga komponen
- **cdf.png** — CDF komparatif (utama untuk paper)
- **breakdown.png** — stacked bar hulu vs hilir per-menit
- **heatmap.png** — latensi vs waktu sebagai heatmap intensitas

Untuk paper IEEE: pakai `cdf.png` (figur utama) + `breakdown.png`.

### 6.4 OptiTrack bridge (v2.5 — GT live POSISI dari marker tag)

Terima OptiTrack NatNet streaming → ambil **posisi marker tag** → transform ke
koordinat sistem (identik `export_gt_synced.py`) → publish ke topik ROS. Untuk
**GT live posisi** (overlay di RViz, demo teknis).

> **Perubahan v2.5 (penting):** node SEBELUMNYA melacak **rigid body**; sekarang
> sumber GT default = **marker tag** (sesuai keputusan: GT posisi saja, pakai
> marker). Mode rigid body masih ada (`source:=rigid_body`) tapi bukan jalur
> utama. Marker tunggal tak punya orientasi → `/gt/pose` memakai orientasi
> identitas.

**⚠ Prasyarat build — entry point WAJIB ada.** `setup.py` paket `ips_nodes`
SEBELUMNYA **tidak** mendaftarkan `optitrack_bridge` di `console_scripts` →
`ros2 run ips_nodes optitrack_bridge` gagal (`executable not found`). Pastikan
baris ini ada di `setup.py` lalu rebuild:
```python
'optitrack_bridge = ips_nodes.optitrack_bridge_node:main',
```
(patch `optitrack_gt_patch.tar.gz` + `setup_optitrack_gt.sh` menambah baris ini
secara idempotent, lalu `colcon build --symlink-install --packages-select ips_nodes`.)

**Setup jaringan** (sekali):
```bash
# Laptop ROS:
sudo apt install ros-jazzy-rosbridge-suite   # untuk demo HTML; bukan untuk bridge ini
sudo ufw allow 1510:1511/udp                  # firewall NatNet
```

**Setup Motive (Windows) — View → Data Streaming:**

| Setelan | Nilai wajib | Alasan |
|---|---|---|
| Enable / Broadcast Frame Data | ON | aktifkan streaming |
| **Local Interface** | **192.168.10.101** (IP LAN PC Motive) | ⚠ JANGAN `loopback` — loopback hanya kirim ke PC Motive sendiri, laptop ROS tak terima apa pun |
| Transmission Type | Multicast | cocok `use_multicast:=true` (default node) |
| **Labeled Markers** | ON | **wajib** agar marker tag ikut di-stream |
| Rigid Bodies | ON | dipakai hanya untuk mode `rigid_body` |
| **Up Axis** | **Y-Axis** | transform mengasumsikan Y = atas (`z_sys = y_opti`) |
| **Scale** | **1** | satuan = meter ��� node default `input_units:=m` |
| Command Port | 1510 | default node |
| Data Port | 1511 | default node |
| Multicast Interface | 239.255.42.99 | default node `multicast_addr` |

(Opsional: buat **rigid body** dari marker tag hanya bila mau pakai
`source:=rigid_body`.)

**Jalankan (mode marker tag — utama):**
```bash
ros2 run ips_nodes optitrack_bridge --ros-args \
    -p server_ip:=192.168.10.101 \
    -p source:=marker \
    -p gt_marker_idx:=2          # ⚠ indeks marker tag — sesuaikan tiap sesi (lihat bawah)
```

**Parameter penting:**

| Param | Default | Keterangan |
|---|---|---|
| `server_ip` | 192.168.10.101 | IP PC Motive |
| `source` | `marker` | `marker` (tag) \| `rigid_body` |
| `marker_set` | `all` | nama marker set; fallback = set terbesar bila tak ketemu |
| `gt_marker_idx` | 2 | indeks marker dalam set — **berubah tiap sesi** |
| `input_units` | `m` | `m` \| `mm` (kalau NatNet kirim mm → dibagi 1000) |
| `rigid_body_id` | 1 | dipakai hanya saat `source:=rigid_body` |
| `x_opti_sa2` | 0.646 | konstanta transform (lihat §9.2 ⚠) |
| `z_opti_sa2` | 3.425 | konstanta transform (lihat §9.2 ⚠) |
| `z_floor` | 0.0 | konstanta transform |

**Menyesuaikan indeks marker tag tiap sesi:** saat start, node mencetak daftar
marker set + jumlah marker, dan tiap 5 dtk mencetak posisi mentah marker
terpilih:
```
[optitrack_bridge] marker set tersedia: "all"(5 marker)
[optitrack_bridge] src=marker['all'][2] raw_opti=(0.612, 1.204, 2.380) → sys=(...)
```
Cocokkan `raw_opti` dgn posisi fisik tag, lalu set `gt_marker_idx`. Marker
ter-oklusi (0,0,0) otomatis ditandai invalid (tak dipublish).

**Output topik:**
- `/gt/position` (PointStamped) — **posisi GT** dalam frame `world` (utama)
- `/gt/pose` (PoseStamped) — posisi sama; **orientasi = identitas** (marker tunggal tak punya orientasi)

**Cek data masuk:**
```bash
ros2 topic echo /gt/position
ros2 topic hz /gt/position    # harus ~120 Hz
```

**Demo overlay di RViz:**
```bash
rviz2
# Tambah PointStamped display untuk /gt/position (warna hijau)
# Tambah PoseStamped display untuk /state/position_compensated (warna merah)
# → GT dan estimasi overlay real-time
```

**⚠ Untuk metrik presisi paper, JANGAN pakai bridge ini.** Timestamp GT =
waktu **terima di laptop ROS** (`get_clock().now()`), BUKAN jam mocap → offset
latensi GT↔sistem tak terselesaikan. Untuk angka akurasi tetap pakai sync
2-langkah offline (`export_gt_synced.py`). Bridge ini untuk **visualisasi &
diagnostik live**, bukan validasi angka.

**Troubleshooting:**
- `executable not found` saat `ros2 run` → entry point belum ada di setup.py
  (lihat prasyarat build di atas) → tambah baris + rebuild ips_nodes.
- "Belum terima frame" → cek (1) **Local Interface BUKAN loopback** (→
  192.168.10.101), (2) Motive streaming ON, (3) IP server benar, (4) firewall
  1510:1511/udp, (5) subnet sama. Multicast over WiFi sering diblokir → pakai
  ethernet untuk PC Motive, atau Unicast di Motive + `use_multicast:=false`.
- Marker set kosong / tag tak terbaca → "Labeled Markers" OFF di Motive, ATAU
  tag hanya muncul di section *Labeled Markers* (bukan *Marker Set*) → node
  perlu ekstensi parser section LabeledMarkers.
- Posisi GT ~1000× meleset → satuan NatNet = mm → jalankan `-p input_units:=mm`.
- GT offset konstan besar → `gt_marker_idx` salah (bukan tag) atau konstanta
  transform beda sesi (§9.2 ⚠).

### 6.5 Demo pameran HTML (v2.3 — visualisasi untuk audiens umum)

Halaman web (`index.html`) yang menampilkan ruangan 3D + drone bergerak +
trail, untuk **pameran/demo ke awam**. Terhubung ke ROS via rosbridge
WebSocket. Smoothing **di display** (EMA), data sistem tidak disentuh.

**Setup laptop ROS (sekali):**
```bash
sudo apt install ros-jazzy-rosbridge-suite
sudo ufw allow 9090/tcp
hostname -I    # catat IP, mis. 192.168.10.100
```

**Jalankan rosbridge** (saat demo, terminal terpisah):
```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

**Di laptop display:**
1. Edit `index.html` → ubah `rosbridgeUrl: 'ws://192.168.10.100:9090'`
   (kalau browser di laptop ROS yang sama, biarkan `localhost`)
2. Double-click `index.html` → buka di browser (Chrome/Firefox)
3. Mode kiosk fullscreen:
   ```bash
   google-chrome --kiosk --app=file:///path/to/index.html
   ```

**Yang ditampilkan:**
- Ruangan 3D wireframe + 5 anchor berlabel (dari `anchors.yaml`)
- Drone cyan dengan halo + pulse halus + drop-line ke lantai
- Trail lintasan ~12 detik terakhir, memudar sesuai usia
- Readout posisi X/Y/Z (m) + update rate + jumlah sampel
- Caption awam ("gelombang radio UWB dari empat titik di sudut ruangan…")
- Auto-reconnect kalau koneksi terputus

**Smoothing display** — di `CONFIG.smoothingAlpha`:
- `0.3` = kurang lag, sedikit goyangan
- `0.18` (default) = halus, lag tak terasa mata (~250 ms)
- `0.08` = sangat halus, lag mulai terasa (~600 ms)

**Untuk pameran tanpa jaminan internet** — gunakan `demo_pameran_offline.tar.gz`
(v2.4). Tarball self-contained dengan Three.js + roslibjs ter-bundle di
folder `lib/`, font sistem (tidak Google Fonts). Cara pakai:

```bash
tar xzf demo_pameran_offline.tar.gz
# Edit index.html: ganti rosbridgeUrl ke IP laptop ROS
# Double-click index.html — siap tampil tanpa internet
```

Versi lama (`demo_pameran.tar.gz`) memuat library dari CDN — **gagal senyap**
di venue tanpa internet (halaman kosong, panel UI muncul tapi scene 3D tidak).
Selalu pakai versi offline untuk pameran.

**Tes seminggu sebelum demo** di lokasi venue — yang paling sering rewel:
firewall venue blokir port 9090, atau WiFi venue bermasalah. Bawa kabel
ethernet sebagai backup.

### 6.6 Konsumsi JSON dari `/state/json` (v2.4 — utama)

state_aggregator v3.3 mempublikasikan snapshot JSON ke topik ROS
`/state/json` (`std_msgs/String`). **Ini cara utama** untuk konsumsi data —
lebih bersih daripada stdout yang mudah tertimpa log node lain.

**Format JSON terstruktur:**
```json
{
  "tag_id": "DRONE_01",
  "timestamp": 1734892800.1234,
  "position": { "x": 2.513, "y": 2.595, "z": 1.75 },
  "velocity": {
    "linear":  { "x": 0.1, "y": 0.0, "z": 0.0 },
    "angular": { "x": 0.0, "y": 0.0, "z": 0.05 }
  },
  "acceleration": {
    "linear":  { "x": 0.0, "y": 0.0, "z": -9.81 },
    "angular": { "x": 0.0, "y": 0.0, "z": 0.0 }
  },
  "orientation": { "roll": -0.56, "pitch": 0.0, "yaw": 130.12 }
}
```

**Konsumsi dari terminal:**
```bash
ros2 topic echo /state/json
```

Setiap snapshot satu pesan JSON, terpisah enter antar pesan oleh
`topic echo` (tidak melebur seperti stdout).

**Konsumsi dari Python (downstream consumer):**
```python
import rclpy, json
from rclpy.node import Node
from std_msgs.msg import String

class Consumer(Node):
    def __init__(self):
        super().__init__('json_consumer')
        self.create_subscription(String, '/state/json',
                                  self.on_json, 10)
    def on_json(self, msg):
        data = json.loads(msg.data)
        print(f"pos x={data['position']['x']}, "
              f"yaw={data['orientation']['yaw']}°")
```

**Field semantik:**
- `tag_id` (string): identitas tag, dari parameter `tag_id` (default `"DRONE_01"`)
- `timestamp` (float): waktu PC saat snapshot, detik epoch
- `position` (dict atau null): posisi terkoreksi (`/state/position_compensated`)
- `velocity.linear` (dict atau null): kecepatan linear dari WoLF EKF
- `velocity.angular` (dict atau null): kecepatan sudut dari IMU (filtered)
- `acceleration.linear` (dict atau null): percepatan linear IMU
- `acceleration.angular` (dict atau null): percepatan sudut SG
- `orientation.{roll,pitch,yaw}` (dict atau null): Euler ZYX **dalam derajat**

**Field kosong = `null`**, BUKAN nol — supaya "belum ada data" tidak
tertukar dengan "nilai nol benar".

**Orientasi opsi A (koreksi BNO055):**
- `roll`  = -roll_BNO_mentah   (flip tanda untuk konvensi user)
- `pitch` =  pitch_BNO_mentah  (biarkan)
- `yaw`   = 360 - yaw_BNO_mentah  → range [0°, 360°)

Verifikasi: drone hadap +X sistem (referensi) → output JSON `yaw ≈ 130°`
(= 360 - 229.88), `roll ≈ -0.5°`, `pitch ≈ 0°`.

**⚠ `/state/orientation` (topik untuk RViz) TIDAK terkoreksi.** RViz akan
menampilkan orientasi dari quaternion BNO mentah, BERBEDA dari nilai
Euler di JSON. Itu disengaja — patch koreksi quaternion via Euler roundtrip
tidak stabil di ZYX (gimbal lock ambiguity). Untuk konsistensi RViz dengan
JSON, butuh pendekatan berbeda yang akan didiskusikan terpisah.

**Parameter relevan di system.yaml:**
```yaml
state_aggregator:
  ros__parameters:
    publish_topic: "/state/json"   # nama topik (bisa diubah)
    display_mode: "off"            # off/stream/refresh (default off di v3.3)
    pretty_json: true
    tag_id: "DRONE_01"
    enable_imu_fields: true
```

`display_mode: "off"` — tidak print ke stdout (topik adalah cara utama).
Set ke `"refresh"` kalau mau tampilan ANSI tetap di tempat (clear screen
per snapshot — hanya berfungsi kalau terminal terisolasi dari log ROS).

---

## 7. Recording CSV

### 7.1 Service calls

```bash
# Mulai
ros2 service call /recorder/control ips_msgs/srv/RecordControl \
    "{action: 'start', label: 'titik_A_diam'}"

# Status
ros2 service call /recorder/control ips_msgs/srv/RecordControl "{action: 'status'}"

# Stop
ros2 service call /recorder/control ips_msgs/srv/RecordControl "{action: 'stop'}"
```

### 7.2 Output per session (14 layers)

```
~/ips_logs/20260515_143022_titik_A_diam/
├── position.csv                 KF raw output (~25 Hz)
├── position_compensated.csv     bias-corrected (UTAMA untuk analisis)
├── position_chan.csv             raw Chan (~25 Hz)
├── corrected_toa.csv            per-anchor ToA + delta_k (~100 Hz)
├── sync_status.csv              LI-KF diagnostik (1 Hz)
├── master_anchor.csv            raw MASTER_CLOCK packets
├── slave_anchor.csv             raw SA MASTER+TAG packets
│   ── IMU (Tingkat 2) ──
├── imu_raw.csv                  IMU mentah + jangkar sinkron (ros_time/tag_ms/blink)
├── orientation.csv              quaternion (fusi BNO055)
│   ── Derived state (Tingkat 2.1) ──
├── translation_velocity.csv     kecepatan DIFF (turunan posisi, differentiator)
├── wolf_velocity.csv            kecepatan EKF (state WoLF — sumber output v2.1)
├── angular_velocity.csv         gyro terfilter (imu_processor)
├── translation_acceleration.csv accel linear terfilter
└── angular_acceleration.csv     percepatan sudut SG (perlu enable_angular)
```

**Checklist agar semua terisi:** `wolf_velocity` perlu `algorithm:=wolf`;
`angular_acceleration` perlu `differentiator: enable_angular: true`. `imu_raw`
punya `blink` = jangkar alignment IMU↔UWB. Rencana sinkron: rekam semua jangkar,
selaraskan offline — IMU↔UWB via blink, sistem↔OptiTrack via cross-correlation.

**Dua kecepatan terekam (sengaja, untuk perbandingan):** `wolf_velocity` (EKF
state, halus & akurat) vs `translation_velocity` (diff, noisy + spike). Lihat
§10c untuk analisis.

### 7.3 Quick RMSE dari CSV

```python
import pandas as pd, numpy as np

df = pd.read_csv('position_compensated.csv')
gt = np.array([1.5, 1.0, 1.15])

t0 = df['pc_time_s'].min()
stable = df[(df['pc_time_s'] - t0) > 30]  # buang 30s pertama

pos = stable[['x','y','z']].values
err = pos - gt
rmse = np.sqrt(np.mean(np.sum(err**2, axis=1)))
std = np.std(pos, axis=0)
print(f"RMSE 3D = {rmse*1000:.1f} mm")
print(f"Std: x={std[0]*1000:.1f} y={std[1]*1000:.1f} z={std[2]*1000:.1f} mm")
```

---

## 8. Diagnostik

### 8.1 Cek sistem

```bash
ros2 node list
ros2 topic list
ros2 topic hz /uwb/anchor_reports   # ~233 Hz
ros2 topic hz /uwb/corrected_toa    # ~100 Hz (25 Hz × 4 SA)
ros2 topic hz /state/position       # ~25 Hz
```

### 8.2 Cek clock sync

```bash
ros2 topic echo /uwb/sync_status
```

- `kf_converged: [True,True,True,True]` → harus True setelah beberapa detik
- `sync_count` → harus naik ~32/s per anchor (CCP rate)
- `reset_count > 0` → MC restart, cek power supply

### 8.3 Cek Layer 1 stats

Log otomatis tiap 5 detik di terminal position_solver:
```
[Layer 1] clip stats: SA3: 23/1250 (1.8%) res_mean=+0.4cm  SA4: 18/1248 (1.4%) res_mean=-0.1cm  SA5: 35/1247 (2.8%) res_mean=+0.9cm
```

- Clip rate sehat: 1-5% per anchor
- Kalau >10%: naikkan `layer1_huber_k` ke 3.0-3.5
- Kalau 0%: filter tidak bekerja (cek apakah `layer1_enabled: true`)
- `res_mean` mendekati 0 → tidak ada bias sistematis per anchor

### 8.4 Cek Layer 2 stats

```
[Layer 2] innovation gate: rejected 12/1250 (0.96%)  last_NIS=4.20  threshold=11.35
```

- Reject rate sehat: 0.5-3%
- Kalau >5%: naikkan `layer2_gate_threshold` ke 16.266 (p=0.999) atau cek R underestimate
- Kalau 0%: threshold terlalu longgar (turunkan ke 7.815 untuk p=0.95)

### 8.5 Cek bias compensator state

```
[bias_compensator] [OPERATIONAL] bias=(+10.37, +2.06, +49.01) cm
```

State: IDLE (belum kalibrasi), CALIBRATING (sedang mengumpulkan sampel), OPERATIONAL (bias aktif).

---

## 9. Konfigurasi YAML

Edit di `~/ips_jazzy_ws/src/ips_bringup/config/` lalu rebuild:
```bash
colcon build --symlink-install --packages-select ips_bringup
source install/setup.bash
```

### 9.1 `system.yaml`

```yaml
udp_gateway:
  ros__parameters:
    udp_port: 5555
    udp_bind_address: "0.0.0.0"
    socket_buffer_bytes: 1048576
    print_stats_every_s: 5.0

clock_sync:
  ros__parameters:
    status_period_s: 1.0
    kf_enabled: true

position_solver:
  ros__parameters:
    frame_id: "world"
    use_kalman_filter: true
    blink_buffer_size: 64
    blink_publish_timeout_s: 0.5
    rx_anchors_ordered: [2, 3, 4, 5]

    # Antenna bias dari self_calibrate.py (arena baru, kerangka tangan-kanan)
    bias_anchor_ids: [2, 3, 4, 5]
    bias_values_ns: [0.0, 1.644, -8.369, -8.663]   # ⚠ SA3/SA4 tertukar vs WoLF (§9.1 catatan)

    # KF tuning
    sigma_a: 0.01
    sigma_t_tdoa: 3.5e-10
    pos_nominal: [2.513, 2.5955, 1.75]   # tengah arena (kerangka baru)
    room_dim: [5.026, 5.191, 3.5]
    dt_nominal_s: 0.20

    # KF type
    kf_type: "student_t"
    stf_eta0: 5.0
    stf_alpha_max: 3.0

    # DZS (MATI untuk tag bergerak — gunakan Layer 1+2)
    dzs_enabled: false
    g1_enabled: true
    g1_margin: 1.5

    # Layer 1 — Predictive TDoA Gate
    layer1_enabled: true
    layer1_sigma_tdoa_m: 0.10    # baseline TDoA noise (m)
    layer1_huber_k: 2.5          # threshold dalam sigma
    layer1_warmup: 20            # blink awal di-skip
    layer1_log_every_s: 5.0

    # Layer 2 — Gated Student-t innovation gate
    # gate_threshold = chi-squared(dim=3) inverse-CDF
    #   p=0.95  -> 7.815
    #   p=0.99  -> 11.345  (default)
    #   p=0.999 -> 16.266
    layer2_enabled: true
    layer2_gate_threshold: 11.345
    layer2_log_every_s: 5.0

wolf_position:
  ros__parameters:
    # ===== I/O (mirror position_solver) =====
    frame_id: "world"
    blink_buffer_size: 64
    blink_publish_timeout_s: 0.5
    rx_anchors_ordered: [2, 3, 4, 5]
    pos_nominal: [2.513, 2.5955, 1.75]   # tengah arena (kerangka baru, init seed)
    room_dim: [5.026, 5.191, 3.5]
    dt_nominal_s: 0.20

    # Antenna bias — INI nilai live/benar (estimator produk = WoLF)
    bias_anchor_ids: [2, 3, 4, 5]
    bias_values_ns: [0.0, -8.366, 1.641, -8.663]
    # ⚠ CATATAN: SA3/SA4 tertukar urutannya vs position_solver
    #   wolf  : SA3=-8.366, SA4=1.641   ← BENAR/live (corr 0.99 vs GT membuktikan;
    #                                       swap 10ns = 3m error, pasti meleset bila salah)
    #   solver: SA3=1.644,  SA4=-8.369  ← stale, position_solver tak dipakai produk
    # Antena delay = properti fisik hardware, seharusnya identik antar-node;
    # rapikan position_solver agar cocok (tak mendesak — produk hanya pakai WoLF).

    # ===== WoLF-EKF-CA tuning (baseline terbaik, hasil eksperimen vs OptiTrack) =====
    wolf_sigma_tdoa: 0.08         # measurement noise R = σ²·I (meter)
    wolf_sigma_a_min: 0.005       # floor process noise (kecil = hover tenang)
    wolf_sigma_a_gain: 1.0        # σ_a_eff = σ_a_min + gain·‖v‖ (responsif saat gerak)
    wolf_sigma_a_max: 3.0         # CAP σ_a_eff (anti-ledakan manuver, v0.3.3); 0=nonaktif
    wolf_c: 0.3                   # IMQ soft threshold (m); kecil = downweight outlier agresif
    wolf_pos_std0: 1.5            # P0 posisi (besar → konvergen dari init kasar)
    wolf_vel_std0: 0.1
    wolf_acc_std0: 0.1
    wolf_log_every_s: 5.0
    # CATATAN: σ_a_min=0.001 BERBAHAYA (meledak 1/2 sesi); 0.005 = titik aman terendah.

    # ===== ZUPT (Zero-Velocity Hold) post-filter — gate terkoreksi v0.3.1 =====
    zupt_enabled: true
    zupt_v_enter: 0.30            # m/s; ambang masuk HOLD (dikoreksi dari 0.08 —
                                  #   jitter menggelembungkan ‖v̂‖ EKF ke ~0.27 saat diam)
    zupt_v_exit: 0.45            # m/s; histeresis keluar (> v_enter)
    zupt_spread_enter: 0.08      # m; RMS std jendela √(σx²+σy²+σz²)
    zupt_window_s: 1.0
    zupt_n_enter: 10             # sampel berturut sebelum HOLD (~0.4s @25Hz)
    zupt_hold_mode: "ema"        # "ema" (cutoff rendah) | "median"
    zupt_hold_cutoff_hz: 0.2     # turunkan ke 0.1 untuk smoothing lebih keras
    zupt_ramp_s: 0.3             # blend masuk/keluar (anti-loncatan)
    zupt_log_every_s: 5.0

bias_compensator:
  ros__parameters:
    bias_yaml_path: ""             # overridden oleh launch arg bias_yaml
    auto_load: true
    auto_save: true
    n_samples_default: 300
    skip_warmup_default: 30
    robust: true                   # median+MAD (true) atau mean+std (false)
    log_status_every_s: 5.0
    calibration_timeout_s: 60.0

differentiator:
  ros__parameters:
    min_dt_s: 0.0001
    max_dt_s: 1.0
    enable_angular: true          # true → /state/angular_acceleration
    deriv_method: "savgol"        # "savgol" (halus) | "backward"
    sg_window: 15                 # jendela Savitzky-Golay
    sg_poly: 3                    # orde polinomial
    ang_accel_clip: 0.0           # clip |percepatan sudut|; 0 = tanpa

imu_processor:
  ros__parameters:
    frame_id: "tag_imu"
    # gyro: raw → [LPF] → [Kalman] (default OFF = passthrough)
    gyro_apply_lpf: false
    gyro_lpf_alpha: 0.05
    gyro_apply_kalman: false
    gyro_kf_q: 0.0005
    gyro_kf_r: 0.05
    # accel: raw → [LPF] → [Kalman]
    accel_apply_lpf: false
    accel_lpf_alpha: 0.05
    accel_apply_kalman: false
    accel_kf_q: 0.0005
    accel_kf_r: 0.05

state_aggregator:
  ros__parameters:
    # ─── v2.4 (state_aggregator v3.3) ─────────────────────────────────
    publish_topic: "/state/json"   # PUBLIKASI ke topik (utama untuk konsumsi)
    display_mode: "off"            # off/stream/refresh (default off di v3.3)
    pretty_json: true              # JSON indent 2 untuk display refresh/stream
    tag_id: "DRONE_01"             # diterbitkan di setiap snapshot
    enable_imu_fields: true        # default TRUE di v3.3 (format butuh IMU)
    # ─── sama dengan sebelumnya ────────────────────────────────────────
    output_rate_hz: 10.0
    log_file_path: ""              # opsional, JSON satu-baris per snapshot
    round_decimals: 4              # presisi angka di output
    # CATATAN: koreksi orientasi opsi A (flip Euler manual) HARDCODED di v3.3.
    #   roll  = -roll_BNO
    #   pitch =  pitch_BNO
    #   yaw   = 360 - yaw_BNO  (range [0, 360))
    # Quaternion BNO mentah TIDAK diubah di /state/orientation — RViz akan
    # menampilkan orientasi yang BERBEDA dari nilai JSON. Lihat §6.6.
    # CATATAN: kecepatan output diambil dari /state/wolf_velocity (EKF),
    # bukan differentiator. Hanya terisi saat algorithm:=wolf.

recorder:
  ros__parameters:
    base_dir: "~/ips_logs"
    auto_start_raw: false
```

### 9.2 `anchors.yaml`

Arena baru, **kerangka tangan-kanan** (right-handed). Posisi anchor dalam
koordinat sistem (meter), MA/SA2 = origin world. Konfig lama (left-handed,
sebelum swap x↔y) USANG.

```yaml
anchors:
  - id: 1
    name: MC          # Master Clock (TX-only CCP broadcaster)
    x: 0.000
    y: 2.593
    z: 2.25
  - id: 2
    name: MA          # = SA2, referensi Chan/WoLF (a_ref), origin world
    x: 0.000
    y: 0.000
    z: 0.38
  - id: 3
    name: SA3
    x: 5.026
    y: 0.000
    z: 3.00
  - id: 4
    name: SA4
    x: 0.000
    y: 5.191
    z: 3.00
  - id: 5
    name: SA5
    x: 5.026
    y: 5.191
    z: 1.00           # anchor terendah selain SA2 — terkait ledakan zona z<0.6 (§12)
```

`ROOM = (5.026, 5.191, 3.5)`.

**Sejarah kerangka (closed):** anchors.yaml dulu left-handed; diperbaiki dengan
**tukar x↔y → tangan-kanan**. User sudah membetulkan anchors.yaml DAN merekam
ulang data di kerangka baru. Toolkit analisis lama punya flag `SWAP_XY` — sudah
**dihapus** di versi baru (kerangka baru di-hardcode langsung).

**Pemetaan OptiTrack→sistem (kerangka baru):**
```
x_sys = x_opti − X_OPTI_SA2     (X_OPTI_SA2 = 0.646 m)
y_sys = Z_OPTI_SA2 − z_opti     (Z_OPTI_SA2 = 3.425 m)
z_sys = y_opti − Z_FLOOR        (Z_FLOOR    = 0.0;  OptiTrack Y = atas/tinggi)
```

**⚠ VERIFIKASI — konstanta transform tidak konsisten antar file referensi:**
`export_gt_synced.py` memakai `X_OPTI_SA2=0.646, Z_OPTI_SA2=3.425`, sedangkan
`ips_analysis_common.py` memakai `X_OPTI_SA2=0.637, Z_OPTI_SA2=3.426` (selisih
~9 mm di X). Kemungkinan dari sesi kalibrasi berbeda. `optitrack_bridge`
memakai 0.646/3.425 sebagai default; konfirmasi nilai kalibrasi arena terbaru
dan samakan di semua tempat (param `x_opti_sa2`/`z_opti_sa2` + kedua skrip).
GT = **marker tag**; indeks marker (`gt_marker_idx`) juga belum konsisten
(0/1/2 antar file) — verifikasi via log `raw_opti` node (§6.4).

**Penting:** ukur posisi fisik semua anchor dengan presisi. Ketidakakuratan 5 cm
di posisi anchor bisa menyebabkan bias puluhan cm di output. Re-ukur posisi
anchor sudah ditetapkan **tidak** dilakukan; koreksi skala ditangani bias affine.

---

## 10. Algoritma — Pipeline Per Blink

```
corrected_toa × 4 anchor terkumpul (satu blink lengkap)
         │
         ▼
  KF.predict_only(dt) → x_pred, P_pos    [TIDAK update KF state]
         │
         ▼
  ┌─── Layer 1 — Predictive TDoA Gate ───────────────────────┐
  │ Untuk tiap anchor non-ref (SA3, SA4, SA5):               │
  │   tdoa_pred = ||x_pred − anchor_i|| − ||x_pred − ref||  │
  │   residual = tdoa_measured − tdoa_pred                   │
  │   σ_pred = sqrt(J · P_pos · Jᵀ + σ_tdoa²)              │
  │   IF |residual| > k_huber × σ_pred:                     │
  │       tdoa[i] ← CLIP ke threshold (Huber)               │
  └──────────────────────────────────────────────────────────┘
         ���
         ▼
    Chan closed-form solver → posisi 3D
         │
         ▼
    publish /state/position_chan
         │
         ▼
  ┌─── Layer 2 — Innovation Gate ────────────────────────────┐
  │   innov = z_chan − H · x_pred                            │
  │   NIS = innovᵀ · S⁻¹ · innov                           │
  │   IF NIS > γ (chi-squared threshold):                   │
  │       SKIP update → x = x_pred  (predict only)         │
  │   ELSE:                                                  │
  │       Student-t update dengan α-scaling                  │
  │       α = (η₀ + NIS) / (η₀ + 3)                        │
  │       P = α · P_kf (inflate saat outlier moderat)       │
  └──────────────────────────────────────────────────────────┘
         │
         ▼
    publish /state/position
         │
         ▼
  bias_compensator: pos_compensated = pos − bias_calibrated
         │
         ▼
    publish /state/position_compensated
```

### Layer 1 vs Layer 2 — saling melengkapi

| Skenario | Yang menangkap |
|---|---|
| 1 TDoA wildly off (multipath, EMI) | Layer 1 Huber-clip |
| 2-3 TDoA off di arah yang sama | Layer 1 clip sebagian, Layer 2 reject sisanya |
| Posisi Chan aneh tapi tiap TDoA terlihat normal | Layer 2 gate |
| Tag bergerak cepat (percepatan tinggi) | Keduanya adaptif via KF velocity state |

---

## 10b. Algoritma — WoLF-EKF-CA (algorithm:=wolf)

WoLF (Weighted Observation Likelihood Filter, Duran-Martin et al. 2024)
menggantikan L1+Chan+L2+KF dengan satu EKF robust. Chan tidak dipakai per-blink
(hanya untuk diagnostic `/state/position_chan`).

```
corrected_toa × 4 anchor terkumpul (satu blink)
         │  − antenna bias (bias_values_ns WoLF)
         ▼
  TDoA_i = c·(toa_i − toa_ref),  i ∈ {SA3,SA4,SA5}
         │
         ▼
  ┌─── PREDICT (constant-acceleration 9-state) ──────────────┐
  │  x = [p(3), v(3), a(3)]                                  │
  │  x_pred = F(dt)·x ;  P_pred = F·P·Fᵀ + Q(dt)            │
  │  σ_a_eff = σ_a_min + gain·‖v‖   (ADAPTIF)               │
  │    → diam: σ_a kecil → P sempit → presisi               │
  │    → gerak: σ_a besar → responsif                       │
  └──────────────────────────────────────────────────────────┘
         │
         ▼
  ┌─── UPDATE dengan IMQ weighting (anti-outlier) ───────────┐
  │  h_i(p) = ‖p−a_i‖ − ‖p−a_ref‖   (TDoA langsung)        │
  │  innov = TDoA − h(x_pred)                                │
  │  w = (1 + ‖innov‖²/c²)^(−1/2)   (Eq.17 paper)          │
  │  R_eff = R / w²                  (Prop 3.1)             │
  │    → innov kecil: w≈1, measurement dipercaya            │
  │    → innov besar: w→0, otomatis di-downweight           │
  │  KF update standar dgn R_eff (Joseph form)              │
  └──────────────────────────────────────────────────────────┘
         │
         ▼
  ┌─── ZUPT post-filter (saat hover) ────────────────────────┐
  │  Gate: ‖v̂‖ state EKF < v_enter DAN spread jendela <    │
  │        spread_enter, bertahan n_enter sampel (histeresis)│
  │  Saat HOLD: output = EMA cutoff-rendah (bukan freeze)   │
  │  Aman ledakan: saat over-ekstrapolasi ‖v̂‖ tinggi →     │
  │                ZUPT tidak aktif                          │
  └──────────────────────────────────────────────────────────┘
         │
         ▼
  ┌─── Anti-divergensi (v0.3.3) ─────────────────────────────┐
  │  predict: σ_a_eff = min(σ_a_min + gain·‖v‖, σ_a_max)    │
  │           CLAMP cegah runaway kuadratik saat manuver     │
  │  guard: state tak finite / posisi absurd → re-init Chan  │
  │         counter diverged=N (kecil=sehat)                 │
  └──────────────────────────────────────────────────────────┘
         │
         ▼
    publish /state/position → bias_compensator → compensated
```

**Kelebihan teoretis (Thm 3.2):** PIF (posterior influence function) BOUNDED —
outlier sekuat apa pun tidak bisa menggeser posterior tanpa batas. Kompleksitas
O(m³) sama dengan KF biasa.

**Karakter empiris** (vs Chan, dari komparasi head-to-head):
- **Dinamis: WoLF unggul** (model CA + IMQ cocok untuk manuver)
- **Statis: Chan sedikit unggul** tanpa ZUPT; **dengan ZUPT, WoLF kompetitif**
  (jitter hover ~24 mm)

### Chan vs WoLF — kapan pakai yang mana

| Kondisi | Rekomendasi |
|---|---|
| Drone banyak manuver / dinamis | WoLF (`algorithm:=wolf`) |
| Hover presisi tinggi statis | Chan + L1/L2, atau WoLF + ZUPT |
| Uji diferensial / baseline | Chan (`algorithm:=chan`, default) |
| Diagnostik ledakan SA5 (§12) | bandingkan keduanya di zona bermasalah |

---

## 10c. IMU & Dua Sumber Kecepatan

### Asal data IMU (`/imu/raw`)

BNO055 di tag mengirim 17-field @~25 Hz (sebelumnya ~20 Hz); udp_gateway parse → `/imu/raw`. Penting
membedakan asal tiap besaran:

| Field | Asal | Olahan |
|---|---|---|
| gyro (angular_velocity) | sensor langsung | tidak (mentah) |
| quaternion / Euler | fusi 9-DOF | di chip BNO055 |
| linear_acceleration | gravity-removal | di chip BNO055 (BODY frame) |

Semua masuk ROS apa adanya (tanpa olahan ROS-side). **Tidak ada** velocity
translasi & angular-accel di `/imu/raw` — itu besaran turunan, dihitung node.

### Pipeline filter (imu_processor + differentiator, C++)

```
gyro  : raw → [LPF EMA] → [Kalman 1D] → /state/angular_velocity
accel : raw → [LPF EMA] → [Kalman 1D] → /state/translation_acceleration
orient: quaternion native → /state/orientation (passthrough)
posisi → [Savitzky-Golay] → /state/translation_velocity (diff)
gyro   → [Savitzky-Golay] → /state/angular_acceleration (enable_angular)
```

Filter di imu_processor **default OFF** (passthrough). SG pakai dt aktual (benar
di rate berapa pun), diverifikasi identik numpy polyfit. Notch 12 Hz TIDAK
disertakan (spesifik frekuensi propeller + tuned 50 Hz — verifikasi spektrum
dulu). Integrasi accel→velocity TIDAK dipakai (drift; itu fusi Tingkat 3).

### Dua kecepatan: wolf (EKF) vs differentiator — wolf yang dipakai

Sistem punya DUA estimasi kecepatan dari sumber berbeda:

- **`wolf_velocity`** = komponen kecepatan dari state EKF WoLF. Diestimasi
  langsung filter Kalman dengan model dinamika → halus alami, kebal spike.
- **`translation_velocity`** = turunan numerik posisi (differentiator). Diferensiasi
  memperkuat noise → jitter & spike ekstrem.

**Analisis vs OptiTrack (sesi tes2-normal, 86s):**

| Metrik | wolf (EKF) | differentiator | Rasio |
|---|---|---|---|
| RMSE vs GT (3D) | **0.36 m/s** | 2.29 m/s | **6.3× lebih buruk** |
| Spike maks ‖v‖ | 2.35 m/s | 88.9 m/s | 38× |
| Jitter antar-sampel (median) | 0.069 | 0.144 m/s | 2× |

**Keputusan (v2.1):** state_aggregator ambil kecepatan output dari
`/state/wolf_velocity`, bukan differentiator. Diferensiasi numerik memperkuat
noise secara fundamental (bukan tuning); EKF estimasi kecepatan sebagai state
dengan peredaman dinamika. Differentiator tetap jalan untuk percepatan sudut
(gyro) & sebagai data perbandingan di recorder. Catatan: RMSE absolut 0.36 m/s
sebagian dari noise GT (GT = turunan OptiTrack); yang valid adalah rasio 6.3×.

### Peluang fusi IMU (Tingkat 3, future)

Akselerasi IMU bisa memperbaiki transien wolf — **bukan** via integrasi langsung
(drift tanpa batas), tapi sebagai **input prediksi EKF** (tightly-coupled). EKF
mengoreksi drift IMU lewat posisi UWB; IMU mempertajam prediksi saat manuver.
Syarat: (1) rotasi accel body→world via quaternion, (2) kualitas IMU terbukti
(waspada offset GYRO_X=16334), (3) alignment temporal via blink. Layak hanya
setelah validasi kualitas IMU mentah.

---

## 10d. Orientasi BNO055 — Koreksi Opsi A (v2.4)

### Setup hardware

BNO055 dipasang dengan PCB **berdiri vertikal** (bukan datar). Remap fisik
di firmware:

```cpp
const uint8_t REMAP_CONFIG = 0x18;   // fusion Z<-chipY, Y<-chipZ, X<-chipX
const uint8_t REMAP_SIGN   = 0x01;   // Z negative (chip +Y points down)
```

Setelah remap, kerangka fusion BNO secara nominal:
- X_fusion = X_chip = kanan drone
- Y_fusion = Z_chip = depan drone
- Z_fusion = −Y_chip = atas drone

### Pengamatan empiris (sumber masalah)

Dengan 7 quaternion empiris (drone diam + 6 rotasi tunggal terisolasi),
ditemukan **sumbu rotasi BNO TIDAK cocok** dengan body drone:

| Rotasi fisik | Sumbu yang diharapkan | Sumbu BNO aktual |
|---|---|---|
| Roll kanan | +X (depan) | +Y ✗ |
| Pitch hidung naik | +Y (kiri) | +X ✗ |
| Yaw CCW (dari atas) | +Z (atas) | +Z ✓ |

Tambahan: tanda Euler roll/pitch/yaw BNO **terbalik** dari konvensi
right-handed standar untuk drone — kemungkinan dari kombinasi remap + konvensi
internal BNO055.

### Koreksi yang dipakai: opsi A user (hardcoded di state_aggregator v3.3)

Diterapkan **hanya di output JSON aggregator**:

```
roll_out  = -roll_BNO_mentah
pitch_out =  pitch_BNO_mentah
yaw_out   = 360 - yaw_BNO_mentah   (range [0°, 360°))
```

**Verifikasi referensi** (drone hadap +X sistem, datar):
- BNO lapor: roll=0.56°, pitch=0°, yaw=229.88°
- Output JSON: roll=-0.56°, pitch=0°, yaw=130.12°  ✓

**Verifikasi delta** (dari 7 sampel empiris):
- Drone miring kanan → output roll **negatif** ✓
- Drone hidung naik → output pitch **positif** (sign cocok ekspektasi) ✓
- Drone putar CCW → output yaw **bertambah** ✓

### Apa yang BUKAN dilakukan (caveat penting)

1. **`/state/orientation` (topik untuk RViz) TIDAK dipatch.** Quaternion
   BNO mentah mengalir apa adanya ke RViz. **RViz menampilkan orientasi
   yang BERBEDA dari nilai Euler di JSON.** Ini disengaja — patch koreksi
   quaternion lewat Euler roundtrip tidak unik di ZYX (gimbal lock
   ambiguity), dan pendekatan mirror axis quaternion (flip_qx/qy/qz/qw)
   tidak menghasilkan transformasi opsi A yang tepat.

2. **Tidak ada koreksi world-frame.** Output Euler "terasa" seperti body
   frame karena tanpa referensi arena tetap + drift magnetometer antar
   sesi, koreksi world frame yang konsisten tidak feasible.

3. **`/imu/raw` tetap utuh** — untuk debug, kalibrasi, atau analisis offline,
   gunakan quaternion mentah dari topik ini.

### Sejarah iterasi (untuk audit & hindari ulang)

Sesi orientasi melalui banyak iterasi:
- v3.0: format JSON terstruktur (tanpa koreksi orientasi)
- v3.1: koreksi `flip_qy` (mirror quaternion) — verifikasi 6/6 untuk sumbu
  rotasi body, tapi yaw absolut salah karena tidak ada referensi tetap
- v3.2: tambah display_mode 'refresh' (ANSI clear screen)
- v3.3 (sekarang): rollback koreksi quaternion, pakai flip Euler manual
  (opsi A user). Tambah publish ke topik `/state/json`.

**Pelajaran**: Euler ↔ Quaternion ↔ Euler tidak unik di ZYX. Untuk koreksi
sederhana (flip tanda komponen Euler), implementasi langsung di Euler
lebih bersih daripada mencari transformasi quaternion ekivalen.

---

## 11. Hasil Performansi

> **Metode (penting untuk validitas):** semua angka de-biased di bawah memakai
> **nearest-neighbor tanpa interpolasi GT** (sampel di gap tracking DIBUANG, tak
> di-interpolasi) + sinkronisasi 2-langkah (coarse xcorr → refine min-RMSE 3D).
> Interpolasi GT melintasi gap dan sync 1-sumbu terbukti **menipu** (lihat
> §catatan metodologi / handover) — angka lama yang memakainya tidak valid.

### 11.1 Status akurasi WoLF vs OptiTrack (semua sesi, metode bersih)

| Sesi | Gerak | RMSE 3D de-biased | x/y/z (deb.) mm | Catatan |
|---|---|---|---|---|
| 06-06 s1 | x,y | ~139 mm | 58/85/— | kerangka baru tervalidasi |
| 06-06 s2 | y,z | 139 mm | —/86/116 | z tervalidasi |
| 06-09 s1 | x,y (z tahan) | **101 mm** | 48/68/57 | z corr rendah (z konstan) |
| 06-09 s2 | y,z | **165 mm** | 61/68/137 | z bergerak → kelemahan z muncul |

**Pola konsisten:**
- **x, y SEHAT** (~50–70 mm de-biased).
- **z = sumbu TERLEMAH** (~120–140 mm saat bergerak). Sebab: VDOP buruk (anchor
  tak sebidang tapi separasi vertikal efektif kecil) + cakupan-z kalibrasi tipis
  (slope z residual 0.88 → skala z kurang dalam).
- **Residual 98–99% SISTEMATIK** (bias spasial fisik), komponen acak hanya
  8–11 mm → WoLF+ZUPT sudah optimal meredam noise; tuning lebih lanjut tidak
  akan memperbaiki akurasi.

### 11.2 Jitter hover (kontribusi ZUPT)

> Diukur vs OptiTrack. ZUPT menekan jitter hover ~38 mm → <15–24 mm.

| Metrik (hover) | WoLF tanpa ZUPT | WoLF + ZUPT |
|---|---|---|
| Jitter hover 3D | 38–51 mm | **24 mm (−53%)** |
| Jitter x/y/z | 28/30/31 mm | 12/17/13 mm |
| HF jitter (tanda EMA aktif) | 10.2 mm | **3.4 mm** |
| Loncatan antar-sampel max | 654 mm | **150 mm** |

### 11.2b Batas affine global & arah perbaikan (temuan kunci)

**Affine global sudah MENTOK ~130–170 mm.** Bukti: bias **bergantung-posisi** —
offset-x berbeda antar-sesi di hari sama (S1 −167 mm vs S2 −71 mm), skala-x 0.92
vs 0.96. Tidak ada satu matriks 3×3 + offset yang memuaskan semua wilayah
ruangan. Konsekuensi untuk arah pengembangan:

- **Bukan** tuning WoLF/ZUPT (domain acak ~10 mm, bukan bottleneck).
- **Adalah** (urut ROI): (a) kalibrasi z-coverage penuh + refit affine (z
  terburuk; perkiraan z 137→60–80 mm); (b) **peta bias spasial** (GP/RBFN —
  preseden LPS RBFN: ~15 cm statis, ~5 cm dinamis); (c) kalibrasi delay antena
  per-anchor; (d) analisis GDOP; (e) fusi sensor ketinggian untuk z.

bias_v5 interim (di-fit dari 06-09 d1+d2, NN tanpa interpolasi) menurunkan train
192→133 mm, CV antar-sesi 172 mm, per-sumbu [63,65,98] vs v4 [126,70,121] — tapi
suku silang M sebagian menyerap korelasi trajektori (bukan murni fisik), jadi
**interim**; fit definitif butuh kalibrasi hover sumbu-independen + peta GP.

### 11.3 Spesifikasi sistem

| Parameter | Nilai |
|---|---|
| KF/WoLF update rate | ~25 Hz (update v2.5 — selaras blink 25 Hz) |
| CCP period | ~30–150 ms |
| Blink rate | ~25 Hz (stabil, tanpa loss — update v2.5) |
| CPU load C++ nodes | <5% (i7-1355U, estimasi konservatif — ukur dgn `top`/`pidstat`) |
| **Latensi pemrosesan** (ingest→output) | **diukur via `latency_monitor` (§6.3)** — orde satuan-belasan ms |
| **Drop rate** (blink hilang) | **~0% (update v2.5 — blink 25 Hz tanpa loss)**; histori: ~16% — `1 − match_rate` |
| Clock sync (LI-KF) | terverifikasi setia paper Zhang 2024 |
| Aggregator snapshot rate | 10 Hz (jeda rata-rata ~50 ms — dilaporkan analitik, terpisah dari latensi) |

> Untuk paper, laporkan **tiga komponen latensi terpisah** (jujur):
> (a) pemrosesan ingest→output (diukur `latency_monitor`), (b) jeda snapshot
> aggregator ~50 ms (analitik), (c) jaringan tag→laptop (disclaimer — tak
> terukur karena jam tag tak sinkron, orde ms WiFi).

---

## 12. Troubleshooting

| Gejala | Penyebab | Solusi |
|---|---|---|
| Posisi compensated meleset ratusan mm | bias.yaml tidak valid (setelah add L1/L2) | Re-kalibrasi: service call /bias_compensator/calibrate |
| `/state/position` tidak muncul | anchor_config belum load | `ros2 topic echo /uwb/anchor_config --once` |
| Layer 1 clip rate >10% | `huber_k` terlalu kecil atau `sigma_tdoa` terlalu kecil | Naikkan `layer1_huber_k: 3.0` atau `layer1_sigma_tdoa_m: 0.15` |
| Layer 2 reject rate >5% | R underestimate atau threshold terlalu ketat | Naikkan `layer2_gate_threshold: 16.266` |
| Posisi stuck saat tag bergerak | DZS reject motion | `dzs_enabled: false` (gunakan L1+L2) |
| `kf_updates=0` di sync_status | `_run_step1` missing return True (Python) | Pakai C++ clock_sync_node |
| Reset_count naik terus (>1/min) | MA firmware/WiFi instability | Cek serial MA, cek jarak ke router |
| Drift posisi >30mm dalam 10 menit | Thermal warmup belum selesai | Tunggu warmup ≥10 menit |
| `colcon build` error virtual destructor | position_kf.hpp missing `virtual ~PositionKF()` | Pastikan pakai file terbaru dari layer2_patch |
| `rmw_qos_profile deprecated` warning | API lama di create_service | Ganti ke `rclcpp::ServicesQoS()` |
| **WoLF: posisi `null` terus** | anchor_config belum diterima / clock_sync error | Cek `/uwb/anchor_config`; pastикan bukan bug init lama (v0.2.0 sudah fix) |
| **WoLF: ledakan di zona x>1.6,y>1.6,z<0.6** | ground-bounce multipath dekat SA5 (z=1.0) — open problem | Hindari zona (flight envelope z≥0.8); uji diferensial vs Chan; lihat catatan §12b |
| **WoLF: jitter hover tidak turun (ZUPT diam)** | gate `v_enter` terlalu ketat / HOLD tak trigger | Pakai gate terkoreksi (v_enter=0.30); cek log `ZUPT=HOLD` muncul; HF jitter <3mm = aktif |
| **WoLF: blow-up saat drone nyala SETELAH ROS** | first-blink shock dari clock_sync warming-up | Nyalakan drone DULU, baru launch ROS |
| **WoLF: estimasi lag saat manuver cepat** | σ_a_gain terlalu kecil | Naikkan `wolf_sigma_a_gain` 1.0→2.0 |
| **WoLF: ledakan JUTAAN meter saat manuver (arena besar)** | σ_a runaway — umpan balik kuadratik tanpa batas (TERATASI v0.3.3) | Pastikan `wolf_sigma_a_max: 3.0` ada; cek log `diverged=N` (kecil=sehat); turunkan ke 2.0 kalau masih meledak |
| **v2.3 — latency_monitor: match=0%** | `tag_seq` tak ada di pesan posisi (versi node lama) | Pastikan WoLF + bias_compensator versi terbaru; lihat §6.3 |
| **v2.3 — latency_monitor: TOTAL negatif** | jam laptop melompat (NTP sync mendadak) | Tunggu beberapa detik; node sudah punya filter |
| **v2.5 — optitrack_bridge: `executable not found`** | entry point `optitrack_bridge` tak ada di setup.py ips_nodes | Tambah `'optitrack_bridge = ips_nodes.optitrack_bridge_node:main',` lalu rebuild ips_nodes (§6.4) |
| **v2.3 — optitrack_bridge: "belum terima frame"** | (a) **Local Interface = loopback**, (b) Motive streaming OFF, (c) IP server salah, (d) firewall, (e) WiFi blokir multicast | Set Local Interface ke 192.168.10.101 (bukan loopback); cek urut; kalau WiFi: ethernet atau Unicast (`use_multicast:=false`) |
| **v2.5 — optitrack_bridge: marker set kosong / tag tak terbaca** | "Labeled Markers" OFF, atau tag hanya di section Labeled Markers (bukan Marker Set) | Aktifkan Labeled Markers di Motive; bila tetap kosong, parser perlu baca section LabeledMarkers |
| **v2.5 — optitrack_bridge: posisi GT ~1000× meleset** | satuan NatNet = mm, node asumsi m | Jalankan `-p input_units:=mm` |
| **v2.5 — optitrack_bridge: GT offset konstan besar** | (a) `gt_marker_idx` salah (marker bukan tag), (b) konstanta transform beda sesi (0.646 vs 0.637) | Cek `raw_opti` di log → set `gt_marker_idx`; sesuaikan `x_opti_sa2`/`z_opti_sa2` (§9.2 ⚠) |
| **v2.3 — demo HTML: "Koneksi gagal"** | (a) rosbridge tidak jalan, (b) firewall port 9090, (c) URL salah | `ros2 launch rosbridge_server rosbridge_websocket_launch.xml`; `sudo ufw allow 9090/tcp`; cek `CONFIG.rosbridgeUrl` di index.html |
| **v2.3 — demo HTML: scene 3D kosong** | Three.js CDN diblokir (venue tanpa internet) | Jalankan `./setup_offline.sh` saat ada internet — lib ter-cache lokal |
| **v2.3 — demo HTML: posisi loncat-loncat** | smoothing display terlalu lemah | Turunkan `CONFIG.smoothingAlpha` 0.18→0.10 (lebih halus, sedikit lag) |
| **v2.4 — demo HTML offline: scene kosong** | folder `lib/` tidak ada di samping `index.html` | Extract seluruh folder `demo_pameran_offline/`, bukan hanya HTML; cek di F12 console: `ERR_FILE_NOT_FOUND` |
| **v2.4 — `/state/json` tidak ada di topic list** | (a) state_aggregator belum v3.3, (b) parameter `publish_topic` di-rename | Cek log startup `state_aggregator v3.3`; pastikan `ros2 topic list \| grep json` muncul |
| **v2.4 — JSON output melebur tanpa enter di stdout** | log node lain menabrak stdout sama | Jangan pakai stdout — `ros2 topic echo /state/json` di terminal terpisah |
| **v2.4 — yaw output JSON beda dengan tampilan RViz** | EXPECTED — `/state/orientation` (RViz) tidak dipatch | Lihat §6.6 + §10d; JSON pakai opsi A, RViz pakai quaternion BNO mentah |
| **v2.4 — Build C++ bias_compensator error `?:` Eigen** | tipe ekspresi Eigen mismatch di operator ternary | Pakai `if` statement: `if (mode==AFFINE) return M*p+b; return p-b;` (lihat §13 sejarah bug) |
| **v2.4 — Python nodes PackageNotFoundError setelah patch** | build artifacts korup di install/ | Clean rebuild: `rm -rf build/ips_nodes install/ips_nodes log/build_*; colcon build --symlink-install --packages-select ips_nodes` |

### 12b. Open problem — ledakan posisi dekat SA5 (z<0.6) [UNSOLVED]

**Gejala:** di zona x>1.6, y>1.6, z<0.6 (persis di bawah SA5 yang di z=1.0),
estimasi meledak. Saat tag naik ke z>0.6 (x,y sama), normal. Threshold z
spesifik & reproducible.

**Diagnosis (hipotesis kuat): ground-bounce multipath.** SA5 di z=1.0 + tag
dekat lantai → selisih path direct vs pantulan lantai mengecil (~3 ns saat
z=0.5, di bawah durasi pulsa UWB ~2 ns) → leading-edge detection SA5 terganggu
→ TDoA SA5 bias → spiral EKF. Saat z>0.6, selisih path >4 ns, pulsa terpisah,
normal. Perhitungan geometris mendukung threshold z≈0.6.

**Belum diverifikasi:** apakah pipeline Chan (`algorithm:=chan`, ada G=1+L1+L2)
JUGA meledak di zona sama? Uji diferensial diperlukan:
- Jika ya → fenomena fisik (multipath), tidak bisa di-fix algoritma; G=1 tidak
  menolong (bias ~0.5m masih di bawah threshold G=1 ~4.4m).
- Jika tidak → spesifik WoLF, bisa di-patch (per-anchor innovation gating).

**Mitigasi sementara (operasional):** flight envelope minimum z=0.8, atau zona
terlarang z<0.6 di pojok SA5. **Solusi fundamental (hardware):** angkat SA5,
absorber RF di lantai, atau pindah SA5 ke langit-langit (z=3) seperti SA3/SA4.

---

---

## 13. Checklist Sebelum Pengukuran

- [ ] Laptop IP statis `192.168.10.100`: `ip addr`
- [ ] `sudo ufw allow 5555/udp`
- [ ] MC menyala, LED berkedip (CCP berjalan)
- [ ] Semua SA menyala, konek WiFi `indoorpos`
- [ ] **Warm-up ≥ 10 menit** (thermal stabilisasi)
- [ ] Verifikasi `sync_count` naik ~32/s semua anchor
- [ ] Posisi anchor di `anchors.yaml` diukur akurat (meteran laser)
- [ ] `bias.yaml` sesuai (kalibrasi ulang jika ada perubahan filter/anchor)
- [ ] **Verifikasi `bias_compensator` versi AFFINE** (silent no-op risk):
  `grep -l "bias_matrix" ~/ips_jazzy_ws/src/ips_nodes_cpp/src/bias_compensator_node.cpp`
- [ ] Layer 1 clip rate 1-5% di log
- [ ] Layer 2 reject rate 0.5-3% di log
- [ ] Bias compensator state = OPERATIONAL  mode=**AFFINE** (cek log startup)

**Checklist tambahan saat butuh fitur v2.3 (opsional):**
- [ ] Latensi/diagnostik → `latency_monitor` jalan di terminal terpisah (§6.3)
- [ ] GT live (RViz demo) → PC Motive di subnet sama, rigid body dibuat,
  `optitrack_bridge` jalan, cek `ros2 topic hz /gt/position` ~120 Hz (§6.4)
- [ ] Demo pameran → `rosbridge_websocket` jalan, port 9090 buka, laptop
  display sudah edit `CONFIG.rosbridgeUrl` di index.html (§6.5)

---

## 14. Build Commands

```bash
cd ~/ips_jazzy_ws
source /opt/ros/jazzy/setup.bash

# C++ compute nodes (clock_sync + position_solver + bias_compensator)
colcon build --packages-select ips_nodes_cpp

# Python I/O nodes
colcon build --symlink-install --packages-select ips_nodes

# YAML config
colcon build --symlink-install --packages-select ips_bringup

# Message/service (wajib clean build jika ada .msg/.srv baru)
rm -rf build/ips_msgs install/ips_msgs
colcon build --packages-select ips_msgs ips_nodes ips_nodes_cpp ips_bringup

# Full rebuild
colcon build --symlink-install

source install/setup.bash
```

Dependency build: `sudo apt install libeigen3-dev libyaml-cpp-dev`

**Dependency v2.3 (untuk demo pameran HTML, opsional):**
```bash
sudo apt install ros-jazzy-rosbridge-suite
```
rosbridge adalah paket standar ROS — tinggal install dan launch, tidak perlu
build manual. Hanya dibutuhkan saat **demo pameran HTML** (§6.5).

---

## 15. Peta File

```
~/ips_jazzy_ws/src/
├── ips_msgs/
│   ├── msg/
│   │   ├── UwbAnchorReport.msg
│   │   ├── SessionEvent.msg
│   │   ├── CorrectedToA.msg
│   │   ├── SyncStatus.msg
│   │   ├── SyncState.msg
│   │   ├── BlinkObservation.msg
│   │   ├── AntennaDelays.msg
│   │   └── ImuTelemetry.msg         (reserved)
│   ├── srv/
│   │   ├── SetAnchorConfig.srv
│   │   ├── RecordControl.srv
│   │   └── Calibrate.srv            [NEW v1.0]
│   ├── CMakeLists.txt
│   └── package.xml
│
├── ips_nodes_cpp/                    [NEW v1.0 — C++ compute]
│   ├── include/ips_nodes_cpp/
│   │   ├── dw1000_constants.hpp
│   │   ├── timestamp_unwrapper.hpp
│   │   ├── kalman_3state.hpp
│   │   ���── sync_engine.hpp
│   │   ├── chan_solver.hpp
│   │   ├── position_kf.hpp          (PositionKF + StudentTFilter + L2 gate)
│   │   ├── dzs_filter.hpp
│   │   └── predictive_tdoa_gate.hpp  [Layer 1]
│   ├── src/
│   │   ├── algorithms/
│   │   │   ├── sync_engine.cpp
│   │   │   ├── chan_solver.cpp
│   │   │   ├── position_kf.cpp       (L2 innovation gate logic)
│   │   │   ├── dzs_filter.cpp
│   │   │   └── predictive_tdoa_gate.cpp  [Layer 1]
│   │   ├── clock_sync_node.cpp
│   │   ├── position_solver_node.cpp   (algorithm=chan: L1+L2 integrated)
│   │   ├── wolf_position_node.cpp     [v2.0] (algorithm=wolf: WoLF-EKF-CA + ZUPT)
│   │   └── bias_compensator_node.cpp  (C++ affine M·p+b)
│   ├── CMakeLists.txt
│   └── package.xml
│
├── ips_nodes/ips_nodes/              (Python I/O nodes)
│   ├── algorithms/
│   │   ├── sync_engine.py            (Python fallback)
│   │   └── position.py              (Python fallback)
│   ├── udp_gateway_node.py
│   ├── clock_sync_node.py            (Python fallback)
│   ├── position_solver_node.py       (Python fallback)
│   ├── bias_compensator_node.py      (Python fallback)
│   ├── differentiator_node.py        (sub: position_compensated)
│   ├── state_aggregator_node.py      (sub: position_compensated) [v2.3 format lines]
│   ├── calibration_service_node.py
│   ��── recorder_node.py              (7 CSV layers incl. compensated)
│   ├── latency_monitor_node.py       [NEW v2.3 — ukur latensi pipeline]
│   ├── optitrack_bridge_node.py      [v2.3/v2.5 — NatNet → /gt/position (marker tag) + /gt/pose; butuh entry point di setup.py]
│   └── common.py
│
└── ips_bringup/
    ├── launch/
    │   ├── ips_system.launch.py       (Python I/O + C++ bias_comp)
    │   └── ips_system_cpp.launch.py   (all C++ compute)  ← UTAMA
    └── config/
        ├── system.yaml
        └── anchors.yaml

Persistence:
  ~/ips_jazzy_ws/bias.yaml              kalibrasi bias affine (auto-load/save)
  ~/ips_logs/                           recording sessions

Tooling validasi (di luar workspace):
  eval_harness_v2.py                    evaluasi vs OptiTrack (GT Marker3, ref v2)
  export_gt_synced.py                   GT OptiTrack tersinkron ke timestamp sistem
  analyze_latency.py                    analisis CSV latency_monitor (5 plot)  [NEW v2.4]

Demo pameran HTML (di luar workspace):
  demo_pameran/                         [LEGACY v2.3 — CDN, gagal offline]
  ├── index.html                        visualisasi 3D via rosbridge
  ├── setup_offline.sh                  download lib (butuh internet sekali)
  └── README.md                         setup + troubleshooting

  demo_pameran_offline/                                            [NEW v2.4]
  ├── index.html                        sudah patched ke lib lokal
  ├── lib/three.min.js                  Three.js r128 (590 KB) bundled
  ├── lib/roslib.min.js                 roslibjs 1.3.0 (65 KB) bundled
  └── README.md                         setup + troubleshooting

Tarball patch sesi v2.4 (di /mnt/user-data/outputs/):
  aggregator_v33_patch.tar.gz           state_aggregator v3.3 (publish /state/json)
  orient_affine_patch.tar.gz            bias_compensator v2 (affine + orientation)
  demo_pameran_offline.tar.gz           demo HTML offline (Three.js + roslibjs lokal)
  analyze_latency.py                    analisis CSV (skrip standalone)
  optitrack_gt_patch.tar.gz             [v2.5] optitrack_bridge marker-tag + fix entry point setup.py
```

---

## 16. A/B Testing

### 16.1 Layer 1 / Layer 2 (estimator Chan)

Semua filter bisa di-toggle runtime tanpa restart:

```bash
# Test 1: Both ON (default)
ros2 launch ips_bringup ips_system_cpp.launch.py
# Test 2: Layer 2 OFF, Layer 1 ON
ros2 param set /position_solver layer2_enabled false
# Test 3: Both OFF (baseline)
ros2 param set /position_solver layer1_enabled false
ros2 param set /position_solver layer2_enabled false
# Test 4: Layer 2 ON, Layer 1 OFF
ros2 param set /position_solver layer1_enabled false
ros2 param set /position_solver layer2_enabled true
```

### 16.2 Chan vs WoLF (estimator)

Jalankan sesi terpisah dengan estimator berbeda, bandingkan vs OptiTrack:

```bash
# Sesi A: Chan
ros2 launch ips_bringup ips_system_cpp.launch.py algorithm:=chan
# rekam + OptiTrack...

# Sesi B: WoLF
ros2 launch ips_bringup ips_system_cpp.launch.py algorithm:=wolf
# rekam + OptiTrack...
```

### 16.3 ZUPT on/off (estimator WoLF)

Bandingkan dalam sesi terpisah (atau runtime param):

```bash
# ZUPT ON (default WoLF)
ros2 launch ips_bringup ips_system_cpp.launch.py algorithm:=wolf
# ZUPT OFF — runtime tanpa rebuild
ros2 param set /wolf_position zupt_enabled false
```

### 16.4 Evaluasi

Pakai `eval_harness_v2.py` (GT=Marker3 OptiTrack, referensi v2, gap-mask 0.1s):

```bash
python3 eval_harness_v2.py <opti_csv> position_compensated.csv "label"
```

Bandingkan: std hover (jitter), RMSE 3D de-biased, lag-corr (~0 = tanpa lag),
HF jitter (tanda ZUPT aktif <3mm), loncatan transisi, max error (no ledakan).

**Acceptance ZUPT:** jitter hover <15 mm, max error 3D <0.5 m, lag-corr ≤0.2,
no loncatan >5cm di transisi, `zupt_enabled:false` = identik baseline.
