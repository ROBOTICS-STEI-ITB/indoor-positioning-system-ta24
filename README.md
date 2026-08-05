
pencil accessory
/
coba buka repo ini, bantu saya buatkan readme cover untuk petunjuk tentang repo. menjelaskan struktur repo dan cara penggunaanya

https://github.com/ROBOTICS-STEI-ITB/indoor-positioning-system-ta24.git
August 4 at 10:11 AM
I'll open the repo and look at its structure.
Repo-nya tidak bisa saya buka — GitHub mengembalikan "content not available", yang biasanya berarti repositori private (atau org-nya membatasi akses publik). Saya juga tidak punya koneksi GitHub di workspace ini, jadi saya belum bisa membaca struktur folder/file-nya secara langsung.
Supaya README yang saya buat akurat (bukan mengarang struktur), saya butuh sedikit input dari Anda.
Bagaimana Anda ingin melanjutkan?
Buat template README IPS dulu (placeholder)
README-nya mau dikirim dalam bentuk apa?
File README.md siap download
README
.md
README.md sudah siap — file-nya ada di atas.
Catatan penting: repo indoor-positioning-system-ta24 tidak bisa saya buka (private), jadi isi README ini adalah template terstruktur dengan penanda <TODO: ...> di setiap bagian yang perlu Anda isi, bukan hasil pembacaan kode asli.
Isi template:
Cover — logo, judul, badge status/platform/lisensi, identitas Robotics Lab STEI ITB
Daftar isi dengan anchor link
Tentang proyek — plus tabel spesifikasi (teknologi sensor, metode estimasi, akurasi, update rate, format output)
Arsitektur sistem — diagram ASCII anchor → tag → positioning core → visualizer/logger
Struktur repositori — pohon folder tipikal proyek IPS (firmware/, src/core/, config/, data/raw/, notebooks/, docs/) + tabel "kapan Anda menyentuh folder ini"
Instalasi — clone (termasuk catatan akses repo private), venv Python, flash firmware
Konfigurasi — contoh config.yaml berisi koordinat anchor dan parameter filter
Cara penggunaan — alur langkah, perintah CLI, contoh output log, tabel argumen
Kalibrasi & pengambilan data — termasuk metrik evaluasi (MAE, RMSE, CEP50/95) dan konvensi penamaan file data
Troubleshooting — tabel gejala umum IPS (jitter, offset posisi, anchor tidak terdeteksi)
Konvensi kontribusi — strategi branch, Conventional Commits, aturan data
Tim & lisensi
Cara tercepat menyesuaikannya: jalankan tree -L 2 -I 'node_modules|.git|__pycache__' di root repo, lalu kirimkan hasilnya ke saya bersama isi main.py/platformio.ini — saya akan ganti seluruh bagian struktur dan perintah dengan yang asli.
PANDUAN_IPS_ROS2_v2_5
.md
saya jelaskan strukturnya. ada 3 folder utama, yaitu firmware, hardware, dan software. 
Berdasarkan data yang diberikan, repositori ini merupakan proyek Indoor Positioning System (Sistem Pemosisian Dalam Ruangan) yang dibangun secara komprehensif [source: 1]. Strukturnya terbagi menjadi tiga komponen utama, yaitu Firmware, Hardware, dan Software [source: 1].
Berikut adalah penjelasan detail dari struktur repositori tersebut:
• File Root
    ◦ Terdapat file README.md utama di root repositori yang umumnya digunakan untuk memberikan penjelasan dan dokumentasi ringkas mengenai proyek [source: 1].
• Direktori firmware/
    ◦ Direktori ini memuat kode program (firmware) untuk mikrokontroler yang dikembangkan menggunakan environment PlatformIO, ditandai dengan adanya file platformio.ini dan kode sumber di folder src/main.cpp [source: 1].
    ◦ Terdapat 3 subsistem utama di dalamnya: master-anchor/ (anchor utama), slave-anchor/ (anchor sekunder), dan tag/ (perangkat yang dilacak posisinya) [source: 1].
    ◦ Setiap subsistem tersebut dilengkapi dengan folder lib/DWM1000/, yang mengindikasikan bahwa sistem ini menggunakan modul Ultra-Wideband (UWB) seri DWM1000 untuk komunikasi dan kalkulasi jarak [source: 1].
• Direktori hardware/
    ◦ Direktori ini berisi seluruh file desain Printed Circuit Board (PCB) dari sistem [source: 1].
    ◦ Terdapat dua subdirektori desain, yaitu untuk anchor/ dan tag/ [source: 1].
    ◦ File-file di dalamnya meliputi skematik kelistrikan (.SchDoc seperti Core.SchDoc, Power.SchDoc), desain papan PCB (.PcbDoc), dan file proyek (.PrjPcb) [source: 1].
    ◦ Terdapat juga folder Project Outputs yang menyimpan file siap produksi/pabrikasi seperti file Gerber (ekstensi .GTL, .GBL, .GTO, dll.), file pengeboran/Drill (.TXT), daftar komponen BOM (.xlsx), dan instruksi mesin perakit atau Pick Place (.csv dan .txt) [source: 1].
• Direktori software/
    ◦ Direktori ini memuat perangkat lunak pemrosesan tingkat tinggi yang menggunakan kerangka kerja ROS 2 (Robot Operating System) [source: 1].
    ◦ Terdapat file dokumentasi PANDUAN_IPS_ROS2_v2_5 .md sebagai petunjuk penggunaan perangkat lunak [source: 1].
    ◦ Di dalamnya terdapat workspace ROS 2 bernama ips_jazzy_ws/ yang memuat beberapa paket (packages) ROS 2 [source: 1]:
        ▪ ips_bringup/: Berisi file konfigurasi sistem berekstensi .yaml (seperti anchors.yaml, system.yaml, antenna_delays.yaml) dan file peluncuran sistem atau launch file (ips_system.launch.py) [source: 1].
        ▪ ips_msgs/: Berisi definisi pesan kustom (.msg) dan layanan (.srv) untuk komunikasi antar node ROS, seperti UwbAnchorReport.msg, CorrectedToA.msg, ImuTelemetry.msg, dan Calibrate.srv [source: 1].
        ▪ ips_nodes/: Berisi kode program pemrosesan atau algoritma utama (node), contohnya node kompensasi bias (bias_compensator_node) [source: 1].
9:36 AM
Shared file
README
.md
README.md sudah saya tulis ulang — kali ini berdasarkan struktur asli yang Anda jelaskan plus isi PANDUAN_IPS_ROS2_v2_5.md yang Anda lampirkan, jadi seluruh perintah, nama topic, dan angka performansi di dalamnya nyata (bukan placeholder).
Isi README:
Bagian
Cakupan
Cover
Judul, badge (PlatformIO/Altium/ROS 2 Jazzy/v2.5), identitas Robotics Lab STEI ITB
§1 Ringkasan
Tabel spesifikasi: DWM1000, TDoA, WoLF-EKF-CA + ZUPT, LI-KF, bias affine, BNO055, ~25 Hz, arena 5.026 × 5.191 × 3.5 m
§2 Firmware
Peran master-anchor (pemancar CCP), slave-anchor (pelapor timestamp), tag (blink + IMU), perintah pio run -t upload, info jaringan indoorpos / 192.168.10.100:5555
§3 Hardware
Tabel jenis file Altium (.PrjPcb, Core.SchDoc, Power.SchDoc, .PcbDoc) dan isi Project Outputs (Gerber, Drill, BOM, Pick & Place) + peringatan regenerasi dari Output Job
§4 Software
Tabel 4 paket ROS 2 (termasuk ips_nodes_cpp untuk node C++), diagram alur data UDP → clock_sync → WoLF → bias_compensator → /state/position_compensated, dan tabel topic penting
§5 Struktur repo
Pohon direktori lengkap 3 folder utama + tabel "kebutuhan → lokasi kerja"
§6 Instalasi
Prasyarat, colcon build penuh & selektif, catatan clean build wajib untuk .msg/.srv
§7 Penggunaan
Checklist lapangan (IP statis, ufw 5555/udp, warm-up ≥10 menit), ros2 launch ips_bringup ips_system_cpp.launch.py algorithm:=wolf, monitoring, recording 14 layer CSV, dua jenis kalibrasi bias, tabel troubleshooting
§8 Performansi
RMSE 3D 101–165 mm, error x/y ~50–70 mm, z ~120–140 mm, jitter hover 24 mm, temuan 98–99% residual sistematik
§9 Alur kerja
Branch, Conventional Commits, aturan file yang tidak boleh di-commit
§10 Dokumentasi
Peta §1–§16 panduan software
Dua catatan kecil untuk Anda cek sebelum commit:
Saya menambahkan paket ips_nodes_cpp (node C++: clock_sync_node, wolf_position_node, bias_compensator_node) yang ada di panduan tapi tidak Anda sebutkan — pastikan paket ini memang ada di ips_jazzy_ws/src/.
Nama file panduan di repo mengandung spasi (PANDUAN_IPS_ROS2_v2_5 .md). Di README saya tulis tanpa spasi; sebaiknya rename file-nya agar link tidak rusak di GitHub.
Buku Tugas Akhir 13222079 no ttd
.pdf
13222041_Kania Ika Putri_TA
.pdf
BukuTA_13222020_Matthew Wijaya Chandra
.pdf
baca dokumen perancangan ini.
9:44 AM
Ketiga buku tugas akhir sudah saya baca lengkap. Ternyata ini satu proyek yang dikerjakan tiga orang dengan pembagian subsistem yang persis memetakan ke tiga folder repo Anda.
Peta dokumen → repo
Penulis
Judul / lingkup
Folder repo terkait
Priya Qolbu Dhiya'an Amar (13222079)
Perangkat keras (tag + anchor) & integrasi top-level
hardware/, firmware/, pipeline ROS 2 di software/
Kania Ika Putri (13222041)
TDoA Positioning Calculation — estimasi posisi & kecepatan translasi
software/…/ips_nodes_cpp (wolf_position_node, bias_compensator_node)
Matthew Wijaya Chandra (13222020)
Sinkronisasi wireless clock UWB + pengolahan data IMU
software/…/ips_nodes_cpp (clock_sync_node, imu_processor_node, differentiator_node)
Pembimbing sama untuk ketiganya: Anggera Bayuwindra, S.T., M.T., Ph.D. dan Ishak Hilton Pujantoro Tnunay, Ph.D. Sidang Juli 2026.
Ringkasan tiap dokumen
1. Priya — Perangkat Keras & Integrasi Top-Level
Tag: ESP32-S3FH4R2 (barechip) + BNO055 + DWM1000 + 2× LDO AP2112K + BMS MCP73831. Blink UWB & kirim IMU tiap 40 ms, dual-core (core 0 = IMU/UDP, core 1 = UWB).
Anchor: ESP32-WROOM-32 + DWM1000 + AMS1117/AP2112K + CH340 (UART-USB). 1 master clock (CCP tiap 30 ms) + 4 slave.
PCB 4 lapis (sinyal-power / 2× ground plane / sinyal), desain EMC: decoupling, ferrite bead, matching circuit 50 Ω untuk antena keramik 2450AT18B100.
Hasil: tag 5 × 4 cm, 43 g termasuk baterai 600 mAh, tahan ~175 menit. Pipeline ROS 2 Jazzy: CPU 14–23%, latensi p95 ~19 ms (17 ms transport + 2 ms komputasi), update posisi ~24,7 Hz, output JSON. Biaya ~Rp4,25 juta vs budget Rp10 juta. Semua spesifikasi ✓.
2. Kania — Estimasi Posisi & Kecepatan Translasi
Ini bagian yang paling menarik secara metodologis — perancangannya dua fase iteratif:
Fase 1: asumsi noise Gaussian (dari datasheet DW1000) → pipeline Chan + KF konvensional (state CV 6-dimensi). Simulasi MATLAB lolos (RMSE 1,5–2,5 cm).
Karakterisasi noise aktual: kurtosis > 0, Q-Q plot bengkok di kedua ujung → noise ternyata contaminated-Gaussian (ekor tebal 17–32× baseline Gaussian) akibat multipath. Asumsi fase 1 gugur.
Fase 2, dua pendekatan:
Outlier removal: pre-filter berlapis — triangle inequality (|d| ≤ 1.5·d_geom) + predictive TDoA gate (saturasi, bukan buang, karena Chan butuh 3 TDoA sekaligus).
Robust filtering: WoLF-EKF state CA 9-dimensi, pembobotan Inverse Multi-Quadratic, R_eff = R/w², plus ZUPT untuk jitter saat hover.
Hasil uji vs OptiTrack PrimeX 13 (8 kamera, akurasi 3 mm):
Pipeline
Statis (RMSE x/y/z, cm)
Dinamis (RMSE x/y/z, cm)
Chan + KF konvensional
1,2 / 2,2 / 3,7
19,3 / 22,8 / 45,7
Chan + Robust KF (Student's-t)
0,6 / 1,0 / 1,7
132,2 / 98,8 / 95,9
WoLF-EKF
1,3 / 1,0 / 1,3
5,2 / 9,8 / 8,2 ✓
Hanya WoLF-EKF yang lolos spesifikasi ±10 cm di kedua skenario → dipakai di sistem final. Kecepatan translasi diambil langsung dari state EKF (bukan finite difference), RMSE 0,11–0,15 m/s — belum memenuhi ±0,1 m/s. Penyebab: kecepatan hanya dipropagasi lewat matriks transisi tanpa measurement update, plus GT kecepatan sendiri berisik karena hasil diferensiasi posisi OptiTrack. PDOP sistem 2,26 (ambang ideal ≤ 2) → error ~13% lebih besar dari kondisi ideal.
3. Matthew — Sinkronisasi Clock UWB & IMU
Clock sync: interpolasi linear (hitung δ = (Tₖ−Tₖ₋₁)/(Rₖ−Rₖ₋₁) dan offset θ = Tₖ−Rₖ) → 3-state Kalman Filter [ToA, δ, φ] dengan model linear acceleration, agar bisa memprediksi ToA_{k+1} (penting karena real-time). Hasil: residual sinkronisasi < 0,5 ns ✓.
IMU (BNO055): LPF orde-1 → KF 1-state → Savitzky-Golay polinomial orde 3 untuk turunan (percepatan sudut) tanpa amplifikasi noise. Orientasi tidak difilter.
Hasil: orientasi ±2° ✓; percepatan translasi, kecepatan sudut, percepatan sudut ✗ — bukan karena gagal, tapi karena GT tidak memadai. Perhitungan propagasi error-nya menunjukkan σ_v OptiTrack = 2,12 m/s, σ_a = 1500 m/s², σ_ω = 514,8°/s (dari σ_p = 3 mm, Δt = 2 ms) — jauh lebih besar dari besaran yang mau divalidasi, jadi validasi dilakukan kualitatif.
Catatan penting untuk README
Ada dua inkonsistensi antara dokumen-dokumen ini dan README yang sudah saya buat:
Posisi MC berbeda. Kania (Tabel 2): MC = (0.000, 2.359, 2.250). anchors.yaml di panduan: MC = (0.000, 2.593, 2.25). Selisih 23 cm — perlu dicek mana yang benar.
Ukuran arena. Buku Priya menyebut lab 8 × 6 × 3,5 m sebagai konstrain, sementara arena implementasi aktual (Kania & panduan) adalah 5,026 × 5,191 × 3,5 m. Di README saya sudah pakai yang 5,026 × 5,191 × 3,5 m — ini yang benar untuk konfigurasi anchor.
Selain itu ada angka yang bisa memperkaya README: RMSE per-sumbu resmi dari buku TA (5,2/9,8/8,2 cm dinamis) lebih spesifik daripada "RMSE 3D 101–165 mm" yang saya ambil dari panduan software.
Mau saya perbarui README.md dengan tiga hal ini — kredit tiga penulis beserta pembagian subsistemnya, tabel hasil pengujian resmi dari buku TA, dan sitasi ketiga dokumen TA sebagai referensi?
README contoh
.md
ini readme contoh sebagai referensi saja. skrg update readme versi proyek ku. tidak perlu persis seperti readme contoh, sesuaikan apa yg cocok dan buatlah sejelas dan selengkap mungkin. keluarkan readme di output agar aku bisa download
9:52 AM
Contoh README-nya sudah saya baca. Sekarang saya susun ulang README proyek Anda dengan gaya serupa (badge, quick nav, tabel, collapsible) tapi isinya disesuaikan penuh dengan struktur 3 folder + data dari panduan ROS 2 dan ketiga buku TA.
Wrote README.md
README
.md
README.md sudah jadi — filenya ada di atas, tinggal klik untuk download.
Yang saya adopsi dari README contoh
Header terpusat + kumpulan badge (ROS 2, Ubuntu, C++, Python, PlatformIO, ESP32, DWM1000, Altium)
Tabel Quick Navigation di awal
Section beremoji, blok > [!NOTE] / [!IMPORTANT] / [!WARNING] / [!CAUTION]
Blok <details> untuk "output yang diharapkan" dan opsi lanjutan
Section Kontributor dengan anggota + pembimbing di akhir
Yang saya sesuaikan dengan proyek Anda
Contoh itu proyek single-device (Raspberry Pi di drone), sementara proyek Anda punya tiga domain terpisah — jadi strukturnya saya perluas:
Section
Kenapa ada
🏗️ Arsitektur Sistem
Diagram ASCII topologi tag↔anchor↔laptop + dua pipeline (UWB & IMU)
🗂️ Struktur Repositori
Pohon direktori lengkap 3 folder dengan anotasi fungsi tiap berkas
🛠️ Kebutuhan Sistem
Dipecah jadi 4 tabel: tag, anchor, pendukung, perangkat lunak
⚙️ Instalasi
5 langkah: prasyarat → build → jaringan → flashing → konfigurasi anchor
🎯 Kalibrasi
Section tersendiri — dua tahap yang sering tertukar (ns vs meter)
📊 Hasil & Performansi
Tabel ketercapaian spesifikasi, perbandingan 3 pipeline, metrik operasional, biaya
💾 Perekaman & Analisis
14 layer CSV + tooling di luar workspace
🩺 Troubleshooting
9 kasus dalam blok collapsible
⚠️ Batasan & Known Issues
Termasuk open problem zona SA5
Beberapa keputusan yang perlu Anda cek
Koordinat MC — saya pakai (0.000, 2.593, 2.25) dari anchors.yaml, bukan 2.359 dari buku Kania. Kalau ternyata buku yang benar, tinggal ubah satu baris di tabel konfigurasi anchor.
Angka performansi — saya gabungkan keduanya: RMSE per-sumbu resmi dari buku TA (5,2/9,8/8,2 cm) di tabel spesifikasi, dan RMSE 3D de-biased dari panduan (101–165 mm) di tabel metrik operasional, karena keduanya mengukur hal berbeda.
Judul TA Kania & Matthew — untuk Kania saya susun dari lingkup kerjanya karena halaman judul tidak terekstrak; sitasi Matthew persis seperti tertulis di buku.
Lisensi MIT — saya ikut contoh. Ganti kalau repo Anda memakai lisensi lain atau belum ada berkas LICENSE.
Placeholder gambar — contoh punya assets/HeaderRepo.svg dan assets/ArsitekturLv1.png. Saya tidak menyertakannya karena bisa jadi broken image. Kalau Anda punya banner atau diagram arsitektur Level 1/2 dari buku TA, kirimkan saja — saya sisipkan menggantikan diagram ASCII-nya.
apa itu MIT license
10:01 AM
Thought
MIT License adalah lisensi perangkat lunak sumber terbuka (open source) yang paling permisif dan paling banyak dipakai di dunia. Namanya berasal dari Massachusetts Institute of Technology, tempat lisensi ini pertama kali dirumuskan pada akhir 1980-an.
Inti isinya
Teksnya sangat pendek — hanya sekitar 170 kata. Intinya dua hal:
1. Izin yang diberikan (sangat luas)
Siapa pun boleh menggunakan, menyalin, memodifikasi, menggabungkan, menerbitkan, mendistribusikan, mensublisensikan, bahkan menjual perangkat lunak Anda — untuk keperluan apa pun, termasuk komersial dan tertutup (proprietary).
2. Dua syarat saja
Salinan pemberitahuan hak cipta dan teks lisensi harus tetap disertakan dalam setiap salinan atau bagian substansial dari perangkat lunak.
Perangkat lunak diberikan "apa adanya" (as is), tanpa jaminan apa pun. Penulis tidak bertanggung jawab atas kerusakan, kerugian, atau kegagalan yang timbul dari penggunaannya.
Jadi, ringkasnya: "Pakai bebas, silakan. Tapi jangan hapus nama saya, dan jangan tuntut saya kalau rusak."
Perbandingan dengan lisensi lain
Lisensi
Sifat
Wajib buka kode turunan?
Cocok untuk
MIT
Permisif
❌ Tidak
Proyek yang ingin dipakai seluas mungkin
Apache 2.0
Permisif
❌ Tidak
Sama seperti MIT, tapi ada klausul paten eksplisit
BSD 3-Clause
Permisif
❌ Tidak
Mirip MIT, plus larangan pakai nama institusi untuk promosi
GPL v3
Copyleft
✅ Ya
Proyek yang ingin memaksa turunannya tetap terbuka
CC BY-NC
Non-komersial
—
Dokumentasi/data, bukan untuk kode
Bentuk berkasnya
Biasanya berupa file bernama LICENSE (tanpa ekstensi) di root repositori, isinya:
MIT License

Copyright (c) 2026 Priya Qolbu Dhiya'an Amar, Kania Ika Putri,
                   Matthew Wijaya Chandra

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE, USED OR OTHER DEALINGS IN THE
SOFTWARE.
​
Kaitannya dengan repo Anda
Di README yang saya buat, badge MIT itu saya ikutkan dari README contoh — belum tentu benar untuk repo Anda. Ada tiga kemungkinan:
Repo sudah punya berkas LICENSE → cek isinya, sesuaikan badge di README.
Belum punya, dan Anda ingin repo terbuka → MIT pilihan aman dan sederhana. Tapi karena repo ada di bawah organisasi ROBOTICS-STEI-ITB, sebaiknya konfirmasi dulu ke pembimbing atau pengelola organisasi — hasil Tugas Akhir kadang terikat kebijakan kekayaan intelektual institusi.
Repo memang private dan tidak untuk dipublikasikan → hapus saja badge dan section lisensi dari README, atau ganti dengan keterangan seperti "Hak cipta © 2026 Program Studi Teknik Elektro STEI ITB. Penggunaan terbatas untuk keperluan akademik."
Satu catatan tambahan: firmware Anda memakai pustaka DWM1000. Kalau pustaka itu diadaptasi dari proyek pihak ketiga (misalnya thotro/arduino-dw1000 yang berlisensi Apache 2.0 atau MIT), lisensi aslinya harus tetap dipertahankan di dalam folder lib/DWM1000/, terpisah dari lisensi repo Anda sendiri.
Kalau Anda sudah tahu mau pakai lisensi yang mana, beri tahu saja — saya sesuaikan badge dan section lisensi di README.
Opus 5 consumes usage limits faster than other models
<div align="center">
​
  <h1>📡 Indoor Positioning System — UWB TDoA</h1>
  <h3>Sistem Penentuan Posisi 3D Dalam Ruangan untuk Pengujian Drone</h3>
​
  <p>
    <a href="https://github.com/ROBOTICS-STEI-ITB/indoor-positioning-system-ta24/stargazers"><img src="https://img.shields.io/github/stars/ROBOTICS-STEI-ITB/indoor-positioning-system-ta24?style=flat-square&color=yellow" alt="Stars"/></a>
    <a href="https://github.com/ROBOTICS-STEI-ITB/indoor-positioning-system-ta24/network/members"><img src="https://img.shields.io/github/forks/ROBOTICS-STEI-ITB/indoor-positioning-system-ta24?style=flat-square&color=blue" alt="Forks"/></a>
    <a href="https://github.com/ROBOTICS-STEI-ITB/indoor-positioning-system-ta24/commits/main"><img src="https://img.shields.io/github/last-commit/ROBOTICS-STEI-ITB/indoor-positioning-system-ta24/main?style=flat-square" alt="Last Commit"/></a>
    <img src="https://img.shields.io/badge/License-MIT-green?style=flat-square" alt="License"/>
  </p>
  <p>
    <img src="https://img.shields.io/badge/ROS%202-Jazzy%20Jalisco-22314E?style=flat-square&logo=ros&logoColor=white" alt="ROS 2 Jazzy"/>
    <img src="https://img.shields.io/badge/Ubuntu-24.04%20LTS-E95420?style=flat-square&logo=ubuntu&logoColor=white" alt="Ubuntu"/>
    <img src="https://img.shields.io/badge/C%2B%2B-17-00599C?style=flat-square&logo=cplusplus&logoColor=white" alt="C++17"/>
    <img src="https://img.shields.io/badge/Python-3.12-3776AB?style=flat-square&logo=python&logoColor=white" alt="Python"/>
    <img src="https://img.shields.io/badge/PlatformIO-Firmware-FF7F00?style=flat-square&logo=platformio&logoColor=white" alt="PlatformIO"/>
    <img src="https://img.shields.io/badge/ESP32-S3%20%7C%20WROOM--32-E7352C?style=flat-square&logo=espressif&logoColor=white" alt="ESP32"/>
    <img src="https://img.shields.io/badge/DWM1000-Ultra%20Wideband-8A2BE2?style=flat-square" alt="DWM1000"/>
    <img src="https://img.shields.io/badge/Altium-PCB%204%20Layer-A5915F?style=flat-square&logo=altiumdesigner&logoColor=white" alt="Altium"/>
  </p>
​
</div>
​
---
​
> **Indoor Positioning System (IPS)** adalah sistem penentuan posisi tiga dimensi dalam ruangan berbasis **Ultra-Wideband (UWB)** dengan metode **Time Difference of Arrival (TDoA)**. Sistem ini mengestimasi **posisi, orientasi, kecepatan, dan percepatan** (translasi maupun angular) sebuah drone secara *real-time* di dalam laboratorium, sebagai pengganti terjangkau dari motion capture komersial. Repositori ini memuat seluruh hasil kerja: **desain PCB**, **firmware embedded**, dan **pipeline pemrosesan ROS 2**.
​
---
​
## 📋 Quick Navigation
​
| Bagian | Deskripsi |
|---|---|
| [🌍 Tentang Proyek](#-tentang-proyek) | Latar belakang, motivasi, dan tujuan sistem |
| [🏗️ Arsitektur Sistem](#️-arsitektur-sistem) | Komponen, alur data, dan pipeline pemrosesan |
| [🗂️ Struktur Repositori](#️-struktur-repositori) | Penjelasan isi folder `firmware`, `hardware`, `software` |
| [🛠️ Kebutuhan Sistem](#️-kebutuhan-sistem) | Perangkat keras dan perangkat lunak yang dibutuhkan |
| [⚙️ Instalasi](#️-instalasi) | Setup workspace ROS 2, firmware, dan jaringan |
| [🎮 Cara Penggunaan](#-cara-penggunaan) | Panduan operasional harian sistem |
| [🎯 Kalibrasi](#-kalibrasi) | Dua tahap kalibrasi wajib sebelum pengukuran |
| [📊 Hasil & Performansi](#-hasil--performansi) | Ketercapaian spesifikasi dan hasil pengujian |
| [💾 Perekaman & Analisis Data](#-perekaman--analisis-data) | Recorder 14 layer dan tooling evaluasi |
| [🩺 Troubleshooting](#-troubleshooting) | Solusi masalah yang sering muncul |
| [⚠️ Batasan & Known Issues](#️-batasan--known-issues) | Hal yang perlu diketahui sebelum memakai sistem |
| [📚 Dokumentasi & Referensi](#-dokumentasi--referensi) | Panduan lengkap dan buku tugas akhir |
| [🤝 Kontributor](#-kontributor) | Tim pengembang dan pembimbing |
​
---
​
## 🌍 Tentang Proyek
​
Peneliti robotika di **Institut Teknologi Bandung (ITB)** melakukan pengujian sistem kendali dan algoritma drone di skala laboratorium. Untuk memvalidasi kinerja sistem tersebut, dibutuhkan informasi *real-time* mengenai **posisi, orientasi, kecepatan, dan percepatan** drone. Masalahnya:
​
- 🛰️ **GPS tidak dapat digunakan di dalam ruangan** — sinyal satelit teratenuasi oleh material bangunan.
- 💸 **Motion capture komersial sangat mahal** — OptiTrack setup minimum 4 kamera ≈ **Rp86,2 juta**, Marvelmind (ultrasound) ≈ **Rp13,7 juta**.
- 📉 **Alternatif murah kurang akurat** — IMU murni, Wi-Fi, dan Bluetooth hanya mencapai akurasi sub-meter, tidak cukup untuk validasi sistem kendali.
​
**Indoor Positioning System** hadir sebagai solusi berbiaya rendah dengan total biaya produksi **± Rp4,25 juta**, yang mampu:
​
- 📍 **Mengestimasi posisi 3D** drone dengan akurasi RMSE per-sumbu **< 10 cm** menggunakan UWB-TDoA
- 🧭 **Mengestimasi orientasi** (roll, pitch, yaw) dengan akurasi **± 2°** melalui IMU BNO055 on-board
- 🏃 **Menurunkan kecepatan & percepatan** translasi maupun angular dari state estimator dan sensor IMU
- ⏱️ **Menyinkronkan clock antar-anchor secara nirkabel** dengan residual **< 0,5 ns** — tanpa kabel sinkronisasi
- ⚡ **Bekerja real-time** dengan latensi *end-to-end* p95 **≈ 19 ms** (jauh di bawah konstrain 60 ms)
- 🪶 **Ringan & tidak mengganggu manuver** — tag hanya **43 g** termasuk baterai, bertahan **± 175 menit**
- 🔌 **Berdaya mandiri** — tag memiliki baterai LiPo sendiri, tidak membebani sistem daya drone
- 📤 **Menyediakan output terbuka** — topic ROS 2, string JSON, WebSocket (rosbridge), dan rekaman CSV
​
> 💡 Proyek ini dikembangkan sebagai **Tugas Akhir** di Program Studi Sarjana Teknik Elektro, Sekolah Teknik Elektro dan Informatika (STEI), **Institut Teknologi Bandung**, tahun 2026.
​
---
​
## 🏗️ Arsitektur Sistem
​
Sistem terdiri atas **tiga bagian besar** yang berkorespondensi langsung dengan tiga folder utama repositori ini.
​
```
┌──────────────────┐        UWB blink (40 ms)         ┌──────────────────────┐
│       TAG        │ ───────────────────────────────► │   5 × ANCHOR         │
│  ESP32-S3 +      │                                  │  1 Master Clock (MC) │
│  DWM1000 +       │        IMU via Wi-Fi/UDP         │  4 Slave Anchor (SA) │
│  IMU BNO055      │ ───────────────────────────┐     │  ESP32-WROOM-32 +    │
│  Baterai LiPo    │                            │     │  DWM1000             │
└──────────────────┘                            │     └──────────┬───────────┘
                                                │                │
     MC broadcast CCP tiap 30 ms ◄──────────────┼────────────────┘
                                                │                │ UDP :5555
                                                ▼                ▼
                            ┌───────────────────────────────────────────┐
                            │   LAPTOP — Ubuntu 24.04 + ROS 2 Jazzy     │
                            │   Pipeline pemrosesan & estimasi posisi   │
                            └───────────────────────────────────────────┘
```
​
### 🔄 Alur Kerja Sistem
​
1. **Anchor dipasang** pada tripod di lima titik dengan ketinggian bervariasi (non-coplanar), lalu dinyalakan.
2. **Master Clock (MC)** memancarkan *Clock Calibration Packet* (CCP) secara periodik tiap **30 ms**.
3. **Slave anchor** mencatat waktu kedatangan CCP untuk menyelaraskan clock lokalnya ke domain waktu master.
4. **Tag** yang terpasang pada drone memancarkan sinyal *blink* UWB tiap **40 ms**, sekaligus mengirim data IMU.
5. **Setiap anchor** mencatat *Time of Arrival* (ToA) blink dan meneruskannya ke laptop melalui **UDP over Wi-Fi**.
6. **Node `clock_sync`** mengoreksi ToA mentah menjadi ToA tersinkronisasi dalam satu domain waktu.
7. **Node `wolf_position`** menyelesaikan persamaan hiperboloid TDoA dan mengestimasi posisi + kecepatan.
8. **Node `bias_compensator`** menerapkan koreksi bias affine untuk menghasilkan posisi final.
9. **Node `imu_processor`** & **`differentiator`** menghasilkan orientasi, kecepatan sudut, dan percepatan.
10. **Data dipublikasikan** ke topic ROS 2, di-*stream* sebagai JSON, dan direkam ke CSV untuk analisis.
​
### 🧩 Pipeline Pemrosesan Detail
​
**Jalur UWB (estimasi posisi):**
​
```
[MC + SA2-SA5] ──UDP──► udp_gateway ──► /uwb/anchor_reports  (~233 Hz)
                                             │
                                             ▼
                          clock_sync  (interpolasi linear + KF 3-state)
                                             │
                                             ▼
                                   /uwb/corrected_toa  (~25 Hz × 4 anchor)
                                             │
                                             ▼
                       wolf_position  (WoLF-EKF 9-state CA + ZUPT)
                                             │
                                             ▼
                                      /state/position
                                             │
                                             ▼
                       bias_compensator  (koreksi affine  p_true = M·p + b)
                                             │
                                             ▼
                        ⭐ /state/position_compensated  ← TOPIC UTAMA (~25 Hz)
                                             │
                          ┌──────────────────┴──────────────────┐
                          ▼                                     ▼
                   differentiator                        state_aggregator
              (kecepatan & percepatan)                  (agregasi → JSON)
                                                                │
                                                                ▼
                                                          recorder → CSV
```
​
**Jalur IMU (estimasi orientasi):**
​
```
[Tag BNO055] ──UDP──► udp_gateway ──► /imu/raw ──► imu_processor
                                                         │
                             ┌───────────────────────────┼───────────────────────────┐
                             ▼                           ▼                           ▼
                    /state/orientation        /state/angular_velocity   /state/translation_acceleration
                                                         │
                                                         ▼  (Savitzky-Golay)
                                              /state/angular_acceleration
```
​
> [!NOTE]
> Estimator produk akhir adalah **WoLF-EKF** (`algorithm:=wolf`). Algoritma **Chan** hanya digunakan sebagai *seed* awal, mekanisme *recovery*, dan pembanding diagnostik melalui topic `/state/position_chan`.
​
---
​
## 🗂️ Struktur Repositori
​
```
indoor-positioning-system-ta24/
│
├── README.md                      ← Dokumen ini
│
├── firmware/                      ← 🔧 Kode embedded (PlatformIO)
│   ├── master-anchor/
│   │   ├── platformio.ini
│   │   ├── src/main.cpp           ← Broadcast CCP + terima blink + kirim UDP
│   │   └── lib/DWM1000/           ← Driver transceiver DWM1000
│   ├── slave-anchor/
│   │   ├── platformio.ini
│   │   ├── src/main.cpp           ← Terima CCP & blink, catat ToA, kirim UDP
│   │   └── lib/DWM1000/
│   └── tag/
│       ├── platformio.ini
│       ├── src/main.cpp           ← Pancarkan blink 40 ms + baca IMU BNO055
│       └── lib/DWM1000/
│
├── hardware/                      ← 🔌 Desain PCB (Altium Designer)
│   ├── anchor/
│   │   ├── *.PrjPcb               ← Berkas proyek Altium
│   │   ├── Core.SchDoc            ← Skematik MCU, DWM1000, UART-USB
│   │   ├── Power.SchDoc           ← Skematik regulator daya
│   │   ├── *.PcbDoc               ← Layout PCB 4 lapis
│   │   └── Project Outputs/
│   │       ├── *.GTL / *.GBL / *.GTO   ← Berkas Gerber (fabrikasi)
│   │       ├── *.TXT                   ← Berkas Drill (NC Drill)
│   │       ├── BOM *.xlsx              ← Bill of Materials
│   │       └── Pick Place *.csv/.txt   ← Data penempatan komponen (SMT)
│   └── tag/
│       └── (struktur sama dengan anchor)
│
└── software/                      ← 💻 Pemrosesan ROS 2
    ├── PANDUAN_IPS_ROS2_v2_5.md   ← 📖 Panduan operasional lengkap
    └── ips_jazzy_ws/
        └── src/
            ├── ips_msgs/          ← Definisi pesan & service kustom
            │   ├── msg/           ← UwbAnchorReport, CorrectedToA, ImuTelemetry, ...
            │   └── srv/           ← Calibrate, RecordControl, SetAnchorConfig
            ├── ips_nodes_cpp/     ← Node C++ (jalur kritis, performa tinggi)
            │   ├── include/       ← sync_engine, chan_solver, position_kf,
            │   │                    dzs_filter, kalman_3state, ...
            │   └── src/           ← clock_sync_node, wolf_position_node,
            │                        bias_compensator_node, position_solver_node
            ├── ips_nodes/         ← Node Python (I/O, utilitas, tooling)
            │   └── ips_nodes/     ← udp_gateway, state_aggregator, recorder,
            │                        calibration_service, latency_monitor,
            │                        optitrack_bridge, differentiator
            └── ips_bringup/       ← Konfigurasi & launch file
                ├── launch/        ← ips_system_cpp.launch.py  (⭐ utama)
                └── config/        ← system.yaml, anchors.yaml
```
​
> [!WARNING]
> Jika nama berkas panduan di repositori masih mengandung spasi (`PANDUAN_IPS_ROS2_v2_5 .md`), sebaiknya **di-rename tanpa spasi** agar tautan relatif di README tidak rusak saat dirender GitHub.
​
---
​
## 🛠️ Kebutuhan Sistem
​
### Perangkat Keras — Sub-sistem Tag
​
| Komponen | Spesifikasi |
|---|---|
| **Mikrokontroler** | ESP32-S3FH4R2 (Xtensa LX7 dual-core, 4 MB Flash, 2 MB PSRAM) |
| **Kristal Osilator** | 40 MHz |
| **Transceiver UWB** | DWM1000 (antarmuka SPI) |
| **IMU** | Bosch BNO055 (antarmuka I²C, pull-up 10 kΩ, kristal 32 kHz) |
| **Antena** | Keramik 2450AT18B100 + *matching circuit* 50 Ω |
| **Regulator** | 2 × LDO AP2112K (3,3 V) |
| **Battery Management** | MCP73831/2 (pengisian LiPo 1S) |
| **Baterai** | LiPo 1S 600 mAh |
| **Dimensi PCB** | 5 cm × 4 cm (+1 cm bagian antena) |
| **Berat Total** | 43 g (tag + mounting 29 g, baterai 14 g) |
| **Durasi Operasi** | ± 175 menit (4,19 V → 3,52 V ≈ 30% SoC) |
​
### Perangkat Keras — Sub-sistem Anchor
​
| Komponen | Spesifikasi |
|---|---|
| **Mikrokontroler** | ESP32-WROOM-32 (Xtensa LX6 dual-core) |
| **Transceiver UWB** | DWM1000 (antarmuka SPI) |
| **Regulator** | AMS1117 (MCU) + AP2112K (DWM1000) |
| **UART-to-USB** | CH340 + auto-reset BJT S8050 |
| **PCB** | 4 lapis — sinyal+power (atas), 2 × ground plane (dalam), sinyal (bawah) |
| **Jumlah** | 5 unit (1 Master Clock + 4 Slave Anchor) |
| **Mounting** | Tripod dengan ketinggian dapat diatur |
| **Catu Daya** | Adaptor switching 5 V |
​
### Perangkat Keras — Pendukung
​
| Komponen | Spesifikasi |
|---|---|
| **Laptop Pemroses** | Ubuntu 24.04 LTS, CPU setara Intel Core i7-1355U, RAM ≥ 16 GB |
| **Router / Access Point** | Wi-Fi 2,4 GHz, SSID `indoorpos`, subnet `192.168.10.0/24` |
| **Drone Uji** | Micro-UAV, dimensi ± 304 × 380 × 91 mm, berat ± 250 g |
| **Ground Truth (opsional)** | OptiTrack PrimeX 13, 8 kamera, akurasi posisi ± 3 mm |
​
### Perangkat Lunak
​
| Kategori | Teknologi |
|---|---|
| **Sistem Operasi** | Ubuntu 24.04 LTS (Noble Numbat) |
| **Middleware Robotika** | ROS 2 Jazzy Jalisco |
| **Bahasa — Jalur Kritis** | C++17 (`ips_nodes_cpp`) |
| **Bahasa — I/O & Tooling** | Python 3.12 (`ips_nodes`) |
| **Bahasa — Firmware** | C/C++ (Arduino framework via PlatformIO) |
| **Build System** | colcon, CMake, ament |
| **Aljabar Linear** | Eigen3 (`libeigen3-dev`) |
| **Parser Konfigurasi** | yaml-cpp (`libyaml-cpp-dev`) |
| **Jembatan Web** | rosbridge_suite (WebSocket) |
| **Visualisasi** | PlotJuggler, Three.js r128 + roslibjs 1.3.0 (demo pameran) |
| **Desain PCB** | Altium Designer |
| **IDE Firmware** | PlatformIO (VS Code) |
| **Analisis Data** | Python + NumPy, pandas, matplotlib |
| **Simulasi Algoritma** | MATLAB |
​
---
​
## ⚙️ Instalasi
​
### 1️⃣ Prasyarat
​
> [!IMPORTANT]
> Pastikan **ROS 2 Jazzy** sudah terpasang di Ubuntu 24.04. Ikuti [panduan instalasi resmi ROS 2](https://docs.ros.org/en/jazzy/Installation.html) apabila belum.
​
```bash
# Dependensi build untuk node C++
sudo apt update
sudo apt install libeigen3-dev libyaml-cpp-dev
​
# Paket ROS 2 pendukung
sudo apt install ros-jazzy-rosbridge-suite ros-jazzy-plotjuggler-ros
```
​
### 2️⃣ Kloning & Build Workspace
​
```bash
git clone https://github.com/ROBOTICS-STEI-ITB/indoor-positioning-system-ta24.git
cd indoor-positioning-system-ta24/software/ips_jazzy_ws
​
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```
​
<details>
<summary>📦 Build per-paket (untuk iterasi cepat saat pengembangan)</summary>
​
```bash
# Node C++ — perlu rebuild penuh setiap ada perubahan
colcon build --packages-select ips_nodes_cpp
​
# Node Python & konfigurasi — symlink, cukup sekali build
colcon build --symlink-install --packages-select ips_nodes
colcon build --symlink-install --packages-select ips_bringup
​
# Jika definisi pesan berubah, bersihkan dulu lalu build berantai
rm -rf build/ips_msgs install/ips_msgs
colcon build --packages-select ips_msgs ips_nodes ips_nodes_cpp ips_bringup
```
​
</details>
​
<details>
<summary>🧰 Tambahkan sourcing otomatis ke <code>~/.bashrc</code></summary>
​
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/ips_jazzy_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
​
</details>
​
### 3️⃣ Konfigurasi Jaringan
​
Laptop harus memakai **IP statis** agar anchor dapat mengirim paket UDP ke alamat yang tetap.
​
| Parameter | Nilai |
|---|---|
| SSID Wi-Fi | `indoorpos` |
| IP Laptop (statis) | `192.168.10.100` |
| IP Anchor | `192.168.10.x` |
| Port UDP | `5555` |
​
```bash
# Izinkan lalu lintas UDP masuk pada port 5555
sudo ufw allow 5555/udp
​
# Verifikasi paket benar-benar sampai
sudo tcpdump -i any udp port 5555 -c 20
```
​
### 4️⃣ Flashing Firmware
​
```bash
# Ulangi untuk masing-masing: master-anchor, slave-anchor, tag
cd firmware/master-anchor
pio run -t upload
​
# Pantau keluaran serial untuk verifikasi
pio device monitor -b 115200
```
​
<details>
<summary>✅ Output serial yang diharapkan</summary>
​
```
[BOOT] DWM1000 init...
[BOOT] DEV_ID = 0xDECA0130          ← Transceiver terdeteksi dengan benar
[BOOT] WiFi connecting to indoorpos...
[BOOT] IP = 192.168.10.11
[BOOT] UDP target = 192.168.10.100:5555
[RUN ] CCP tx ok, seq=1024
```
​
Jika `DEV_ID` bukan `0xDECA0130`, periksa kembali penyolderan jalur SPI dan tegangan LDO (harus berada di rentang **3,28 – 3,30 V**).
​
</details>
​
### 5️⃣ Konfigurasi Posisi Anchor
​
Edit `software/ips_jazzy_ws/src/ips_bringup/config/anchors.yaml` sesuai penempatan aktual anchor di arena Anda. Konfigurasi baku hasil implementasi:
​
| ID | Nama | x (m) | y (m) | z (m) | Peran |
|:--:|:--|--:|--:|--:|:--|
| 1 | `MC` | 0,000 | 2,593 | 2,250 | Master Clock — sumber CCP |
| 2 | `MA` / SA2 | 0,000 | 0,000 | 0,380 | Anchor referensi (origin, `a_ref`) |
| 3 | `SA3` | 5,026 | 0,000 | 3,000 | Slave anchor |
| 4 | `SA4` | 0,000 | 5,191 | 3,000 | Slave anchor |
| 5 | `SA5` | 5,026 | 5,191 | 1,000 | Slave anchor |
​
**Dimensi arena:** `ROOM = (5,026 × 5,191 × 3,5) m` — kerangka koordinat tangan-kanan, satuan meter.
​
> [!IMPORTANT]
> Keempat *slave anchor* **wajib ditempatkan non-coplanar** (ketinggian bervariasi). Jika seluruh anchor sebidang, matriks Jacobian kehilangan *full column rank* sehingga koordinat **z tidak dapat diobservasi sama sekali** dan solusi posisi menjadi ambigu (cermin atas–bawah). Konfigurasi di atas menghasilkan **HDOP 1,23 · VDOP 1,86 · PDOP 2,26**.
​
---
​
## 🎮 Cara Penggunaan
​
> [!IMPORTANT]
> Bagian ini mengasumsikan sistem sudah pernah di-setup dan **dikalibrasi**. Jika belum, lakukan [🎯 Kalibrasi](#-kalibrasi) terlebih dahulu — tanpa kalibrasi, posisi keluaran dapat meleset ratusan milimeter.
​
### 1. Nyalakan perangkat & tunggu pemanasan
​
Nyalakan kelima anchor dan tag, lalu **tunggu minimal 10 menit**. Kristal osilator DWM1000 memerlukan waktu untuk mencapai kesetimbangan termal — tanpa pemanasan, akan muncul *drift* posisi bertahap.
​
### 2. Sourcing environment
​
```bash
cd ~/ips_jazzy_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```
​
### 3. Jalankan sistem
​
```bash
ros2 launch ips_bringup ips_system_cpp.launch.py algorithm:=wolf
```
​
<details>
<summary>⚙️ Menjalankan dengan parameter lengkap (kalibrasi, perekaman, laju keluaran)</summary>
​
```bash
ros2 launch ips_bringup ips_system_cpp.launch.py \
    algorithm:=wolf \
    bias_yaml:=~/ips_jazzy_ws/bias.yaml \
    record_dir:=~/ips_logs \
    output_rate_hz:=10.0
```
​
| Argumen | Fungsi |
|---|---|
| `algorithm` | Estimator yang dipakai — `wolf` (produk) atau `chan` (diagnostik) |
| `bias_yaml` | Lokasi berkas kalibrasi bias affine |
| `record_dir` | Direktori penyimpanan rekaman CSV |
| `output_rate_hz` | Laju publikasi agregat / JSON |
​
</details>
​
### 4. Verifikasi kesehatan sistem
​
Sebelum mulai mengambil data, pastikan **seluruh indikator berikut** terpenuhi:
​
| Indikator | Nilai Sehat | Cara Memeriksa |
|---|---|---|
| `sync_count` | naik ± 32 per detik | `ros2 topic echo /uwb/sync_status` |
| Laju blink diterima | ± 24,9 Hz (loss < 0,5%) | `ros2 topic hz /uwb/corrected_toa` |
| Layer-1 clip rate | 1 – 5% | log node `wolf_position` |
| Layer-2 reject rate | 0,5 – 3% | log node `wolf_position` |
| `bias_compensator` | `state=OPERATIONAL`, `mode=AFFINE` | log node `bias_compensator` |
| Latensi pipeline | p95 < 25 ms | `ros2 run ips_nodes latency_monitor` |
​
### 5. Baca data posisi
​
```bash
# ⭐ Topic utama — posisi final terkompensasi
ros2 topic echo /state/position_compensated --field pose.pose.position
​
# Keluaran agregat dalam format JSON (posisi + orientasi + turunan)
ros2 topic echo /state/json
```
​
<details>
<summary>📡 Daftar lengkap topic yang tersedia</summary>
​
| Topic | Laju | Keterangan |
|---|---|---|
| `/uwb/anchor_reports` | ~233 Hz | Laporan mentah dari seluruh anchor |
| `/uwb/corrected_toa` | ~25 Hz × 4 | ToA tersinkronisasi per anchor |
| `/uwb/sync_status` | 1 Hz | Status & metrik sinkronisasi clock |
| `/uwb/anchor_config` | latched | Konfigurasi posisi anchor |
| `/state/position` | ~25 Hz | Posisi WoLF-EKF sebelum kompensasi bias |
| `/state/position_chan` | ~25 Hz | Posisi Chan — diagnostik/pembanding |
| **`/state/position_compensated`** | **~25 Hz** | **⭐ Posisi final — gunakan topic ini** |
| `/state/translation_velocity` | ~25 Hz | Kecepatan translasi |
| `/state/wolf_velocity` | ~25 Hz | Kecepatan langsung dari state EKF |
| `/state/translation_acceleration` | ~25 Hz | Percepatan translasi (dari IMU) |
| `/imu/raw` | ~25 Hz | Telemetri IMU mentah |
| `/state/orientation` | ~25 Hz | Orientasi (roll, pitch, yaw) |
| `/state/angular_velocity` | ~25 Hz | Kecepatan sudut |
| `/state/angular_acceleration` | ~25 Hz | Percepatan sudut (Savitzky-Golay) |
| `/state/json` | konfigurabel | Agregat seluruh state dalam JSON |
| `/diag/pipeline_latency_ms` | ~25 Hz | Diagnostik latensi end-to-end |
| `/gt/pose`, `/gt/position` | ~120 Hz | Ground truth OptiTrack (bila aktif) |
​
</details>
​
### 6. Utilitas tambahan
​
```bash
# Monitor latensi pipeline secara real-time
ros2 run ips_nodes latency_monitor
​
# Jembatan ground truth OptiTrack (untuk validasi)
ros2 run ips_nodes optitrack_bridge
​
# Jembatan WebSocket untuk visualisasi web / demo
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```
​
---
​
## 🎯 Kalibrasi
​
Sistem memerlukan **dua tahap kalibrasi yang berbeda**. Keduanya wajib dilakukan ulang setiap kali sistem dipindahkan ke arena baru.
​
### Tahap 1 — Antenna Delay (bias per-anchor)
​
Mengompensasi *delay* propagasi sinyal dari antena hingga register pencatat waktu. Nilainya **konstan per-anchor** karena bergantung pada karakteristik perangkat keras masing-masing.
​
- **Satuan:** nanodetik
- **Posisi di pipeline:** pra-Chan (koreksi paling awal)
- **Skrip:** `self_calibrate.py`
- **Tersimpan di:** `bias_values_ns` dalam `system.yaml`
​
```bash
# Tempatkan tag pada posisi statis yang diketahui, lalu jalankan
python3 self_calibrate.py
```
​
### Tahap 2 — Bias Affine (koreksi global pasca-filter)
​
Mengompensasi bias sistematis yang bergantung posisi (akibat geometri anchor dan multipath) dengan model `p_true = M · p_meas + b`.
​
- **Satuan:** meter
- **Posisi di pipeline:** pasca-Kalman Filter
- **Tersimpan di:** `bias.yaml` (skema v4)
​
**Alur kerja lengkap:**
​
| Langkah | Perintah / Tindakan |
|:--:|---|
| 1 | `python3 calib_targets.py` — hasilkan ± 12 titik target D-optimal |
| 2 | Pemanasan sistem **30 menit** |
| 3 | Hover drone **10–30 detik** di setiap titik target sambil merekam |
| 4 | `python3 export_gt_synced.py` — sinkronkan rekaman dengan ground truth |
| 5 | `python3 fit_bias.py` — hitung matriks `M` dan offset `b` → `bias.yaml` |
​
**Alternatif — kalibrasi cepat via ROS 2 service:**
​
```bash
ros2 service call /bias_compensator/calibrate ips_msgs/srv/Calibrate \
    '{gt_x: 2.5, gt_y: 2.6, gt_z: 1.15, n_samples: 500, skip_warmup: 100}'
```
​
> [!NOTE]
> Residual galat sistem bersifat **98–99% sistematik**, artinya sebagian besar galat memang dapat dikoreksi lewat kalibrasi. Namun model affine global memiliki batas kemampuan di kisaran **130–170 mm** karena bias sebenarnya bervariasi terhadap posisi di dalam arena.
​
---
​
## 📊 Hasil & Performansi
​
### Ketercapaian Spesifikasi Sistem
​
| Spesifikasi | Target | Hasil | Status |
|---|---|---|:--:|
| Dimensi tag | ≤ 7 cm × 4 cm | 5 cm × 4 cm | ✅ |
| Berat tag (termasuk baterai) | ≤ 100 g | 43 g | ✅ |
| Kapasitas baterai | ≥ 120 mAh | 600 mAh | ✅ |
| Berat baterai | ≤ 40 g | 14 g | ✅ |
| Format keluaran | JSON | JSON + topic ROS 2 + CSV | ✅ |
| Latensi komputasi | < 60 ms | ± 19 ms (p95) | ✅ |
| Akurasi posisi *P(x, y, z)* | ± 10 cm | RMSE 5,2 / 9,8 / 8,2 cm | ✅ |
| Residual sinkronisasi clock | orde sub-ns | < 0,5 ns | ✅ |
| Akurasi orientasi | ± 2° | memenuhi | ✅ |
| Akurasi kecepatan translasi | ± 0,1 m/s | RMSE 0,11 – 0,15 m/s | ⚠️ |
| Akurasi kecepatan & percepatan sudut | ± 4,2 °/s · ± 2,5 °/s² | tidak tervalidasi kuantitatif | ⚠️ |
​
### Perbandingan Kandidat Pipeline Estimasi Posisi
​
Tiga kandidat algoritma diuji secara *real-time* di arena, divalidasi terhadap **OptiTrack PrimeX 13** (8 kamera, akurasi 3 mm).
​
**Skenario statis** — RMSE per sumbu (cm):
​
| Pipeline | Sumbu-x | Sumbu-y | Sumbu-z |
|---|--:|--:|--:|
| Chan + KF Konvensional | 1,2 | 2,2 | 3,7 |
| Chan + Robust KF (Student's-t) | 0,6 | 1,0 | 1,7 |
| **WoLF-EKF** | 1,3 | 1,0 | 1,3 |
​
**Skenario dinamis** — RMSE per sumbu (cm):
​
| Pipeline | Sumbu-x | Sumbu-y | Sumbu-z | Lolos spesifikasi |
|---|--:|--:|--:|:--:|
| Chan + KF Konvensional | 19,3 | 22,8 | 45,7 | ❌ |
| Chan + Robust KF (Student's-t) | 132,2 | 98,8 | 95,9 | ❌ |
| **WoLF-EKF** | **5,2** | **9,8** | **8,2** | ✅ |
​
> 🏆 **WoLF-EKF adalah satu-satunya kandidat yang memenuhi spesifikasi ± 10 cm pada kedua skenario**, sehingga dipilih sebagai algoritma produk. Keunggulannya berasal dari *Posterior Influence Function* yang terbatas — pengaruh satu *outlier* terhadap estimasi state dijamin memiliki batas tetap.
​
### Metrik Operasional
​
| Metrik | Nilai |
|---|---|
| Laju pembaruan posisi | ± 24,7 Hz |
| Laju blink diterima | ± 24,9 Hz (packet loss 0,12 – 0,48%) |
| Laju CCP (sinkronisasi) | ± 33,33 Hz |
| Latensi p95 end-to-end | ± 19 ms (transport 17 ms + komputasi 2 ms) |
| Penggunaan CPU | 14 – 23% per core (Intel Core i7-1355U) |
| Penggunaan RAM | ± 4,3 GB (26,1%) |
| Jitter perioda IMU | 2,45 ms (nominal 40 ms) |
| Jitter posisi saat hover | 38 – 51 mm → **24 mm dengan ZUPT (−53%)** |
| RMSE 3D *de-biased* | 101 – 165 mm (x/y ± 50–70 mm; z ± 120–140 mm) |
| Tegangan keluaran LDO | 3,28 – 3,30 V |
​
### Estimasi Biaya Produksi
​
| Item | Satuan | Jumlah | Subtotal |
|---|--:|--:|--:|
| Tag | Rp 669.881 | 1 | Rp 669.881 |
| Anchor | Rp 378.375 | 5 | Rp 1.891.876 |
| Setup, mounting, & lain-lain | — | — | Rp 1.690.591 |
| **Total** | | | **Rp 4.252.348** |
​
> 💰 Sekitar **5%** dari biaya OptiTrack setup minimum (± Rp86,2 juta) dan **31%** dari Marvelmind Starter (± Rp13,7 juta).
​
---
​
## 💾 Perekaman & Analisis Data
​
### Kontrol Perekaman
​
Perekaman dikendalikan lewat ROS 2 service, sehingga dapat dimulai/dihentikan tanpa merestart sistem.
​
```bash
# Mulai merekam dengan label sesi
ros2 service call /recorder/control ips_msgs/srv/RecordControl \
    "{action: 'start', label: 'titik_A_diam'}"
​
# Cek status perekaman
ros2 service call /recorder/control ips_msgs/srv/RecordControl "{action: 'status'}"
​
# Hentikan perekaman
ros2 service call /recorder/control ips_msgs/srv/RecordControl "{action: 'stop'}"
```
​
### Struktur Keluaran
​
Setiap sesi menghasilkan **14 berkas CSV** di `~/ips_logs/<YYYYMMDD_HHMMSS>_<label>/`:
​
| Berkas | Isi |
|---|---|
| `position.csv` | Posisi WoLF-EKF sebelum kompensasi |
| **`position_compensated.csv`** | **⭐ Posisi final — dataset utama untuk analisis** |
| `position_chan.csv` | Posisi Chan (pembanding diagnostik) |
| `corrected_toa.csv` | ToA tersinkronisasi per anchor |
| `sync_status.csv` | Metrik kualitas sinkronisasi clock |
| `master_anchor.csv` | Laporan mentah master anchor |
| `slave_anchor.csv` | Laporan mentah slave anchor |
| `imu_raw.csv` | Telemetri IMU mentah |
| `orientation.csv` | Roll, pitch, yaw |
| `translation_velocity.csv` | Kecepatan translasi |
| `wolf_velocity.csv` | Kecepatan dari state EKF |
| `angular_velocity.csv` | Kecepatan sudut |
| `translation_acceleration.csv` | Percepatan translasi |
| `angular_acceleration.csv` | Percepatan sudut |
​
### Tooling Analisis
​
Skrip berikut berada **di luar workspace ROS 2** dan digunakan untuk evaluasi *post-processing*:
​
| Skrip / Folder | Fungsi |
|---|---|
| `eval_harness_v2.py` | Evaluasi menyeluruh RMSE, jitter, dan statistik galat |
| `export_gt_synced.py` | Sinkronisasi temporal rekaman sistem dengan ground truth |
| `analyze_latency.py` | Analisis distribusi latensi pipeline |
| `calib_targets.py` | Pembangkit titik kalibrasi D-optimal |
| `fit_bias.py` | Regresi least-squares untuk matriks affine `M` dan offset `b` |
| `demo_pameran/` | Visualisasi 3D real-time (Three.js + roslibjs, via rosbridge) |
| `demo_pameran_offline/` | Versi offline dari demo, memutar ulang rekaman CSV |
​
### Transformasi Koordinat OptiTrack → Sistem
​
```
x_sys = x_opti − 0,646
y_sys = 3,425 − z_opti
z_sys = y_opti
```
​
> [!WARNING]
> Terdapat **inkonsistensi konstanta** antar-skrip: `ips_analysis_common.py` menggunakan `0,637` dan `3,426`, sedangkan nilai `gt_marker_idx` yang beredar bervariasi (`0`, `1`, atau `2`). Verifikasi konstanta yang benar sebelum melakukan evaluasi kuantitatif, karena selisih ini langsung menggeser seluruh perhitungan RMSE.
​
---
​
## 🩺 Troubleshooting
​
<details>
<summary><b>Posisi terkompensasi meleset ratusan milimeter</b></summary>
​
Kalibrasi bias affine sudah kedaluwarsa atau tidak sesuai dengan arena saat ini. Lakukan **re-kalibrasi** mengikuti [Tahap 2](#tahap-2--bias-affine-koreksi-global-pasca-filter). Pastikan juga sistem sudah melewati masa pemanasan.
​
</details>
​
<details>
<summary><b>Topic <code>/state/position</code> tidak pernah muncul</b></summary>
​
Node estimator belum menerima konfigurasi anchor. Periksa apakah topic konfigurasi sudah terisi:
​
```bash
ros2 topic echo /uwb/anchor_config --once
```
​
Jika kosong, periksa `anchors.yaml` dan pastikan node `udp_gateway` berjalan.
​
</details>
​
<details>
<summary><b>WoLF mengeluarkan posisi <code>null</code></b></summary>
​
Sama seperti kasus di atas — `anchor_config` belum diterima node `wolf_position`. Restart launch file setelah memastikan konfigurasi anchor termuat.
​
</details>
​
<details>
<summary><b>Layer-1 clip rate melebihi 10%</b></summary>
​
Filter *triangle inequality* terlalu ketat untuk kondisi multipath saat ini. Longgarkan parameter di `system.yaml`:
​
```yaml
layer1_huber_k: 3.0
layer1_sigma_tdoa_m: 0.15
```
​
</details>
​
<details>
<summary><b>Layer-2 reject rate melebihi 5%</b></summary>
​
*Predictive TDoA gate* terlalu agresif. Naikkan ambang di `system.yaml`:
​
```yaml
layer2_gate_threshold: 16.266
```
​
</details>
​
<details>
<summary><b>Posisi "macet" / tidak bergerak padahal drone berpindah</b></summary>
​
Filter *dead-zone suppression* (ZUPT) salah mendeteksi kondisi diam. Nonaktifkan sementara di `system.yaml`:
​
```yaml
dzs_enabled: false
```
​
</details>
​
<details>
<summary><b>Drift posisi lebih dari 30 mm per 10 menit</b></summary>
​
Osilator DWM1000 belum mencapai kesetimbangan termal. Tunggu pemanasan penuh (minimal 10 menit, idealnya 30 menit sebelum kalibrasi).
​
</details>
​
<details>
<summary><b><code>DEV_ID</code> tidak terbaca <code>0xDECA0130</code></b></summary>
​
Modul DWM1000 tidak merespons SPI. Periksa penyolderan jalur SPI (MOSI, MISO, SCK, CS), pastikan tegangan LDO berada di **3,28 – 3,30 V**, dan cek pin `RST` tidak tertahan rendah.
​
</details>
​
<details>
<summary><b>Tidak ada paket UDP yang masuk ke laptop</b></summary>
​
```bash
# Pastikan IP laptop benar
ip addr show | grep 192.168.10
​
# Buka firewall
sudo ufw allow 5555/udp
​
# Sadap lalu lintas untuk memastikan anchor mengirim
sudo tcpdump -i any udp port 5555 -c 20
```
​
</details>
​
---
​
## ⚠️ Batasan & Known Issues
​
> [!CAUTION]
> **Ledakan estimasi WoLF di zona dekat SA5 — OPEN PROBLEM.**
> Pada zona `x > 1,6 m`, `y > 1,6 m`, dan `z < 0,6 m` (area di sekitar anchor SA5), estimator WoLF-EKF dapat menghasilkan lonjakan posisi yang tidak wajar. Penyebabnya belum sepenuhnya teridentifikasi. **Mitigasi:** batasi *flight envelope* pada `z ≥ 0,8 m`.
​
Batasan lain yang perlu diketahui:
​
- **Satu tag pada satu waktu.** Sistem saat ini dirancang untuk melacak satu drone; penambahan tag memerlukan skema penjadwalan blink.
- **Kalibrasi terikat arena.** Setiap perpindahan arena mewajibkan kalibrasi antenna delay dan bias affine dari awal.
- **PDOP 2,26 melebihi ambang ideal (≤ 2).** Secara teoretis galat posisi menjadi ± 13% lebih besar dari kondisi geometri ideal. Perbaikan: tambah jumlah anchor.
- **Akurasi kecepatan translasi belum tercapai** (0,11–0,15 m/s vs target 0,1 m/s). Penyebab: kecepatan hanya dipropagasi lewat matriks transisi tanpa *measurement update*, dan ground truth kecepatan sendiri berisik karena hasil diferensiasi numerik posisi OptiTrack. Perbaikan: *sensor fusion* dengan IMU.
- **Validasi kuantitatif turunan angular tidak tersedia.** OptiTrack hanya menyediakan posisi & orientasi; propagasi galat pada diferensiasi numerik menghasilkan σ yang jauh lebih besar dari besaran yang diukur, sehingga validasi dilakukan secara kualitatif.
- **Bias affine bersifat global.** Model tunggal `M`, `b` tidak dapat menangkap bias yang bervariasi terhadap posisi, sehingga galat mentok di kisaran 130–170 mm.
​
---
​
## 📚 Dokumentasi & Referensi
​
### Dokumentasi Internal
​
| Dokumen | Lokasi | Isi |
|---|---|---|
| **Panduan IPS ROS 2 v2.5** | `software/PANDUAN_IPS_ROS2_v2_5.md` | Panduan operasional lengkap 16 bab — arsitektur, parameter, kalibrasi, troubleshooting mendalam |
| **Konfigurasi sistem** | `software/ips_jazzy_ws/src/ips_bringup/config/system.yaml` | Seluruh parameter filter dan pipeline |
| **Konfigurasi anchor** | `software/ips_jazzy_ws/src/ips_bringup/config/anchors.yaml` | Koordinat anchor dan dimensi arena |
​
### Buku Tugas Akhir
​
 Sistem ini merupakan hasil kerja kolaboratif tiga tugas akhir yang saling melengkapi:
​
1. **Amar, P. Q. D. (2026).** *Desain dan Implementasi Sistem Indoor Positioning Berbasis Time Difference of Arrival dengan Ultra-Wideband: Perangkat Keras dan Integrasi Top-Level.* Tugas Akhir Program Sarjana, Institut Teknologi Bandung.
   → Perancangan PCB tag & anchor, firmware, dan integrasi pipeline ROS 2.
​
2. **Putri, K. I. (2026).** *TDoA Positioning Calculation pada Sistem Indoor Positioning.* Tugas Akhir Program Sarjana, Institut Teknologi Bandung.
   → Algoritma estimasi posisi: Chan, Kalman Filter, WoLF-EKF, penanganan outlier, dan ZUPT.
​
3. **Chandra, M. W. (2026).** *Implementasi Sinkronisasi Wireless Clock Ultra-wideband dan Inertial Measurement Unit pada Sistem Indoor Positioning.* Tugas Akhir Program Sarjana, Institut Teknologi Bandung.
   → Sinkronisasi clock nirkabel (interpolasi linear + KF 3-state) dan pemrosesan data IMU.
​
### Referensi Eksternal
​
- [ROS 2 Jazzy Jalisco Documentation](https://docs.ros.org/en/jazzy/)
- [Qorvo DWM1000 Module Datasheet](https://www.qorvo.com/products/p/DWM1000)
- [Bosch BNO055 Datasheet](https://www.bosch-sensortec.com/products/smart-sensor-systems/bno055/)
- [PlatformIO Documentation](https://docs.platformio.org/)
- Duran-Martin, G. et al. (2024). *Outlier-robust Kalman Filtering through Generalised Bayes* — dasar teoretis WoLF.
​
---
​
## 🤝 Kontributor
​
**Institusi** : Program Studi Sarjana Teknik Elektro — Sekolah Teknik Elektro dan Informatika (STEI), Institut Teknologi Bandung
**Tahun** : 2026
​
**Anggota Tim** :
​
| Nama | NIM | Sub-sistem yang Dikerjakan |
|---|:--:|---|
| Priya Qolbu Dhiya'an Amar | 13222079 | Perangkat keras (tag & anchor) + integrasi top-level |
| Kania Ika Putri | 13222041 | TDoA positioning calculation (estimasi posisi & kecepatan) |
| Matthew Wijaya Chandra | 13222020 | Sinkronisasi wireless clock UWB + pemrosesan data IMU |
​
**Dosen Pembimbing** :
​
1. Anggera Bayuwindra, S.T., M.T., Ph.D.
2. Ishak Hilton Pujantoro Tnunay, S.T., Ph.D.
​
---
​
## 📄 Lisensi
​
Distributed under the MIT License. Lihat berkas `LICENSE` untuk informasi selengkapnya.
​
---
​
<div align="center">
  <sub>Dibuat dengan 📡 di Laboratorium Robotika STEI — Institut Teknologi Bandung</sub>
</div>
​
Notion AI finished.
PANDUAN_IPS_ROS2_v2_5 .md
