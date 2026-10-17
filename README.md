# Konoha Driver - Turtlesim Menggambar

Node ROS 2 dengan C++, rclcpp yang menggerakkan turtle di turtlesim untuk menggambar tulisan
KONOHAGAKURE dan logo konoha

## 1. Ringkasan Komunikasi ROS 2

| Jenis | Nama | Tipe | Peran | Fungsi |
|-------|------|------|-------|--------|
| **Topic** | `/turtle1/cmd_vel` | `geometry_msgs/msg/Twist` | Publisher | Kirim kecepatan (linear.x, angular.z) ke turtle |
| **Topic** | `/turtle1/pose` | `turtlesim/msg/Pose` | Subscriber | Baca posisi (x, y) & arah (theta) turtle |
| **Service** | `/turtle1/set_pen` | `turtlesim/srv/SetPen` | Client | Angkat/turunkan pen |
| **Action** | — | — | tidak dipakai | Putaran dilakukan manual via `angular.z` |
| **Parameter** | — | — | tidak dipakai | Konfigurasi pakai `constexpr` |

### Diagram hubungan

```
                    publish Twist                  +------------------+
  +-----------+  ---------------------------->     |                  |
  |  konoha_  |       /turtle1/cmd_vel             |                  |
  |  driver   |                                    |    turtlesim     |
  |  (node)   |  <----------------------------     |   (turtlesim_    |
  |           |    subscribe Pose                  |      node)       |
  |           |       /turtle1/pose                |                  |
  |           |                                    |                  |
  |           |  ---------------------------->     |                  |
  +-----------+    request SetPen (service)        +------------------+
                    /turtle1/set_pen
```
### Penjelasan Tiap Komunikasi

**🔵 Topic `/turtle1/cmd_vel` (Publisher, Twist)**
- Node → turtlesim, ~50 Hz (tiap 20 ms)
- Pakai field `linear.x` (maju, max 2.0) & `angular.z` (putar, max 10.0)
- Publish `Twist()` kosong saat job selesai → turtle berhenti

**🔵 Topic `/turtle1/pose` (Subscriber, Pose)**
- Turtlesim → node, ~60 Hz
- Ambil `x`, `y`, `theta` — dipakai buat hitung error sudut di pure pursuit
- **Flag `got_pose_`**: node nunggu pose pertama dulu baru mulai gerak

**🟢 Service `/turtle1/set_pen` (Client, SetPen)**
- Node → turtlesim, 1× per job (47 job total)
- Request: `r=g=b=255` (putih), `width=3`, `off=!down` (1=angkat, 0=turun)
- Dipanggil pakai `async_send_request` (non-blocking)
- State `WAIT_PEN` nunggu callback, baru lanjut `DRIVE`

**🟡 Action: Tidak Dipakai**
- Turtlesim punya `/turtle1/rotate_absolute` (muter ke sudut tertentu)

**🟣 Parameter: Tidak Dipakai**
- Semua config pakai `constexpr` (compile-time)


## 2. Alur Program

```
buildStrokes()  ->  densify()  ->  daftar Job  ->  tick() tiap 20 ms
(titik gambar)     (rapatkan)     (naik/turun)     SET_PEN -> WAIT_PEN -> DRIVE -> (Job berikut)
```

### a. Membuat goresan (`buildStrokes`)
Gambar dibagi menjadi **goresan** (stroke), yaitu satu garis sambung tanpa angkat pen.
Totalnya 23 goresan:
- **Teks "KONOHAGAKURE"**: 12 huruf, 21 goresan, dibuat oleh `letter()`. Huruf dihitung dari
  lebar `TW`, tinggi `TH`, dan jarak `TP`, lalu dipusatkan di tengah kanvas. Huruf lengkung (O, G, U, R)
  memakai `arc()`.
- **Daun**: 1 goresan (ekor + busur luar), dari titik hasil tracing gambar.
- **Spiral**: 1 goresan, juga dari titik hasil tracing.

Titik tracing berupa koordinat piksel, lalu dikonversi ke koordinat turtlesim oleh `fromPixels()`
(digeser ke tengah kanvas dan diskalakan `IMG_SCALE = 0.030`, sumbu y dibalik).

### b. Merapatkan titik (`densify`)
Tiap segmen dipecah supaya jarak antar titik tidak lebih dari `STEP = 0.10`,
agar turtle mengikuti garis lurus dengan mulus.

### c. Menyusun pekerjaan (`Job`)
Tiap goresan menjadi **2 Job**:
1. **Pindah**: pen **naik**, turtle menuju titik awal goresan.
2. **Gambar**: pen **turun**, turtle mengikuti seluruh titik goresan.

Ditambah 1 Job terakhir: pen naik, turtle menuju posisi akhir `(FIN_X, FIN_Y)`.
Total = 23 x 2 + 1 = 47 Job.

### d. Menjalankan Job (state machine di `tick`)

| State | Yang dilakukan | Pindah ke |
|-------|----------------|-----------|
| `SET_PEN` | Memanggil service `set_pen` (naik atau turun sesuai Job) | `WAIT_PEN` |
| `WAIT_PEN` | Menunggu respon service | `DRIVE` (diatur oleh callback) |
| `DRIVE` | Mengikuti jalur dengan `pursue()` | `SET_PEN` (Job berikut) |

Saat Job terakhir selesai, node mencetak `=== SELESAI ===` dan berhenti mengirim perintah.

### e. Mengikuti jalur (`drive` + `pursue`, metode *pure pursuit*)
1. **Look-ahead**: titik di jalur yang jaraknya kurang dari `LOOK = 0.20` dari turtle dilewati,
   sehingga target selalu titik di depan turtle.
2. **Hitung error sudut** antara arah hadap turtle dan arah ke target.
3. **Pilih gerakan**:
   - Error sudut besar (> 0.8 rad): **putar di tempat** (`linear.x = 0`).
   - Error sudut kecil: **maju sambil belok**. Kecepatan maju makin kecil kalau sudut makin besar
     (`VMAX * cos^3(a)`), dan kecepatan putar mengikuti kelengkungan ke target.
4. **Di titik terakhir**, kecepatan diperlambat mendekati target.
   Job dianggap selesai bila jarak ke titik terakhir kurang dari `TOL = 0.04`.

---

## 3. Konfigurasi (konstanta di kode)

| Konstanta | Nilai | Arti |
|-----------|-------|------|
| `STEP` | 0.10 | Jarak maksimum antar titik jalur |
| `LOOK` | 0.20 | Jarak look-ahead pure pursuit |
| `TOL` | 0.04 | Toleransi dianggap sampai di titik akhir |
| `VMAX` | 2.0 | Kecepatan linear maksimum |
| `WMAX` | 10.0 | Kecepatan sudut maksimum |
| `CX` | 5.544 | Tengah kanvas turtlesim (sumbu x) |
| `TW`, `TH` | 0.6, 1.2 | Lebar dan tinggi huruf |
| `TY` | 7.0 | Posisi y bawah huruf |
| `TP` | 0.85 | Jarak antar huruf |
| `IMG_CY`, `IMG_SCALE` | 4.8, 0.030 | Pusat y dan skala gambar daun |
| `FIN_X`, `FIN_Y` | 5.544, 1.5 | Posisi akhir turtle |

---

## 4. Struktur Kode

| Bagian | Isi |
|--------|-----|
| Geometri | `Pt`, `Path`, `append()`, `arc()`, `densify()` |
| Gambar | `letter()`, `fromPixels()`, `buildStrokes()` |
| Kontrol | `pursue()` untuk menghitung kecepatan menuju target |
| Node | `Job`, kelas `Konoha` (publisher, subscriber, client, timer, state machine) |

---

## 5. Cara Menjalankan

**Dependensi** (`package.xml` / `CMakeLists.txt`): `rclcpp`, `geometry_msgs`, `turtlesim`.

```bash
# Terminal 1 - jalankan turtlesim
ros2 run turtlesim turtlesim_node

# Terminal 2 - build dan jalankan node ini (ganti <nama_package> sesuai package Anda)
colcon build
source install/setup.bash
ros2 run <nama_package> konoha_driver
```

Opsional, untuk memeriksa komunikasi saat program berjalan:

```bash
ros2 node info /konoha_driver      # melihat topic dan service yang dipakai node
ros2 topic echo /turtle1/pose      # memantau posisi turtle
ros2 topic hz /turtle1/cmd_vel     # memeriksa frekuensi perintah (sekitar 50 Hz)
rqt_graph                          # melihat diagram node dan topic
```

vid penjelasan: https://drive.google.com/file/d/19mXsdFexaXNhL30ZbEb3RUaENJl25v3Q/view?usp=sharing