# Khageswara FC-2026 — Panduan Pengujian HITL & TECS

Panduan ini menjelaskan cara menjalankan pengujian **Hardware-In-The-Loop (HITL)** pada Teensy 4.1 untuk memvalidasi:
1. **Auto Mode dengan Full Groundspeed** (Non-Airspeed Mode tanpa Pitot Tube fisik).
2. **Cascaded Control Architecture** (Inner-loop FBWA PID dipakai langsung oleh Outer-loop TECS & L1).
3. **Respon Throttle & Pitch TECS yang Mulus** (Anti-Jerk, Slew Rate Limiter, Anti-Windup).

---

## Persiapan & Langkah Menjalankan

### Langkah 1: Aktifkan Mode HITL di Firmware Teensy
Buka file `include/FW_config.h`:
```cpp
#define HITL_ENABLED 1   // Ubah dari 0 ke 1
```
Lalu flash/upload firmware ke Teensy 4.1 menggunakan PlatformIO:
```bash
pio run -t upload
```

### Langkah 2: Install Dependensi Python di Laptop
Buka terminal dan jalankan:
```bash
pip install -r tools/hitl_sim/requirements.txt
```

### Langkah 3: Hubungkan Teensy ke Laptop via USB dan Jalankan Simulator

```bash
python tools/hitl_sim/tecs_hitl_sim.py
```
*(Script akan mendeteksi port `/dev/ttyACM0` atau `COMx` secara otomatis).*

---

## Parameter Uji (CLI Options)

| Perintah | Deskripsi | Target Uji |
|---|---|---|
| `python tecs_hitl_sim.py` | Mode default: **Full Groundspeed** (Pitot dinonaktifkan). | **Uji Poin 1**: Membuktikan auto mode terbang stabil murni dari GPS Groundspeed & Baro. |
| `python tecs_hitl_sim.py --wind 5.0` | Injeksi angin sakal (headwind 5 m/s). | **Uji Poin 1 & 3**: Membuktikan groundspeed safety floor & kompensasi throttle TECS. |
| `python tecs_hitl_sim.py --pitot` | Mengaktifkan virtual pitot sensor (Airspeed Mode). | Membandingkan performa Non-Airspeed vs Airspeed TECS. |
| `python tecs_hitl_sim.py -p /dev/ttyACM1` | Menentukan port serial secara spesifik. | Konfigurasi port USB. |

---

## 4 Grafik yang Muncul di Live Dashboard

1. **Subplot 1 (Ketinggian & Climb Rate)**:
   - Garis hijau: Ketinggian aktual pesawat ($h$).
   - Garis biru putus-putus: Target ketinggian dari waypoint ($h_{target}$).
   - Garis magenta: Laju pendakian ($\dot{h}$).
   - *Evaluasi*: Pastikan pesawat menanjak dan meluncur turun secara proporsional saat waypoint berubah (50m $\to$ 80m $\to$ 40m $\to$ 60m).

2. **Subplot 2 (Kecepatan Udara & Groundspeed)**:
   - Garis kuning: Airspeed TAS ($V_{TAS}$).
   - Garis putih: GPS Groundspeed ($V_{gnd}$).
   - *Evaluasi*: Dalam mode Full Groundspeed, kecepatan terjaga di sekitar 14 m/s tanpa drop ke stall speed.

3. **Subplot 3 (Tracking Sudut Pitch)**:
   - Garis hijau: Sudut Pitch aktual pesawat ($\theta$).
   - Garis merah putus-putus: Demanded Pitch dari TECS ($\theta_{dem}$).
   - *Evaluasi*: Membuktikan Poin 2; inner-loop FBWA PID mampu men-tracking setpoint TECS tanpa osilasi pitch hunting.

4. **Subplot 4 (Daya Motor Throttle & Energy Error)**:
   - Garis oranye: Throttle motor yang dieksekusi ($0 - 100\%$).
   - Garis biru: Specific Total Energy Error ($\Delta STE$).
   - *Evaluasi*: Membuktikan Poin 3; pergerakan throttle halus (*smooth ramp*), tidak ada spiking / chattering.

