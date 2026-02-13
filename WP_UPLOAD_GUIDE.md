# Waypoint Upload via MAVLink Guide

## Overview
OTA (On-The-Air) waypoint upload telah diimplementasikan di FC Anda. Anda bisa upload mission langsung dari GCS tanpa perlu flash firmware ulang.

## Features
- ✅ Upload waypoint via telemetri (Serial2)
- ✅ Support MISSION_COUNT dan MISSION_ITEM messages
- ✅ Auto-request waypoint one-by-one untuk reliability
- ✅ Timeout detection (5 detik per upload)
- ✅ Serial monitoring untuk debug

## Setup & Penggunaan

### 1. Hardware Koneksi
- **Telemetri**: Serial2 @ 57600 baud
- **USB**: Serial @ 115200 baud  
- **Raspberry Pi**: Serial7

### 2. GCS yang Didukung
**QGroundControl** (Recommended):
1. Download: https://qgroundcontrol.com/
2. Buka aplikasi
3. Connect ke Vehicle (USB atau Telemetry)
4. Masuk ke **Plan View**
5. Add waypoints sesuai kebutuhan
6. Click **Upload to Vehicle** (saat terbang atau udara)

**Mission Planner**:
1. Download: https://ardupilot.org/planner/
2. Connect to Vehicle
3. Buka **Flight Plan**
4. Add waypoints
5. Click **Write WayPoints**

### 3. How It Works
1. GCS mengirim `MISSION_COUNT` message dengan jumlah waypoint
2. FC membalas dengan `MISSION_REQUEST` untuk WP#0
3. GCS kirim `MISSION_ITEM` dengan data WP#0
4. FC simpan ke `waypoint[0]` array
5. FC request WP#1, dan seterusnya...
6. Setelah semua waypoint diterima, FC kirim `MISSION_ACK`
7. GCS menampilkan status berhasil

### 4. Monitoring via Serial Monitor
Buka **Serial Monitor @ 115200 baud**:
```
Mission upload started: expecting 5 waypoints
Requesting WP#0
WP#0 received: lat=-7.1234567, lng=110.9876543, alt=100.00 m
Requesting WP#1
WP#1 received: lat=-7.1245678, lng=110.9845678, alt=105.00 m
...
Mission upload complete! Total waypoints: 5
Mission ACK sent (type=0)
```

### 5. Waypoint Storage
- **Array Size**: 100 waypoints max
- **Storage Type**: RAM (hilang saat reset/power off)
- **Data Format**:
  - `lat`, `lng`: degrees × 1e7 (e.g., -7123456 = -7.123456°)
  - `alt`: centimeters (e.g., 10500 = 105 m)

### 6. Testing Steps
1. **Build & Upload firmware** ke FC
2. **Connect telemetri** ke GCS
3. **Buka GCS** (QGroundControl atau Mission Planner)
4. **Create mission** dengan beberapa waypoints
5. **Click "Upload to Vehicle"**
6. **Monitor serial console** untuk status
7. **Verify** di serial monitor atau GCS

### 7. Implementasi di Navigation
Setelah upload berhasil, gunakan waypoint di auto mode:
```cpp
// Di Mode_Manager atau AutoMode
if(auto_state.wp_reached) {
    set_next_WP(waypoint[flag_wp++]);  // Pindah ke waypoint berikutnya
}
```

## Troubleshooting

### Upload Gagal / Timeout
- ❌ Check koneksi telemetri
- ❌ Check baud rate (57600 untuk telemetri)
- ❌ Cek serial monitor untuk error message

### Waypoint Tidak Tersimpan
- ❌ Check jumlah waypoint < 100
- ❌ Verify `wp_sum` di serial monitor
- ❌ Reset FC dan coba lagi

### GCS Tidak Detect FC
- ❌ Check USB/Telemetri connected
- ❌ Verifikasi SYSTEM_ID = 1
- ❌ Cek mavlink version compatibility

## MAVLink Messages Used
- **MISSION_COUNT** (44): GCS → FC (jumlah WP)
- **MISSION_ITEM** (39): GCS → FC (detail WP)
- **MISSION_ITEM_INT** (73): GCS → FC (detail WP, format baru)
- **MISSION_REQUEST** (40): FC → GCS (request WP)
- **MISSION_ACK** (47): FC → GCS (completion status)

## Code Structure
- **File**: `include/Mavlink.h`
- **Class**: `MavlinkHandler`
- **Methods**:
  - `handleMissionCount()` - Process MISSION_COUNT
  - `handleMissionItem()` - Process MISSION_ITEM
  - `sendMissionRequest()` - Request next WP
  - `sendMissionAck()` - Acknowledge completion
  - `processAsFC()` - Route messages

## Next Steps (Optional)
1. **EEPROM Storage**: Simpan waypoint ke EEPROM agar persist setelah reset
2. **Parameter Upload**: Implement parameter list (PARAM_REQUEST_LIST)
3. **Mission Download**: Implement MISSION_REQUEST_LIST untuk download mission dari FC
4. **Geofence**: Implement geofence upload 

---
**Last Updated**: Feb 2026
**Status**: Working & Tested ✅
