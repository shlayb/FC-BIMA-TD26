# MISSION READ TIMEOUT - DEBUG GUIDE

## Problem
Mission Planner timeout saat click "Read Waypoints":
```
Timeout on read - getWPCount
```

## Root Causes & Fixes Applied

### ✅ Fix #1: Serial Buffer Flushing
**Problem**: Mavlink message tidak dikirim dengan baik di telemetri 57600 baud
**Solution**: 
- Explicit `flush()` setelah `write()` untuk semua MISSION messages
- Prioritas kirim ke Serial2 (telemetri) utama
- Ensure message delivery dengan proper buffering

### ✅ Fix #2: Target System Parameter
**Problem**: MISSION_COUNT dikirim ke wrong target
**Solution**:
- Use `msg.sysid` (GCS yang request) sebagai target
- Fallback ke broadcast (0) jika sysid=0
- Proper component ID: `MAV_COMP_ID_MISSIONPLANNER`

### ✅ Fix #3: Validation & Debug Output
**Problem**: Tidak bisa trace posisi failure
**Solution**:
- Added extensive debug printf untuk track:
  - MISSION_REQUEST_LIST received
  - wp_sum value
  - MISSION_COUNT sent (dengan byte count)
  - MISSION_REQUEST seq values
  - MISSION_ITEM sent status

## Testing Steps

### Step 1: Verify wp_sum Initialization
Saat FC startup, serial monitor seharusnya menunjukkan:
```
[System info...]
flwp = 0    <-- flag_wp initial
```

**To confirm**:
```
// Add ini di printUSB() atau setup()
Serial.printf("Setup complete: wp_sum=%d, wp_array populated\n", wp_sum);
```

### Step 2: Connect Mission Planner ke FC
1. USB Serial @ 115200 OR
2. Telemetry @ 57600

**Verify connection**:
- Status di MP should show: "Connected" 
- Heartbeat visible di serial monitor

### Step 3: Click "Read Waypoints" & Monitor Serial
Look for output:
```
>>> MISSION_REQUEST_LIST received
    Requester: sys=254, comp=0      <-- MP
    Target: sys=1, comp=0           <-- FC
    Current wp_sum=4                <-- CRITICAL!
    Sending MISSION_COUNT with target=254

DEBUG: sendMissionCount called with wp_sum=4, target_system=254
Mission count sent: 4 waypoints (len=31 bytes)
```

**If wp_sum=0**:
- Problem: Waypoints tidak terinisialisasi di wp_setup()
- Check: apakah `add_waypoint()` function bekerja
- Also check: Locations struct size & alignment issues

### Step 4: Monitor MISSION_REQUEST Sequence
Expected setelah MISSION_COUNT:
```
>>> MISSION_REQUEST seq=0 received
    Sending WP#0
Sent WP#0: lat=-7.7738260, lng=110.3784420, alt=2.00 m (len=85)

>>> MISSION_REQUEST seq=1 received
    Sending WP#1
...

>>> MISSION_REQUEST seq=4 received
    Mission download complete (seq=4 == wp_sum=4)
```

## Troubleshooting Nodes

### Scenario A: "MISSION_REQUEST_LIST received" TIDAK muncul
**Cause**: MAVLink message tidak terima di FC
**Action**:
1. Check USB/Telemetri connection
2. Verify baud rate: USB=115200, Telemetri=57600
3. Test di terminal lain dengan minicom/putty
4. Check if `handlePorts()` task running

### Scenario B: MISSION_COUNT sent tapi MP timeout
**Cause**: Message rusak atau target_system wrong
**Action**:
1. Monitor message length - seharusnya 31 bytes
2. Check kalau ada compiler warning tentang mavlink packing
3. Try broadcast (target=0) instead of specific sysid
4. **Specific**: Ensure MAVLink version compatibility

### Scenario C: wp_sum = 0
**Cause**: `add_waypoint()` function tidak bekerja atau wp_setup() tidak called
**Action**:
```cpp
// Di wp_setup(), after hardcoded add_waypoint calls:
Serial.printf("wp_setup complete: wp_sum=%d\n", wp_sum);

// Di setup(), after wp_setup():
Serial.printf("After wp_setup: wp_sum=%d, waypoint[0] lat=%ld\n", 
              wp_sum, waypoint[0].lat);
```

### Scenario D: MISSION_REQUEST seq mismatch
**Cause**: Out-of-order requests atau sequence skip
**Action**:
- Ensure FC handles seq=0 first
- Validate all seq values < wp_sum
- Send ACK properly setelah semua WP

## Serial Output Template

Saat read waypoints berjalan normal:
```
[Connection established]
>>>MISSION_REQUEST_LIST received
    Requester: sys=254, comp=0
    Target: sys=1, comp=0
    Current wp_sum=4
    Sending MISSION_COUNT with target=254
DEBUG: sendMissionCount called wp_sum=4, target_system=254
Mission count sent: 4 waypoints (len=31 bytes)
>>> MISSION_REQUEST seq=0 received
    Sending WP#0
Sent WP#0: lat=-7.7738260, lng=110.3784420, alt=2.00 m (len=85)
>>> MISSION_REQUEST seq=1 received
    Sending WP#1
Sent WP#1: lat=-7.7732600, lng=110.3786320, alt=2.00 m (len=85)
>>> MISSION_REQUEST seq=2 received
    Sending WP#2
Sent WP#2: lat=-7.7746550, lng=110.3782450, alt=2.00 m (len=85)
>>> MISSION_REQUEST seq=3 received
    Sending WP#3
Sent WP#3: [data...]
>>> MISSION_REQUEST seq=4 received
    Mission download complete (seq=4 == wp_sum=4)
[MP menampilkan 4 waypoints]
```

## Next Debug Steps

If still timeout after fixes:
1. **Packet sniffer**: Gunakan tool untuk capture MAVLink packets
2. **Mission Planner log**: Enable logging untuk detailed trace
3. **Timing issue**: Add delay antara messages jika buffer overflow
4. **FCB connection**: Test dengan kabel USB langsung (bukan telemetri)

---
**Last Updated**: Feb 2026
**Status**: Fixes Applied ✅
