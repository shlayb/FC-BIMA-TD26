#!/usr/bin/env python3
"""
=============================================================================
  KHAGESWARA FC-2026 — HITL (Hardware-In-The-Loop) Simulator & TECS Tester
=============================================================================
  Simulasi 6-DOF Longitudinal & Lateral Fixed-Wing Aerodynamics Bridge
  untuk memvalidasi:
    1. Auto Mode Full Groundspeed (Non-Airspeed Mode tanpa Pitot Sensor)
    2. Cascaded Control Architecture (Outer L1/TECS -> Inner FBWA PID)
    3. Smoothness, Throttle Slew, Anti-Windup & Anti-Jerk TECS
=============================================================================
"""

import sys
import time
import math
import struct
import argparse
import threading
from collections import deque

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("[ERROR] pyserial belum terinstall. Install dengan: pip install pyserial")
    sys.exit(1)

try:
    import numpy as np
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
except ImportError:
    print("[ERROR] matplotlib/numpy belum terinstall. Install dengan: pip install matplotlib numpy")
    sys.exit(1)

# =============================================================================
#  BINARY PROTOCOL DEFINITIONS (Matching HITL_Bridge.h)
# =============================================================================
HITL_MAGIC_0 = 0xAA
HITL_MAGIC_1 = 0x55

# struct HITL_SensorPacket:
# uint8 magic0, magic1, uint32 timestamp_ms
# float roll_deg, pitch_deg, yaw_deg
# float gyro_x, gyro_y, gyro_z
# float accel_x, accel_y, accel_z
# float baro_alt_m, climb_rate, tas_mps, eas_mps
# int32 lat_1e7, lon_1e7, float gps_alt_m, groundspeed_mps
# float vel_n_mps, vel_e_mps, vel_d_mps
# uint8 flags, uint16 checksum
SENSOR_FMT = "<BB I fff fff fff ffff ii fffff B H"
SENSOR_PKT_SIZE = struct.calcsize(SENSOR_FMT)

# struct HITL_ActuatorPacket:
# uint8 magic0, magic1, uint32 timestamp_ms
# int16 servo_ail_R, servo_ail_L, servo_ele, servo_rud
# uint16 throttle_pwm
# float demanded_pitch_deg, demanded_roll_deg, demanded_throttle_pct
# float tecs_integ_seb, tecs_integ_thr, tecs_ste_error, tecs_target_alt_m
# uint8 current_mode, is_armed, uint16 checksum
ACTUATOR_FMT = "<BB I hhhh H fff ffff BB H"
ACTUATOR_PKT_SIZE = struct.calcsize(ACTUATOR_FMT)

def calc_fletcher16(data: bytes) -> int:
    """Menghitung Checksum Fletcher-16 (sama dengan implementasi Teensy)."""
    sum1 = 0
    sum2 = 0
    for b in data:
        sum1 = (sum1 + b) % 255
        sum2 = (sum2 + sum1) % 255
    return (sum2 << 8) | sum1


# =============================================================================
#  FIXED-WING 6-DOF AERODYNAMIC PHYSICS ENGINE
# =============================================================================
class FixedWingPhysics:
    def __init__(self):
        # Spesifikasi Pesawat Khageswara UAV (Estimasi Fixed-Wing)
        self.mass = 6.0             # kg
        self.S = 0.558              # m^2 (Luas sayap)
        self.b = 2.2             # m (Wingspan)
        self.c = self.S / self.b   # m (Mean Chord)
        self.rho = 1.225           # kg/m^3 (Densitas udara)
        self.g = 9.80665           # m/s^2
        self.Iyy = 0.6166            # kg*m^2 (Inersia Pitch)
        self.Ixx = 0.239            # kg*m^2 (Inersia Roll)
        self.Izz = 0.843            # kg*m^2 (Inersia Yaw)
        self.T_max = 38.0          # N (Max thrust bersih yang tersedia untuk akselerasi & climb)
                                    # Catatan: 45% cruise thr = 12.6N vs drag ~10N di 21m/s → margin tipis utk climb

        # Koefisien Aerodinamika — Dikalibrasi realistis untuk UAV 6kg fixed-wing
        # Target: throttle ~45-50% pada level cruise 21 m/s
        # Referensi: di 21m/s level → Drag ≈ 12.5N → T_max=28N → ~45% thr ✓
        self.CL0 = 0.25
        self.CLa = 5.34             # rad^-1
        self.CD0 = 0.055            # Parasite drag realistis (frame + payload + interference drag)
        self.k_induced = 0.070      # Induced drag factor
        self.Cm0 = 0.0
        self.Cma = -1.20           # Longitudinal static stability
        self.Cmq = -3.50           # Pitch rate damping
        self.Cm_de = 3.50          # Elevator control authority
        self.Cl_p = -3.2           # Roll rate damping
        self.Cl_da = 3.0           # Aileron control authority

        # State Variables
        self.x = 0.0               # North (m)
        self.y = 0.0               # East (m)
        self.altitude = 50.0       # m (Altitude start at 50m)
        self.TAS = 21.0            # m/s (Cruise target airspeed)
        self.climb_rate = 0.0      # m/s
        self.groundspeed = 21.0    # m/s

        # Attitude & Rates
        self.roll = 0.0            # rad
        self.pitch = 0.0           # rad
        self.yaw = 0.0             # rad (Heading)
        self.p = 0.0               # rad/s (Roll rate)
        self.q = 0.0               # rad/s (Pitch rate)
        self.r = 0.0               # rad/s (Yaw rate)

        # Body Accelerations
        self.accel_x = 0.0
        self.accel_y = 0.0
        self.accel_z = -self.g

        # Environment & Wind
        self.wind_north = 0.0      # m/s
        self.wind_east = 0.0       # m/s
        self.wind_down = 0.0       # m/s

        # Home Coordinates (UGM / Lapangan Grha Sabha Pramana)
        self.lat_home = -7.768000 #-7.7738260
        self.lon_home = 110.234000 #110.3784420

    def step(self, dt: float, elevator_cmd: float, aileron_cmd: float, rudder_cmd: float, throttle_pct: float):
        """
        Integrasi fisika pesawat maju dt detik (Stabilized Aero-Kinematic Model).
        elevator_cmd: -1.0 .. +1.0 (defleksi elevator dari -400..400)
        aileron_cmd : -1.0 .. +1.0
        throttle_pct: 0.0 .. 1.0 (persen daya dorong motor)
        """
        # Sanitasi input perintah dari FC
        ele_cmd = float(np.clip(np.nan_to_num(elevator_cmd), -1.0, 1.0))
        ail_cmd = float(np.clip(np.nan_to_num(aileron_cmd), -1.0, 1.0))
        thr_cmd = float(np.clip(np.nan_to_num(throttle_pct), 0.0, 1.0))

        # 1. Respon Dinamika Pitch (Elevator -> Pitch Rate -> Pitch)
        # elevator_cmd > 0 (FC minta Nose UP) -> target pitch rate positif
        q_target = ele_cmd * math.radians(35.0)  # Max 35 deg/s pitch rate
        self.q += (q_target - self.q) * (dt / 0.18)
        self.q = float(np.clip(self.q, math.radians(-40.0), math.radians(40.0)))
        self.pitch += self.q * dt
        self.pitch = float(np.clip(self.pitch, math.radians(-25.0), math.radians(25.0)))

        # 2. Respon Dinamika Roll (Aileron -> Roll Rate -> Roll)
        p_target = ail_cmd * math.radians(60.0)  # Max 60 deg/s roll rate
        self.p += (p_target - self.p) * (dt / 0.15)
        self.p = float(np.clip(self.p, math.radians(-80.0), math.radians(80.0)))
        self.roll += self.p * dt
        self.roll = float(np.clip(self.roll, math.radians(-45.0), math.radians(45.0)))

        # 3. Daya Dorong & Hambatan Udara (Berdasarkan Hukum Aerodinamika Baku)
        Thrust = thr_cmd * self.T_max
        q_dyn = 0.5 * self.rho * (self.TAS ** 2)
        
        # Gaya Angkat (Lift) yang dibutuhkan untuk menopang berat pesawat & banking load factor
        # L = m * g / cos(roll)
        cos_roll = max(math.cos(self.roll), 0.5)
        lift_needed = (self.mass * self.g) / cos_roll
        CL = float(np.clip(lift_needed / max(q_dyn * self.S, 1.0), 0.1, 1.5))
        
        # Drag Polar Baku: CD = CD0 (Parasite Drag) + k * CL^2 (Induced Drag)
        cd_total = self.CD0 + self.k_induced * (CL ** 2)
        Drag = q_dyn * self.S * cd_total
        
        # Flight path angle (gamma)
        gamma = math.asin(np.clip(self.climb_rate / max(self.TAS, 1.0), -0.75, 0.75))
        a_lon = (Thrust - Drag - self.mass * self.g * math.sin(gamma)) / self.mass
        self.TAS += a_lon * dt
        self.TAS = float(np.clip(self.TAS, 5.0, 40.0))

        # 4. Ketinggian & Climb Rate (Kinematis sesuai Pitch dan Daya)
        # Steady state climb rate: V * sin(pitch - trim_alpha)
        trim_alpha = math.radians(2.5)
        target_climb = self.TAS * math.sin(self.pitch - trim_alpha)
        self.climb_rate += (target_climb - self.climb_rate) * (dt / 0.25)
        self.climb_rate = float(np.clip(self.climb_rate, -6.0, 6.0))
        self.altitude += self.climb_rate * dt
        self.altitude = float(np.clip(self.altitude, 0.0, 2000.0))

        # 5. Navigasi Sumbu Datar (Heading & Position)
        yaw_rate = (self.g / max(self.TAS, 5.0)) * math.tan(self.roll)
        self.r = float(np.clip(yaw_rate, -1.0, 1.0))
        self.yaw += self.r * dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        v_n = float(self.TAS * math.cos(self.pitch) * math.cos(self.yaw) + self.wind_north)
        v_e = float(self.TAS * math.cos(self.pitch) * math.sin(self.yaw) + self.wind_east)
        v_d = float(-self.climb_rate + self.wind_down)

        self.groundspeed = float(math.sqrt(v_n ** 2 + v_e ** 2))
        self.x += v_n * dt
        self.y += v_e * dt

        # Akselerasi Sensor IMU
        self.accel_x = float(a_lon)
        self.accel_y = float(self.TAS * self.r)
        self.accel_z = float(-self.g * math.cos(self.roll))

        return v_n, v_e, v_d

    def get_lat_lon(self):
        """Konversi pergeseran lokal (x=North, y=East) dalam meter ke koordinat GPS."""
        d_lat = self.x / 111132.954
        d_lon = self.y / (111132.954 * math.cos(math.radians(self.lat_home)))
        return float(self.lat_home + d_lat), float(self.lon_home + d_lon)


# =============================================================================
#  HITL SIMULATION BRIDGE RUNNER
# =============================================================================
class HITLSimRunner:
    def __init__(self, port: str, baud: int = 115200, enable_pitot: bool = False):
        self.port = port
        self.baud = baud
        self.enable_pitot = enable_pitot
        self.physics = FixedWingPhysics()
        self.ser = None
        self.running = False
        self.lock = threading.Lock()

        # Telemetry Buffers for Live Plotting (History window)
        self.buf_len = 300
        self.t_hist = deque(maxlen=self.buf_len)
        self.alt_hist = deque(maxlen=self.buf_len)
        self.alt_tgt_hist = deque(maxlen=self.buf_len)
        self.climb_hist = deque(maxlen=self.buf_len)
        self.tas_hist = deque(maxlen=self.buf_len)
        self.gndspd_hist = deque(maxlen=self.buf_len)
        self.pitch_hist = deque(maxlen=self.buf_len)
        self.pitch_dem_hist = deque(maxlen=self.buf_len)
        self.roll_hist = deque(maxlen=self.buf_len)
        self.roll_dem_hist = deque(maxlen=self.buf_len)
        self.thr_pct_hist = deque(maxlen=self.buf_len)
        self.thr_dem_hist = deque(maxlen=self.buf_len)
        self.ste_err_hist = deque(maxlen=self.buf_len)
        self.ele_pwm_hist = deque(maxlen=self.buf_len)

        # Current Actuator Command from Teensy
        self.curr_elevator = 0.0   # -1.0 .. 1.0
        self.curr_aileron = 0.0
        self.curr_rudder = 0.0
        self.curr_throttle_pct = 0.45 # Initial 45%
        self.curr_dem_pitch = 0.0
        self.curr_dem_roll = 0.0
        self.curr_dem_thr = 45.0
        self.curr_target_alt = 50.0
        self.curr_active_wp = 1
        self.curr_ste_err = 0.0
        self.mode_str = "AUTO"
        self.is_armed = False
        self.arm_cmd = True  # Status perintah arming (Default: True untuk HITL)

        # Stats
        self.rx_count = 0
        self.tx_count = 0
        self.start_time = time.time()

    def arm(self):
        with self.lock:
            self.arm_cmd = True
        print("\n[HITL] Perintah: ARM")

    def disarm(self):
        with self.lock:
            self.arm_cmd = False
        print("\n[HITL] Perintah: DISARM")

    def toggle_arm(self):
        with self.lock:
            self.arm_cmd = not self.arm_cmd
            target = "ARM" if self.arm_cmd else "DISARM"
        print(f"\n[HITL] Perintah: Toggle -> {target}")

    def connect(self):
        print(f"[HITL] Menghubungkan ke Teensy pada {self.port} ({self.baud} baud)...")
        self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
        time.sleep(0.5)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        print(f"[HITL] Terhubung berhasil!")

    def pack_sensor_packet(self, v_n, v_e, v_d) -> bytes:
        lat, lon = self.physics.get_lat_lon()
        now_ms = int((time.time() - self.start_time) * 1000) & 0xFFFFFFFF

        # Flags: bit0 = airspeed_valid, bit1 = arming (1/0), bit2 = force_auto (1)
        flags = 0x04  # Force AUTO
        if self.arm_cmd:
            flags |= 0x02
        if self.enable_pitot:
            flags |= 0x01

        # Sanitasi tipe data & nilai float agar tidak pernah overflow
        roll_deg  = float(np.clip(np.nan_to_num(math.degrees(self.physics.roll)), -90.0, 90.0))
        pitch_deg = float(np.clip(np.nan_to_num(math.degrees(self.physics.pitch)), -85.0, 85.0))
        yaw_deg   = float(np.nan_to_num(math.degrees(self.physics.yaw)) % 360.0)
        p_dps     = float(np.clip(np.nan_to_num(math.degrees(self.physics.p)), -300.0, 300.0))
        q_dps     = float(np.clip(np.nan_to_num(math.degrees(self.physics.q)), -300.0, 300.0))
        r_dps     = float(np.clip(np.nan_to_num(math.degrees(self.physics.r)), -300.0, 300.0))
        acc_x     = float(np.clip(np.nan_to_num(self.physics.accel_x), -50.0, 50.0))
        acc_y     = float(np.clip(np.nan_to_num(self.physics.accel_y), -50.0, 50.0))
        acc_z     = float(np.clip(np.nan_to_num(self.physics.accel_z), -50.0, 50.0))
        alt       = float(np.clip(np.nan_to_num(self.physics.altitude), -100.0, 5000.0))
        climb     = float(np.clip(np.nan_to_num(self.physics.climb_rate), -20.0, 20.0))
        tas       = float(np.clip(np.nan_to_num(self.physics.TAS), 0.0, 100.0))
        lat_i     = int(np.clip(np.nan_to_num(lat * 1e7), -900000000, 900000000))
        lon_i     = int(np.clip(np.nan_to_num(lon * 1e7), -1800000000, 1800000000))
        gndspd    = float(np.clip(np.nan_to_num(self.physics.groundspeed), 0.0, 100.0))
        vn        = float(np.clip(np.nan_to_num(v_n), -100.0, 100.0))
        ve        = float(np.clip(np.nan_to_num(v_e), -100.0, 100.0))
        vd        = float(np.clip(np.nan_to_num(v_d), -100.0, 100.0))

        raw_data = struct.pack(
            "<BB I fff fff fff ffff ii fffff B",
            HITL_MAGIC_0,
            HITL_MAGIC_1,
            now_ms,
            roll_deg,
            pitch_deg,
            yaw_deg,
            p_dps,
            q_dps,
            r_dps,
            acc_x,
            acc_y,
            acc_z,
            alt,
            climb,
            tas,
            tas,  # EAS ~ TAS at sea level
            lat_i,
            lon_i,
            alt,
            gndspd,
            vn,
            ve,
            vd,
            flags
        )
        cs = calc_fletcher16(raw_data)
        return raw_data + struct.pack("<H", cs)

    def parse_actuator_packet(self, data: bytes):
        if len(data) != ACTUATOR_PKT_SIZE:
            return
        
        expected_cs = calc_fletcher16(data[:-2])
        actual_cs = struct.unpack("<H", data[-2:])[0]
        if expected_cs != actual_cs:
            return  # Checksum mismatch

        unpacked = struct.unpack(ACTUATOR_FMT, data)
        # ACTUATOR_FMT = "<BB I hhhh H fff ffff BB H"
        # Index mapping:
        # 0: magic0, 1: magic1, 2: timestamp_ms
        # 3: ail_R, 4: ail_L, 5: ele, 6: rud, 7: thr_pwm (h h h h H)
        # 8: dem_pitch, 9: dem_roll, 10: dem_thr_pct (f f f)
        # 11: integ_seb, 12: integ_thr, 13: ste_err, 14: target_alt_m (f f f f)
        # 15: current_mode (= flag_wp aktif), 16: is_armed (B B)
        # 17: checksum (H)
        with self.lock:
            # FC u_roll > 0 saat error roll positif (roll > roll_cmd), meminta defleksi aileron untuk roll rate negatif
            # Di fisika simulator p_target = ail_cmd * 60 deg/s, sehingga ail_cmd harus bernilai negatif dari u_roll
            self.curr_elevator = np.clip(unpacked[5] / 400.0, -1.0, 1.0)
            self.curr_aileron  = -np.clip(unpacked[3] / 400.0, -1.0, 1.0)
            self.curr_rudder   = np.clip(unpacked[6] / 400.0, -1.0, 1.0)
            self.curr_dem_pitch  = unpacked[8]
            self.curr_dem_roll   = unpacked[9]
            self.curr_dem_thr    = unpacked[10]
            self.curr_ste_err    = unpacked[13]
            self.curr_target_alt = unpacked[14]

            # Decode flight mode & active WP dari packet
            mode_val = int(unpacked[15])
            if mode_val == 200:
                self.mode_str = "RTL"
                self.curr_active_wp = 0
            elif mode_val >= 10 and mode_val < 50:
                self.mode_str = "AUTO"
                self.curr_active_wp = mode_val - 10
            elif mode_val == 1:
                self.mode_str = "MANUAL"
                self.curr_active_wp = 0
            elif mode_val == 2:
                self.mode_str = "FBWA"
                self.curr_active_wp = 0
            elif mode_val == 4:
                self.mode_str = "GUIDED"
                self.curr_active_wp = 0
            else:
                self.mode_str = f"MODE_{mode_val}"
                self.curr_active_wp = mode_val

            # Map throttle PWM (1000..2000) ke 0.0..1.0
            self.curr_throttle_pct = np.clip((unpacked[7] - 1000) / 1000.0, 0.0, 1.0)

            # Sinkronisasi status Arming dari Teensy FC
            fc_armed = bool(unpacked[16])
            if fc_armed != self.is_armed:
                self.is_armed = fc_armed
                # Sinkronkan command arm simulator agar tidak menimpa status FC secara tidak sengaja
                self.arm_cmd = fc_armed
                status_str = "ARMED" if fc_armed else "DISARMED"
                print(f"\n[HITL] Status Arm FC berubah -> {status_str}")

            self.rx_count += 1

    def sim_loop(self):
        """Thread simulasi fisika 50 Hz."""
        dt = 0.02 # 50 Hz
        rx_buffer = bytearray()

        while self.running:
            t0 = time.time()

            # 1. Step physics forward
            with self.lock:
                if self.is_armed:
                    # Throttle: gunakan demanded throttle pct dari TECS (0..100 → 0.0..1.0)
                    thr_base = np.clip(self.curr_dem_thr / 100.0, 0.0, 1.0)
                    thr_for_physics = thr_base

                    # Speed governor: jika TAS turun di bawah 12 m/s (safety floor MIN_AIRSPEED),
                    # override ke 85% throttle untuk recovery cepat ke kecepatan cruise 21 m/s.
                    # Ini mencegah TECS yang belum warm-up menyebabkan pesawat melambat terus.
                    if self.physics.TAS < 12.0:
                        thr_for_physics = max(thr_for_physics, 0.85)
                else:
                    thr_for_physics = 0.0
                    self.curr_throttle_pct = 0.0

                v_n, v_e, v_d = self.physics.step(
                    dt,
                    self.curr_elevator,
                    self.curr_aileron,
                    self.curr_rudder,
                    thr_for_physics
                )
                t_curr = time.time() - self.start_time
                self.t_hist.append(t_curr)
                self.alt_hist.append(self.physics.altitude)
                self.alt_tgt_hist.append(self.curr_target_alt)
                self.climb_hist.append(self.physics.climb_rate)
                self.tas_hist.append(self.physics.TAS)
                self.gndspd_hist.append(self.physics.groundspeed)
                self.pitch_hist.append(math.degrees(self.physics.pitch))
                self.pitch_dem_hist.append(self.curr_dem_pitch)
                self.roll_hist.append(math.degrees(self.physics.roll))
                self.roll_dem_hist.append(self.curr_dem_roll)
                self.thr_pct_hist.append(self.curr_throttle_pct * 100.0)
                self.thr_dem_hist.append(self.curr_dem_thr)
                self.ste_err_hist.append(self.curr_ste_err)
                self.ele_pwm_hist.append(self.curr_elevator * 400.0)

            # 2. Send sensor packet to Teensy
            pkt = self.pack_sensor_packet(v_n, v_e, v_d)
            try:
                self.ser.write(pkt)
                self.tx_count += 1
            except Exception as e:
                print(f"[ERROR] Serial write error: {e}")

            # 3. Read incoming actuator telemetry
            try:
                if self.ser.in_waiting > 0:
                    rx_buffer.extend(self.ser.read(self.ser.in_waiting))
                    while len(rx_buffer) >= ACTUATOR_PKT_SIZE:
                        # Find sync header
                        if rx_buffer[0] == HITL_MAGIC_0 and rx_buffer[1] == HITL_MAGIC_1:
                            pkt_data = bytes(rx_buffer[:ACTUATOR_PKT_SIZE])
                            self.parse_actuator_packet(pkt_data)
                            rx_buffer = rx_buffer[ACTUATOR_PKT_SIZE:]
                        else:
                            rx_buffer.pop(0)
            except Exception as e:
                print(f"[ERROR] Serial read error: {e}")

            # Debug print setiap 2 detik (100 iterations)
            if self.tx_count % 50 == 0:
                lat, lon = self.physics.get_lat_lon()
                mode_str = getattr(self, 'mode_str', 'AUTO')
                arm_tag = "ARMED" if self.is_armed else "DISARMED"
                mode_tag = f"[{arm_tag}] {mode_str}" + (f" (WP {self.curr_active_wp})" if mode_str == "AUTO" else "")
                print(f"[SIM t={t_curr:5.1f}s] "
                      f"{mode_tag:<22} "
                      f"x={self.physics.x:6.1f}m N  y={self.physics.y:6.1f}m E  "
                      f"yaw={math.degrees(self.physics.yaw):6.1f}°  "
                      f"roll={math.degrees(self.physics.roll):5.1f}°  "
                      f"roll_dem={self.curr_dem_roll:5.1f}°  "
                      f"TAS={self.physics.TAS:4.1f}m/s  "
                      f"thr={self.curr_throttle_pct*100:2.0f}%  "
                      f"alt={self.physics.altitude:4.1f}m  "
                      f"GPS=({lat:.7f},{lon:.7f})")

            # Sleep to maintain 50 Hz loop
            elapsed = time.time() - t0
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def start(self):
        self.connect()
        self.running = True
        self.thread = threading.Thread(target=self.sim_loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()


# =============================================================================
#  REAL-TIME DASHBOARD & LIVE PLOTTER
# =============================================================================
def run_dashboard(runner: HITLSimRunner):
    plt.style.use('dark_background')
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(14, 9))
    fig.canvas.manager.set_window_title("Khageswara FC — HITL TECS & Auto Control Live Dashboard")

    # Subplot 1: Altitude & Climb Rate
    line_alt, = ax1.plot([], [], 'g-', label="Actual Alt (m)", lw=2)
    line_alt_tgt, = ax1.plot([], [], 'c--', label="Target Alt (m)", lw=1.5)
    line_climb, = ax1.plot([], [], 'm:', label="Climb Rate (m/s)", lw=1.2)
    arm_badge = ax1.text(0.03, 0.88, "[STATUS: ARMED]", transform=ax1.transAxes, fontsize=10, fontweight='bold', color='lime', bbox=dict(boxstyle='round,pad=0.3', facecolor='black', edgecolor='lime', alpha=0.8))
    wp_badge = ax1.text(0.68, 0.88, "Target: WP 1 (80m)", transform=ax1.transAxes, fontsize=10, fontweight='bold', color='lime', bbox=dict(boxstyle='round,pad=0.3', facecolor='black', edgecolor='lime', alpha=0.8))
    ax1.set_title("1. Altitude & Climb Rate Tracking", fontsize=11, fontweight='bold', color='lime')
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Meters / (m/s)")
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc="upper left")

    # Subplot 2: Speed & Groundspeed (Proving Groundspeed-Only Operation)
    line_tas, = ax2.plot([], [], 'y-', label="Airspeed TAS (m/s)", lw=2)
    line_gnd, = ax2.plot([], [], 'w--', label="Groundspeed (m/s)", lw=1.5)
    ax2.axhline(21.0, color='r', linestyle=':', label="Cruise Target (21 m/s)")
    ax2.set_title("2. Speed: TAS vs Groundspeed (GPS Mode)", fontsize=11, fontweight='bold', color='yellow')
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Speed (m/s)")
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc="upper left")

    # Subplot 3: Pitch Response & Cascaded Tracking (Proving Inner-Loop PID)
    line_pitch, = ax3.plot([], [], 'g-', label="Actual Pitch (deg)", lw=2)
    line_pitch_dem, = ax3.plot([], [], 'r--', label="TECS Demanded Pitch (deg)", lw=1.5)
    line_ele, = ax3.plot([], [], 'm:', label="Elevator Deflection (scaled)", lw=1)
    ax3.set_title("3. Cascaded Pitch Tracking: TECS Setpoint -> FBWA PID", fontsize=11, fontweight='bold', color='cyan')
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Degrees")
    ax3.grid(True, alpha=0.3)
    ax3.legend(loc="upper left")

    # Subplot 4: Throttle & Specific Energy Error (Proving Smoothness & Slew)
    line_thr, = ax4.plot([], [], 'orange', label="Applied Motor Throttle (%)", lw=2)
    line_thr_dem, = ax4.plot([], [], 'r--', label="TECS Demanded Thr (%)", lw=1.2)
    line_ste, = ax4.plot([], [], 'b:', label="Energy Error STE (scaled)", lw=1)
    ax4.set_title("4. TECS Energy Error & Throttle Slew Smoothness", fontsize=11, fontweight='bold', color='orange')
    ax4.set_xlabel("Time (s)")
    ax4.set_ylabel("Percentage (%)")
    ax4.grid(True, alpha=0.3)
    ax4.legend(loc="upper left")

    # Keyboard shortcut listener di Matplotlib
    def on_key_press(event):
        if event.key in ['d', 'D']:
            runner.disarm()
        elif event.key in ['a', 'A']:
            runner.arm()
        elif event.key == ' ':
            runner.toggle_arm()

    fig.canvas.mpl_connect('key_press_event', on_key_press)
    fig.text(0.5, 0.008, "Kontrol Keyboard: [SPACE] Toggle Arm/Disarm  |  [D] Disarm  |  [A] Arm", ha='center', fontsize=9, color='lightgray')

    plt.tight_layout()

    def update_plot(frame):
        with runner.lock:
            # Update badge Arming status
            if runner.is_armed:
                arm_badge.set_text("[STATUS: ARMED]")
                arm_badge.set_color('lime')
                arm_badge.set_bbox(dict(boxstyle='round,pad=0.3', facecolor='black', edgecolor='lime', alpha=0.8))
            else:
                arm_badge.set_text("[STATUS: DISARMED]")
                arm_badge.set_color('red')
                arm_badge.set_bbox(dict(boxstyle='round,pad=0.3', facecolor='black', edgecolor='red', alpha=0.8))

            mode_name = getattr(runner, 'mode_str', 'AUTO')
            wp_num = getattr(runner, 'curr_active_wp', 1)
            tgt_alt = getattr(runner, 'curr_target_alt', 50.0)
            if mode_name == "RTL":
                wp_badge.set_text(f"[MODE: RTL] Loitering HOME ({tgt_alt:.0f}m)")
                wp_badge.set_color('cyan')
            elif mode_name == "AUTO":
                wp_badge.set_text(f"[MODE: AUTO] Target WP {wp_num} ({tgt_alt:.0f}m)")
                wp_badge.set_color('lime')
            else:
                wp_badge.set_text(f"[MODE: {mode_name}] ({tgt_alt:.0f}m)")
                wp_badge.set_color('yellow')

            if len(runner.t_hist) < 2:
                return line_alt, line_alt_tgt, line_climb, line_tas, line_gnd, line_pitch, line_pitch_dem, line_ele, line_thr, line_thr_dem, line_ste

            t = list(runner.t_hist)
            t_min, t_max = max(0, t[-1] - 30), t[-1] + 1

            # Subplot 1
            line_alt.set_data(t, list(runner.alt_hist))
            line_alt_tgt.set_data(t, list(runner.alt_tgt_hist))
            line_climb.set_data(t, list(runner.climb_hist))
            ax1.set_xlim(t_min, t_max)
            all_alt = list(runner.alt_hist) + list(runner.alt_tgt_hist)
            ax1.set_ylim(min(all_alt) - 5, max(all_alt) + 10)

            # Subplot 2
            line_tas.set_data(t, list(runner.tas_hist))
            line_gnd.set_data(t, list(runner.gndspd_hist))
            ax2.set_xlim(t_min, t_max)
            ax2.set_ylim(0, 30)

            # Subplot 3
            line_pitch.set_data(t, list(runner.pitch_hist))
            line_pitch_dem.set_data(t, list(runner.pitch_dem_hist))
            ele_scaled = [e / 20.0 for e in runner.ele_pwm_hist] # Scaled for overlay
            line_ele.set_data(t, ele_scaled)
            ax3.set_xlim(t_min, t_max)
            ax3.set_ylim(-25, 30)

            # Subplot 4
            line_thr.set_data(t, list(runner.thr_pct_hist))
            line_thr_dem.set_data(t, list(runner.thr_dem_hist))
            ste_scaled = [s * 0.1 for s in runner.ste_err_hist]
            line_ste.set_data(t, ste_scaled)
            ax4.set_xlim(t_min, t_max)
            ax4.set_ylim(-5, 105)

        return line_alt, line_alt_tgt, line_climb, line_tas, line_gnd, line_pitch, line_pitch_dem, line_ele, line_thr, line_thr_dem, line_ste

    ani = FuncAnimation(fig, update_plot, interval=50, blit=False, cache_frame_data=False)
    plt.show()


# =============================================================================
#  TERMINAL INPUT LISTENER THREAD
# =============================================================================
def terminal_input_loop(runner: HITLSimRunner):
    """Membaca input keyboard di terminal (d / a / space / disarm / arm)."""
    while runner.running:
        try:
            line = sys.stdin.readline()
            if not line:
                break
            cmd = line.strip().lower()
            if cmd in ['d', 'disarm']:
                runner.disarm()
            elif cmd in ['a', 'arm']:
                runner.arm()
            elif cmd in [' ', 'toggle', 't']:
                runner.toggle_arm()
            elif cmd in ['h', 'help']:
                print("\n[BANTUAN HITL] d/disarm = Disarm | a/arm = Arm | t/space = Toggle | h = Help")
        except Exception:
            break


# =============================================================================
#  AUTO-DETECT SERIAL PORT & ENTRY POINT
# =============================================================================
def auto_detect_port():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if "Teensy" in p.description or "USB Serial" in p.description or "ACM" in p.device or "USB" in p.device:
            return p.device
    if ports:
        return ports[0].device
    return "/dev/ttyACM0"

def main():
    parser = argparse.ArgumentParser(description="Khageswara FC-2026 HITL TECS Simulator")
    parser.add_argument("--port", "-p", default=None, help="Serial port Teensy (e.g. /dev/ttyACM0 or COM3)")
    parser.add_argument("--baud", "-b", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--pitot", action="store_true", help="Aktifkan virtual Pitot Tube sensor (Default: False = Full Groundspeed Mode)")
    parser.add_argument("--wind", type=float, default=0.0, help="Headwind magnitude in m/s (e.g. 5.0)")
    parser.add_argument("--disarm", action="store_true", help="Mulai simulasi dalam kondisi DISARMED (Default: Mulai ARMED)")
    args = parser.parse_args()

    port = args.port or auto_detect_port()

    print("=" * 70)
    print("  KHAGESWARA FC-2026 — HITL SIMULATION & TECS VALIDATION")
    print("=" * 70)
    print(f"  Target Port        : {port}")
    print(f"  Baud Rate          : {args.baud}")
    print(f"  Airspeed Mode      : {'Pitot Tube Enabled' if args.pitot else 'FULL GROUNDSPEED (Non-Airspeed TECS)'}")
    print(f"  Simulated Headwind : {args.wind} m/s")
    print(f"  Initial State      : {'DISARMED' if args.disarm else 'ARMED'}")
    print("=" * 70)
    print("  [INFO] Teensy Dual Serial Aktif:")
    print(f"         • Simulator HITL  : {port}")
    print("         • Mission Planner : Port serial kedua (misal /dev/ttyACM1 atau COM4)")
    print("  [KONTROL ARM / DISARM]:")
    print("         • Tekan [SPACE] di Dashboard / Terminal : Toggle Arm/Disarm")
    print("         • Tekan [D] / ketik 'd'                 : DISARM")
    print("         • Tekan [A] / ketik 'a'                 : ARM")
    print("         • Lewat Mission Planner / MAVLink       : Otomatis sinkron!")
    print("=" * 70)

    runner = HITLSimRunner(port=port, baud=args.baud, enable_pitot=args.pitot)
    if args.disarm:
        runner.arm_cmd = False
    if args.wind > 0:
        runner.physics.wind_north = -args.wind # North headwind

    try:
        runner.start()
        # Jalankan thread pembaca input terminal (background)
        threading.Thread(target=terminal_input_loop, args=(runner,), daemon=True).start()
        print("\n[HITL] Simulator aktif! Membuka Live Dashboard...")
        run_dashboard(runner)
    except KeyboardInterrupt:
        print("\n[HITL] Menghentikan simulasi...")
    finally:
        runner.stop()
        print("[HITL] Selesai.")

if __name__ == "__main__":
    main()

