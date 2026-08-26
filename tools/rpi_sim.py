#!/usr/bin/env python3
"""
rpi_sim.py — Simulasi RPi commands ke FC via MAVLink (USB/Serial)

Gunakan ini untuk test RPi lock & velocity control TANPA RPi fisik.
Koneksi: Teensy USB ke laptop (COM port atau /dev/ttyACM0)

Install dependency:
    pip install pymavlink

Jalankan:
    python rpi_sim.py --port COM3           (Windows)
    python rpi_sim.py --port /dev/ttyACM0   (Linux)

Kontrol interaktif (ketik + Enter):
    L/U   : RPi LOCK ON / OFF
    T     : TKOF
    G     : GUIDED
    F     : FW-TRANS (transisi ke fixed-wing)
    A     : AUTO (waypoint)
    Q     : QHOV (pilot emergency)
    arm   : ARM
    dis   : DISARM
    hover : velocity 0,0,0 (diam di tempat)
    up    : naik 0.5 m/s
    dn    : turun 0.5 m/s
    fwd   : maju ke utara 1 m/s
    stop  : reset velocity
    status: print telemetri terakhir
    h     : tampilkan menu
    quit  : keluar
"""

import sys
import time
import threading
import argparse
from pymavlink import mavutil

# ─── Konfigurasi ──────────────────────────────────────────────
SYSID  = 1     # Akan di-update setelah heartbeat
COMPID = 1
BAUD   = 57600
# ──────────────────────────────────────────────────────────────

latest_hb  = None
latest_pos = None

def listen_loop(conn):
    """Background thread: terima dan print pesan dari FC."""
    global latest_hb, latest_pos
    while True:
        try:
            msg = conn.recv_match(blocking=True, timeout=1.0)
            if msg is None:
                continue
            t = msg.get_type()
            if t == 'HEARTBEAT':
                latest_hb = msg
            elif t == 'GLOBAL_POSITION_INT':
                latest_pos = msg
            elif t == 'STATUSTEXT':
                print(f"\n  [FC] {msg.text.strip()}")
            elif t == 'COMMAND_ACK':
                results = {0:'ACCEPTED', 1:'TEMP_REJ', 2:'DENIED', 3:'UNSUPPORTED', 4:'FAILED'}
                print(f"\n  [ACK] cmd={msg.command} → {results.get(msg.result, msg.result)}")
        except Exception:
            pass

def cmd_rpi_lock(conn, on: bool):
    """MAV_CMD_USER_1 — toggle rpi_mode_lock di FC."""
    print(f"[TX] MAV_CMD_USER_1 param1={'1.0' if on else '0.0'} → RPI LOCK {'ON' if on else 'OFF'}")
    conn.mav.command_long_send(
        SYSID, COMPID,
        31010,   # MAV_CMD_USER_1
        0,
        1.0 if on else 0.0,
        0, 0, 0, 0, 0, 0)

def cmd_set_mode(conn, custom_mode: int, name: str):
    """MAV_CMD_DO_SET_MODE — set mode via MAVLink."""
    print(f"[TX] SET_MODE custom={custom_mode} → {name}")
    conn.mav.command_long_send(
        SYSID, COMPID,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        float(custom_mode),
        0, 0, 0, 0, 0)

def cmd_arm(conn, arm: bool, force: bool = False):
    """MAV_CMD_COMPONENT_ARM_DISARM. Jika force=True, bypass GPS check (bench test)."""
    p2 = 21196.0 if (arm and force) else 0.0
    print(f"[TX] {'FORCE ARM' if (arm and force) else ('ARM' if arm else 'DISARM')}")
    conn.mav.command_long_send(
        SYSID, COMPID,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1.0 if arm else 0.0,
        p2, # param2: 21196 untuk bypass pre-arm check
        0, 0, 0, 0, 0)

def cmd_set_velocity(conn, vx=0.0, vy=0.0, vz=0.0):
    """
    SET_POSITION_TARGET_LOCAL_NED — velocity only, NED frame.
    vz: NED convention (+= turun, -= naik)
    """
    # Ignore position, acceleration, yaw — hanya VX VY VZ aktif
    TYPE_MASK = (0b111111000111)  # pos+accel ignored, vel active
    # bit 0-2 = pos ignored, bit 3-5 = vel active (0=used), bit 6-8 = accel ignored
    # Shorthand: 0b111_000_111 = 0x1C7... mari pakai konstanta yang jelas:
    POS_IGN  = 0b000000111   # ignore X Y Z
    ACC_IGN  = 0b111000000   # ignore ax ay az
    YAW_IGN  = 0b100000000000  # ignore yaw
    YRTE_IGN = 0b010000000000  # ignore yaw_rate
    mask = POS_IGN | ACC_IGN | YAW_IGN | YRTE_IGN

    direction = "turun" if vz > 0 else ("naik" if vz < 0 else "diam")
    print(f"[TX] SET_POSITION_TARGET vx={vx:.2f} vy={vy:.2f} vz={vz:.2f} ({direction})")
    conn.mav.set_position_target_local_ned_send(
        int(time.time() * 1000) & 0xFFFFFFFF,
        SYSID, COMPID,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        mask,
        0.0, 0.0, 0.0,  # x y z (ignored)
        vx, vy, vz,      # velocity NED
        0.0, 0.0, 0.0,  # accel (ignored)
        0.0, 0.0        # yaw, yaw_rate (ignored)
    )

def print_status():
    print("\n--- STATUS FC ---------------------")
    if latest_hb:
        armed = bool(latest_hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        print(f"  Heartbeat : custom_mode={latest_hb.custom_mode}, armed={armed}")
    else:
        print("  Heartbeat : (belum diterima)")
    if latest_pos:
        alt = latest_pos.relative_alt / 1000.0
        vx  = latest_pos.vx / 100.0
        vy  = latest_pos.vy / 100.0
        vz  = latest_pos.vz / 100.0
        print(f"  Altitude  : {alt:.2f} m (relative)")
        print(f"  Velocity  : vx={vx:.2f} vy={vy:.2f} vz={vz:.2f} m/s")
    else:
        print("  Position  : (belum diterima)")
    print("-----------------------------------\n")

MENU = """
+=========================================+
|   RPi Mission Simulator                 |
+----------+------------------------------+
|  L / U   | RPi LOCK ON / OFF            |
|  T       | TKOF (custom_mode=0)         |
|  G       | GUIDED (custom_mode=15)      |
|  F       | FW-TRANS (custom_mode=5)     |
|  A       | AUTO (custom_mode=12)        |
|  Q       | QHOV emergency (cm=10)       |
+----------+------------------------------+
|  arm     | ARM                          |
|  farm    | FORCE ARM (Bypass GPS)       |
|  dis     | DISARM                       |
+----------+------------------------------+
|  hover   | velocity 0,0,0              |
|  up      | naik 0.5 m/s (vz=-0.5)      |
|  dn      | turun 0.5 m/s (vz=+0.5)     |
|  fwd     | maju utara 1 m/s            |
|  stop    | stop velocity               |
+----------+------------------------------+
|  status  | print telemetri             |
|  h       | menu ini                    |
|  quit    | keluar                      |
+=========================================+
"""

def interactive(conn):
    print(MENU)
    while True:
        try:
            raw = input("rpi> ").strip().upper()
        except (EOFError, KeyboardInterrupt):
            break

        if   raw == 'L':      cmd_rpi_lock(conn, True)
        elif raw == 'U':      cmd_rpi_lock(conn, False)
        elif raw == 'T':      cmd_set_mode(conn, 0,  "TKOF/MC")
        elif raw == 'G':      cmd_set_mode(conn, 15, "GUIDED")
        elif raw == 'F':      cmd_set_mode(conn, 5,  "FW-TRANS")
        elif raw == 'A':      cmd_set_mode(conn, 12, "AUTO")
        elif raw == 'Q':      cmd_set_mode(conn, 10, "QHOV")
        elif raw == 'ARM':    cmd_arm(conn, True)
        elif raw == 'FARM':   cmd_arm(conn, True, force=True)
        elif raw == 'DIS':    cmd_arm(conn, False)
        elif raw == 'HOVER':  cmd_set_velocity(conn, 0.0, 0.0, 0.0)
        elif raw == 'UP':     cmd_set_velocity(conn, 0.0, 0.0, -0.5)
        elif raw == 'DN':     cmd_set_velocity(conn, 0.0, 0.0, +0.5)
        elif raw == 'FWD':    cmd_set_velocity(conn, 1.0, 0.0, 0.0)
        elif raw == 'STOP':   cmd_set_velocity(conn, 0.0, 0.0, 0.0)
        elif raw == 'STATUS': print_status()
        elif raw == 'H':      print(MENU)
        elif raw in ('QUIT', 'EXIT'):
            break
        elif raw == '':
            pass
        else:
            print(f"  Perintah tidak dikenal: '{raw}'. Ketik H untuk help.")

    print("[EXIT] Selesai.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Simulasi RPi Mission Control via MAVLink")
    parser.add_argument('--port', default='COM3',
                        help='Serial port (e.g. COM3, COM10, /dev/ttyUSB0)')
    parser.add_argument('--baud', default=BAUD, type=int,
                        help=f'Baud rate (default: {BAUD})')
    parser.add_argument('--no-heartbeat', action='store_true',
                        help='Skip tunggu heartbeat (langsung ke menu)')
    args = parser.parse_args()

    # Fix untuk Windows: pymavlink perlu format \\.\ untuk semua COM port
    # agar tidak salah diinterpretasi sebagai file path
    port = args.port
    import platform
    if platform.system() == 'Windows':
        # Normalisasi: COM3 → \\.\COM3, sudah \\.\COM3 → tetap
        if not port.startswith('\\\\.\\') and not port.startswith('//./'):
            port = f'\\\\.\\{port}'

    print(f"[CONN] Menghubungi FC di {args.port} -> {port} @ {args.baud} baud...")
    try:
        conn = mavutil.mavlink_connection(port, baud=args.baud)
    except Exception as e:
        print(f"[ERROR] Tidak bisa buka port: {e}")
        print("[HINT] Cek apakah port benar dan tidak dipakai aplikasi lain.")
        sys.exit(1)

    if args.no_heartbeat:
        print("[CONN] Skip heartbeat (--no-heartbeat). Langsung ke menu.\n")
        SYSID = 1
    else:
        print("[CONN] Tunggu heartbeat FC di Serial1 (Pin 0/1, 57600 baud)...")
        print("[HINT] Jika FC tidak kirim heartbeat, coba --no-heartbeat")
        hb = conn.wait_heartbeat(timeout=15)
        if hb is None:
            print("[ERROR] Tidak ada heartbeat dalam 15 detik.")
            print("  → Cek wiring TX/RX (harus silang: FC-TX → CH340-RX)")
            print("  → Pastikan FC sudah powered dan MAVLINK_RPI_ENABLE_TX=1")
            print("  → Coba jalankan dengan --no-heartbeat untuk test kirim saja")
            sys.exit(1)
        SYSID = conn.target_system
        print(f"[CONN] FC ditemukan! sysid={SYSID}\n")

    # Background listener
    t = threading.Thread(target=listen_loop, args=(conn,), daemon=True)
    t.start()

    interactive(conn)
