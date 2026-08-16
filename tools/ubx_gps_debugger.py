#!/usr/bin/env python3
"""
UBX Real-time GPS Packet Inspector & Parser for Holybro / u-blox M10
Supports all UBX messages (NAV-PVT, NAV-POSLLH, NAV-STATUS, NAV-DOP, etc.)
"""

import sys
import time
import struct
import argparse
import serial
import serial.tools.list_ports

# Struct formats
UBX_NAV_PVT_FMT = "<IHBBBBBBIIBBBi i i i I I i i i i i I I H H 4s i h H".replace(" ", "")
UBX_NAV_POSLLH_FMT = "<IiiiiII"
UBX_NAV_STATUS_FMT = "<IBBBBII"

UBX_MSG_NAMES = {
    (0x01, 0x07): "UBX-NAV-PVT (Position/Velocity/Time)",
    (0x01, 0x02): "UBX-NAV-POSLLH (Geodetic Position)",
    (0x01, 0x03): "UBX-NAV-STATUS (Receiver Navigation Status)",
    (0x01, 0x04): "UBX-NAV-DOP (Dilution of Precision)",
    (0x01, 0x06): "UBX-NAV-SOL (Navigation Solution)",
    (0x01, 0x12): "UBX-NAV-VELNED (Velocity Solution in NED)",
    (0x01, 0x21): "UBX-NAV-TIMEUTC (UTC Time)",
    (0x01, 0x35): "UBX-NAV-SAT (Satellite Information)",
    (0x01, 0x30): "UBX-NAV-SVINFO (Space Vehicle Info)",
    (0x05, 0x01): "UBX-ACK-ACK (Message Acknowledged)",
    (0x05, 0x00): "UBX-ACK-NAK (Message Rejected)",
    (0x0A, 0x09): "UBX-MON-VER (Receiver/Software Version)",
    (0x0A, 0x38): "UBX-MON-RF (RF Information)",
}

FIX_TYPES = {
    0: "NO FIX",
    1: "DEAD RECKONING",
    2: "2D FIX",
    3: "3D FIX",
    4: "GNSS + DR",
    5: "TIME ONLY"
}

def calculate_fletcher_checksum(data: bytes) -> tuple:
    ck_a = 0
    ck_b = 0
    for b in data:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return ck_a, ck_b

def create_ubx_packet(msg_class: int, msg_id: int, payload: bytes = b"") -> bytes:
    header = bytes([0xB5, 0x62, msg_class, msg_id, len(payload) & 0xFF, (len(payload) >> 8) & 0xFF])
    ck_a, ck_b = calculate_fletcher_checksum(header[2:] + payload)
    return header + payload + bytes([ck_a, ck_b])

def list_available_ports():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("[-] Tidak ada port serial yang terdeteksi.")
        return []
    print("[*] Port Serial yang Terdeteksi:")
    for p in ports:
        print(f"    - {p.device} ({p.description})")
    return [p.device for p in ports]

def parse_ubx_packet(msg_class: int, msg_id: int, payload: bytes) -> str:
    """Menguraikan paket UBX berdasarkan Class dan ID"""
    msg_name = UBX_MSG_NAMES.get((msg_class, msg_id), f"UBX-0x{msg_class:02X}-0x{msg_id:02X}")

    # 1. NAV-PVT
    if msg_class == 0x01 and msg_id == 0x07 and len(payload) == 92:
        unpacked = struct.unpack(UBX_NAV_PVT_FMT, payload)
        hour, minute, sec = unpacked[4], unpacked[5], unpacked[6]
        fix_type = unpacked[10]
        num_sv   = unpacked[13]
        lon      = unpacked[14] * 1e-7
        lat      = unpacked[15] * 1e-7
        alt_m    = unpacked[17] * 1e-3
        speed_km = (unpacked[23] * 1e-3) * 3.6
        pdop     = unpacked[27] * 1e-2

        fix_str = FIX_TYPES.get(fix_type, f"Fix({fix_type})")
        return (f"[{msg_name}]\n"
                f"  -> Fix: {fix_str} | Sats: {num_sv} | Time: {hour:02d}:{minute:02d}:{sec:02d} UTC\n"
                f"  -> Lat: {lat:.7f}° | Lon: {lon:.7f}° | Alt: {alt_m:.1f} m\n"
                f"  -> Speed: {speed_km:.2f} km/h | pDOP: {pdop:.2f}")

    # 2. NAV-POSLLH
    elif msg_class == 0x01 and msg_id == 0x02 and len(payload) >= 28:
        unpacked = struct.unpack(UBX_NAV_POSLLH_FMT, payload[:28])
        lon = unpacked[1] * 1e-7
        lat = unpacked[2] * 1e-7
        alt_m = unpacked[4] * 1e-3
        return f"[{msg_name}] Lat: {lat:.7f}° | Lon: {lon:.7f}° | Alt: {alt_m:.1f} m"

    # 3. NAV-STATUS
    elif msg_class == 0x01 and msg_id == 0x03 and len(payload) >= 16:
        unpacked = struct.unpack(UBX_NAV_STATUS_FMT, payload[:16])
        fix_type = unpacked[1]
        fix_str = FIX_TYPES.get(fix_type, f"Fix({fix_type})")
        return f"[{msg_name}] GPS Fix Status: {fix_str}"

    # 4. ACK-ACK / ACK-NAK
    elif msg_class == 0x05:
        ack_cls = payload[0] if len(payload) > 0 else 0
        ack_id  = payload[1] if len(payload) > 1 else 0
        return f"[{msg_name}] Command (0x{ack_cls:02X}, 0x{ack_id:02X}) {'ACCEPTED' if msg_id == 1 else 'REJECTED'}"

    # 5. MON-VER
    elif msg_class == 0x0A and msg_id == 0x09:
        sw_ver = payload[:30].decode('ascii', errors='ignore').strip('\x00')
        hw_ver = payload[30:40].decode('ascii', errors='ignore').strip('\x00')
        return f"[{msg_name}] Hardware: {hw_ver} | Software: {sw_ver}"

    # Generic UBX info
    return f"[{msg_name}] (Length: {len(payload)} bytes)"

def configure_m10(ser: serial.Serial):
    """Kirim perintah inisialisasi UBX ke Holybro M10"""
    print("[*] Mengirim perintah aktivasi UBX-NAV-PVT...")
    # 1. CFG-VALSET (u-blox M10)
    valset_payload = bytes([
        0x00, 0x07, 0x00, 0x00,
        0x07, 0x00, 0x91, 0x20, 0x01,  # CFG-MSGOUT-UBX_NAV_PVT_UART1 = 1
        0x01, 0x00, 0x74, 0x10, 0x01   # CFG-UART1OUTPROT-UBX = 1
    ])
    ser.write(create_ubx_packet(0x06, 0x8A, valset_payload))
    
    # 2. CFG-MSG Legacy
    ser.write(create_ubx_packet(0x06, 0x01, bytes([0x01, 0x07, 0x01])))
    
    # 3. Poll MON-VER & NAV-PVT
    ser.write(create_ubx_packet(0x0A, 0x09, b"")) # Poll Hardware version
    ser.write(create_ubx_packet(0x01, 0x07, b"")) # Poll NAV-PVT
    ser.flush()

def run_inspector(port: str, baudrate: int = 115200):
    print(f"\n[*] Membuka {port} pada {baudrate} baud...")
    try:
        ser = serial.Serial(port, baudrate=baudrate, timeout=0.1)
    except Exception as e:
        print(f"[!] Gagal membuka port {port}: {e}")
        return

    configure_m10(ser)

    print("\n[+] STREAM UBX AKTIF! Menunggu data paket...")
    print("=" * 80)

    state = "SYNC1"
    msg_class = 0
    msg_id = 0
    msg_len = 0
    payload = bytearray()

    total_bytes = 0
    total_ubx_packets = 0
    last_poll_time = time.time()

    try:
        while True:
            # Poll NAV-PVT setiap 1 detik
            if time.time() - last_poll_time > 1.0:
                ser.write(create_ubx_packet(0x01, 0x07, b""))
                last_poll_time = time.time()

            raw_bytes = ser.read(ser.in_waiting or 1)
            if not raw_bytes:
                continue

            total_bytes += len(raw_bytes)

            for b in raw_bytes:
                if state == "SYNC1":
                    if b == 0xB5:
                        state = "SYNC2"
                elif state == "SYNC2":
                    if b == 0x62:
                        state = "CLASS"
                    elif b == 0xB5:
                        state = "SYNC2"
                    else:
                        state = "SYNC1"
                elif state == "CLASS":
                    msg_class = b
                    state = "ID"
                elif state == "ID":
                    msg_id = b
                    state = "LEN1"
                elif state == "LEN1":
                    msg_len = b
                    state = "LEN2"
                elif state == "LEN2":
                    msg_len |= (b << 8)
                    payload = bytearray()
                    if msg_len > 1024:  # Ukuran paket tidak wajar
                        state = "SYNC1"
                    else:
                        state = "PAYLOAD" if msg_len > 0 else "CK_A"
                elif state == "PAYLOAD":
                    payload.append(b)
                    if len(payload) >= msg_len:
                        state = "CK_A"
                elif state == "CK_A":
                    ck_a = b
                    state = "CK_B"
                elif state == "CK_B":
                    ck_b = b
                    state = "SYNC1"

                    # Hitung dan verifikasi checksum
                    hdr_and_pl = bytes([msg_class, msg_id, msg_len & 0xFF, (msg_len >> 8) & 0xFF]) + bytes(payload)
                    calc_a, calc_b = calculate_fletcher_checksum(hdr_and_pl)

                    if calc_a == ck_a and calc_b == ck_b:
                        total_ubx_packets += 1
                        result = parse_ubx_packet(msg_class, msg_id, bytes(payload))
                        print(f"[#{total_ubx_packets:04d}] {result}\n", flush=True)

    except KeyboardInterrupt:
        print("\n\n[*] Inspector dihentikan oleh user.")
    finally:
        ser.close()
        print("=" * 80)
        print(f"[*] Total Bytes Diterima : {total_bytes} bytes")
        print(f"[*] Total Paket UBX Valid: {total_ubx_packets} paket")

def main():
    parser = argparse.ArgumentParser(description="UBX Real-time GPS Inspector")
    parser.add_argument("-p", "--port", type=str, default="COM17", help="Port COM (default: COM17)")
    parser.add_argument("-b", "--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    args = parser.parse_args()

    run_inspector(args.port, args.baud)

if __name__ == "__main__":
    main()
