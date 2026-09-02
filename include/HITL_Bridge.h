#pragma once

#include <Arduino.h>
#include "AHRS.h"
#include "Barometer.h"
#include "bno055.h"
#include "gps.h"
#include "airspeed.h"
#include "Radio.h"
#include "TECS.h"
#include "L1_Controller.h"
#include "FW_ControlModes.h"
#include "FW_control.h"
#include "Mode_setup.h"

// ============================================================
//  HITL CONFIGURATION & STRUCTS
// ============================================================
#define HITL_MAGIC_HEADER_0  0xAA
#define HITL_MAGIC_HEADER_1  0x55

#pragma pack(push, 1)

// Packet Sensor: Dikirim dari PC (Simulator) -> Teensy (FC)
struct HITL_SensorPacket {
    uint8_t  magic0;
    uint8_t  magic1;
    uint32_t timestamp_ms;
    
    // IMU (Attitude & Rates & Accel)
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float gyro_x;     // deg/s
    float gyro_y;     // deg/s
    float gyro_z;     // deg/s
    float accel_x;    // m/s^2 body X (forward)
    float accel_y;    // m/s^2 body Y (right)
    float accel_z;    // m/s^2 body Z (down)
    
    // Barometer & Airspeed
    float baro_alt_m; // Altitude relative to home (m)
    float climb_rate; // m/s
    float tas_mps;    // True Airspeed (m/s)
    float eas_mps;    // Equivalent Airspeed (m/s)
    
    // GPS & Groundspeed
    int32_t lat_1e7;  // Latitude * 1e7
    int32_t lon_1e7;  // Longitude * 1e7
    float   gps_alt_m;
    float   groundspeed_mps;
    float   vel_n_mps; // North velocity
    float   vel_e_mps; // East velocity
    float   vel_d_mps; // Down velocity
    
    // Control Flags
    uint8_t flags;    // bit0: is_airspeed_valid, bit1: arming_override, bit2: force_auto_mode
    uint16_t checksum;
};

// Packet Actuator: Dikirim dari Teensy (FC) -> PC (Simulator)
struct HITL_ActuatorPacket {
    uint8_t  magic0;
    uint8_t  magic1;
    uint32_t timestamp_ms;
    
    // Servo & Motor Signals
    int16_t  servo_ail_R;      // -400 .. 400
    int16_t  servo_ail_L;      // -400 .. 400
    int16_t  servo_ele;        // -400 .. 400
    int16_t  servo_rud;        // -400 .. 400
    uint16_t throttle_pwm;     // 1000 .. 2000 us
    
    // TECS & Outer-Loop Demands
    float demanded_pitch_deg;  // Output dari TECS / calc_nav_pitch
    float demanded_roll_deg;   // Output dari L1 Navigation
    float demanded_throttle_pct; // 0.0 - 100.0% dari TECS
    
    // TECS Internal Diagnostics
    float tecs_integ_seb;      // Pitch energy balance integrator
    float tecs_integ_thr;      // Throttle integrator
    float tecs_ste_error;      // Total specific energy error
    float tecs_target_alt_m;   // Demanded target altitude (m)
    
    uint8_t  current_mode;     // 1=MANU, 2=FBWA, 3=AUTO, 4=GUIDED
    uint8_t  is_armed;
    uint16_t checksum;
};

#pragma pack(pop)

class HITLBridge {
public:
    bool enabled = false;
    uint32_t last_packet_time_ms = 0;
    uint32_t packets_received = 0;
    uint32_t packets_sent = 0;

    void init(bool enable_hitl) {
        enabled = enable_hitl;
    }

    // Hitung Checksum Fletcher-16
    static uint16_t calcChecksum(const uint8_t* data, size_t len) {
        uint16_t sum1 = 0, sum2 = 0;
        for (size_t i = 0; i < len; ++i) {
            sum1 = (sum1 + data[i]) % 255;
            sum2 = (sum2 + sum1) % 255;
        }
        return (sum2 << 8) | sum1;
    }

    // Process incoming serial bytes from USB
    void processSerial() {
        if (!enabled) return;

        static uint8_t rx_buf[sizeof(HITL_SensorPacket)];
        static size_t  rx_idx = 0;

        while (Serial.available()) {
            uint8_t b = Serial.read();

            if (rx_idx == 0) {
                if (b == HITL_MAGIC_HEADER_0) {
                    rx_buf[rx_idx++] = b;
                }
            } else if (rx_idx == 1) {
                if (b == HITL_MAGIC_HEADER_1) {
                    rx_buf[rx_idx++] = b;
                } else {
                    rx_idx = 0; // Invalid sync
                }
            } else {
                rx_buf[rx_idx++] = b;

                if (rx_idx >= sizeof(HITL_SensorPacket)) {
                    // Packet fully received
                    HITL_SensorPacket* pkt = (HITL_SensorPacket*)rx_buf;
                    
                    // Verify checksum (excluding the 2 checksum bytes)
                    uint16_t expected_cs = calcChecksum(rx_buf, sizeof(HITL_SensorPacket) - sizeof(uint16_t));
                    if (pkt->checksum == expected_cs) {
                        injectSensorData(*pkt);
                        packets_received++;
                        last_packet_time_ms = millis();
                        
                        // Send actuator telemetry immediately back to simulator
                        sendActuatorFeedback();
                    }
                    rx_idx = 0; // Reset for next packet
                }
            }
        }
    }

    // Inject simulated physics state into firmware objects
    void injectSensorData(const HITL_SensorPacket& pkt) {
        // 1. IMU injection
        imu.roll       = pkt.roll_deg;
        imu.pitch      = pkt.pitch_deg;
        imu.yaw        = pkt.yaw_deg;
        imu.heading    = pkt.yaw_deg;
        imu.rad_roll   = radians(pkt.roll_deg);
        imu.rad_pitch  = radians(pkt.pitch_deg);
        imu.rad_yaw    = radians(pkt.yaw_deg);
        
        // Mapping gyro sesuai konvensi sumbu FC Khageswara:
        // updateAUTO_FW / updateFBWA_FW memetakan gyro_x ke pitch D-term dan gyro_y ke roll D-term
        imu.gyro_x     = pkt.gyro_y; // Pitch Rate (q)
        imu.gyro_y     = pkt.gyro_x; // Roll Rate (p)
        imu.gyro_z     = pkt.gyro_z; // Yaw Rate (r)
        
        imu._accel.x   = pkt.accel_x;
        imu._accel.y   = pkt.accel_y;
        imu._accel.z   = pkt.accel_z;

        // 2. Barometer injection
        baro.altitude  = pkt.baro_alt_m;

        // 3. Airspeed injection & sensor enable flag
        bool aspd_valid = (pkt.flags & 0x01) != 0;
        ahrs.is_airspeed_sensor_enabled = aspd_valid;
        arspd.v_ms     = pkt.tas_mps;
        arspd.v_kmh    = pkt.tas_mps * 3.6f;

        // 4. GPS injection
        gepees.latitude   = pkt.lat_1e7 / 10000000.0f;
        gepees.longitude  = pkt.lon_1e7 / 10000000.0f;
        gepees.alt_m      = pkt.gps_alt_m;
        gepees.speed_mps  = pkt.groundspeed_mps;
        gepees.speed_kmph = pkt.groundspeed_mps * 3.6f;
        gepees.gps_status = GPS_FIX_3D;
        gepees.age        = 10;
        gepees.velocity   = Vector3f(pkt.vel_n_mps, pkt.vel_e_mps, pkt.vel_d_mps);

        // 5. AHRS state update
        ahrs.groundspeed  = pkt.groundspeed_mps;

        // 6. Arming & Mode control overrides from simulator
        if (pkt.flags & 0x02) {
            arming = true;
        } else {
            arming = false;
        }

        if (pkt.flags & 0x04) {
            // Force AUTO Mode: hanya saat mission belum mulai/di tengah jalan
            // JANGAN force AUTO jika:
            //  - current sudah RTL (misi selesai)
            //  - mode_now sudah 5 (navigate() baru saja minta RTL, belum diproses setup_mode())
            //  - flag_wp >= wp_sum (semua waypoint misi selesai)
            //  - current sudah AUTO (tidak perlu force ulang)
            if (current != ModeId::AUTO && current != ModeId::RTL && mode_now != 5 && (wp_sum == 0 || flag_wp < wp_sum)) {
                mode_now = 3;
                modes.setMode(ModeId::AUTO);
                active_mode = ModeId::AUTO;
                current = ModeId::AUTO;
            }
        }
    }

    // Send actuator commands & TECS telemetry to Python
    void sendActuatorFeedback() {
        HITL_ActuatorPacket out_pkt;
        memset(&out_pkt, 0, sizeof(out_pkt));

        out_pkt.magic0 = HITL_MAGIC_HEADER_0;
        out_pkt.magic1 = HITL_MAGIC_HEADER_1;
        out_pkt.timestamp_ms = millis();

        // Control surfaces & motor PWM
        out_pkt.servo_ail_R  = fw_control.servos[0];
        out_pkt.servo_ail_L  = fw_control.servos[1];
        out_pkt.servo_ele    = fw_control.servos[2];
        out_pkt.servo_rud    = fw_control.servos[3];
        out_pkt.throttle_pwm = fw_control.get_auto_throttle();

        // Demanded outer loop targets
        out_pkt.demanded_pitch_deg    = get_pitch_demand();
        out_pkt.demanded_roll_deg     = nav_roll_cd() / 100.0f;
        out_pkt.demanded_throttle_pct = (float)get_throttle_demand();

        // TECS Internal Diagnostics
        out_pkt.tecs_integ_seb    = _integSEBdot;
        out_pkt.tecs_integ_thr    = _integTHR_state;
        out_pkt.tecs_ste_error    = _STE_error;
        out_pkt.tecs_target_alt_m = target_altitude.amsl_cm * 0.01f;

        // Encode flight mode & active WP:
        // 1=MANU, 2=FBWA, 4=GUIDED, 200=RTL, 10+wp=AUTO (e.g. 11=WP1, 12=WP2, 13=WP3, 14=WP4)
        uint8_t mode_val = 0;
        if (current == ModeId::MANU) mode_val = 1;
        else if (current == ModeId::FBWA) mode_val = 2;
        else if (current == ModeId::GUIDED) mode_val = 4;
        else if (current == ModeId::RTL) mode_val = 200;
        else if (current == ModeId::AUTO) {
            mode_val = (uint8_t)(10 + (flag_wp >= 0 ? flag_wp : 0));
        }

        out_pkt.current_mode = mode_val;
        out_pkt.is_armed     = arming ? 1 : 0;

        out_pkt.checksum = calcChecksum((const uint8_t*)&out_pkt, sizeof(HITL_ActuatorPacket) - sizeof(uint16_t));

        Serial.write((const uint8_t*)&out_pkt, sizeof(out_pkt));
        packets_sent++;
    }
};

inline HITLBridge hitl;

