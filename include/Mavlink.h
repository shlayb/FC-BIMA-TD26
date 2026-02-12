// #ifndef MAV_H
// #define MAV_H

// #include "TimingUtils.h"
// #include "bno055.h"
// #include "AHRS.h"
// #include "Radio.h"
// #include "gps.h"
// #include <common/mavlink.h>
// // --- CONFIG ---
// // #define SDA_PIN 21
// // #define SCL_PIN 22
// // #define MAV_RPI  Serial7    // UART ke Raspberry Pi
// // #define MAV_GCS  Serial     // USB untuk GCS
// // #define PARAM_REQUEST_INTERVAL 1000  // 1 second

// // // --- GPS DATA STRUCTURE ---
// // struct {
// //     int32_t lat;        // latitude in degrees * 1e7
// //     int32_t lon;        // longitude in degrees * 1e7
// //     int32_t alt;        // altitude in mm
// //     uint16_t eph;       // horizontal dilution of precision (cm)
// //     uint16_t epv;       // vertical dilution of precision (cm)
// //     uint16_t vel;       // ground speed (cm/s)
// //     uint16_t cog;       // course over ground (degrees * 100)
// //     uint8_t fix_type;   // 0=No Fix, 1=GPS, 2=DGPS, 3=RTK Fixed, 4=RTK Float
// //     uint8_t satellites_visible;
// // } gps = {0, 0, 0, 0, 0, 0, 0, 0, 0};

// const uint8_t SYSTEM_ID = 1;
// const uint8_t COMPONENT_ID =  MAV_COMP_ID_AUTOPILOT1;
// enum PortId { PORT_USB=0, PORT_TLM=1, PORT_RPI=2 };
// static mavlink_status_t st_usb, st_tlm, st_rpi;
// static mavlink_message_t rxmsg;


// // ALERT THRESHOLDS AND INTERVALS
// const unsigned long VFR_MS = 200;
// unsigned long lastVfr = 0;
// const float ALERT_THRESHOLD_DEG = 40.0f;
// const unsigned long ALERT_DEBOUNCE_MS = 2000;
// const unsigned long HEARTBEAT_MS = 1000;
// const unsigned long ATT_MS       = 50;
// const unsigned long ALT_MS       = 300;
// const unsigned long PARAM_REQUEST_INTERVAL = 1000;
// const unsigned long GPS_MS = 500;
// unsigned long lastGps = 0;
// bool MAV_BOOL_FALSE = false;
// unsigned long lastHb  = 0;
// unsigned long lastParamRequest = 0;
// unsigned long lastAtt = 0;
// unsigned long lastAlt = 0;
// unsigned long lastAlert = 0;

// // SEND MAVLINK THROUGH USB
// // void mavSend(mavlink_message_t *msg) {
// //     uint8_t buf[MAVLINK_MAX_PACKET_LEN];
// //     uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
// //     Serial.write(buf, len);
// // }

// //enum & status
// static inline void mavWrite(HardwareSerial& port, const mavlink_message_t& msg) {
//   uint8_t buf[MAVLINK_MAX_PACKET_LEN];
//   uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
//   port.write(buf, len);
// }

// // overload untuk USB Serial (bukan HardwareSerial) // basic mavlink write function
// static inline void mavWriteUSB(Stream& port, const mavlink_message_t& msg) {
//   uint8_t buf[MAVLINK_MAX_PACKET_LEN];
//   uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
//   port.write(buf, len);
// }

// void mavSendAll(const mavlink_message_t& msg) {
//   mavWriteUSB(Serial, msg);     // USB ke GCS
//   mavWrite(Serial2, msg);       // Telemetry UART
//   mavWrite(Serial7, msg);       // RPi UART
// }

// //=========HELPER FUNCTIONS FOR MAVLINK MESSAGE HANDLING========//

// // Fungsi untuk mengirim COMMAND_ACK
// void sendCommandAck(uint16_t command, uint8_t result) {
//     mavlink_message_t msg;
//     mavlink_msg_command_ack_pack(
//         SYSTEM_ID,
//         COMPONENT_ID,
//         &msg,
//         command,
//         result,
//         0, 0, 0, 0
//     );
//     mavSendAll(msg);
// }

// // Fungsi untuk mengirim STATUSTEXT
// void sendStatusText(uint8_t severity, const char* text) {
//     mavlink_message_t msg;
//     mavlink_msg_statustext_pack(
//         SYSTEM_ID,
//         COMPONENT_ID,
//         &msg,
//         severity,
//         text
//     );
//     mavSendAll(msg);
// }

// //=========== HANDLER FUNCTIONS FOR MAVLINK COMMANDS ==========//
// void handleCommandLong(const mavlink_message_t& msg) {
//     mavlink_command_long_t cmd;
//     mavlink_msg_command_long_decode(&msg, &cmd);
    
//     // Cek apakah ditujukan untuk system ini
//     if (cmd.target_system != SYSTEM_ID && cmd.target_system != 0) {
//         return;
//     }
    
//     uint8_t result = MAV_RESULT_UNSUPPORTED;
    
//     switch (cmd.command) {
//         case MAV_CMD_COMPONENT_ARM_DISARM: {
//             float arm_value = cmd.param1;
//             float force = cmd.param2;
            
//             Serial.printf("ARM REQ: arm=%.1f, force=%.1f, GPS=%d\n", arm_value, force, gepees.getStatus());
            
//             if (arm_value == 1.0f) {
//                 // ARM REQUEST
//                 bool can_arm = true;
//                 // Pre-arm checks
//                 if (gepees.getStatus() < 2 && force != 21196.0f) {
//                     result = MAV_RESULT_DENIED;
//                     sendStatusText(MAV_SEVERITY_WARNING, "ARM DENIED: MAKANYA JANGAN JOMBLO");
//                     break;
//                 }

//                 // if (!imu.isValid()) {
//                 //     result = MAV_RESULT_DENIED;
//                 //     sendStatusText(MAV_SEVERITY_WARNING, "KAMU JOMBLO SIH");
//                 //     break;
//                 // }
                
//                 if (can_arm || force == 21196.0f) {
//                     arming = true;
//                     result = MAV_RESULT_ACCEPTED;
//                     sendStatusText(MAV_SEVERITY_INFO, "ARMED");
//                     Serial.println("ARMED");
//                 } else {
//                     result = MAV_RESULT_DENIED;
//                     sendStatusText(MAV_SEVERITY_WARNING, "ARM DENIED");
//                 }
                
//             } else if (arm_value == 0.0f) {
//                 // DISARM REQUEST
//                 arming = false;
//                 result = MAV_RESULT_ACCEPTED;
//                 sendStatusText(MAV_SEVERITY_INFO, "DISARMED");
//                 Serial.println("DISARMED");
//             }
//             break;
//         }
        
//         default:
//             result = MAV_RESULT_UNSUPPORTED;
//             break;
//     }
    
//     // Kirim ACK
//     sendCommandAck(cmd.command, result);
// }

// // Ini tempat kamu proses command/mission sebagai flight controller kamu
// void processAsFC(const mavlink_message_t& msg, PortId from) {
//     switch (msg.msgid) {
//         case MAVLINK_MSG_ID_COMMAND_LONG:
//             handleCommandLong(msg);
//             break;
            
//         case MAVLINK_MSG_ID_MISSION_COUNT:
//             // TODO: Handle mission upload
//             break;
            
//         case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
//             // TODO: Handle parameter request
//             break;
            
//         default:
//             // Message tidak diproses
//             break;
//     }
// }

// void forwardToOthers(const mavlink_message_t& msg, PortId from) {
//   if (from != PORT_USB) mavWriteUSB(Serial, msg);
//   if (from != PORT_TLM) mavWrite(Serial2, msg);
//   if (from != PORT_RPI) mavWrite(Serial7, msg);
// }

// //=========== PORT HANDLER FUNCTIONS ==========//
// void handlePortUSB() {
//   while (Serial.available()) {
//     uint8_t c = Serial.read();
//     if (mavlink_parse_char(MAVLINK_COMM_0, c, &rxmsg, &st_usb)) {
//       processAsFC(rxmsg, PORT_USB);
//       forwardToOthers(rxmsg, PORT_USB);
//     }
//   }
// }

// void handlePortTLM() {
//   while (Serial2.available()) {
//     uint8_t c = Serial2.read();
//     if (mavlink_parse_char(MAVLINK_COMM_1, c, &rxmsg, &st_tlm)) {
//       processAsFC(rxmsg, PORT_TLM);
//       forwardToOthers(rxmsg, PORT_TLM);
//     }
//   }
// }

// void handlePortRPI() {
//   while (Serial7.available()) {
//     uint8_t c = Serial7.read();
//     if (mavlink_parse_char(MAVLINK_COMM_2, c, &rxmsg, &st_rpi)) {
//       processAsFC(rxmsg, PORT_RPI);
//       forwardToOthers(rxmsg, PORT_RPI);
//     }
//   }
// }

// // HEARTBEAT
// // void sendHeartbeat() {
// //     mavlink_message_t msg;
// //     mavlink_msg_heartbeat_pack(
// //         SYSTEM_ID,
// //         COMPONENT_ID,
// //         &msg,
// //         MAV_TYPE_GENERIC,
// //         MAV_AUTOPILOT_GENERIC,
// //         MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
// //         0,
// //         MAV_STATE_ACTIVE
// //     );
// //     // uint8_t buf[MAVLINK_MAX_PACKET_LEN];
// //     // uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
// //     // Serial7.write(buf, len);
// //     mavSendAll(msg);
// // }

// // =========== SEND MESSAGES ==========//
// void sendHeartbeat() {
//     mavlink_message_t msg;
    
//     uint8_t system_status = arming ? MAV_STATE_ACTIVE : MAV_STATE_STANDBY;
//     uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    
//     if (arming) {
//         base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
//     }
    
//     mavlink_msg_heartbeat_pack(
//         SYSTEM_ID,
//         COMPONENT_ID,
//         &msg,
//         MAV_TYPE_QUADROTOR,  // atau MAV_TYPE_GENERIC
//         MAV_AUTOPILOT_GENERIC,
//         base_mode,
//         0,
//         system_status
//     );
//     mavSendAll(msg);
// }


// // STATUSTEXT
// void sendAlert(const char *txt) {
//     mavlink_message_t msg;
//     mavlink_msg_statustext_pack(
//         SYSTEM_ID,
//         COMPONENT_ID,
//         &msg,
//         MAV_SEVERITY_WARNING,
//         txt
//     );
//     mavSendAll(msg);
// }

// //Altitude
// void sendAltitude(float altitude) {
//     mavlink_message_t msg;
//     uint64_t t_usec = (uint64_t)getMillis() * 1000ULL;
//     mavlink_msg_altitude_pack(
//         SYSTEM_ID,
//         COMPONENT_ID,
//         &msg,
//         t_usec, // time_usec
//         altitude, // altitude_monotonic
//         altitude, // altitude_amsl
//         0, // altitude_local
//         altitude, // altitude_relative, ini yang digunakan di MAVLink Inspector
//         0, // altitude_terrain
//         0.0f      // bottom_clearance
//     );
//     mavSendAll(msg);
// }

// // ATTITUDE
// void sendAttitude(float roll_rad, float pitch_rad, float yaw_rad) {
//     mavlink_message_t msg;
//     mavlink_msg_attitude_pack(
//         SYSTEM_ID,
//         COMPONENT_ID,
//         &msg,
//         getMillis(),
//         roll_rad,
//         pitch_rad,
//         yaw_rad,     // yaw
//         0,0,0  // angular velocities
//     );
//     mavSendAll(msg);
// }

// // GPS_RAW_INT (untuk raw GPS data)
// void sendGpsRawInt(float fix_type, float lat, float lon, float hdop, float vel, float satellites_visible) {
//     mavlink_message_t msg;
//     uint64_t t_usec = (uint64_t)getMillis() * 1000ULL;
//     mavlink_msg_gps_raw_int_pack(
//         SYSTEM_ID,
//         COMPONENT_ID,
//         &msg,
//         t_usec,     // time_usec (uint64_t)
//         fix_type,           // fix_type (uint8_t: 0-5)
//         lat,                // lat (int32_t, degrees * 1e7)
//         lon,                // lon (int32_t, degrees * 1e7)
//         0,                // alt (int32_t, mm above MSL)
//         hdop,                // eph (uint16_t, cm)
//         0,                // epv (uint16_t, cm)
//         vel,                // vel (uint16_t, cm/s)
//         0,                // cog (uint16_t, degrees * 100)
//         satellites_visible, // satellites_visible (uint8_t)
//         0,                      // alt_ellipsoid (int32_t, mm)
//         0,                      // h_acc (uint32_t, mm)
//         0,                      // v_acc (uint32_t, mm)
//         0,                      // vel_acc (uint32_t, mm/s)
//         0                       // hdg_acc (uint32_t, degrees * 100000)
//     );
//     mavSendAll(msg);
// }

// // void handleIncomingMavlink() {
// //     mavlink_message_t msg;
// //     mavlink_status_t status;
    
// //     while (Serial.available() > 0) {
// //         uint8_t c = Serial.read(); //keUSB
// //         if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
// //             // Handle PARAM_REQUEST_LIST
// //             if (msg.msgid == MAVLINK_MSG_ID_PARAM_REQUEST_LIST) {
// //                 // Send your parameters here
// //             }
// //             // Handle PARAM_REQUEST_READ
// //             if (msg.msgid == MAVLINK_MSG_ID_PARAM_REQUEST_READ) {
// //                 // Send specific parameter
// //             }
// //         }
// //     }
// // }

// // void sendVfrHud(float alt_m, float climb_mps) {
// //   mavlink_message_t msg;
// //   mavlink_msg_vfr_hud_pack(
// //     SYSTEM_ID,
// //     COMPONENT_ID,
// //     &msg,
// //     0.0f,          // airspeed (m/s) - boleh 0
// //     0.0f,          // groundspeed (m/s) - boleh 0
// //     0,             // heading (deg) - boleh 0
// //     0,             // throttle (%) - boleh 0
// //     alt_m,         // altitude (m)
// //     climb_mps      // climb rate (m/s)
// //   );
// //   mavSendAll(msg);
// // }

// // mavlink_msg_command_long_pack
// // void armDisarmComponent(uint8_t arm, uint8_t force) {
// //     mavlink_message_t msg;
// //     mavlink_msg_command_long_pack(
// //         SYSTEM_ID,
// //         COMPONENT_ID,
// //         &msg,
// //         SYSTEM_ID,          // target_system
// //         COMPONENT_ID,       // target_component
// //         MAV_CMD_COMPONENT_ARM_DISARM, // command
// //         0,                  // confirmation
// //         arm,                // param1: 1 to arm, 0 to disarm
// //         force,              // param2: force flag
// //         0,0,0,0,0          // param3 - param7: unused
// //     );
// //     mavSendAll(msg);
// // }

// // GLOBAL_POSITION_INT (dengan GPS + altitude)
// // void sendGlobalPositionInt() {
// //     mavlink_message_t msg;
// //     mavlink_msg_global_position_int_pack(
// //         SYSTEM_ID,
// //         COMPONENT_ID,
// //         &msg,
// //         getMillis(),            // time_boot_ms
// //         gps.lat,                // lat (degrees * 1e7)
// //         gps.lon,                // lon (degrees * 1e7)
// //         gps.alt,                // alt in mm above MSL
// //         (int32_t)(baro.altitude * 1000),  // relative_alt in mm
// //         0,                      // vx (velocity x in cm/s)
// //         0,                      // vy (velocity y in cm/s)
// //         0,                      // vz (velocity z in cm/s)
// //         0                       // hdg (heading in degrees * 100)
// //     );
// //     mavSend(&msg);
// // }

// void mavlinkTask() {
//     unsigned long now = getMillis();
//     // float roll_rad = imu.rad_roll;
//     // float pitch_rad = imu.rad_pitch;
//     // float yaw_rad = imu.rad_yaw;

//     if (now - lastHb >= HEARTBEAT_MS) {
//         lastHb = now;
//         sendHeartbeat();
//     }

//     // ALTITUDE
//     if (now - lastAlt >= ALT_MS) {
//         lastAlt = now;
//         float alt = baro.altitude;  // Ganti dengan sensor altitude sebenarnya
//         sendAltitude(alt);
//         // Serial.printf("Altitude sent: %.2f m\n", alt);  // Debug
//     }

//     // GPS
//     if (now - lastGps >= GPS_MS) {
//         lastGps = now;
//         sendGpsRawInt(
//             gepees.getStatus(),
//             gepees.latitude * 1e7,
//             gepees.longitude * 1e7,
//             gepees.hdop * 100, // convert to cm
//             gepees.speed_mps * 100, // convert m/s to cm/s
//             gepees.satellites
//         );
//         // sendGlobalPositionInt();
//         // Serial.printf("GPS sent: lat=%.7f, lon=%.7f, alt=%.2f m, sats=%d\n", 
//         //     gepees.latitude/1e7, gepees.longitude/1e7, gepees.satellites);
//     }

//     // ATTITUDE
//     if (now - lastAtt >= ATT_MS) {
//         lastAtt = now;
//         sendAttitude(imu.rad_roll, imu.rad_pitch, imu.rad_yaw);
//     }

//     // ALERT logic
//     if (fabs(imu.roll) > ALERT_THRESHOLD_DEG) {
//         if (now - lastAlert >= ALERT_DEBOUNCE_MS) {
//             char msg[50];
//             snprintf(msg, sizeof(msg), "ALERT: ROLL LIMIT (%.1f deg)", imu.roll);
//             sendAlert(msg);
//             // Serial.println(msg);
//             lastAlert = now;
//         }
//     }
    
//     // ARM/DISARM LOGIC
//     // armDisarmComponent(arming, MAV_BOOL_FALSE);
    
//     // if (now - lastVfr >= VFR_MS) {
//     // lastVfr = now;
//     // sendVfrHud(baro.altitude, 0.0f);
//     // }


//     // if (now - lastParamRequest >= PARAM_REQUEST_INTERVAL) {      //dihapus aja
//     //     lastParamRequest = now;
//     //     // handleIncomingMavlink();  // Replace with the actual parameter name
//     // }
// }
    
// #endif

#ifndef MAV_H
#define MAV_H

#include "TimingUtils.h"
#include "bno055.h"
#include "AHRS.h"
#include "Radio.h"
#include "gps.h"
#include <common/mavlink.h>
#include <cstring>  // Untuk memset

// Enum for port IDs
enum PortId { PORT_USB = 0, PORT_TLM = 1, PORT_RPI = 2 };

class MavlinkHandler {
private:
    // Constants
    static const uint8_t SYSTEM_ID = 1;
    static const uint8_t COMPONENT_ID = MAV_COMP_ID_AUTOPILOT1;
    static const unsigned long VFR_MS = 200;
    static constexpr float ALERT_THRESHOLD_DEG = 40.0f;  // Gunakan constexpr untuk float
    static const unsigned long ALERT_DEBOUNCE_MS = 2000;
    static const unsigned long HEARTBEAT_MS = 1000;
    static const unsigned long ATT_MS = 50;
    static const unsigned long ALT_MS = 300;
    static const unsigned long GPS_MS = 500;
    static const unsigned long PARAM_REQUEST_INTERVAL = 1000;

    // Timers
    unsigned long lastVfr = 0;
    unsigned long lastGps = 0;
    unsigned long lastHb = 0;
    unsigned long lastParamRequest = 0;
    unsigned long lastAtt = 0;
    unsigned long lastAlt = 0;
    unsigned long lastAlert = 0;

    // MAVLink status and message buffers
    mavlink_status_t st_usb;
    mavlink_status_t st_tlm;
    mavlink_status_t st_rpi;
    mavlink_message_t rxmsg;

    // Private methods for internal use
    void mavWrite(HardwareSerial& port, const mavlink_message_t& msg);
    void mavWriteUSB(Stream& port, const mavlink_message_t& msg);
    void mavSendAll(const mavlink_message_t& msg);
    void sendCommandAck(uint16_t command, uint8_t result);
    void sendStatusText(uint8_t severity, const char* text);
    void handleCommandLong(const mavlink_message_t& msg);
    void handleStatusText(const mavlink_message_t& msg);  // decode dan print STATUSTEXT
    void processAsFC(const mavlink_message_t& msg, PortId from);
    void forwardToOthers(const mavlink_message_t& msg, PortId from);
    void sendHeartbeat();
    void sendAlert(const char* txt);
    void sendAltitude(float altitude);
    void sendAttitude(float roll_rad, float pitch_rad, float yaw_rad);
    void sendGpsRawInt(uint8_t fix_type, int32_t lat, int32_t lon, uint16_t eph, uint16_t vel, uint8_t satellites_visible);

public:
    // Constructor for initialization
    MavlinkHandler();

    // Public methods
    void mavlinkTask();  // Main periodic task
    void handlePorts();  // Handle incoming data from all ports
    void sendCustomTextToGCS(const char* text);  // Method baru: Kirim pesan custom ke GCS
};

/////////////////////// Implementation ///////////////////////

// Constructor
MavlinkHandler::MavlinkHandler() {
    // Initialize statuses to zero
    memset(&st_usb, 0, sizeof(st_usb));
    memset(&st_tlm, 0, sizeof(st_tlm));
    memset(&st_rpi, 0, sizeof(st_rpi));
}

// Write to HardwareSerial
void MavlinkHandler::mavWrite(HardwareSerial& port, const mavlink_message_t& msg) {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    port.write(buf, len);
}

// Write to Stream (USB)
void MavlinkHandler::mavWriteUSB(Stream& port, const mavlink_message_t& msg) {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    port.write(buf, len);
}

// Send to all ports
void MavlinkHandler::mavSendAll(const mavlink_message_t& msg) {
    mavWriteUSB(Serial, msg);  // USB to GCS
    mavWrite(Serial2, msg);    // Telemetry UART
    mavWrite(Serial7, msg);    // RPi UART
}

// Send COMMAND_ACK
void MavlinkHandler::sendCommandAck(uint16_t command, uint8_t result) {
    mavlink_message_t msg;
    mavlink_msg_command_ack_pack(SYSTEM_ID, COMPONENT_ID, &msg, command, result, 0, 0, 0, 0);
    mavSendAll(msg);
}

// Send STATUSTEXT (hanya 5 argumen untuk kompatibilitas)
void MavlinkHandler::sendStatusText(uint8_t severity, const char* text) {
    mavlink_message_t msg;
    mavlink_msg_statustext_pack(SYSTEM_ID, COMPONENT_ID, &msg, severity, text);
    mavSendAll(msg);
}

// Handle incoming STATUSTEXT (decode dan print ke Serial) - Fitur baru
void MavlinkHandler::handleStatusText(const mavlink_message_t& msg) {
    mavlink_statustext_t status;
    mavlink_msg_statustext_decode(&msg, &status);

    // Cek jika pesan dari diri sendiri (untuk cegah loop infinite)
    if (msg.sysid == SYSTEM_ID && msg.compid == COMPONENT_ID) {
        return;  // Jangan process jika dari FC sendiri
    }

    // Buat text echo back (tambah prefix untuk logging)
    char echo_text[50];
    snprintf(echo_text, sizeof(echo_text), "Echo: %s", status.text);

    // Kirim balik ke GCS dengan severity INFO
    sendStatusText(MAV_SEVERITY_INFO, echo_text);
}

void MavlinkHandler::sendCustomTextToGCS(const char* text) {
    sendStatusText(MAV_SEVERITY_INFO, text);  // Kirim dengan severity INFO (bisa ubah)
}

// Handle COMMAND_LONG
void MavlinkHandler::handleCommandLong(const mavlink_message_t& msg) {
    mavlink_command_long_t cmd;
    mavlink_msg_command_long_decode(&msg, &cmd);

    if (cmd.target_system != SYSTEM_ID && cmd.target_system != 0) {
        return;
    }

    uint8_t result = MAV_RESULT_UNSUPPORTED;

    switch (cmd.command) {
        case MAV_CMD_COMPONENT_ARM_DISARM: {
            float arm_value = cmd.param1;
            float force = cmd.param2;

            Serial.printf("ARM REQ: arm=%.1f, force=%.1f, GPS=%d\n", arm_value, force, gepees.getStatus());

            if (arm_value == 1.0f) {
                bool can_arm = true;
                if (gepees.getStatus() < 2 && force != 21196.0f) {
                    result = MAV_RESULT_DENIED;
                    sendStatusText(MAV_SEVERITY_WARNING, "ARM DENIED: MAKANYA JANGAN JOMBLO");
                    break;
                }

                if (can_arm || force == 21196.0f) {
                    arming = true;
                    result = MAV_RESULT_ACCEPTED;
                    sendStatusText(MAV_SEVERITY_INFO, "ARMED");
                    Serial.println("ARMED");
                } else {
                    result = MAV_RESULT_DENIED;
                    sendStatusText(MAV_SEVERITY_WARNING, "ARM DENIED");
                }
            } else if (arm_value == 0.0f) {
                arming = false;
                result = MAV_RESULT_ACCEPTED;
                sendStatusText(MAV_SEVERITY_INFO, "DISARMED");
                Serial.println("DISARMED");
            }
            break;
        }
        default:
            result = MAV_RESULT_UNSUPPORTED;
            break;
    }

    sendCommandAck(cmd.command, result);
}

// Process as Flight Controller
void MavlinkHandler::processAsFC(const mavlink_message_t& msg, PortId from) {
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_COMMAND_LONG:
            handleCommandLong(msg);
            break;
        case MAVLINK_MSG_ID_STATUSTEXT:  // handle STATUSTEXT dari GCS
            handleStatusText(msg);
            break;
        case MAVLINK_MSG_ID_MISSION_COUNT:
            // TODO: Handle mission upload
            break;
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
            // TODO: Handle parameter request
            break;
        default:
            // Message not processed
            break;
    }
}

// Forward to other ports
void MavlinkHandler::forwardToOthers(const mavlink_message_t& msg, PortId from) {
    if (from != PORT_USB) mavWriteUSB(Serial, msg);
    if (from != PORT_TLM) mavWrite(Serial2, msg);
    if (from != PORT_RPI) mavWrite(Serial7, msg);
}

// Send HEARTBEAT
void MavlinkHandler::sendHeartbeat() {
    mavlink_message_t msg;
    uint8_t system_status = arming ? MAV_STATE_ACTIVE : MAV_STATE_STANDBY;
    uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    if (arming) base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;

    mavlink_msg_heartbeat_pack(SYSTEM_ID, COMPONENT_ID, &msg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, base_mode, 0, system_status);
    mavSendAll(msg);
}

// Send ALERT (hanya 5 argumen untuk kompatibilitas)
void MavlinkHandler::sendAlert(const char* txt) {
    mavlink_message_t msg;
    mavlink_msg_statustext_pack(SYSTEM_ID, COMPONENT_ID, &msg, MAV_SEVERITY_WARNING, txt);
    mavSendAll(msg);
}

// Send ALTITUDE
void MavlinkHandler::sendAltitude(float altitude) {
    mavlink_message_t msg;
    uint64_t t_usec = (uint64_t)getMillis() * 1000ULL;
    mavlink_msg_altitude_pack(SYSTEM_ID, COMPONENT_ID, &msg, t_usec, altitude, altitude, 0, altitude, 0, 0.0f);
    mavSendAll(msg);
}

// Send ATTITUDE
void MavlinkHandler::sendAttitude(float roll_rad, float pitch_rad, float yaw_rad) {
    mavlink_message_t msg;
    mavlink_msg_attitude_pack(SYSTEM_ID, COMPONENT_ID, &msg, getMillis(), roll_rad, pitch_rad, yaw_rad, 0, 0, 0);
    mavSendAll(msg);
}

// Send GPS_RAW_INT
void MavlinkHandler::sendGpsRawInt(uint8_t fix_type, int32_t lat, int32_t lon, uint16_t eph, uint16_t vel, uint8_t satellites_visible) {
    mavlink_message_t msg;
    uint64_t t_usec = (uint64_t)getMillis() * 1000ULL;
    mavlink_msg_gps_raw_int_pack(SYSTEM_ID, COMPONENT_ID, &msg, t_usec, fix_type, lat, lon, 0, eph, 0, vel, 0, satellites_visible, 0, 0, 0, 0, 0);
    mavSendAll(msg);
}

// Handle incoming data from all ports
void MavlinkHandler::handlePorts() {
    // USB
    while (Serial.available()) {
        uint8_t c = Serial.read();
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &rxmsg, &st_usb)) {
            processAsFC(rxmsg, PORT_USB);
            forwardToOthers(rxmsg, PORT_USB);
        }
    }

    // Telemetry
    while (Serial2.available()) {
        uint8_t c = Serial2.read();
        if (mavlink_parse_char(MAVLINK_COMM_1, c, &rxmsg, &st_tlm)) {
            processAsFC(rxmsg, PORT_TLM);
            forwardToOthers(rxmsg, PORT_TLM);
        }
    }

    // RPi
    while (Serial7.available()) {
        uint8_t c = Serial7.read();
        if (mavlink_parse_char(MAVLINK_COMM_2, c, &rxmsg, &st_rpi)) {
            processAsFC(rxmsg, PORT_RPI);
            forwardToOthers(rxmsg, PORT_RPI);
        }
    }
}

// Main periodic task
void MavlinkHandler::mavlinkTask() {
    unsigned long now = getMillis();
    static unsigned long lastCustom = 0;

    if (now - lastHb >= HEARTBEAT_MS) {
        lastHb = now;
        sendHeartbeat();
    }

    if (now - lastAlt >= ALT_MS) {
        lastAlt = now;
        float alt = baro.altitude;  // Replace with actual sensor
        sendAltitude(alt);
    }

    if (now - lastGps >= GPS_MS) {
        lastGps = now;
        sendGpsRawInt(gepees.getStatus(), (int32_t)(gepees.latitude * 1e7), (int32_t)(gepees.longitude * 1e7),
                      (uint16_t)(gepees.hdop * 100), (uint16_t)(gepees.speed_mps * 100), gepees.satellites);
    }

    if (now - lastAtt >= ATT_MS) {
        lastAtt = now;
        sendAttitude(imu.rad_roll, imu.rad_pitch, imu.rad_yaw);
    }

    if (fabs(imu.roll) > ALERT_THRESHOLD_DEG) {
        if (now - lastAlert >= ALERT_DEBOUNCE_MS) {
            char msg[50];
            snprintf(msg, sizeof(msg), "ALERT: ROLL LIMIT (%.1f deg)", imu.roll);
            sendAlert(msg);
            lastAlert = now;
        }
    }
   
    // if (millis() - lastCustom >= 5000) {
    //     lastCustom = millis();
    //     sendCustomTextToGCS("Pesan custom dari FC!");
    // }
}

#endif