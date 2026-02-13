#ifndef MAV_H
#define MAV_H

#include "TimingUtils.h"
#include "bno055.h"
#include "AHRS.h"
#include "Radio.h"
#include "gps.h"
#include <common/mavlink.h>
#include <cstring>  // Untuk memset

// External declarations untuk waypoint storage
extern Locations waypoint[100];
extern int wp_sum;

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

    // Mission upload state
    uint16_t mission_count = 0;
    uint16_t mission_items_received = 0;
    bool mission_upload_in_progress = false;
    unsigned long mission_timeout = 0;
    const unsigned long MISSION_TIMEOUT_MS = 5000;  // 5 second timeout

    // Mission download state
    uint16_t mission_download_seq = 0;
    bool mission_download_in_progress = false;

    // Private methods for internal use
    void mavWrite(HardwareSerial& port, const mavlink_message_t& msg);
    void mavWriteUSB(Stream& port, const mavlink_message_t& msg);
    void mavSendAll(const mavlink_message_t& msg);
    void sendCommandAck(uint16_t command, uint8_t result);
    void sendStatusText(uint8_t severity, const char* text);
    void handleCommandLong(const mavlink_message_t& msg);
    void handleStatusText(const mavlink_message_t& msg);  // decode dan print STATUSTEXT
    void handleMissionCount(const mavlink_message_t& msg);
    void handleMissionItem(const mavlink_message_t& msg);
    void sendMissionRequest(uint16_t sequence);
    void sendMissionAck(uint8_t type);
    void sendMissionCount(uint8_t target_system);
    void sendMissionItem(uint16_t sequence);
    void handleMissionRequestList(const mavlink_message_t& msg);
    void handleMissionRequest(const mavlink_message_t& msg);
    void handleClearMission(const mavlink_message_t& msg);
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

// Send MISSION_REQUEST to request waypoint from GCS
void MavlinkHandler::sendMissionRequest(uint16_t sequence) {
    mavlink_message_t msg;
    mavlink_msg_mission_request_pack(
        SYSTEM_ID,
        COMPONENT_ID,
        &msg,
        0,              // target_system (0 = broadcast)
        0,              // target_component
        sequence,       // seq
        MAV_MISSION_TYPE_MISSION  // mission_type
    );
    mavSendAll(msg);
    Serial.printf("Requesting WP#%d\n", sequence);
}

// Send MISSION_ACK when mission upload complete
void MavlinkHandler::sendMissionAck(uint8_t type) {
    mavlink_message_t msg;
    mavlink_msg_mission_ack_pack(
        SYSTEM_ID,
        COMPONENT_ID,
        &msg,
        0,              // target_system (0 = broadcast)
        0,              // target_component
        type,           // type: 0=success, 1=error, etc
        MAV_MISSION_TYPE_MISSION  // mission_type
    );
    mavSendAll(msg);
    Serial.printf("Mission ACK sent (type=%d)\n", type);
}

// Handle MISSION_COUNT message from GCS
void MavlinkHandler::handleMissionCount(const mavlink_message_t& msg) {
    mavlink_mission_count_t count_msg;
    mavlink_msg_mission_count_decode(&msg, &count_msg);

    mission_count = count_msg.count;
    mission_items_received = 0;
    mission_upload_in_progress = true;
    mission_timeout = getMillis();

    Serial.printf("Mission upload started: expecting %d waypoints\n", mission_count);
    
    // Request first waypoint (index 0)
    if (mission_count > 0) {
        sendMissionRequest(0);
    } else {
        // No waypoints to upload
        sendMissionAck(0);  // MAV_MISSION_ACCEPTED
        mission_upload_in_progress = false;
    }
}

// Handle MISSION_ITEM message from GCS
void MavlinkHandler::handleMissionItem(const mavlink_message_t& msg) {
    if (!mission_upload_in_progress) {
        Serial.println("Warning: Received MISSION_ITEM but no upload in progress");
        return;
    }

    // Check timeout
    if (getMillis() - mission_timeout > MISSION_TIMEOUT_MS) {
        Serial.println("ERROR: Mission upload timeout!");
        mission_upload_in_progress = false;
        sendMissionAck(1);  // MAV_MISSION_ERROR
        return;
    }

    mavlink_mission_item_int_t mission_item;
    
    // Try to decode as mission_item_int first (more precise)
    if (msg.msgid == MAVLINK_MSG_ID_MISSION_ITEM_INT) {
        mavlink_msg_mission_item_int_decode(&msg, &mission_item);
    } else {
        // Fallback untuk mission_item (old format)
        mavlink_mission_item_t old_format;
        mavlink_msg_mission_item_decode(&msg, &old_format);
        mission_item.x = (int32_t)(old_format.x * 1e7);
        mission_item.y = (int32_t)(old_format.y * 1e7);
        mission_item.z = old_format.z;
        mission_item.seq = old_format.seq;
    }

    uint16_t wp_index = mission_item.seq;

    // Store waypoint in array
    if (wp_index < 100) {
        waypoint[wp_index].lat = mission_item.x;      // lat in degrees * 1e7
        waypoint[wp_index].lng = mission_item.y;      // lng in degrees * 1e7
        waypoint[wp_index].alt = (int32_t)(mission_item.z * 100);  // alt in cm

        mission_items_received++;
        Serial.printf("WP#%d received: lat=%.7f, lng=%.7f, alt=%.2f m\n",
                      wp_index,
                      mission_item.x / 1e7,
                      mission_item.y / 1e7,
                      mission_item.z);

        // Request next waypoint or complete upload
        if (mission_items_received < mission_count) {
            sendMissionRequest(mission_items_received);
        } else {
            // All waypoints received
            wp_sum = mission_count;  // Update total waypoint count
            Serial.printf("Mission upload complete! Total waypoints: %d\n", wp_sum);
            sendMissionAck(0);  // MAV_MISSION_ACCEPTED
            mission_upload_in_progress = false;
        }
    } else {
        Serial.printf("ERROR: Waypoint index %d out of range!\n", wp_index);
        sendMissionAck(1);  // MAV_MISSION_ERROR
        mission_upload_in_progress = false;
    }
}

// Send MISSION_COUNT to GCS (when GCS requests mission list)
void MavlinkHandler::sendMissionCount(uint8_t target_system) {
    mavlink_message_t msg;
    mavlink_msg_mission_count_pack(
        SYSTEM_ID,
        COMPONENT_ID,
        &msg,
        target_system,
        0,              // target_component
        wp_sum,         // count: jumlah waypoint yang ada
        MAV_MISSION_TYPE_MISSION
    );
    mavSendAll(msg);
    Serial.printf("Mission count sent: %d waypoints\n", wp_sum);
}

// Send individual MISSION_ITEM to GCS
void MavlinkHandler::sendMissionItem(uint16_t sequence) {
    if (sequence >= wp_sum) {
        Serial.printf("ERROR: Sequence %d out of range (total WP: %d)\n", sequence, wp_sum);
        return;
    }

    mavlink_message_t msg;
    
    // Gunakan MISSION_ITEM_INT untuk precision lebih baik
    mavlink_msg_mission_item_int_pack(
        SYSTEM_ID,
        COMPONENT_ID,
        &msg,
        0,              // target_system (broadcast)
        0,              // target_component
        sequence,       // seq
        MAV_FRAME_GLOBAL_RELATIVE_ALT,  // frame
        MAV_CMD_NAV_WAYPOINT,           // command
        0,              // current (0=not current)
        1,              // autocontinue
        0, 0, 0, 0,     // param1-4 (unused for waypoint)
        waypoint[sequence].lat,         // x = latitude (int32, degrees * 1e7)
        waypoint[sequence].lng,         // y = longitude (int32, degrees * 1e7)
        waypoint[sequence].alt / 100.0f, // z = altitude (float, in meters)
        MAV_MISSION_TYPE_MISSION
    );
    mavSendAll(msg);
    Serial.printf("Sent WP#%d: lat=%.7f, lng=%.7f, alt=%.2f m\n",
                  sequence,
                  waypoint[sequence].lat / 1e7,
                  waypoint[sequence].lng / 1e7,
                  waypoint[sequence].alt / 100.0f);
}

// Handle MISSION_REQUEST_LIST from GCS (user click "Read Waypoints")
void MavlinkHandler::handleMissionRequestList(const mavlink_message_t& msg) {
    mavlink_mission_request_list_t req_list;
    mavlink_msg_mission_request_list_decode(&msg, &req_list);

    Serial.printf("Mission request list received from sys %d, comp %d\n", 
        req_list.target_system, req_list.target_component);

    // Start mission download
    mission_download_in_progress = true;
    mission_download_seq = 0;

    // Send total mission count
    sendMissionCount(msg.sysid);  // Send count to the requester
}

// Handle MISSION_REQUEST from GCS (request individual waypoint during download)
void MavlinkHandler::handleMissionRequest(const mavlink_message_t& msg) {
    if (!mission_download_in_progress) {
        Serial.println("Warning: Received MISSION_REQUEST but no download in progress");
        return;
    }

    mavlink_mission_request_int_t req;
    
    // Try MISSION_REQUEST_INT first
    if (msg.msgid == MAVLINK_MSG_ID_MISSION_REQUEST_INT) {
        mavlink_msg_mission_request_int_decode(&msg, &req);
    } else {
        // Fallback to old MISSION_REQUEST format
        mavlink_mission_request_t old_req;
        mavlink_msg_mission_request_decode(&msg, &old_req);
        req.seq = old_req.seq;
    }

    mission_download_seq = req.seq;

    // Send the requested waypoint
    if (req.seq < wp_sum) {
        sendMissionItem(req.seq);
    } else {
        // Download complete or invalid index
        if (req.seq == wp_sum) {
            // GCS is requesting the mission end marker
            Serial.printf("Mission download complete\n");
            mission_download_in_progress = false;
            // Send mission ack to indicate completion
            sendMissionAck(0);  // MAV_MISSION_ACCEPTED
        } else {
            Serial.printf("ERROR: Invalid mission sequence %d\n", req.seq);
        }
    }
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
            handleMissionCount(msg);
            break;
        case MAVLINK_MSG_ID_MISSION_ITEM:
            handleMissionItem(msg);
            break;
        case MAVLINK_MSG_ID_MISSION_ITEM_INT:
            handleMissionItem(msg);
            break;
        case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
            handleMissionRequestList(msg);
            break;
        case MAVLINK_MSG_ID_MISSION_REQUEST:
            handleMissionRequest(msg);
            break;
        case MAVLINK_MSG_ID_MISSION_REQUEST_INT:
            handleMissionRequest(msg);
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