#ifndef MAV_H
#define MAV_H

#include "TimingUtils.h"
#include "bno055.h"
#include "AHRS.h"
#include "Radio.h"
#include "gps.h"
#include "Mode_setup.h"
#include "Auto_setup.h"  // Untuk akses auto_state struct
#include "Voltage.h"  // Untuk battery voltage dan percentage
#include <common/mavlink.h>
#include <cstring>  // Untuk memset
#include <Locations.h>  // Untuk akses waypoint global
#include "WP_EEPROM.h"  // Untuk EEPROM waypoint storage

// Declare external global waypoint data from AutoMode.h
extern Locations waypoint[100];
extern int wp_sum;

// Mission state enum
enum MissionState {
    MISSION_IDLE,
    MISSION_RECEIVING
};

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
    static const unsigned long BATTERY_MS = 1000;  // Battery status interval
    static const unsigned long NAV_CONTROLLER_MS = 200;  // Navigation controller output interval
    static const unsigned long PARAM_REQUEST_INTERVAL = 1000;
    static const uint16_t MAX_MISSION = 100;  // Same as waypoint[] size

    // Timers
    unsigned long lastVfr = 0;
    unsigned long lastGps = 0;
    unsigned long lastHb = 0;
    unsigned long lastParamRequest = 0;
    unsigned long lastAtt = 0;
    unsigned long lastAlt = 0;
    unsigned long lastAlert = 0;
    unsigned long lastBattery = 0;
    unsigned long lastNavController = 0;

    // MAVLink status and message buffers
    mavlink_status_t st_usb;
    mavlink_status_t st_tlm;
    mavlink_status_t st_rpi;
    mavlink_message_t rxmsg;

    // Mission state (storage menggunakan waypoint[] global)
    MissionState mission_state;
    uint16_t expected_count;
    uint16_t current_upload_seq;

    // Private methods for internal use
    void mavWrite(HardwareSerial& port, const mavlink_message_t& msg);
    void mavWriteUSB(Stream& port, const mavlink_message_t& msg);
    void mavSendAll(const mavlink_message_t& msg);
    void sendCommandAck(uint16_t command, uint8_t result);
    void sendStatusText(uint8_t severity, const char* text);
    void handleCommandLong(const mavlink_message_t& msg);
    void handleStatusText(const mavlink_message_t& msg);  // decode dan print STATUSTEXT
    void handleMissionRequestList(const mavlink_message_t& msg);
    void handleMissionRequest(const mavlink_message_t& msg);
    void handleMissionRequestInt(const mavlink_message_t& msg);
    void handleMissionCount(const mavlink_message_t& msg);
    void handleMissionItemInt(const mavlink_message_t& msg);
    void handleMissionItem(const mavlink_message_t& msg);
    void handleMissionClearAll(const mavlink_message_t& msg);
    void processAsFC(const mavlink_message_t& msg, PortId from);
    void forwardToOthers(const mavlink_message_t& msg, PortId from);
    void sendHeartbeat();
    void sendAlert(const char* txt);
    void sendAltitude(float altitude);
    void sendAttitude(float roll_rad, float pitch_rad, float yaw_rad);
    void sendGpsRawInt(uint8_t fix_type, int32_t lat, int32_t lon, uint16_t eph, uint16_t vel, uint8_t satellites_visible);
    void sendMissionItemReached(uint16_t seq);
    void sendBatteryStatus(float voltage, int16_t current_amp, int8_t battery_remaining);
    void sendNavControllerOutput(float nav_roll, float nav_pitch, float nav_bearing, float target_bearing, float wp_dist, float alt_error, float aspd_error, float xtrack_error);
    uint8_t getBaseMode() const;
    uint32_t getCustomMode() const;

public:
    // Constructor for initialization
    MavlinkHandler();

    // Public methods
    void mavlinkTask();  // Main periodic task
    void handlePorts();  // Handle incoming data from all ports
    void sendCustomTextToGCS(const char* text);  // Method baru: Kirim pesan custom ke GCS
    void notifyWaypointReached(uint16_t seq);  // Notify when waypoint reached
    uint16_t getMissionCount() const { return wp_sum; }  // Get mission count (dari wp_sum global)
};

// Global helper function for waypoint notification
void notifyWaypointReached(uint16_t seq);

/////////////////////// Implementation ///////////////////////

// Constructor
MavlinkHandler::MavlinkHandler() {
    // Initialize statuses to zero
    memset(&st_usb, 0, sizeof(st_usb));
    memset(&st_tlm, 0, sizeof(st_tlm));
    memset(&st_rpi, 0, sizeof(st_rpi));
    
    // Initialize mission state (waypoint[] diinit di tempat lain)
    mission_state = MISSION_IDLE;
    expected_count = 0;
    current_upload_seq = 0;
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

    // Check for DROP command dari Raspberry Pi untuk trigger autopayload
    if (strncmp(status.text, "DROP", 4) == 0) {
        payload_drop_command = true;
        char confirm_msg[50];
        snprintf(confirm_msg, sizeof(confirm_msg), "PAYLOAD DROP TRIGGERED");
        sendStatusText(MAV_SEVERITY_INFO, confirm_msg);
        // Serial.println("PAYLOAD DROP COMMAND RECEIVED from RPI via MAVLINK");
        return;
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

        case MAV_CMD_DO_SET_MODE: {
            // param1: mode (base mode bitmask or custom mode number)
            // param2: custom mode (ArduPlane mode number)
            uint32_t custom_mode = (uint32_t)cmd.param2;
            
            Serial.printf("MODE CHANGE REQ: custom_mode=%d\n", custom_mode);
            
            // Map ArduPlane custom modes to internal mode_now
            // ArduPlane: MANUAL=0, FBWA=5, AUTO=10
            int new_mode = 0;
            const char* mode_name = "UNKNOWN";
            
            if (custom_mode == 0) {
                // MANUAL mode
                new_mode = 1;
                mode_name = "MANUAL";
            } else if (custom_mode == 5) {
                // FBWA mode
                new_mode = 2;
                mode_name = "FBWA";
            } else if (custom_mode == 10) {
                // AUTO mode
                new_mode = 3;
                mode_name = "AUTO";
            } else {
                // Unsupported mode
                result = MAV_RESULT_UNSUPPORTED;
                sendStatusText(MAV_SEVERITY_WARNING, "MODE NOT SUPPORTED");
                break;
            }
            
            // Set the mode
            mode_now = new_mode;
            result = MAV_RESULT_ACCEPTED;
            
            char msg_buf[50];
            snprintf(msg_buf, sizeof(msg_buf), "MODE: %s", mode_name);
            sendStatusText(MAV_SEVERITY_INFO, msg_buf);
            Serial.printf("Mode switched to: %s (mode_now=%d)\n", mode_name, new_mode);
            break;
        }

        default:
            result = MAV_RESULT_UNSUPPORTED;
            break;
    }

    sendCommandAck(cmd.command, result);
}

// Handle MISSION_REQUEST_LIST - GCS meminta jumlah waypoint
void MavlinkHandler::handleMissionRequestList(const mavlink_message_t& msg) {
    mavlink_message_t reply;
    
    mavlink_msg_mission_count_pack(
        SYSTEM_ID,
        COMPONENT_ID,
        &reply,
        msg.sysid,
        msg.compid,
        wp_sum,  // Gunakan wp_sum global
        MAV_MISSION_TYPE_MISSION
    );
    
    mavSendAll(reply);
    Serial.printf("Sent mission count: %d\n", wp_sum);
}

// Handle MISSION_REQUEST - GCS meminta waypoint tertentu (format lama)
void MavlinkHandler::handleMissionRequest(const mavlink_message_t& msg) {
    mavlink_mission_request_t request;
    mavlink_msg_mission_request_decode(&msg, &request);
    
    if (request.seq >= wp_sum) {
        Serial.printf("Mission request seq out of range: %d\n", request.seq);
        return;
    }
    
    mavlink_message_t reply;
    
    // Konversi dari waypoint[] (Locations) ke MAVLink format
    // waypoint[].alt dalam CM, MAVLink perlu METER
    mavlink_msg_mission_item_int_pack(
        SYSTEM_ID,
        COMPONENT_ID,
        &reply,
        msg.sysid,
        msg.compid,
        request.seq,
        MAV_FRAME_GLOBAL_RELATIVE_ALT,
        MAV_CMD_NAV_WAYPOINT,
        0,  // current
        1,  // autocontinue
        0, 0, 0, 0,  // param1-4
        waypoint[request.seq].lat,  // Already in 1e7 format
        waypoint[request.seq].lng,  // Already in 1e7 format
        waypoint[request.seq].alt / 100.0f,  // Convert CM to METER
        MAV_MISSION_TYPE_MISSION
    );
    
    mavSendAll(reply);
    Serial.printf("Sent mission item %d\n", request.seq);
    
    // Jika ini item terakhir, kirim ACK
    if (request.seq == wp_sum - 1) {
        mavlink_message_t ack;
        
        mavlink_msg_mission_ack_pack(
            SYSTEM_ID,
            COMPONENT_ID,
            &ack,
            msg.sysid,
            msg.compid,
            MAV_MISSION_ACCEPTED,
            MAV_MISSION_TYPE_MISSION
        );
        
        mavSendAll(ack);
        Serial.println("Mission request completed");
    }
}

// Handle MISSION_REQUEST_INT - GCS meminta waypoint tertentu (format int, lebih baru)
void MavlinkHandler::handleMissionRequestInt(const mavlink_message_t& msg) {
    mavlink_mission_request_int_t request;
    mavlink_msg_mission_request_int_decode(&msg, &request);
    
    if (request.seq >= wp_sum) {
        Serial.printf("Mission request int seq out of range: %d\n", request.seq);
        return;
    }
    
    mavlink_message_t reply;
    
    // Konversi dari waypoint[] (Locations) ke MAVLink format
    mavlink_msg_mission_item_int_pack(
        SYSTEM_ID,
        COMPONENT_ID,
        &reply,
        msg.sysid,
        msg.compid,
        request.seq,
        MAV_FRAME_GLOBAL_RELATIVE_ALT,
        MAV_CMD_NAV_WAYPOINT,
        0,  // current
        1,  // autocontinue
        0, 0, 0, 0,  // param1-4
        waypoint[request.seq].lat,  // Already in 1e7 format
        waypoint[request.seq].lng,  // Already in 1e7 format
        waypoint[request.seq].alt / 100.0f,  // Convert CM to METER
        MAV_MISSION_TYPE_MISSION
    );
    
    mavSendAll(reply);
    Serial.printf("Sent mission item int %d\n", request.seq);
    
    // Jika ini item terakhir, kirim ACK
    if (request.seq == wp_sum - 1) {
        mavlink_message_t ack;
        
        mavlink_msg_mission_ack_pack(
            SYSTEM_ID,
            COMPONENT_ID,
            &ack,
            msg.sysid,
            msg.compid,
            MAV_MISSION_ACCEPTED,
            MAV_MISSION_TYPE_MISSION
        );
        
        mavSendAll(ack);
        Serial.println("Mission request int completed");
    }
}

// Handle MISSION_COUNT - GCS mulai upload waypoint
void MavlinkHandler::handleMissionCount(const mavlink_message_t& msg) {
    mavlink_mission_count_t count_msg;
    mavlink_msg_mission_count_decode(&msg, &count_msg);
    
    expected_count = count_msg.count;
    Serial.printf("Receiving mission count: %d\n", expected_count);
    
    if (expected_count > MAX_MISSION) {
        // Tolak kalau terlalu besar
        mavlink_message_t ack;
        mavlink_msg_mission_ack_pack(
            SYSTEM_ID, COMPONENT_ID,
            &ack,
            msg.sysid, msg.compid,
            MAV_MISSION_ERROR,
            MAV_MISSION_TYPE_MISSION
        );
        mavSendAll(ack);
        sendStatusText(MAV_SEVERITY_WARNING, "Mission count too large");
        return;
    }
    
    mission_state = MISSION_RECEIVING;
    current_upload_seq = 0;
    
    // Minta item pertama
    mavlink_message_t req;
    mavlink_msg_mission_request_int_pack(
        SYSTEM_ID, COMPONENT_ID,
        &req,
        msg.sysid, msg.compid,
        0,
        MAV_MISSION_TYPE_MISSION
    );
    mavSendAll(req);
}

// Handle MISSION_ITEM_INT - Terima waypoint dari GCS (format int)
void MavlinkHandler::handleMissionItemInt(const mavlink_message_t& msg) {
    if (mission_state != MISSION_RECEIVING) {
        Serial.println("Not in receiving state");
        return;
    }
    
    mavlink_mission_item_int_t item;
    mavlink_msg_mission_item_int_decode(&msg, &item);
    
    // Validasi urutan
    if (item.seq != current_upload_seq) {
        // sequence salah → error
        mavlink_message_t ack;
        mavlink_msg_mission_ack_pack(
            SYSTEM_ID, COMPONENT_ID,
            &ack,
            msg.sysid, msg.compid,
            MAV_MISSION_INVALID_SEQUENCE,
            MAV_MISSION_TYPE_MISSION
        );
        mavSendAll(ack);
        
        mission_state = MISSION_IDLE;
        sendStatusText(MAV_SEVERITY_WARNING, "Invalid mission sequence");
        return;
    }
    
    // Simpan waypoint ke global waypoint[] array
    // Konversi dari MAVLink format ke Locations
    waypoint[item.seq].lat = item.x;  // Already in 1e7 format
    waypoint[item.seq].lng = item.y;  // Already in 1e7 format
    waypoint[item.seq].alt = (int32_t)(item.z * 100.0f);  // Convert METER to CM
    
    Serial.printf("Received mission item %d: lat=%d, lon=%d, alt=%.1fm (=%dcm)\n", 
                  item.seq, item.x, item.y, item.z, waypoint[item.seq].alt);
    
    current_upload_seq++;
    
    if (current_upload_seq < expected_count) {
        // Minta berikutnya
        mavlink_message_t req;
        mavlink_msg_mission_request_int_pack(
            SYSTEM_ID, COMPONENT_ID,
            &req,
            msg.sysid, msg.compid,
            current_upload_seq,
            MAV_MISSION_TYPE_MISSION
        );
        mavSendAll(req);
    } else {
        // Selesai - update wp_sum global
        wp_sum = expected_count;
        
        mavlink_message_t ack;
        mavlink_msg_mission_ack_pack(
            SYSTEM_ID, COMPONENT_ID,
            &ack,
            msg.sysid, msg.compid,
            MAV_MISSION_ACCEPTED,
            MAV_MISSION_TYPE_MISSION
        );
        mavSendAll(ack);
        
        mission_state = MISSION_IDLE;
        sendStatusText(MAV_SEVERITY_INFO, "Mission upload complete");
        Serial.printf("Mission upload complete, total: %d\n", wp_sum);
        
        // Save to EEPROM
        if (saveWaypointsToEEPROM()) {
            sendStatusText(MAV_SEVERITY_INFO, "Saved to EEPROM");
        }
    }
}

// Handle MISSION_ITEM - Terima waypoint dari GCS (format float, lama)
void MavlinkHandler::handleMissionItem(const mavlink_message_t& msg) {
    if (mission_state != MISSION_RECEIVING) {
        Serial.println("Not in receiving state");
        return;
    }
    
    mavlink_mission_item_t item;
    mavlink_msg_mission_item_decode(&msg, &item);
    
    if (item.seq != current_upload_seq) {
        mavlink_message_t ack;
        mavlink_msg_mission_ack_pack(
            SYSTEM_ID, COMPONENT_ID,
            &ack,
            msg.sysid, msg.compid,
            MAV_MISSION_INVALID_SEQUENCE,
            MAV_MISSION_TYPE_MISSION
        );
        mavSendAll(ack);
        mission_state = MISSION_IDLE;
        sendStatusText(MAV_SEVERITY_WARNING, "Invalid mission sequence");
        return;
    }
    
    // Simpan ke waypoint[] global
    // item.x dan item.y dalam float degrees, convert ke int32 1e7
    waypoint[item.seq].lat = (int32_t)(item.x * 1e7);
    waypoint[item.seq].lng = (int32_t)(item.y * 1e7);
    waypoint[item.seq].alt = (int32_t)(item.z * 100.0f);  // Convert METER to CM
    
    Serial.printf("Received mission item (float) %d\n", item.seq);
    
    current_upload_seq++;
    
    if (current_upload_seq < expected_count) {
        mavlink_message_t req;
        mavlink_msg_mission_request_pack(
            SYSTEM_ID, COMPONENT_ID,
            &req,
            msg.sysid, msg.compid,
            current_upload_seq,
            MAV_MISSION_TYPE_MISSION
        );
        mavSendAll(req);
    } else {
        wp_sum = expected_count;
        
        mavlink_message_t ack;
        mavlink_msg_mission_ack_pack(
            SYSTEM_ID, COMPONENT_ID,
            &ack,
            msg.sysid, msg.compid,
            MAV_MISSION_ACCEPTED,
            MAV_MISSION_TYPE_MISSION
        );
        mavSendAll(ack);
        
        mission_state = MISSION_IDLE;
        sendStatusText(MAV_SEVERITY_INFO, "Mission upload complete");
        Serial.printf("Mission upload complete, total: %d\n", wp_sum);
        
        // Save to EEPROM
        if (saveWaypointsToEEPROM()) {
            sendStatusText(MAV_SEVERITY_INFO, "Saved to EEPROM");
        }
    }
}

// Handle MISSION_CLEAR_ALL - Hapus semua waypoint
void MavlinkHandler::handleMissionClearAll(const mavlink_message_t& msg) {
    wp_sum = 0;  // Reset global counter
    // Optional: clear waypoint array content
    // memset(waypoint, 0, sizeof(waypoint));
    
    // Clear EEPROM
    clearWaypointEEPROM();
    
    mavlink_message_t ack;
    mavlink_msg_mission_ack_pack(
        SYSTEM_ID, COMPONENT_ID,
        &ack,
        msg.sysid, msg.compid,
        MAV_MISSION_ACCEPTED,
        MAV_MISSION_TYPE_MISSION
    );
    mavSendAll(ack);
    
    sendStatusText(MAV_SEVERITY_INFO, "Missions cleared");
    Serial.println("All missions cleared (RAM + EEPROM)");
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
        case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
            handleMissionRequestList(msg);
            break;
        case MAVLINK_MSG_ID_MISSION_REQUEST:
            handleMissionRequest(msg);
            break;
        case MAVLINK_MSG_ID_MISSION_REQUEST_INT:
            handleMissionRequestInt(msg);
            break;
        case MAVLINK_MSG_ID_MISSION_COUNT:
            handleMissionCount(msg);
            break;
        case MAVLINK_MSG_ID_MISSION_ITEM_INT:
            handleMissionItemInt(msg);
            break;
        case MAVLINK_MSG_ID_MISSION_ITEM:
            handleMissionItem(msg);
            break;
        case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
            handleMissionClearAll(msg);
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
    uint8_t base_mode = getBaseMode();
    if (arming) {
        base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }

    uint32_t custom_mode = getCustomMode();
    mavlink_msg_heartbeat_pack(
        SYSTEM_ID,
        COMPONENT_ID,
        &msg,
        MAV_TYPE_FIXED_WING,
        MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode,
        custom_mode,
        system_status
    );
    mavSendAll(msg);
}

uint8_t MavlinkHandler::getBaseMode() const {
    // Provide base mode hints for Mission Planner
    switch (current) {
        case ModeId::MANU:
            return MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
        case ModeId::FBWA:
            return MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_MANUAL_INPUT_ENABLED | MAV_MODE_FLAG_STABILIZE_ENABLED;
        case ModeId::AUTO:
            return MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_AUTO_ENABLED | MAV_MODE_FLAG_GUIDED_ENABLED;
        default:
            return MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    }
}

uint32_t MavlinkHandler::getCustomMode() const {
    // ArduPlane mode mapping for Mission Planner display
    switch (current) {
        case ModeId::MANU:
            return 0;   // MANUAL
        case ModeId::FBWA:
            return 5;   // FBWA
        case ModeId::AUTO:
            return 10;  // AUTO
        default:
            return 0;   // fallback to MANUAL
    }
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

// Send BATTERY_STATUS
void MavlinkHandler::sendBatteryStatus(float voltage, int16_t current_amp, int8_t battery_remaining) {
    mavlink_message_t msg;
    // voltage: dalam millivolt (V * 1000)
    // current: dalam centiamp (A * 100), -1 jika unknown
    uint16_t voltage_mv = (uint16_t)(voltage * 1000.0f);
    int16_t current_ca = (current_amp == 0) ? -1 : (current_amp * 100);
    
    mavlink_msg_battery_status_pack(
        SYSTEM_ID,
        COMPONENT_ID,
        &msg,
        0,              // id (battery ID)
        0,              // battery_function (0 = unknown)
        0,              // type (0 = unknown)
        INT16_MAX,      // temperature (kelvin, -327.68 = unknown)
        &voltage_mv,    // voltages array pointer
        current_ca,     // current_battery (centiamps, -1 = unknown)
        -1,             // current_consumed (mAh, -1 = unknown)
        -1,             // energy_consumed (hJ, -1 = unknown)
        battery_remaining, // battery_remaining (percentage, -1 = unknown)
        0,              // time_remaining (minutes, 0 = unknown)
        0               // charge_state (0 = unknown)
    );
    mavSendAll(msg);
    Serial.printf("Battery: %.2fV, %d%%\n", voltage, battery_remaining);
}

// Send NAV_CONTROLLER_OUTPUT - Navigation controller output dengan distance ke next WP
void MavlinkHandler::sendNavControllerOutput(float nav_roll, float nav_pitch, float nav_bearing, float target_bearing, float wp_dist, float alt_error, float aspd_error, float xtrack_error) {
    mavlink_message_t msg;
    mavlink_msg_nav_controller_output_pack(
        SYSTEM_ID,
        COMPONENT_ID,
        &msg,
        nav_roll,       // roll commanded by autopilot (rad)
        nav_pitch,      // pitch commanded by autopilot (rad)
        nav_bearing,    // bearing to waypoint (rad)
        target_bearing, // bearing to current waypoint (rad)
        wp_dist,        // distance to active waypoint (m) ← auto_state.distance_next_wp
        alt_error,      // altitude error (m)
        aspd_error,     // airspeed error (m/s)
        xtrack_error    // crosstrack error (m)
    );
    mavSendAll(msg);
}

// Send MISSION_ITEM_REACHED
void MavlinkHandler::sendMissionItemReached(uint16_t seq) {
    mavlink_message_t msg;
    mavlink_msg_mission_item_reached_pack(SYSTEM_ID, COMPONENT_ID, &msg, seq);
    mavSendAll(msg);
    Serial.printf("WP %d reached\n", seq);
}

// Public method: notify when waypoint reached
void MavlinkHandler::notifyWaypointReached(uint16_t seq) {
    sendMissionItemReached(seq);
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

    if (now - lastBattery >= BATTERY_MS) {
        lastBattery = now;
        // Get battery data dari Voltage.h
        // voltage: dalam volt, current: dalam mA (default 0 jika tidak ada sensor arus)
        sendBatteryStatus(batt_v, 0, (int8_t)display_batt());
    }

    if (now - lastNavController >= NAV_CONTROLLER_MS) {
        lastNavController = now;
        // Send navigation controller output dengan distance to next waypoint
        float nav_roll = imu.rad_roll;      // Roll commanded
        float nav_pitch = imu.rad_pitch;    // Pitch commanded
        float nav_bearing = atan2(0, 0);   // TODO: heading to waypoint (rad)
        float target_bearing = atan2(0, 0); // TODO: current waypoint bearing (rad)
        float wp_dist = auto_state.distance_next_wp;   // Distance to next waypoint (m)
        float alt_error = 0;                // TODO: altitude error from navigation
        float aspd_error = 0;               // TODO: airspeed error from FW controller
        float xtrack_error = 0;             // TODO: crosstrack error from L1 controller
        
        sendNavControllerOutput(nav_roll, nav_pitch, nav_bearing, target_bearing, wp_dist, alt_error, aspd_error, xtrack_error);
    }
   
    // if (millis() - lastCustom >= 5000) {
    //     lastCustom = millis();
    //     sendCustomTextToGCS("Pesan custom dari FC!");
    // }
}

// Global helper function implementation
void notifyWaypointReached(uint16_t seq) {
    extern MavlinkHandler mavHandler;
    mavHandler.notifyWaypointReached(seq);
}

#endif