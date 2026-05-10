#ifndef MAV_H
#define MAV_H

/**
 * ============================================================
 *  MAV.h — MAVLink Handler + ELRS RC Channel Reader
 *  Teensy 4.1 | Autopilot FC
 * ============================================================
 *  Integrasi Program:
 *    - MAVLink Telemetry Handler (heartbeat, GPS, IMU, dsb)
 *    - ELRS RC Channel Reader via MAVLink
 *
 *  RC Priority (sesuai ELRS MAVLink mode):
 *    1. ID 70  RC_CHANNELS_OVERRIDE  ← ELRS utama
 *    2. ID 65  RC_CHANNELS           ← fallback
 *    3. ID 35  RC_CHANNELS_RAW       ← fallback legacy
 *
 *  RC Mapping:
 *    ch_roll     = CH1  (aileron)
 *    ch_pitch    = CH2  (elevator)
 *    ch_throttle = CH3  (throttle)
 *    ch_yaw      = CH4  (rudder)
 *    ch5 – ch8           (aux channels)
 *
 *  Wiring ELRS -> Teensy:
 *    ELRS TX  ->  Pin 9  (RX2 / Serial2)
 *    ELRS RX  ->  Pin 10 (TX2 / Serial2)  [opsional]
 * ============================================================
 */

#include "TimingUtils.h"
#include "bno055.h"
#include "AHRS.h"
#include "Radio.h"
#include "gps.h"
#include "airspeed.h"
#include "Mode_setup.h"
#include "Auto_setup.h"
#include "Voltage.h"
#include <common/mavlink.h>
#include <cstring>
#include <Locations.h>
#include "WP_EEPROM.h"

// External globals
extern Locations waypoint[100];
extern int       wp_sum;
extern Airspeed  arspd;

// ============================================================
//  RC CHANNEL VARIABLES — Hasil parsing ELRS RC
//  Rentang nilai: 1000–2000 us (microsecond PWM)
//  Nilai 0 atau 65535 = channel tidak aktif / tidak valid
// ============================================================

// ============================================================
//  RC STATUS
// ============================================================
struct RCStatus {
    bool     valid         = false;   // true jika sudah terima RC
    uint8_t  sourceId      = 0;       // 70 / 65 / 35
    uint32_t lastUpdateMs  = 0;
    uint8_t  rssi          = 255;     // 255 = unknown
    uint32_t rcMsgCount    = 0;
};
RCStatus g_rcStatus;

// RC timeout threshold
static const uint32_t RC_TIMEOUT_MS = 500;

// ============================================================
//  HELPER: apakah RC data masih fresh?
// ============================================================
inline bool rcIsHealthy() {
    return g_rcStatus.valid &&
           (millis() - g_rcStatus.lastUpdateMs < RC_TIMEOUT_MS);
}

// ============================================================
//  MISSION STATE
// ============================================================
enum MissionState { MISSION_IDLE, MISSION_RECEIVING };

// ============================================================
//  PORT IDs
// ============================================================
enum PortId { PORT_USB = 0, PORT_TLM = 1, PORT_RPI = 2 };

// ============================================================
//  MavlinkHandler CLASS
// ============================================================
class MavlinkHandler {
private:
    // --- Constants ---
    static const uint8_t  SYSTEM_ID    = 1;
    static const uint8_t  COMPONENT_ID = MAV_COMP_ID_AUTOPILOT1;

    // static const unsigned long HEARTBEAT_MS        = 1000;
    // static const unsigned long ATT_MS              = 100;
    // static const unsigned long ALT_MS              = 300;
    // static const unsigned long GPS_MS              = 500;
    // static const unsigned long VFR_MS              = 200;
    // static const unsigned long BATTERY_MS          = 1000;
    // static const unsigned long NAV_CONTROLLER_MS   = 200;
    // static const unsigned long RC_PRINT_MS         = 100;   // Debug print RC @ 10 Hz
    // Di MAV.h, ubah semua interval jadi lebih jarang
    static const unsigned long HEARTBEAT_MS      = 1000;  // tetap
    static const unsigned long ATT_MS            = 200;   // 5Hz  (dari 50ms/20Hz)
    static const unsigned long ALT_MS            = 500;   // 2Hz  (dari 300ms)
    static const unsigned long GPS_MS            = 1000;  // 1Hz  (dari 500ms)
    static const unsigned long VFR_MS            = 500;   // 2Hz  (dari 200ms)
    static const unsigned long BATTERY_MS        = 5000;  // 0.2Hz (dari 1000ms)
    static const unsigned long NAV_CONTROLLER_MS = 1000;  // 1Hz  (dari 200ms)

    static constexpr float     ALERT_THRESHOLD_DEG = 40.0f;
    static const unsigned long ALERT_DEBOUNCE_MS   = 2000;

    static const uint16_t MAX_MISSION = 100;

    // --- Timers ---
    unsigned long lastHb            = 0;
    unsigned long lastAtt           = 0;
    unsigned long lastAlt           = 0;
    unsigned long lastGps           = 0;
    unsigned long lastVfr           = 0;
    unsigned long lastBattery       = 0;
    unsigned long lastNavController = 0;
    unsigned long lastAlert         = 0;
    unsigned long lastRcPrint       = 0;

    // --- MAVLink buffers ---
    mavlink_status_t  st_usb, st_tlm, st_rpi;
    mavlink_message_t rxmsg;

    // --- Mission state ---
    MissionState mission_state      = MISSION_IDLE;
    uint16_t     expected_count     = 0;
    uint16_t     current_upload_seq = 0;

    // --------------------------------------------------------
    //  PRIVATE: Output helpers
    // --------------------------------------------------------
    void mavWrite(HardwareSerial& port, const mavlink_message_t& msg);
    void mavWriteUSB(Stream& port,      const mavlink_message_t& msg);
    void mavSendAll(const mavlink_message_t& msg);

    // --------------------------------------------------------
    //  PRIVATE: Telemetry senders
    // --------------------------------------------------------
    void sendCommandAck(uint16_t command, uint8_t result);
    void sendStatusText(uint8_t severity, const char* text);
    void sendHeartbeat();
    void sendAlert(const char* txt);
    void sendAltitude(float altitude);
    void sendAttitude(float roll_rad, float pitch_rad, float yaw_rad);
    void sendGpsRawInt(uint8_t fix_type, int32_t lat, int32_t lon,
                       uint16_t eph, uint16_t vel, uint16_t cog,
                       uint8_t satellites_visible);
    void sendBatteryStatus(float voltage, int16_t current_amp, int8_t battery_remaining);
    void sendNavControllerOutput(float nav_roll, float nav_pitch,
                                 float nav_bearing, float target_bearing,
                                 float wp_dist, float alt_error,
                                 float aspd_error, float xtrack_error);
    void sendVfrHud(float airspeed, float groundspeed, float heading,
                    float altitude, float climb_rate);
    void sendMissionItemReached(uint16_t seq);

    uint8_t  getBaseMode()   const;
    uint32_t getCustomMode() const;

    // --------------------------------------------------------
    //  PRIVATE: Incoming message handlers (telemetry)
    // --------------------------------------------------------
    void handleCommandLong(const mavlink_message_t& msg);
    void handleStatusText(const mavlink_message_t& msg);
    void handleMissionRequestList(const mavlink_message_t& msg);
    void handleMissionRequest(const mavlink_message_t& msg);
    void handleMissionRequestInt(const mavlink_message_t& msg);
    void handleMissionCount(const mavlink_message_t& msg);
    void handleMissionItemInt(const mavlink_message_t& msg);
    void handleMissionItem(const mavlink_message_t& msg);
    void handleMissionClearAll(const mavlink_message_t& msg);
    void handleSetPositionTargetLocalNed(const mavlink_message_t& msg);
    void handleSetPositionTargetGlobalInt(const mavlink_message_t& msg);
    void processAsFC(const mavlink_message_t& msg, PortId from);
    void forwardToOthers(const mavlink_message_t& msg, PortId from);

    // --------------------------------------------------------
    //  PRIVATE: RC Channel handlers (dari ELRS)
    // --------------------------------------------------------
    void handleRcChannelsOverride(const mavlink_message_t& msg);   // ID 70 — utama
    void handleRcChannels(const mavlink_message_t& msg);           // ID 65 — fallback
    void handleRcChannelsRaw(const mavlink_message_t& msg);        // ID 35 — fallback legacy
    void handleRadioStatus(const mavlink_message_t& msg);          // ID 109 — RSSI
    void applyRcToGlobals(uint16_t c1, uint16_t c2, uint16_t c3,
                          uint16_t c4, uint16_t c5, uint16_t c6,
                          uint16_t c7, uint16_t c8,
                          uint8_t src_id);

    // --------------------------------------------------------
    //  PRIVATE: RC Debug
    // --------------------------------------------------------
    void printRCStatus();

public:
    MavlinkHandler();

    void mavlinkTask();              // Periodic telemetry sender
    void handlePorts();              // Parse incoming bytes dari semua port

    void sendCustomTextToGCS(const char* text);
    void notifyWaypointReached(uint16_t seq);

    uint16_t getMissionCount() const { return wp_sum; }
    bool     rcValid()         const { return rcIsHealthy(); }
};

// ============================================================
//  CONSTRUCTOR
// ============================================================
MavlinkHandler::MavlinkHandler() {
    memset(&st_usb, 0, sizeof(st_usb));
    memset(&st_tlm, 0, sizeof(st_tlm));
    memset(&st_rpi, 0, sizeof(st_rpi));
}

// ============================================================
//  OUTPUT HELPERS
// ============================================================
void MavlinkHandler::mavWrite(HardwareSerial& port, const mavlink_message_t& msg) {
    uint8_t  buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    port.write(buf, len);
    port.flush();  //  FIX: ensure data is sent immediately
}

void MavlinkHandler::mavWriteUSB(Stream& port, const mavlink_message_t& msg) {
    uint8_t  buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    port.write(buf, len);    // Note: USB Serial does not have flush() or it's a no-op}
}

void MavlinkHandler::mavSendAll(const mavlink_message_t& msg) {
    // CATATAN: Serial USB TIDAK dikirimi binary MAVLink agar Serial Monitor
    // tetap bisa baca debug text (printRCStatus, Serial.printf, dsb).
    // Aktifkan baris ini HANYA jika GCS (Mission Planner/QGC) konek via USB:
    mavWriteUSB(Serial, msg);
    mavWrite(Serial2, msg);   // Telemetry Radio / ELRS
    mavWrite(Serial7, msg);   // RPi UART
}

// ============================================================
//  RC: Apply raw channel values ke global variables
//  Nilai 0 = channel tidak dipakai oleh ELRS → abaikan
// ============================================================
void MavlinkHandler::applyRcToGlobals(uint16_t c1, uint16_t c2, uint16_t c3,
                                       uint16_t c4, uint16_t c5, uint16_t c6,
                                       uint16_t c7, uint16_t c8,
                                       uint8_t src_id) {
    // Nilai 0 = "not provided" pada RC_CHANNELS_OVERRIDE → skip channel tsb
    // Nilai 65535 = invalid/disabled → skip juga
    auto apply = [](uint16_t& dst, uint16_t src) {
        if (src != 0 && src != 65535) dst = src;
    };

    apply(ch_roll,     c1);
    apply(ch_pitch,    c2);
    apply(ch_throttle, c3);
    apply(ch_yaw,      c4);
    apply(ch5,         c5);
    arming = ch5 > 1500 ? 1 : 0; // Contoh: channel 5 sebagai switch arming
    apply(ch_mode,         c6);
    apply(ch_mode_backup,         c7);
    //  FIX: RC channels only support 3 modes (MANUAL/FBWA/AUTO)
    // GUIDED mode only via GCS (MAVLink)
    if (ch_mode_backup <= 1000) {
        mode_now = 1;  // MANUAL
    } else if (ch_mode_backup > 1000 && ch_mode_backup <= 1512) {
        mode_now = 2;  // FBWA
    } else {
        mode_now = 3;  // AUTO (bukan GUIDED)
    }
    apply(ch_vehicle_mode,         c8);

    g_rcStatus.valid        = true;
    g_rcStatus.sourceId     = src_id;
    g_rcStatus.lastUpdateMs = millis();
    g_rcStatus.rcMsgCount++;
}

// ============================================================
//  RC HANDLER: ID 70 — RC_CHANNELS_OVERRIDE (ELRS utama)
// ============================================================
void MavlinkHandler::handleRcChannelsOverride(const mavlink_message_t& msg) {
    mavlink_rc_channels_override_t rc;
    mavlink_msg_rc_channels_override_decode(&msg, &rc);

    applyRcToGlobals(rc.chan1_raw, rc.chan2_raw, rc.chan3_raw, rc.chan4_raw,
                     rc.chan5_raw, rc.chan6_raw, rc.chan7_raw, rc.chan8_raw,
                     70);

    static bool first = true;
    if (first) {
        first = false;
        Serial.println("[RC OK] RC_CHANNELS_OVERRIDE (ID 70) diterima dari ELRS");
    }
}

// ============================================================
//  RC HANDLER: ID 65 — RC_CHANNELS (fallback)
// ============================================================
void MavlinkHandler::handleRcChannels(const mavlink_message_t& msg) {
    mavlink_rc_channels_t rc;
    mavlink_msg_rc_channels_decode(&msg, &rc);

    g_rcStatus.rssi = rc.rssi;

    applyRcToGlobals(rc.chan1_raw, rc.chan2_raw, rc.chan3_raw, rc.chan4_raw,
                     rc.chan5_raw, rc.chan6_raw, rc.chan7_raw, rc.chan8_raw,
                     65);
}

// ============================================================
//  RC HANDLER: ID 35 — RC_CHANNELS_RAW (fallback legacy)
// ============================================================
void MavlinkHandler::handleRcChannelsRaw(const mavlink_message_t& msg) {
    mavlink_rc_channels_raw_t rc;
    mavlink_msg_rc_channels_raw_decode(&msg, &rc);

    g_rcStatus.rssi = rc.rssi;

    applyRcToGlobals(rc.chan1_raw, rc.chan2_raw, rc.chan3_raw, rc.chan4_raw,
                     rc.chan5_raw, rc.chan6_raw, rc.chan7_raw, rc.chan8_raw,
                     35);
}

// ============================================================
//  RC HANDLER: ID 109 — RADIO_STATUS (RSSI/LQ ELRS)
// ============================================================
void MavlinkHandler::handleRadioStatus(const mavlink_message_t& msg) {
    mavlink_radio_status_t radio;
    mavlink_msg_radio_status_decode(&msg, &radio);
    g_rcStatus.rssi = radio.rssi;
}

// ============================================================
//  RC DEBUG PRINT (opsional, dipanggil dari mavlinkTask)
// ============================================================
void MavlinkHandler::printRCStatus() {
    const char* src = "???";
    if      (g_rcStatus.sourceId == 70) src = "ID70 RC_OVERRIDE";
    else if (g_rcStatus.sourceId == 65) src = "ID65 RC_CHANNELS";
    else if (g_rcStatus.sourceId == 35) src = "ID35 RC_RAW    ";

    uint32_t age = millis() - g_rcStatus.lastUpdateMs;
    // bool timeout = (age > RC_TIMEOUT_MS);

    // Serial.printf("[ RC | %s | RSSI:%3u | Age:%4lums%s ]\n",
    //               src, g_rcStatus.rssi, (unsigned long)age,
    //               timeout ? " TIMEOUT!" : "");
    // Serial.printf("  CH1(Roll)=%4u  CH2(Pitch)=%4u  CH3(Thr)=%4u  CH4(Yaw)=%4u\n",
    //               ch_roll, ch_pitch, ch_throttle, ch_yaw);
    // Serial.printf("  CH5=%4u  CH6=%4u  CH7=%4u  CH8=%4u\n\n",
    //               ch5, ch6, ch7, ch8);
}

// ============================================================
//  HANDLE PORTS — Parse semua byte masuk dari setiap serial port
//  RC (ID 70/65/35/109) akan otomatis diproses di sini juga
// ============================================================
void MavlinkHandler::handlePorts() {
    // --- USB (GCS via Mission Planner / QGC) ---
    // while (Serial.available()) {
    //     uint8_t c = Serial.read();
    //     if (mavlink_parse_char(MAVLINK_COMM_0, c, &rxmsg, &st_usb)) {
    //         processAsFC(rxmsg, PORT_USB);
    //         forwardToOthers(rxmsg, PORT_USB);
    //     }
    // }

    // --- Serial2: Telemetry Radio DAN ELRS RC ---
    //  ELRS dalam MAVLink mode mengirim RC_CHANNELS_OVERRIDE (ID 70)
    //  bersama heartbeat melalui port ini. Semua message dari Serial2
    //  diparse di sini — FC messages diteruskan ke USB & RPi.
    while (Serial2.available()) {
        uint8_t c = Serial2.read();
        if (mavlink_parse_char(MAVLINK_COMM_1, c, &rxmsg, &st_tlm)) {
            // Cek apakah ini RC message dari ELRS
            switch (rxmsg.msgid) {
                case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
                    handleRcChannelsOverride(rxmsg);
                    break;
                case MAVLINK_MSG_ID_RC_CHANNELS:
                    handleRcChannels(rxmsg);
                    break;
                case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
                    handleRcChannelsRaw(rxmsg);
                    break;
                case MAVLINK_MSG_ID_RADIO_STATUS:
                    handleRadioStatus(rxmsg);
                    break;
                default:
                    // Bukan RC message → proses sebagai perintah FC
                    processAsFC(rxmsg, PORT_TLM);
                    forwardToOthers(rxmsg, PORT_TLM);
                    break;
            }
        }
    }

    // --- Serial7: Raspberry Pi ---
    while (Serial7.available()) {
        uint8_t c = Serial7.read();
        if (mavlink_parse_char(MAVLINK_COMM_2, c, &rxmsg, &st_rpi)) {
            processAsFC(rxmsg, PORT_RPI);
            forwardToOthers(rxmsg, PORT_RPI);
        }
    }
}

// ============================================================
//  PROCESS AS FC — Routing perintah masuk ke handler
// ============================================================
void MavlinkHandler::processAsFC(const mavlink_message_t& msg, PortId from) {
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_COMMAND_LONG:          handleCommandLong(msg);         break;
        case MAVLINK_MSG_ID_STATUSTEXT:            handleStatusText(msg);          break;
        case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:  handleSetPositionTargetLocalNed(msg);  break;
        case MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT: handleSetPositionTargetGlobalInt(msg); break;
        case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:  handleMissionRequestList(msg);  break;
        case MAVLINK_MSG_ID_MISSION_REQUEST:       handleMissionRequest(msg);      break;
        case MAVLINK_MSG_ID_MISSION_REQUEST_INT:   handleMissionRequestInt(msg);   break;
        case MAVLINK_MSG_ID_MISSION_COUNT:         handleMissionCount(msg);        break;
        case MAVLINK_MSG_ID_MISSION_ITEM_INT:      handleMissionItemInt(msg);      break;
        case MAVLINK_MSG_ID_MISSION_ITEM:          handleMissionItem(msg);         break;
        case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:     handleMissionClearAll(msg);     break;
        default: break;
    }
}

// ============================================================
//  FORWARD TO OTHERS
// ============================================================
void MavlinkHandler::forwardToOthers(const mavlink_message_t& msg, PortId from) {
    // Serial USB dinonaktifkan agar debug Serial Monitor tidak banjir binary.
    // Aktifkan jika GCS konek via USB:
    // if (from != PORT_USB) mavWriteUSB(Serial, msg);
    if (from != PORT_TLM) mavWrite(Serial2, msg);
    if (from != PORT_RPI) mavWrite(Serial7, msg);
}

// ============================================================
//  SEND HELPERS
// ============================================================
void MavlinkHandler::sendCommandAck(uint16_t command, uint8_t result) {
    mavlink_message_t msg;
    mavlink_msg_command_ack_pack(SYSTEM_ID, COMPONENT_ID, &msg, command, result, 0, 0, 0, 0);
    mavSendAll(msg);
}

void MavlinkHandler::sendStatusText(uint8_t severity, const char* text) {
    mavlink_message_t msg;
    mavlink_msg_statustext_pack(SYSTEM_ID, COMPONENT_ID, &msg, severity, text);
    mavSendAll(msg);
}

void MavlinkHandler::sendCustomTextToGCS(const char* text) {
    sendStatusText(MAV_SEVERITY_INFO, text);
}

// ============================================================
//  HANDLE STATUSTEXT (dari GCS/RPi)
// ============================================================
void MavlinkHandler::handleStatusText(const mavlink_message_t& msg) {
    mavlink_statustext_t status;
    mavlink_msg_statustext_decode(&msg, &status);

    if (msg.sysid == SYSTEM_ID && msg.compid == COMPONENT_ID) return;

    if (strncmp(status.text, "DROP", 4) == 0) {
        payload_drop_command = true;
        sendStatusText(MAV_SEVERITY_INFO, "PAYLOAD DROP TRIGGERED");
        return;
    }

    char echo_text[50];
    snprintf(echo_text, sizeof(echo_text), "Echo: %s", status.text);
    sendStatusText(MAV_SEVERITY_INFO, echo_text);
}

// ============================================================
//  HANDLE COMMAND_LONG
// ============================================================
void MavlinkHandler::handleCommandLong(const mavlink_message_t& msg) {
    mavlink_command_long_t cmd;
    mavlink_msg_command_long_decode(&msg, &cmd);

    if (cmd.target_system != SYSTEM_ID && cmd.target_system != 0) return;

    uint8_t result = MAV_RESULT_UNSUPPORTED;

    switch (cmd.command) {
        case MAV_CMD_COMPONENT_ARM_DISARM: {
            float arm_value = cmd.param1;
            float force     = cmd.param2;
            Serial.printf("ARM REQ: arm=%.1f, force=%.1f\n", arm_value, force);

            if (arm_value == 1.0f) {
                if (gepees.getStatus() < 2 && force != 21196.0f) {
                    result = MAV_RESULT_DENIED;
                    sendStatusText(MAV_SEVERITY_WARNING, "ARM DENIED: NO GPS FIX");
                } else {
                    arming = true;
                    result = MAV_RESULT_ACCEPTED;
                    sendStatusText(MAV_SEVERITY_INFO, "ARMED");
                }
            } else if (arm_value == 0.0f) {
                arming = false;
                result = MAV_RESULT_ACCEPTED;
                sendStatusText(MAV_SEVERITY_INFO, "DISARMED");
            }
            break;
        }

        case MAV_CMD_DO_SET_MODE: {
            uint32_t    custom_mode = (uint32_t)cmd.param2;
            int         new_mode   = 0;
            const char* mode_name  = "UNKNOWN";

            if      (custom_mode == 0)  { new_mode = 1; mode_name = "MANUAL"; }
            else if (custom_mode == 5)  { new_mode = 2; mode_name = "FBWA"; }
            else if (custom_mode == 10) { new_mode = 3; mode_name = "AUTO"; }
            else if (custom_mode == 15) { new_mode = 4; mode_name = "GUIDED"; }  //  GCS-only mode
            else {
                result = MAV_RESULT_UNSUPPORTED;
                sendStatusText(MAV_SEVERITY_WARNING, "MODE NOT SUPPORTED");
                break;
            }

            mode_now = new_mode;
            result   = MAV_RESULT_ACCEPTED;
            char msg_buf[50];
            snprintf(msg_buf, sizeof(msg_buf), "MODE: %s", mode_name);
            sendStatusText(MAV_SEVERITY_INFO, msg_buf);
            break;
        }

        default: result = MAV_RESULT_UNSUPPORTED; break;
    }

    sendCommandAck(cmd.command, result);
}

// ============================================================
//  MISSION HANDLERS
// ============================================================
void MavlinkHandler::handleMissionRequestList(const mavlink_message_t& msg) {
    mavlink_message_t reply;
    mavlink_msg_mission_count_pack(SYSTEM_ID, COMPONENT_ID, &reply,
                                   msg.sysid, msg.compid, wp_sum,
                                   MAV_MISSION_TYPE_MISSION);
    mavSendAll(reply);
    Serial.printf("Sent mission count: %d\n", wp_sum);
}

void MavlinkHandler::handleMissionRequest(const mavlink_message_t& msg) {
    mavlink_mission_request_t request;
    mavlink_msg_mission_request_decode(&msg, &request);
    if (request.seq >= wp_sum) return;

    mavlink_message_t reply;
    mavlink_msg_mission_item_int_pack(
        SYSTEM_ID, COMPONENT_ID, &reply,
        msg.sysid, msg.compid,
        request.seq, MAV_FRAME_GLOBAL_RELATIVE_ALT,
        MAV_CMD_NAV_WAYPOINT, 0, 1,
        0, 0, 0, 0,
        waypoint[request.seq].lat,
        waypoint[request.seq].lng,
        waypoint[request.seq].alt / 100.0f,
        MAV_MISSION_TYPE_MISSION);
    mavSendAll(reply);
    Serial.printf("Sent mission item %d\n", request.seq);

    if (request.seq == wp_sum - 1) {
        mavlink_message_t ack;
        mavlink_msg_mission_ack_pack(SYSTEM_ID, COMPONENT_ID, &ack,
                                     msg.sysid, msg.compid,
                                     MAV_MISSION_ACCEPTED,
                                     MAV_MISSION_TYPE_MISSION);
        mavSendAll(ack);
    }
}

void MavlinkHandler::handleMissionRequestInt(const mavlink_message_t& msg) {
    mavlink_mission_request_int_t request;
    mavlink_msg_mission_request_int_decode(&msg, &request);
    if (request.seq >= wp_sum) return;

    mavlink_message_t reply;
    mavlink_msg_mission_item_int_pack(
        SYSTEM_ID, COMPONENT_ID, &reply,
        msg.sysid, msg.compid,
        request.seq, MAV_FRAME_GLOBAL_RELATIVE_ALT,
        MAV_CMD_NAV_WAYPOINT, 0, 1,
        0, 0, 0, 0,
        waypoint[request.seq].lat,
        waypoint[request.seq].lng,
        waypoint[request.seq].alt / 100.0f,
        MAV_MISSION_TYPE_MISSION);
    mavSendAll(reply);
    Serial.printf("Sent mission item int %d\n", request.seq);

    if (request.seq == wp_sum - 1) {
        mavlink_message_t ack;
        mavlink_msg_mission_ack_pack(SYSTEM_ID, COMPONENT_ID, &ack,
                                     msg.sysid, msg.compid,
                                     MAV_MISSION_ACCEPTED,
                                     MAV_MISSION_TYPE_MISSION);
        mavSendAll(ack);
    }
}

void MavlinkHandler::handleMissionCount(const mavlink_message_t& msg) {
    mavlink_mission_count_t count_msg;
    mavlink_msg_mission_count_decode(&msg, &count_msg);
    expected_count = count_msg.count;
    Serial.printf("[MAV] Receiving mission count: %d\n", expected_count);

    if (expected_count > MAX_MISSION) {
        mavlink_message_t ack;
        mavlink_msg_mission_ack_pack(SYSTEM_ID, COMPONENT_ID, &ack,
                                     msg.sysid, msg.compid,
                                     MAV_MISSION_ERROR,
                                     MAV_MISSION_TYPE_MISSION);
        mavSendAll(ack);
        sendStatusText(MAV_SEVERITY_WARNING, "Mission count too large");
        return;
    }

    mission_state       = MISSION_RECEIVING;
    current_upload_seq  = 0;

    //  FIX: Small delay to ensure ACK is sent before requesting first item
    delayMicroseconds(100);

    mavlink_message_t req;
    mavlink_msg_mission_request_int_pack(SYSTEM_ID, COMPONENT_ID, &req,
                                          msg.sysid, msg.compid, 0,
                                          MAV_MISSION_TYPE_MISSION);
    mavSendAll(req);
    Serial.printf("[MAV] Requesting mission item 0\n");
}

void MavlinkHandler::handleMissionItemInt(const mavlink_message_t& msg) {
    if (mission_state != MISSION_RECEIVING) {
        Serial.printf("[MAV] Reject: not in MISSION_RECEIVING state\n");
        return;
    }

    mavlink_mission_item_int_t item;
    mavlink_msg_mission_item_int_decode(&msg, &item);

    if (item.seq != current_upload_seq) {
        Serial.printf("[MAV] SEQ MISMATCH: got %d, expected %d\n", item.seq, current_upload_seq);
        mavlink_message_t ack;
        mavlink_msg_mission_ack_pack(SYSTEM_ID, COMPONENT_ID, &ack,
                                     msg.sysid, msg.compid,
                                     MAV_MISSION_INVALID_SEQUENCE,
                                     MAV_MISSION_TYPE_MISSION);
        mavSendAll(ack);
        mission_state = MISSION_IDLE;
        sendStatusText(MAV_SEVERITY_WARNING, "Invalid mission sequence");
        return;
    }

    waypoint[item.seq].lat = item.x;
    waypoint[item.seq].lng = item.y;
    waypoint[item.seq].alt = (int32_t)(item.z * 100.0f);
    Serial.printf("[MAV] WP %d: lat=%ld lon=%ld alt=%.1fm\n", item.seq, (long)item.x, (long)item.y, item.z);

    current_upload_seq++;

    if (current_upload_seq < expected_count) {
        //  FIX: Small delay between requests to avoid overwhelming Raspi
        delayMicroseconds(500);
        
        mavlink_message_t req;
        mavlink_msg_mission_request_int_pack(SYSTEM_ID, COMPONENT_ID, &req,
                                              msg.sysid, msg.compid,
                                              current_upload_seq,
                                              MAV_MISSION_TYPE_MISSION);
        mavSendAll(req);
        Serial.printf("[MAV] Requesting mission item %d\n", current_upload_seq);
    } else {
        wp_sum = expected_count;
        mavlink_message_t ack;
        mavlink_msg_mission_ack_pack(SYSTEM_ID, COMPONENT_ID, &ack,
                                     msg.sysid, msg.compid,
                                     MAV_MISSION_ACCEPTED,
                                     MAV_MISSION_TYPE_MISSION);
        mavSendAll(ack);
        Serial.printf("[MAV] Mission upload complete: %d waypoints\n", wp_sum);
        mission_state = MISSION_IDLE;
        sendStatusText(MAV_SEVERITY_INFO, "Mission upload complete");
        if (saveWaypointsToEEPROM()) sendStatusText(MAV_SEVERITY_INFO, "Saved to EEPROM");
    }
}

void MavlinkHandler::handleMissionItem(const mavlink_message_t& msg) {
    if (mission_state != MISSION_RECEIVING) {
        Serial.printf("[MAV] Reject: not in MISSION_RECEIVING state\\n");
        return;
    }

    mavlink_mission_item_t item;
    mavlink_msg_mission_item_decode(&msg, &item);

    if (item.seq != current_upload_seq) {
        Serial.printf("[MAV] SEQ MISMATCH: got %d, expected %d\\n", item.seq, current_upload_seq);
        mavlink_message_t ack;
        mavlink_msg_mission_ack_pack(SYSTEM_ID, COMPONENT_ID, &ack,
                                     msg.sysid, msg.compid,
                                     MAV_MISSION_INVALID_SEQUENCE,
                                     MAV_MISSION_TYPE_MISSION);
        mavSendAll(ack);
        mission_state = MISSION_IDLE;
        sendStatusText(MAV_SEVERITY_WARNING, "Invalid mission sequence");
        return;
    }

    waypoint[item.seq].lat = (int32_t)(item.x * 1e7);
    waypoint[item.seq].lng = (int32_t)(item.y * 1e7);
    waypoint[item.seq].alt = (int32_t)(item.z * 100.0f);
    Serial.printf("[MAV] WP %d (legacy): lat=%.7f lon=%.7f alt=%.1fm\\n", item.seq, item.x, item.y, item.z);
    current_upload_seq++;

    if (current_upload_seq < expected_count) {
        //  FIX: Small delay between requests to avoid overwhelming Raspi
        delayMicroseconds(500);
        
        mavlink_message_t req;
        mavlink_msg_mission_request_pack(SYSTEM_ID, COMPONENT_ID, &req,
                                          msg.sysid, msg.compid,
                                          current_upload_seq,
                                          MAV_MISSION_TYPE_MISSION);
        mavSendAll(req);
        Serial.printf("[MAV] Requesting mission item %d\\n", current_upload_seq);
    } else {
        wp_sum = expected_count;
        mavlink_message_t ack;
        mavlink_msg_mission_ack_pack(SYSTEM_ID, COMPONENT_ID, &ack,
                                     msg.sysid, msg.compid,
                                     MAV_MISSION_ACCEPTED,
                                     MAV_MISSION_TYPE_MISSION);
        mavSendAll(ack);
        Serial.printf("[MAV] Mission upload complete: %d waypoints\\n", wp_sum);
        mission_state = MISSION_IDLE;
        sendStatusText(MAV_SEVERITY_INFO, "Mission upload complete");
        if (saveWaypointsToEEPROM()) sendStatusText(MAV_SEVERITY_INFO, "Saved to EEPROM");
    }
}

void MavlinkHandler::handleMissionClearAll(const mavlink_message_t& msg) {
    wp_sum = 0;
    clearWaypointEEPROM();

    mavlink_message_t ack;
    mavlink_msg_mission_ack_pack(SYSTEM_ID, COMPONENT_ID, &ack,
                                 msg.sysid, msg.compid,
                                 MAV_MISSION_ACCEPTED,
                                 MAV_MISSION_TYPE_MISSION);
    mavSendAll(ack);
    sendStatusText(MAV_SEVERITY_INFO, "Missions cleared");
}

void MavlinkHandler::handleSetPositionTargetLocalNed(const mavlink_message_t& msg) {
    mavlink_set_position_target_local_ned_t sp;
    mavlink_msg_set_position_target_local_ned_decode(&msg, &sp);

    const uint16_t ignore_x = (1U << 0);
    const uint16_t ignore_y = (1U << 1);
    const uint16_t ignore_z = (1U << 2);
    if ((sp.type_mask & (ignore_x | ignore_y | ignore_z)) != 0) {
        return;
    }

    Locations origin = home_wp_loc;
    if (!origin.initialised()) {
        origin = current_loc;
    }

    Locations target = origin;
    target.offset(sp.x, sp.y);
    target.alt = origin.alt - (int32_t)lroundf(sp.z * 100.0f);

    rpi_external_setpoint_target = target;
    rpi_external_setpoint_last_ms = millis();
    rpi_external_setpoint_active = true;
    rpi_external_setpoint_yaw_deg = degrees(sp.yaw);
    rpi_external_setpoint_yaw_valid = !isnan(sp.yaw);
}

void MavlinkHandler::handleSetPositionTargetGlobalInt(const mavlink_message_t& msg) {
    mavlink_set_position_target_global_int_t sp;
    mavlink_msg_set_position_target_global_int_decode(&msg, &sp);

    const uint16_t ignore_x = (1U << 0);
    const uint16_t ignore_y = (1U << 1);
    const uint16_t ignore_z = (1U << 2);
    if ((sp.type_mask & (ignore_x | ignore_y | ignore_z)) != 0) {
        return;
    }

    Locations target;
    target.lat = sp.lat_int;
    target.lng = sp.lon_int;
    target.alt = (int32_t)lroundf(sp.alt * 100.0f);

    rpi_external_setpoint_target = target;
    rpi_external_setpoint_last_ms = millis();
    rpi_external_setpoint_active = true;
    rpi_external_setpoint_yaw_deg = degrees(sp.yaw);
    rpi_external_setpoint_yaw_valid = !isnan(sp.yaw);
}

// ============================================================
//  HEARTBEAT
// ============================================================
void MavlinkHandler::sendHeartbeat() {
    mavlink_message_t msg;
    uint8_t system_status = arming ? MAV_STATE_ACTIVE : MAV_STATE_STANDBY;
    uint8_t base_mode     = getBaseMode();
    if (arming) base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;

    mavlink_msg_heartbeat_pack(SYSTEM_ID, COMPONENT_ID, &msg,
                               MAV_TYPE_FIXED_WING,
                               MAV_AUTOPILOT_ARDUPILOTMEGA,
                               base_mode, getCustomMode(), system_status);
    mavSendAll(msg);
}

uint8_t MavlinkHandler::getBaseMode() const {
    switch (current) {
        case ModeId::MANU: return MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
        case ModeId::FBWA: return MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_MANUAL_INPUT_ENABLED | MAV_MODE_FLAG_STABILIZE_ENABLED;
        case ModeId::AUTO: return MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_AUTO_ENABLED | MAV_MODE_FLAG_GUIDED_ENABLED;
        default:           return MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    }
}

uint32_t MavlinkHandler::getCustomMode() const {
    switch (current) {
        case ModeId::MANU: return 0;
        case ModeId::FBWA: return 5;
        case ModeId::AUTO: return 10;
        default:           return 0;
    }
}

// ============================================================
//  TELEMETRY SENDERS
// ============================================================
void MavlinkHandler::sendAlert(const char* txt) {
    mavlink_message_t msg;
    mavlink_msg_statustext_pack(SYSTEM_ID, COMPONENT_ID, &msg, MAV_SEVERITY_WARNING, txt);
    mavSendAll(msg);
}

void MavlinkHandler::sendAltitude(float altitude) {
    mavlink_message_t msg;
    uint64_t t_usec = (uint64_t)getMillis() * 1000ULL;
    mavlink_msg_altitude_pack(SYSTEM_ID, COMPONENT_ID, &msg,
                              t_usec, altitude, altitude, 0, altitude, 0, 0.0f);
    mavSendAll(msg);
}

void MavlinkHandler::sendAttitude(float roll_rad, float pitch_rad, float yaw_rad) {
    mavlink_message_t msg;
    mavlink_msg_attitude_pack(SYSTEM_ID, COMPONENT_ID, &msg,
                              getMillis(), roll_rad, pitch_rad, yaw_rad, 0, 0, 0);
    mavSendAll(msg);
}

void MavlinkHandler::sendGpsRawInt(uint8_t fix_type, int32_t lat, int32_t lon,
                                    uint16_t eph, uint16_t vel, uint16_t cog,
                                    uint8_t satellites_visible) {
    mavlink_message_t msg;
    uint64_t t_usec = (uint64_t)getMillis() * 1000ULL;
    mavlink_msg_gps_raw_int_pack(SYSTEM_ID, COMPONENT_ID, &msg,
                                 t_usec, fix_type, lat, lon, 0,
                                 eph, 0, vel, cog, satellites_visible,
                                 0, 0, 0, 0, 0);
    mavSendAll(msg);
}

void MavlinkHandler::sendBatteryStatus(float voltage, int16_t current_amp, int8_t battery_remaining) {
    mavlink_message_t msg;
    uint16_t voltage_mv = (uint16_t)(voltage * 1000.0f);
    int16_t  current_ca = (current_amp == 0) ? -1 : (current_amp * 100);

    mavlink_msg_battery_status_pack(SYSTEM_ID, COMPONENT_ID, &msg,
                                    0, 0, 0, INT16_MAX,
                                    &voltage_mv, current_ca,
                                    -1, -1, battery_remaining, 0, 0);
    mavSendAll(msg);
}

void MavlinkHandler::sendNavControllerOutput(float nav_roll, float nav_pitch,
                                              float nav_bearing, float target_bearing,
                                              float wp_dist, float alt_error,
                                              float aspd_error, float xtrack_error) {
    mavlink_message_t msg;
    mavlink_msg_nav_controller_output_pack(SYSTEM_ID, COMPONENT_ID, &msg,
                                           nav_roll, nav_pitch,
                                           nav_bearing, target_bearing,
                                           wp_dist, alt_error,
                                           aspd_error, xtrack_error);
    mavSendAll(msg);
}

void MavlinkHandler::sendVfrHud(float airspeed, float groundspeed, float heading,
                                 float altitude, float climb_rate) {
    mavlink_message_t msg;
    mavlink_msg_vfr_hud_pack(SYSTEM_ID, COMPONENT_ID, &msg,
                             airspeed, groundspeed, heading,
                             0, altitude, climb_rate);
    mavSendAll(msg);
}

void MavlinkHandler::sendMissionItemReached(uint16_t seq) {
    mavlink_message_t msg;
    mavlink_msg_mission_item_reached_pack(SYSTEM_ID, COMPONENT_ID, &msg, seq);
    mavSendAll(msg);
    Serial.printf("WP %d reached\n", seq);
}

void MavlinkHandler::notifyWaypointReached(uint16_t seq) {
    sendMissionItemReached(seq);
}

// ============================================================
//  MAIN PERIODIC TASK
// ============================================================
void MavlinkHandler::mavlinkTask() {
    unsigned long now = getMillis();

    // Heartbeat
    if (now - lastHb >= HEARTBEAT_MS) {
        lastHb = now;
        sendHeartbeat();
    }

    // Altitude
    if (now - lastAlt >= ALT_MS) {
        lastAlt = now;
        sendAltitude(baro.altitude);
    }

    // GPS
    if (now - lastGps >= GPS_MS) {
        lastGps = now;
        uint16_t cog_centi = 65535;
        if (gepees.getStatus() >= 2)
            cog_centi = (uint16_t)(gepees.getCourse() * 100.0f);

        sendGpsRawInt(gepees.getStatus(),
                      (int32_t)(gepees.latitude  * 1e7),
                      (int32_t)(gepees.longitude * 1e7),
                      (uint16_t)(gepees.hdop * 100),
                      (uint16_t)(gepees.speed_mps * 100),
                      imu.heading,
                      gepees.satellites);
    }

    // Attitude
    if (now - lastAtt >= ATT_MS) {
        lastAtt = now;
        sendAttitude(imu.rad_roll, imu.rad_pitch, imu.rad_yaw);
    }

    // Roll alert
    if (fabs(imu.roll) > ALERT_THRESHOLD_DEG && (now - lastAlert >= ALERT_DEBOUNCE_MS)) {
        char msg[50];
        snprintf(msg, sizeof(msg), "ALERT: ROLL LIMIT (%.1f deg)", imu.roll);
        sendAlert(msg);
        lastAlert = now;
    }

    // Battery
    if (now - lastBattery >= BATTERY_MS) {
        lastBattery = now;
        sendBatteryStatus(batt_v, 0, (int8_t)display_batt());
    }

    // Nav controller
    if (now - lastNavController >= NAV_CONTROLLER_MS) {
        lastNavController = now;
        sendNavControllerOutput(imu.rad_roll, imu.rad_pitch,
                                atan2(0, 0), atan2(0, 0),
                                auto_state.distance_next_wp,
                                0, 0, 0);
    }

    // VFR HUD
    if (now - lastVfr >= VFR_MS) {
        lastVfr = now;
        float heading = (imu.yaw < 0) ? imu.yaw + 360 : imu.yaw;
        sendVfrHud(arspd.v_ms, gepees.speed_mps, heading, baro.altitude, 0);
    }

    // RC debug print - aktif agar bisa monitor RC di Serial Monitor
    // if (now - lastRcPrint >= RC_PRINT_MS) {
    //     lastRcPrint = now;
    //     if (g_rcStatus.valid) {
    //         printRCStatus();
    //     } else {
    //         static uint32_t lastWait = 0;
    //         if (now - lastWait >= 2000) {
    //             lastWait = now;
    //             Serial.printf("[RC WAIT] Belum ada RC | Total bytes masuk Serial2: cek wiring/baud\n");
    //         }
    //     }
    // }
}

// ============================================================
//  GLOBAL HELPER
// ============================================================
void notifyWaypointReached(uint16_t seq) {
    extern MavlinkHandler mavHandler;
    mavHandler.notifyWaypointReached(seq);
}

#endif  // MAV_H