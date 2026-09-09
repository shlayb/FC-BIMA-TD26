#pragma once
#include <Arduino.h>
#include <EEPROM.h>

#include "FW_config.h"

#include "takeoff.h"

// Harus di-include SETELAH FW_CONTROL_FBWA.h karena butuh fw_control
// Harus di-include SETELAH kedua file kontrol
extern FW_CONTROL fw_control;
extern gains gain;
extern float model_uav;
extern float _L1_period;
extern float _L1_damping;

// Actuator Pin variables
extern float SERVO_RUD_L;
extern float MOTOR_1_PIN;
extern float SERVO_ELE;
extern float MOTOR_2_PIN;
extern float SERVO_AIL_L;
extern float MOTOR_3_PIN;
extern float THROTTLE;
extern float MOTOR_4_PIN;
extern float SERVO_RUD_R;
extern float SERVO_AIL_R;
extern float SERVO_PAYLOAD;

// Airspeed & RTL variables
extern float AIRSPEED_CRUISE;
extern float MIN_AIRSPEED;
extern float MAX_AIRSPEED;
extern float AIRSPEED_STALL;
extern float rtl_altitude;

// Loiter & Mission variables
extern float wp_radius;
extern float loiter_mode_radius;
extern float loiter_bank_limit;
extern float mis_done_behave;

// ============================================================
//  Param dengan pointer langsung ke variabel aslinya
//  Ketika nilai diubah, variabel target langsung berubah
// ============================================================
struct Param {
    char    name[17];
    float*  target;
    float   default_val;
    float   min_val;
    float   max_val;
};

static Param g_params[100];
static uint16_t PARAM_COUNT = 0;

inline void initParams() {
    uint16_t i = 0;

    // ============================================================
    //  FIXED WING - fw_control.fw_gain
    // ============================================================
    g_params[i++] = {"FW_ROLL_P",      &fw_control.fw_gain.rate_roll_P,       10.0f,   0.0f, 100.0f};
    g_params[i++] = {"FW_ROLL_I",      &fw_control.fw_gain.rate_roll_I,        0.0f,   0.0f,  10.0f};
    g_params[i++] = {"FW_ROLL_D",      &fw_control.fw_gain.rate_roll_D,        0.0f,   0.0f,  10.0f};
    g_params[i++] = {"FW_ROLL_IMAX",   &fw_control.fw_gain.rate_roll_IMAX,   500.0f,   0.0f,1000.0f};
    g_params[i++] = {"FW_PITCH_P",     &fw_control.fw_gain.rate_pitch_P,      10.0f,   0.0f, 100.0f};
    g_params[i++] = {"FW_PITCH_I",     &fw_control.fw_gain.rate_pitch_I,       0.0f,   0.0f,  10.0f};
    g_params[i++] = {"FW_PITCH_D",     &fw_control.fw_gain.rate_pitch_D,       0.2f,   0.0f,  10.0f};
    g_params[i++] = {"FW_PITCH_IMAX",  &fw_control.fw_gain.rate_pitch_IMAX,  500.0f,   0.0f,1000.0f};
    g_params[i++] = {"FW_YAW_P",       &fw_control.fw_gain.rate_yaw_P,         0.0f,   0.0f, 100.0f};
    g_params[i++] = {"FW_YAW_I",       &fw_control.fw_gain.rate_yaw_I,         0.0f,   0.0f,  10.0f};
    g_params[i++] = {"FW_YAW_D",       &fw_control.fw_gain.rate_yaw_D,         0.0f,   0.0f,  10.0f};
    g_params[i++] = {"FW_YAW_IMAX",    &fw_control.fw_gain.rate_yaw_IMAX,      8.0f,   0.0f,1000.0f};
    g_params[i++] = {"FW_THR_P",       &fw_control.fw_gain.rate_thr_P,        35.0f,   0.0f, 200.0f};
    g_params[i++] = {"FW_THR_I",       &fw_control.fw_gain.rate_thr_I,         0.0f,   0.0f,  10.0f};
    g_params[i++] = {"FW_THR_D",       &fw_control.fw_gain.rate_thr_D,         0.0f,   0.0f,  10.0f};

    // ============================================================
    //  COPTER STABILIZE - gain.stab_*
    // ============================================================
    g_params[i++] = {"CP_STAB_R_P",    &gain.stab_roll_P,                      4.5f,   0.0f,  20.0f};
    g_params[i++] = {"CP_STAB_P_P",    &gain.stab_pitch_P,                     4.0f,   0.0f,  20.0f};
    g_params[i++] = {"CP_STAB_Y_P",    &gain.stab_yaw_P,                       5.0f,   0.0f,  20.0f};

    // ============================================================
    //  COPTER RATE ROLL - gain.rate_roll_*
    // ============================================================
    g_params[i++] = {"CP_RATE_R_P",    &gain.rate_roll_P,                      5.0f,   0.0f, 100.0f};
    g_params[i++] = {"CP_RATE_R_I",    &gain.rate_roll_I,                      0.01f,  0.0f,  10.0f};
    g_params[i++] = {"CP_RATE_R_D",    &gain.rate_roll_D,                      0.05f,  0.0f,  10.0f};
    g_params[i++] = {"CP_RATE_R_IMAX", &gain.rate_roll_IMAX,                 500.0f,   0.0f,1000.0f};

    // ============================================================
    //  COPTER RATE PITCH - gain.rate_pitch_*
    // ============================================================
    g_params[i++] = {"CP_RATE_P_P",    &gain.rate_pitch_P,                     5.0f,   0.0f, 100.0f};
    g_params[i++] = {"CP_RATE_P_I",    &gain.rate_pitch_I,                     0.01f,  0.0f,  10.0f};
    g_params[i++] = {"CP_RATE_P_D",    &gain.rate_pitch_D,                     0.05f,  0.0f,  10.0f};
    g_params[i++] = {"CP_RATE_P_IMAX", &gain.rate_pitch_IMAX,                500.0f,   0.0f,1000.0f};

    // ============================================================
    //  COPTER RATE YAW - gain.rate_yaw_*
    // ============================================================
    g_params[i++] = {"CP_RATE_Y_P",    &gain.rate_yaw_P,                      30.0f,   0.0f, 200.0f};
    g_params[i++] = {"CP_RATE_Y_I",    &gain.rate_yaw_I,                      0.01f,   0.0f,  10.0f};
    g_params[i++] = {"CP_RATE_Y_D",    &gain.rate_yaw_D,                      0.05f,   0.0f,  10.0f};
    g_params[i++] = {"CP_RATE_Y_IMAX", &gain.rate_yaw_IMAX,                    8.0f,   0.0f,1000.0f};

    // ============================================================
    //  COPTER ALTITUDE - gain.k_alt, k_z_velocity
    // ============================================================
    g_params[i++] = {"CP_ALT_P",       &gain.k_alt,                            6.5f,   0.0f,  50.0f};
    g_params[i++] = {"CP_ZVEL_P",      &gain.k_z_velocity,                     3.0f,   0.0f,  20.0f};

    // ============================================================
    //  UAV MODEL TYPE
    //  0 = Fixed Wing
    //  1 = VTOL
    //  Taruh di akhir supaya EEPROM param lama tidak geser.
    // ============================================================
    g_params[i++] = {"MODEL_UAV",      &model_uav,                             0.0f,   0.0f,   1.0f};

    // ============================================================
    //  FIXED WING AUTO & TECS TUNING (PX4 STYLE)
    // ============================================================
    g_params[i++] = {"FW_THR_CRUISE",  &tecs_cruise_throttle,                 55.0f,   0.0f, 100.0f};
    g_params[i++] = {"FW_THR_MIN",     &tecs_min_throttle,                     0.0f,   0.0f, 100.0f};
    g_params[i++] = {"FW_THR_MAX",     &tecs_max_throttle,                    85.0f,   0.0f, 100.0f};
    g_params[i++] = {"FW_GND_SPD_MIN", &min_gndspeed,                          5.0f,   0.0f,  30.0f};
    g_params[i++] = {"FW_R_LIM_MAX",   &roll_max_deg,                         35.0f,  10.0f,  60.0f};
    g_params[i++] = {"FW_P_LIM_MAX",   &pitch_max_deg,                        18.0f,   5.0f,  45.0f};
    g_params[i++] = {"FW_P_LIM_MIN",   &pitch_min_deg,                       -12.0f, -45.0f,  -2.0f};
    g_params[i++] = {"FW_L1_PERIOD",   &_L1_period,                           22.0f,   5.0f,  50.0f};
    g_params[i++] = {"FW_L1_DAMPING",  &_L1_damping,                          0.75f,   0.5f,   1.0f};

    // ============================================================
    //  AIRSPEED & RTL PARAMETERS
    // ============================================================
    g_params[i++] = {"AIRSPEED_CRUISE", &AIRSPEED_CRUISE,                      21.0f,   5.0f,  35.0f};
    g_params[i++] = {"MIN_AIRSPEED",    &MIN_AIRSPEED,                         12.0f,   5.0f,  30.0f};
    g_params[i++] = {"MAX_AIRSPEED",    &MAX_AIRSPEED,                         30.0f,  10.0f,  45.0f};
    g_params[i++] = {"AIRSPEED_STALL",  &AIRSPEED_STALL,                        7.0f,   5.0f,  25.0f};
    g_params[i++] = {"RTL_ALTITUDE",    &rtl_altitude,                         60.0f,  10.0f, 200.0f};

    // ============================================================
    //  TAKEOFF / LAUNCH GUARD (ArduPilot TKOFF_* style)
    // ============================================================
    g_params[i++] = {"TKOFF_LVL_ALT",   &launch_guard.safe_climb_alt,          15.0f,   2.0f, 100.0f};      // Alt selesai climbout (m)
    g_params[i++] = {"TKOFF_LVL_PITCH", &launch_guard.takeoff_pitch_deg,       15.0f,   0.0f,  35.0f};      // Target pitch climbout (deg)
    g_params[i++] = {"TKOFF_THR_MINACC",&launch_guard.throw_accel_thresh,      13.0f,   5.0f,  35.0f};      // Accel lempar (m/s^2)
    g_params[i++] = {"TKOFF_THR_DELAY", &launch_guard.release_delay_ms,       400.0f,   0.0f,2000.0f};      // Jeda lepas tangan (ms)
    g_params[i++] = {"TKOFF_THR_RAMP",  &launch_guard.ramp_time_sec,            1.2f,   0.1f,   5.0f};      // Durasi ramp throttle (s)
    g_params[i++] = {"TKOFF_SHAKE_ACC", &launch_guard.shake_accel_thresh,       5.0f,   3.0f,  25.0f};      // Accel shake-to-wake (m/s^2)
    g_params[i++] = {"TKOFF_GND_SPD",   &launch_guard.min_groundspeed_launch,   3.0f,   0.5f,  15.0f};      // Min groundspeed launch (m/s)

    // ============================================================
    //  ACTUATOR PIN MAPPING (CH_MTR_1 .. CH_MTR_11)
    // ============================================================
    g_params[i++] = {"CH_MTR_1",        &SERVO_RUD_L,                           2.0f,   0.0f,  41.0f};
    g_params[i++] = {"CH_MTR_2",        &SERVO_ELE,                            37.0f,   0.0f,  41.0f};
    g_params[i++] = {"CH_MTR_3",        &SERVO_AIL_L,                           6.0f,   0.0f,  41.0f};
    g_params[i++] = {"CH_MTR_4",        &THROTTLE,                             10.0f,   0.0f,  41.0f};
    g_params[i++] = {"CH_MTR_5",        &SERVO_AIL_R,                          33.0f,   0.0f,  41.0f};
    g_params[i++] = {"CH_MTR_6",        &SERVO_PAYLOAD,                        36.0f,   0.0f,  41.0f};
    g_params[i++] = {"CH_MTR_7",        &MOTOR_1_PIN,                           3.0f,   0.0f,  41.0f};
    g_params[i++] = {"CH_MTR_8",        &MOTOR_2_PIN,                           5.0f,   0.0f,  41.0f};
    g_params[i++] = {"CH_MTR_9",        &MOTOR_3_PIN,                           9.0f,   0.0f,  41.0f};
    g_params[i++] = {"CH_MTR_10",       &MOTOR_4_PIN,                          11.0f,   0.0f,  41.0f};
    g_params[i++] = {"CH_MTR_11",       &SERVO_RUD_R,                          13.0f,   0.0f,  41.0f};

    // ============================================================
    //  WAYPOINT, LOITER & MISSION PARAMETERS (ArduPilot Standard)
    // ============================================================
    g_params[i++] = {"WP_RADIUS",       &wp_radius,                           40.0f,    5.0f, 200.0f};
    g_params[i++] = {"WP_LOITER_RAD",   &loiter_mode_radius,                  50.0f, -500.0f, 500.0f};
    g_params[i++] = {"FW_LOIT_BANK",    &loiter_bank_limit,                   35.0f,   15.0f,  60.0f};
    g_params[i++] = {"MIS_DONE_BEHAVE", &mis_done_behave,                      1.0f,    0.0f,   1.0f};

    PARAM_COUNT = i;
}

// ============================================================
//  EEPROM - Teensy 4.1
//  Teensy 4.1 punya EEPROM emulation 4284 byte.
//  Tidak perlu EEPROM.begin() dan tidak perlu EEPROM.commit().
// ============================================================
namespace PARAM_EEPROM {
    const uint16_t MAGIC      = 0xCDAB;
    const uint16_t ADDR_MAGIC = 650;
    const uint16_t ADDR_COUNT = 652;
    const uint16_t ADDR_DATA  = 654;

    const uint16_t TEENSY41_EEPROM_SIZE = 4284;
}

inline float clampParamValue(float val, float min_val, float max_val) {
    if (isnan(val) || isinf(val)) {
        return min_val;
    }
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

inline void applyDefaultParams(uint16_t start_index = 0) {
    if (start_index > PARAM_COUNT) return;

    for (uint16_t i = start_index; i < PARAM_COUNT; i++) {
        *g_params[i].target = g_params[i].default_val;
    }
}

inline bool paramEEPROMRangeOK() {
    uint16_t end_addr = PARAM_EEPROM::ADDR_DATA + (PARAM_COUNT * sizeof(float));

    if (end_addr > PARAM_EEPROM::TEENSY41_EEPROM_SIZE) {
        Serial.printf("[PARAM] EEPROM range overflow: end=%u, max=%u\n",
                      end_addr,
                      PARAM_EEPROM::TEENSY41_EEPROM_SIZE);
        return false;
    }

    return true;
}

inline int findParam(const char* name) {
    for (uint16_t i = 0; i < PARAM_COUNT; i++) {
        if (strncmp(g_params[i].name, name, 16) == 0) {
            return i;
        }
    }
    return -1;
}

inline void saveParamsToEEPROM() {
    if (PARAM_COUNT == 0) {
        Serial.println("[PARAM] Save failed: initParams() not called");
        return;
    }

    if (!paramEEPROMRangeOK()) {
        Serial.println("[PARAM] Save failed: EEPROM range invalid");
        return;
    }

    EEPROM.put(PARAM_EEPROM::ADDR_MAGIC, PARAM_EEPROM::MAGIC);
    EEPROM.put(PARAM_EEPROM::ADDR_COUNT, PARAM_COUNT);

    uint16_t addr = PARAM_EEPROM::ADDR_DATA;

    for (uint16_t i = 0; i < PARAM_COUNT; i++) {
        float val = clampParamValue(*g_params[i].target,
                                    g_params[i].min_val,
                                    g_params[i].max_val);

        *g_params[i].target = val;
        EEPROM.put(addr, val);
        addr += sizeof(float);
    }

    Serial.printf("[PARAM] Saved %u params (addr %u-%u)\n",
                  PARAM_COUNT,
                  PARAM_EEPROM::ADDR_DATA,
                  PARAM_EEPROM::ADDR_DATA + PARAM_COUNT * sizeof(float));
}

inline bool loadParamsFromEEPROM() {
    if (PARAM_COUNT == 0) {
        Serial.println("[PARAM] Load failed: initParams() not called");
        return false;
    }

    if (!paramEEPROMRangeOK()) {
        Serial.println("[PARAM] Load failed: EEPROM range invalid");
        return false;
    }

    uint16_t magic = 0;
    uint16_t saved_count = 0;

    EEPROM.get(PARAM_EEPROM::ADDR_MAGIC, magic);
    EEPROM.get(PARAM_EEPROM::ADDR_COUNT, saved_count);

    if (magic != PARAM_EEPROM::MAGIC) {
        Serial.println("[PARAM] No EEPROM data. Using defaults and saving once.");
        applyDefaultParams(0);
        saveParamsToEEPROM();
        return false;
    }

    uint16_t load_count = min(saved_count, PARAM_COUNT);
    uint16_t addr = PARAM_EEPROM::ADDR_DATA;

    for (uint16_t i = 0; i < load_count; i++) {
        float val = 0.0f;
        EEPROM.get(addr, val);
        addr += sizeof(float);

        *g_params[i].target = clampParamValue(val,
                                              g_params[i].min_val,
                                              g_params[i].max_val);
    }

    // Kalau firmware baru punya param tambahan, misalnya MODEL_UAV,
    // param yang belum ada di EEPROM akan diisi default lalu disimpan.
    if (saved_count < PARAM_COUNT) {
        applyDefaultParams(saved_count);
        saveParamsToEEPROM();
        Serial.printf("[PARAM] EEPROM schema updated: old=%u, new=%u\n",
                      saved_count,
                      PARAM_COUNT);
    }

    Serial.printf("[PARAM] Loaded %u/%u params\n", load_count, PARAM_COUNT);
    return true;
}

inline void resetParamsToDefault() {
    if (PARAM_COUNT == 0) {
        Serial.println("[PARAM] Reset failed: initParams() not called");
        return;
    }

    applyDefaultParams(0);
    saveParamsToEEPROM();
    Serial.println("[PARAM] Reset to defaults and saved");
}

inline bool setParamValue(const char* name, float value, bool save_after_set = true) {
    int idx = findParam(name);

    if (idx < 0) {
        Serial.printf("[PARAM] Set failed: %s not found\n", name);
        return false;
    }

    float val = clampParamValue(value,
                                g_params[idx].min_val,
                                g_params[idx].max_val);

    *g_params[idx].target = val;

    if (save_after_set) {
        saveParamsToEEPROM();
    }

    Serial.printf("[PARAM] %s = %.4f\n", g_params[idx].name, *g_params[idx].target);
    return true;
}

inline bool getParamValue(const char* name, float& value_out) {
    int idx = findParam(name);

    if (idx < 0) {
        return false;
    }

    value_out = *g_params[idx].target;
    return true;
}

inline void printParams() {
    Serial.println("=== PARAMS ===");

    for (uint16_t i = 0; i < PARAM_COUNT; i++) {
        Serial.printf("  [%2u] %-12s = %8.4f  (%.2f - %.2f)\n",
                      i,
                      g_params[i].name,
                      *g_params[i].target,
                      g_params[i].min_val,
                      g_params[i].max_val);
    }
}