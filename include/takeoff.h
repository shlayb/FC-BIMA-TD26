#pragma once
#include <Arduino.h>
#include "Actuator.h"
#include "Auto_setup.h"
#include "AHRS.h"
#include "Barometer.h"
#include "FW_config.h"
#include "FW_control.h"

enum LaunchState : uint8_t {
    LAUNCH_STATE_IDLE = 0,            // Belum aktif / Disarmed
    LAUNCH_STATE_WAIT_GROUND,         // Di darat: Motor MATI TOTAL (988), menunggu lemparan/shake
    LAUNCH_STATE_RELEASE_DELAY,       // Lemparan terdeteksi: Jeda aman 400ms agar tangan pilot aman lepas
    LAUNCH_STATE_THROTTLE_RAMP,       // Ramp up throttle secara halus
    LAUNCH_STATE_CLIMB,               // Mendaki stabil ke safe altitude
    LAUNCH_STATE_COMPLETE             // Selesai takeoff -> navigasi waypoint L1 + TECS aktif penuh
};

class LaunchGuard {
public:
    LaunchState state = LAUNCH_STATE_IDLE;
    
    // Parameter peluncuran (ArduPilot TKOFF_* style, terdaftar di Params.h)
    float throw_accel_thresh = 13.0f;    // TKOFF_THR_MINACC: m/s^2 linear accel untuk deteksi lemparan
    float shake_accel_thresh = 9.0f;     // TKOFF_SHAKE_ACC:  m/s^2 untuk deteksi goyangan (shake)
    float min_groundspeed_launch = 3.0f; // TKOFF_GND_SPD:   m/s min groundspeed jika dilempar cepat
    float release_delay_ms = 400.0f;     // TKOFF_THR_DELAY:  ms jeda tangan menjauh sebelum motor hidup
    float ramp_time_sec = 1.2f;          // TKOFF_THR_RAMP:   detik untuk ramp throttle dari 0 ke target
    float safe_climb_alt = 15.0f;        // TKOFF_LVL_ALT:    meter AGL selesai fase climbout
    float takeoff_pitch_deg = 15.0f;     // TKOFF_LVL_PITCH:  derajat pitch saat climbout
    
    uint32_t trigger_time_ms = 0;
    uint32_t ramp_start_ms = 0;
    float current_throttle_pwm = 988.0f;
    
    // Shake detection state
    uint32_t last_shake_pos_ms = 0;
    bool shake_pos_detected = false;

    bool is_on_ground() const {
        // Pesawat dianggap di darat jika ketinggian AGL < 4 meter dan groundspeed < 5 m/s
        return (relative_alt < 4.0f && ahrs.groundspeed < 5.0f);
    }

    void reset() {
        if (is_on_ground()) {
            state = LAUNCH_STATE_WAIT_GROUND;
        } else {
            state = LAUNCH_STATE_COMPLETE;
        }
        current_throttle_pwm = 988.0f;
        shake_pos_detected = false;
        trigger_time_ms = 0;
        ramp_start_ms = 0;
    }

    void on_enter_auto() {
        reset();
        if (state == LAUNCH_STATE_WAIT_GROUND) {
            Serial.println("[LAUNCH] AUTO entered on GROUND -> Motor LOCKED (OFF: 988). Waiting for Throw/Shake...");
        } else {
            Serial.println("[LAUNCH] AUTO entered in AIR -> Normal flight active");
        }
    }

    void on_exit_auto() {
        reset();
    }

    bool is_launch_complete() const {
        return (state == LAUNCH_STATE_COMPLETE);
    }

    void update(float &pitch_dem_out, float &roll_dem_out, float &throttle_pwm_out) {
        // JIKA DISARMED: Motor DIJAMIN MATI (988) dan state disiapkan ke WAIT_GROUND
        if (!arming) {
            reset();
            throttle_pwm_out = 988.0f;
            return;
        }

        uint32_t now = millis();
        // Linear acceleration magnitude (orientation-safe horizontal)
        float horiz_accel = sqrtf(imu._linear_acc.x * imu._linear_acc.x + imu._linear_acc.y * imu._linear_acc.y);
        float max_lin_acc = max(fabsf(imu._linear_acc.x), fabsf(imu._linear_acc.y));

        switch (state) {
            case LAUNCH_STATE_IDLE: {
                throttle_pwm_out = 988.0f;
                current_throttle_pwm = 988.0f;
                if (is_on_ground()) {
                    state = LAUNCH_STATE_WAIT_GROUND;
                } else {
                    state = LAUNCH_STATE_COMPLETE;
                }
                break;
            }

            // ------------------------------------------------------------
            // 1. DI DARAT: MOTOR MATI TOTAL (988), TUNGGU LEMPARAN / SHAKE
            // ------------------------------------------------------------
            case LAUNCH_STATE_WAIT_GROUND: {
                throttle_pwm_out = 988.0f; // MOTOR DIJAMIN MATI (988)
                current_throttle_pwm = 988.0f;
                roll_dem_out = 0.0f;       // Sayap datar
                pitch_dem_out = 5.0f;      // Standby pitch

                // Deteksi A: Forward Throw (Lemparan kuat ke depan atau didorong cepat)
                bool throw_detected = (horiz_accel >= throw_accel_thresh) || 
                                     (ahrs.haveGPS() && ahrs.groundspeed >= min_groundspeed_launch);

                // Deteksi B: Shake to Launch (Goyangan maju-mundur/kiri-kanan)
                if (max_lin_acc >= shake_accel_thresh) {
                    shake_pos_detected = true;
                    last_shake_pos_ms = now;
                }
                bool shake_detected = false;
                if (shake_pos_detected) {
                    if (now - last_shake_pos_ms <= 800) {
                        // Goyangan balasan terdeteksi
                        if (max_lin_acc >= (shake_accel_thresh * 0.8f) && (now - last_shake_pos_ms >= 50)) {
                            shake_detected = true;
                        }
                    } else {
                        shake_pos_detected = false;
                    }
                }

                if (throw_detected || shake_detected) {
                    state = LAUNCH_STATE_RELEASE_DELAY;
                    trigger_time_ms = now;
                    Serial.printf("[LAUNCH] Trigger detected! (%s) -> Hand release delay (%.0f ms)...\n",
                                  throw_detected ? "THROW" : "SHAKE", release_delay_ms);
                }
                break;
            }

            // ------------------------------------------------------------
            // 2. JEDA PELEPASAN TANGAN: MOTOR TETAP MATI (400ms)
            // ------------------------------------------------------------
            case LAUNCH_STATE_RELEASE_DELAY: {
                throttle_pwm_out = 988.0f; // MOTOR TETAP MATI
                roll_dem_out = 0.0f;
                pitch_dem_out = takeoff_pitch_deg;

                if (now - trigger_time_ms >= (uint32_t)release_delay_ms) {
                    state = LAUNCH_STATE_THROTTLE_RAMP;
                    ramp_start_ms = now;
                    Serial.println("[LAUNCH] Hand clear -> Ramping throttle up...");
                }
                break;
            }

            // ------------------------------------------------------------
            // 3. THROTTLE RAMP UP: NAIK SECARA HALUS DARI 0 KE TARGET
            // ------------------------------------------------------------
            case LAUNCH_STATE_THROTTLE_RAMP: {
                float progress = (float)(now - ramp_start_ms) / (ramp_time_sec * 1000.0f);
                progress = constrain(progress, 0.0f, 1.0f);

                int target_thr_pwm = fw_control.get_auto_throttle();
                if (target_thr_pwm < 1550) target_thr_pwm = 1650; // Minimum climb throttle

                current_throttle_pwm = 988.0f + progress * (target_thr_pwm - 988.0f);
                throttle_pwm_out = current_throttle_pwm;

                // Batasi roll agar tidak miring saat awal takeoff
                roll_dem_out = constrain(nav_roll_deg, -10.0f, 10.0f);
                pitch_dem_out = takeoff_pitch_deg;

                if (progress >= 1.0f) {
                    state = LAUNCH_STATE_CLIMB;
                    Serial.println("[LAUNCH] Ramp complete -> Climbing to safe altitude...");
                }
                break;
            }

            // ------------------------------------------------------------
            // 4. CLIMB OUT: MENDAKI STABIL KE KETINGGIAN AMAN
            // ------------------------------------------------------------
            case LAUNCH_STATE_CLIMB: {
                throttle_pwm_out = fw_control.get_auto_throttle();
                if (throttle_pwm_out < 1550) throttle_pwm_out = 1650;

                roll_dem_out = constrain(nav_roll_deg, -12.0f, 12.0f);
                pitch_dem_out = takeoff_pitch_deg;

                if (relative_alt >= safe_climb_alt) {
                    state = LAUNCH_STATE_COMPLETE;
                    Serial.println("[LAUNCH] Safe altitude reached -> FULL WAYPOINT NAVIGATION ACTIVE!");
                }
                break;
            }

            // ------------------------------------------------------------
            // 5. SELESAI: KONTROL PENUH OLEH L1 & TECS
            // ------------------------------------------------------------
            case LAUNCH_STATE_COMPLETE: {
                throttle_pwm_out = fw_control.get_auto_throttle();
                pitch_dem_out = nav_pitch_deg;
                roll_dem_out = nav_roll_deg;
                break;
            }
        }
    }
};

inline LaunchGuard launch_guard;