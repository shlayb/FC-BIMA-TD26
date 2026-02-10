#pragma once
#include <Actuator.h>
#include <Auto_setup.h>
#include <bno055.h>

// #define TAKEOFF_WITH_LAUCHER

bool entertakeoff = false;
bool turn_on_throttle = false;
float positive_counter = 0;
float negative_counter = 0;
static uint32_t last_negative_time = 0;

const uint32_t TAKEOFF_DURATION = 3000;
const int32_t MAX_THROTTLE_PERCENT = 99;

bool no_launch() {
    auto_takeoff_state.launchTimerStarted = false;
    auto_takeoff_state.last_tkoff_arm_time = 0;
    return false;
}

// void check_shake() {
//     if (!auto_takeoff_check()) {
//         Serial2.println("takeoff check false");
//     }
//     if (!no_launch()) {
//         Serial2.println("no launch called");
//     }
//     Serial2.println("nothing false shake or no launch");
// }

void takeoff_calc_roll(void) {
    calc_nav_roll();
    // during takeoff use the level flight roll limit to prevent large
    // wing strike. Slowly allow for more roll as we get higher above
    // the takeoff altitude
    int32_t takeoff_roll_limit = 20;   // roll limit untuk takeoff
    float level_roll_limit = 10;       // meter
    int32_t level_alt = 27;            // meter
    int32_t target_alt = 30;           // meter
    int16_t takeoff_rotate_speed = 0;  // For hand launch and catapult launches a TKOFF_ROTATE_SPD of zero should be set.

    if (baro.getEAS2TAS() < takeoff_rotate_speed) {
        // before Vrotate (aka, on the ground)
        takeoff_roll_limit = level_roll_limit;
    } else {
        // lim1 - below altitude TKOFF_LVL_ALT, restrict roll to LEVEL_ROLL_LIMIT
        // lim2 - above altitude (TKOFF_LVL_ALT * 3) allow full flight envelope of ROLL_LIMIT_DEG
        // In between lim1 and lim2 use a scaled roll limit.
        // The *3 scheme should scale reasonably with both small and large aircraft
        const float lim1 = MAX(level_alt, 0);
        const float lim2 = MIN(level_alt * 3, target_alt);
        const float current_baro_alt = baro.altitude;

        takeoff_roll_limit = linear_interpolate(level_roll_limit, 16,  // nilai roll limit
                                                current_baro_alt,
                                                auto_state.baro_takeoff_alt + lim1, auto_state.baro_takeoff_alt + lim2);
    }

    nav_roll_deg = constrain(nav_roll_deg, -takeoff_roll_limit, takeoff_roll_limit);
}

// this function limit pitch during takeoff.
void takeoff_calc_pitch(void) {
    float dHeight;
    float dDistance;

    dHeight = (current_loc.alt - auto_takeoff_state.target_altitude) / 100.0f;
    dDistance = current_loc.get_distance(next_WP_loc);
    nav_pitch_deg = atan2f(dHeight, dDistance) * RAD_TO_DEG;
}

void takeoff_headlock() {
    float head_error = imu.heading;

    if (head_error < 180.0 && auto_takeoff_state.head_setpoint >= 180.0) {
        head_error += 360.0;
    } else if (head_error > 180.0 && auto_takeoff_state.head_setpoint <= 180.0) {
        head_error -= 360.0;
    }
    value2 = head_error;
    nav_roll_deg = auto_takeoff_state.head_setpoint - head_error;
    nav_roll_deg = constrain_float(nav_roll_deg, -15.0f, 15.0f);
}

void takeoff_pitchlock(float deg) {
    nav_pitch_deg = deg;
    // servo_pwm_ele = 25.0f;
}

void trigger_autotakeoff() {
    float takeoff_throttle_min_speed = 0.2f;   // groundspeed minimum untuk takeoff
    float takeoff_throttle_min_accel = 20.0f;  // minimal kecepatan akselerasi untuk menyalakan motor
    float xaccel = imu._linear_acc.y;
    uint32_t current_takeoff_time = millis();

    if (!arming) return;
    if (gepees.gps_status < GPS_FIX_2D) return;

#ifdef TAKEOFF_WITH_LAUCHER
    if (xaccel >= takeoff_throttle_min_accel && groundspeed >= takeoff_throttle_min_accel) {
        auto_takeoff_state.ready_for_throttle = true;
        auto_takeoff_state.start_time_ms = current_takeoff_time;
    }

    if (auto_takeoff_state.ready_for_throttle && reached_approach_alt(altitude)) {
        if ((current_takeoff_time - auto_takeoff_state.start_time_ms) >= 100) {
            entertakeoff = true;
            turn_on_throttle = true;
        }
    }
#else
    if (xaccel >= 6 && (current_takeoff_time - auto_takeoff_state.accel_event_ms) >= 200 && positive_counter < 2) {
        positive_counter++;
        auto_takeoff_state.accel_event_ms = current_takeoff_time;
        auto_takeoff_state.start_time_ms = current_takeoff_time;
    }

    if (xaccel <= -6 && (current_takeoff_time - last_negative_time) >= 200 && negative_counter < 2) {
        negative_counter++;
        last_negative_time = current_takeoff_time;
        auto_takeoff_state.start_time_ms = current_takeoff_time;
    }

    if (positive_counter >= 2 && negative_counter >= 2) {
        if ((current_takeoff_time - auto_takeoff_state.start_time_ms) >= 500) {
            entertakeoff = true;
            turn_on_throttle = true;
        }
    }
#endif
}

void takeoff_throtlle() {
    float pwm;
    // if (!auto_takeoff_state.doing) {
    //     auto_takeoff_state.doing = true;
    //     auto_takeoff_state.now = millis();
    // }

    // auto_takeoff_state.dtime = millis() - auto_takeoff_state.now;

    if (auto_takeoff_state.throttle < 20) {
        auto_takeoff_state.throttle += 5;  //= (int32_t)floor(auto_takeoff_state.dtime / TAKEOFF_DURATION * MAX_THROTTLE_PERCENT);
        pwm = PercenttoPWM(auto_takeoff_state.throttle);
        plane_motors_out_auto(pwm);
        delay(450);

        // if (auto_takeoff_state.dtime <= TAKEOFF_DURATION) {
        //     auto_takeoff_state.throttle = (int32_t)floor(auto_takeoff_state.dtime / TAKEOFF_DURATION * MAX_THROTTLE_PERCENT);
        //     pwm = PercenttoPWM(auto_takeoff_state.throttle);
        //     plane_motors_out_auto(pwm);
    } else {
        pwm = PercenttoPWM(MAX_THROTTLE_PERCENT);
        plane_motors_out_auto(pwm);
        auto_takeoff_state.complete = true;
    }
}

bool auto_takeoff_check(void) {
    // TAKEOFF PARAMETER
    float takeoff_throttle_delay = 2.0f;       // waktu delay setelah diarm
    float takeoff_throttle_min_speed = 0.1f;   // groundspeed minimum untuk takeoff
    float takeoff_throttle_min_accel = 15.0f;  // minimal kecepatan akselerasi untuk menyalakan motor
    int16_t takeoff_throttle_accel_count = 1;  // banyak akselerasi sebelum motor hidup

    // this is a more advanced check that relies on TECS
    uint32_t now = millis();
    uint16_t wait_time_ms = MIN(uint16_t(takeoff_throttle_delay) * 100, 12700);

    // reset all takeoff state if disarmed
    if (!arming) {
        // memset(&auto_takeoff_state, 0, sizeof(auto_takeoff_state));
        auto_state.baro_takeoff_alt = baro.altitude;
        return false;
    }

    // Reset states if process has been interrupted
    if (auto_takeoff_state.last_check_ms && (now - auto_takeoff_state.last_check_ms) > 200) {
        // memset(&auto_takeoff_state, 0, sizeof(auto_takeoff_state));
        return false;
    }

    auto_takeoff_state.last_check_ms = now;

    // Check for bad GPS
    if (gepees.gps_status < GPS_FIX_2D) {
        // no auto takeoff without GPS lock
        // return false;
    }

    // buat shake to takoff ,cariin yg salah ya :)
    if (!auto_takeoff_state.launchTimerStarted && !is_zero(takeoff_throttle_min_accel)) {
        // we are requiring an X acceleration event to launch
        float xaccel = imu._linear_acc.y;
        if (takeoff_throttle_accel_count <= 1) {
            if (xaccel < takeoff_throttle_min_accel) {
                return no_launch();
            }
        } else {
            // we need multiple accel events
            if (now - auto_takeoff_state.accel_event_ms > 500) {
                auto_takeoff_state.accel_event_counter = 0;
            }
            bool odd_event = ((auto_takeoff_state.accel_event_counter & 1) != 0);
            bool got_event = (odd_event ? xaccel < -takeoff_throttle_min_accel : xaccel > takeoff_throttle_min_accel);
            if (got_event) {
                auto_takeoff_state.accel_event_counter++;
                auto_takeoff_state.accel_event_ms = now;
            }
            if (auto_takeoff_state.accel_event_counter < takeoff_throttle_accel_count) {
                return no_launch();
            }
        }
    }

    // we've reached the acceleration threshold, so start the timer
    if (!auto_takeoff_state.launchTimerStarted) {
        auto_takeoff_state.launchTimerStarted = true;
        auto_takeoff_state.last_tkoff_arm_time = now;
        if (now - auto_takeoff_state.last_report_ms > 2000) {
            auto_takeoff_state.last_report_ms = now;
        }
    }

    // Only perform velocity check if not timed out
    if ((now - auto_takeoff_state.last_tkoff_arm_time) > wait_time_ms + 100U) {
        if (now - auto_takeoff_state.last_report_ms > 2000) {
            auto_takeoff_state.last_report_ms = now;
        }
        return no_launch();
    }

    // Check ground speed and time delay
    if (((ahrs.groundspeed > takeoff_throttle_min_speed || is_zero(takeoff_throttle_min_speed))) &&
        ((now - auto_takeoff_state.last_tkoff_arm_time) >= wait_time_ms)) {
        auto_takeoff_state.launchTimerStarted = false;
        auto_takeoff_state.last_tkoff_arm_time = 0;
        auto_takeoff_state.start_time_ms = now;
        entertakeoff = true;
        turn_on_throttle = true;
        return true;
    }

    // we're not launching yet, but the timer is still going
    return false;
}