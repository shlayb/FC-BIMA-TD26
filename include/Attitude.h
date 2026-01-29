#pragma once
#include <Arduino.h>

#include "AHRS.h"
// #include "Barometer.h"
// #include "Control_data.h"
#include "FW_ControlModes.h"
// #include "FW_config.h"
#include "FW_pitch_controller.h"
#include "FW_roll_controller.h"
#include "FW_yaw_controller.h"
// #include "Radio.h"
// #include "Control_data.h"
#include <AP_Math.h>
#include "WP_Loiter.h"

// #include "Automode.h"
// #include "Servos_FW.h"
#include "Actuator.h"



uint32_t last_stabilize_ms;
float relative_altitude = 0.0f;

float calc_nav_yaw_coordinated() {
    float rudder_in = outputScaler(ch_yaw) * MAX_YAW_ANGLE;
    float commanded_yaw;
    commanded_yaw = -u_yaw;
    // commanded_yaw += scaleToAngle(ch_yaw) * RUDDER_MIX;
    commanded_yaw += u_roll * (RUDDER_MIX);
    if (mode_fbwa_plane) {
        commanded_yaw += rudder_in;
    }
    return commanded_yaw;
}

void stabilize_pitch(float speed_scaler) {
    plane_control_t.pitch_setpoint = -nav_pitch_deg;
    plane_control_t.pitch_data = -imu.pitch;
    plane_control_t.body_y_rate = imu.gyro_y;
    plane_control_t.speed_scaler = speed_scaler;
    plane_control_t.airspeed_min = MIN_AIRSPEED;
    plane_control_t.airspeed_max = MAX_AIRSPEED;
    u_pitch = pitch_control_attitude(plane_control_t);
}

void stabilize_roll(float speed_scaler) {
    plane_control_t.roll_setpoint = nav_roll_deg;
    plane_control_t.roll_data = imu.roll; //dicek lagi kadang suka terbalik roll nya
    plane_control_t.body_x_rate = imu.gyro_x; 
    plane_control_t.speed_scaler = speed_scaler;
    plane_control_t.airspeed_min = MIN_AIRSPEED;
    plane_control_t.airspeed_max = MAX_AIRSPEED;
    u_roll = roll_control_attitude(plane_control_t);
}

void stabilize_yaw(float speed_scaler) {
    float commanded_yaw = 0;
    plane_control_t.yaw_setpoint = commanded_yaw;
    plane_control_t.yaw_data = 0;
    plane_control_t.body_z_rate = imu.gyro_z;
    plane_control_t.speed_scaler = speed_scaler;
    plane_control_t.airspeed_min = MIN_AIRSPEED;
    plane_control_t.airspeed_max = MAX_AIRSPEED;
    u_yaw = yaw_control_attitude(plane_control_t);

    u_yaw_cmd = calc_nav_yaw_coordinated();
}

float get_speed_scaler() {
    float speed_scaler, aspeed;
    // TODO: next terbang is_airspeed_sensor_enabled di true, sama gedein SCALIng_SPEED
    if (ahrs.is_airspeed_sensor_enabled == true) {
        aspeed = 0;  // getAirspeedEstimate
        if (aspeed > 0.0001f) {
            speed_scaler = SCALING_SPEED / aspeed;
        } else {
            speed_scaler = 2.0;
        }
        float scale_min = MIN(0.5f, (0.5f * MIN_AIRSPEED) / SCALING_SPEED);
        float scale_max = MAX(2.0f, (1.5f * MAX_AIRSPEED) / SCALING_SPEED);
        speed_scaler = constrain(speed_scaler, scale_min, scale_max);
    } else if (arming) {
        float throttle_out = MAX(scaleToPercent(ch_throttle), 1);
        speed_scaler = sqrtf(THROTTLE_CRUISE / throttle_out);
        speed_scaler = constrain(speed_scaler, 1.2f, 1.67f);
    } else {
        speed_scaler = 1.0f;
    }
    return speed_scaler;
}

void stabilize() {
    if (mode_manual == true) {
        return;
    }

    float speed_scaler = get_speed_scaler();
    uint32_t now = millis();
    if (now - last_stabilize_ms > 2000) {
        reset_integrator_roll();
        reset_integrator_pitch();
        reset_integrator_yaw();
    }
    last_stabilize_ms = now;

    if (mode_fbwa_plane) {
        stabilize_roll(speed_scaler);
        stabilize_pitch(speed_scaler);
        stabilize_yaw(speed_scaler);
    }

    if (scaleToPercent(ch_throttle) == 0 &&
        fabsf(relative_altitude) < 5.0f &&
        fabsf(baro.get_climb_rate()) < 5.0f &&
        ahrs.groundspeed < 3) {
        reset_integrator_roll();
        reset_integrator_pitch();
        reset_integrator_yaw();
    }
}

bool loiter_enter() {
    auto_throttle_mode = true;
    auto_loiter_mode = true;
    next_WP_loc = prev_WP_loc = current_loc;
    flag_wp = -1;
    target_altitude.amsl_cm = 10000.0f; //100 meter
    target_airspeed = AIRSPEED_CRUISE;
    init_loiter();

    return true;
}

