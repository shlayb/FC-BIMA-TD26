#pragma once
#include "Actuator.h"
#include "Auto_setup.h"
#include "Mode_setup.h"
// #include "Radio.h"
#include "TECS.h"
#include "bno055.h"
#include "Copter_Control.h"

class ModeCopter_STB : public ModeBase {
public:
  ModeId id() const override { return ModeId::COPT; }   // pastikan enum ModeId punya COPT

protected:
  bool _enter() override {
    mode_fbwa_plane      = false;    // contoh: disable fixed-wing
    auto_navigation_mode = false;    // auto nav off
    return true;
  }

  void _update() override {
        imu.pitch_mode = 1;
        alt_hold = true;  // disable alt hold
        mode_fbwa_plane = false;
        auto_navigation_mode = false;
        copter_ControlFSFB(ch_roll, ch_pitch, ch_yaw, ch_throttle, imu.roll, imu.pitch, imu.yaw, imu.gyro_y, imu.gyro_x, imu.gyro_z);
        copter_calcOutput(ch_throttle);
        motor_loop(m1_pwm,m2_pwm,m3_pwm,m4_pwm);
        // servos_out_manual();
        // servos_out_manual_flywing();
        // plane_motors_out();  // pusher.writeMicroseconds(988);
        mode_phase = 1;        
        current = ModeId::COPT;
        // auto_state.distance_next_wp = current_loc.get_distance(next_WP_loc);
  }

  void _exit() override {
  }
};

class ModeCopter_HOV : public ModeBase {
public:
  ModeId id() const override { return ModeId::QHOV; }   // pastikan enum ModeId punya COPT

protected:
  bool _enter() override {
    mode_fbwa_plane      = false;    // contoh: disable fixed-wing
    auto_navigation_mode = false;    // auto nav off
    return true;
  }

  void _update() override {
    alt_hold = true;  // enable alt hold
    pos_hold = true;
    copter_ControlFSFB(ch_roll, ch_pitch, ch_yaw, ch_throttle, imu.roll, imu.pitch, imu.yaw, imu.gyro_y, imu.gyro_x, imu.gyro_z);
    copter_calcOutput(ch_throttle);
    motor_loop(m1_pwm, m2_pwm, m3_pwm, m4_pwm);
    mode_phase = 2;
    current = ModeId::QHOV;
  }

  void _exit() override {
    // Serial.println("[COPTER] exit");
  }
};

class ModeCopter_GUIDED : public ModeBase {
public:
  ModeId id() const override { return ModeId::GUIDED; }

protected:
  bool _enter() override {
    mode_fbwa_plane = false;
    auto_navigation_mode = false;
    alt_hold = true;
    pos_hold = true;
    return true;
  }

  void _update() override {
    mode_fbwa_plane = false;
    auto_navigation_mode = false;
    alt_hold = true;
    pos_hold = true;

    // Stick virtual netral agar loop PID copter pakai target posisi dari MAVLink.
    const int16_t guided_ch_roll = 1500;
    const int16_t guided_ch_pitch = 1500;
    const int16_t guided_ch_yaw = 1500;
    const int16_t guided_ch_thr = 1700;

    const bool has_fresh_target = rpi_external_setpoint_active &&
                                  (millis() - rpi_external_setpoint_last_ms <= 1000);

    if (has_fresh_target) {
      lat_pos = (float)rpi_external_setpoint_target.lat * 1.0e-7f;
      lon_pos = (float)rpi_external_setpoint_target.lng * 1.0e-7f;
      alt_ref = (float)rpi_external_setpoint_target.alt / 100.0f;
      alt_hold_on = true;
    } else {
      lat_pos = gepees.latitude;
      lon_pos = gepees.longitude;
      alt_ref = baro.altitude;
      alt_hold_on = true;
    }

    copter_ControlFSFB(guided_ch_roll, guided_ch_pitch, guided_ch_yaw, guided_ch_thr,
                       imu.roll, imu.pitch, imu.yaw, imu.gyro_y, imu.gyro_x, imu.gyro_z);
    copter_calcOutput(guided_ch_thr);
    motor_loop(m1_pwm, m2_pwm, m3_pwm, m4_pwm);
    mode_phase = 3;
    current = ModeId::GUIDED;
  }

  void _exit() override {
    // keep neutral exit
  }
};

ModeCopter_STB    mCOPT;
ModeCopter_HOV    mQHOV;
ModeCopter_GUIDED mGUIDED;