#pragma once
#include "Actuator.h"
#include "Auto_setup.h"
#include "Mode_setup.h"
#include "TECS.h"
#include "bno055.h"
#include "Copter_Control.h"

class ModeCopter_STB : public ModeBase {
public:
  ModeId id() const override { return ModeId::COPT; }

protected:
  bool _enter() override {
    mode_fbwa_plane      = false;
    auto_navigation_mode = false;
    alt_hold             = false;  // STB = pure manual throttle, no alt hold
    pos_hold             = false;
    min_roll             = min_roll_default;
    max_roll             = max_roll_default;
    mode_phase           = 1;
    current              = ModeId::COPT;
    return true;
  }

  void _update() override {
    imu.pitch_mode = 1;
    alt_hold = false;
    pos_hold = false;
    mode_fbwa_plane = false;
    auto_navigation_mode = false;

    copter_ControlFSFB(ch_roll, ch_pitch, ch_yaw, ch_throttle,
                       imu.roll, imu.pitch, imu.yaw, imu.gyro_y, imu.gyro_x, imu.gyro_z);
    copter_calcOutput(ch_throttle);

    if (arming) {
        motor_loop(m1_pwm, m2_pwm, m3_pwm, m4_pwm);
    } else {
        motor_loop(988, 988, 988, 988);
    }

    // Pusher motor fixed-wing dipastikan mati saat mode copter
    pusher.writeMicroseconds(988);
  }

  void _exit() override {
    motor_loop(988, 988, 988, 988);
  }
};

class ModeCopter_HOV : public ModeBase {
public:
  ModeId id() const override { return ModeId::QHOV; }

protected:
  bool _enter() override {
    mode_fbwa_plane      = false;
    auto_navigation_mode = false;
    alt_hold             = true;   // QHOV = altitude hold aktif
    pos_hold             = false;
    alt_hold_on          = false;  // trigger re-latch alt_ref saat pertama hover
    min_roll             = min_roll_default;
    max_roll             = max_roll_default;
    mode_phase           = 2;
    current              = ModeId::QHOV;
    return true;
  }

  void _update() override {
    imu.pitch_mode = 1;
    alt_hold = true;
    pos_hold = false;
    mode_fbwa_plane = false;
    auto_navigation_mode = false;

    copter_ControlFSFB(ch_roll, ch_pitch, ch_yaw, ch_throttle,
                       imu.roll, imu.pitch, imu.yaw, imu.gyro_y, imu.gyro_x, imu.gyro_z);
    copter_calcOutput(ch_throttle);

    if (arming) {
        motor_loop(m1_pwm, m2_pwm, m3_pwm, m4_pwm);
    } else {
        motor_loop(988, 988, 988, 988);
    }

    pusher.writeMicroseconds(988);
  }

  void _exit() override {
    alt_hold_on = false;
    motor_loop(988, 988, 988, 988);
  }
};

class ModeCopter_GUIDED : public ModeBase {
public:
  ModeId id() const override { return ModeId::GUIDED; }

protected:
  bool _enter() override {
    mode_fbwa_plane      = false;
    auto_navigation_mode = false;
    alt_hold             = true;
    pos_hold             = true;
    alt_hold_on          = false;
    mode_phase           = 3;
    current              = ModeId::GUIDED;
    return true;
  }

  void _update() override {
    imu.pitch_mode = 1;
    mode_fbwa_plane = false;
    auto_navigation_mode = false;
    alt_hold = true;
    pos_hold = true;

    // Stick virtual netral agar loop PID copter pakai target posisi
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
      if (gepees.getStatus() >= 2) {
          lat_pos = gepees.latitude;
          lon_pos = gepees.longitude;
      }
      alt_ref = baro.altitude;
      alt_hold_on = true;
    }

    copter_ControlFSFB(guided_ch_roll, guided_ch_pitch, guided_ch_yaw, guided_ch_thr,
                       imu.roll, imu.pitch, imu.yaw, imu.gyro_y, imu.gyro_x, imu.gyro_z);
    copter_calcOutput(guided_ch_thr);

    if (arming) {
        motor_loop(m1_pwm, m2_pwm, m3_pwm, m4_pwm);
    } else {
        motor_loop(988, 988, 988, 988);
    }

    pusher.writeMicroseconds(988);
  }

  void _exit() override {
    alt_hold_on = false;
    motor_loop(988, 988, 988, 988);
  }
};

ModeCopter_STB    mCOPT;
ModeCopter_HOV    mQHOV;
ModeCopter_GUIDED mGUIDED;