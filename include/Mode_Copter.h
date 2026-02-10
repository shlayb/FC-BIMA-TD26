#pragma once
#include "Actuator.h"
#include "Auto_setup.h"
#include "mode_setup.h"
// #include "Radio.h"
#include "TECS.h"
#include "bno055.h"
#include "Copter_control.h"

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

ModeCopter_STB    mCOPT;
ModeCopter_HOV    mQHOV;