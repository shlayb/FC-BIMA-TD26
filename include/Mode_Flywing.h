#pragma once

#include "Actuator.h"
#include "Auto_setup.h"
#include "Attitude.h"
#include "Mode_setup.h"
// #include "Radio.h"
#include "FW_ControlModes.h"
#include "Fuzzy_FW_Roll.h"
#include "flywing_control.h"
#include "TECS.h"
#include "Navigation.h"


// ======================= MODE MANU  =========================
class ModeManual : public ModeBase {
public:
  ModeId id() const override { return ModeId::MANU; }

protected:
  bool _enter() override {
    // set flags manual
    mode_fbwa_plane      = false;
    auto_navigation_mode = false;
    return true;
  }

  void _update() override {
    ahrs.update_ahrs();
    // nav_gps();
    servos_out_manual_flywing();
    plane_motors_out();
    mode_phase = 1;
    current = ModeId::MANU;
  }

  void _exit() override {
    // tidak perlu apa2 untuk manual
  }
};

// ======================= MODE FBWA =========================
class ModeFBWA : public ModeBase {
public:
  ModeId id() const override { return ModeId::FBWA; }

protected:
  bool _enter() override {
    // imu.pitch_mode = 2;
    mode_fbwa_plane = true;
    auto_navigation_mode = false;
    return true;
  }
  void _update() override {
        flywingcontrol.flywing_control_stabilize(ch_roll, ch_pitch, ch_yaw, ch_throttle, imu.roll, imu.pitch, imu.yaw, imu.gyro_y, imu.gyro_x, imu.gyro_z);
        flywing_calcout();
        flywing_servos_out(servo_L, servo_R);
        // motorCopterOff(988, 988, 988, 988);
        motor_loop(988, 988, 988, 988);
        // copter_ControlFSFB(ch_roll, ch_pitch, ch_yaw, ch_throttle, imu.roll, imu.pitch, imu.yaw, imu.gyro_y, imu.gyro_x, imu.gyro_z);
        // copter_calcOutput(ch_throttle);
        // updateFBWA_FW();
        // servos_out_fbwa_flywing();
        // servos_out_fbwa();
        plane_motors_out();  // TODO:crosschecking (sblmnya plane_motors_out_auto(1900))
        mode_phase = 2;
        current = ModeId::FBWA;
  }
  void _exit() override {
  }
};

// ======================= MODE AUTO =========================
class ModeAUTO : public ModeBase {
public:
  ModeId id() const override { return ModeId::AUTO; }

  protected:
  bool _enter() override {
    // ahrs.update_ahrs();
    // nav_gps();
    // mode_fbwa_plane = true;
    // auto_throttle_mode = true;
    // auto_navigation_mode = true;
    // auto_loiter_mode = false;
    // prev_WP_loc = current_loc = next_WP_loc;
    // flag_wp = -1;
    // target_altitude.amsl_cm = 99.0f * M_TO_CM;  // cm
    // target_airspeed = AIRSPEED_CRUISE;
    // // Serial.println("enter auto done!");
    // add_waypoint(gepees.latitude * 1000000, gepees.longitude * 10000000, 90 * M_TO_CM, 0);
    // add_waypoint(latitude, longitude, 90);
    return true;
  }
  void _update() override {
          if (!enterauto) {
            auto_throttle_mode = true;
            auto_navigation_mode = true;
            auto_loiter_mode = false;
            prev_WP_loc = current_loc = next_WP_loc;
            flag_wp = -1;
            target_altitude.amsl_cm = 99.0f * M_TO_CM;  // cm
            target_airspeed = AIRSPEED_CRUISE;
            // Serial.println("enter auto done!");
            add_waypoint(gepees.latitude * 10000000, gepees.longitude * 10000000, 90 * M_TO_CM, 0);
            // add_waypoint(latitude, longitude, 90);
            enterauto = true;  // Set flag menjadi true agar fungsi tidak dipanggil lagi
          }
    // test_fuzzy_roll(); 
    motor_loop(988, 988, 988, 988);
    mode_fbwa_plane = true;
    navigate();
    update_tecs();
    flywingcontrol.flywing_control_auto(ch_roll, ch_pitch, ch_yaw, ch_throttle, imu.roll, imu.pitch, imu.yaw, imu.gyro_y, imu.gyro_x, imu.gyro_z);
    flywing_calcout();
    flywing_servos_out(servo_L, servo_R);
    // updateAuto_FW();
    // stabilize();
    // servos_out_fbwa();
    // servos_out_fbwa_flywing();
    // updateFBWA_FW();
    plane_motors_out();
    // plane_motors_out_auto(988);
    mode_phase = 3;
    current = ModeId::AUTO;
  }
  void _exit() override {
  }
};

ModeManual        mMANU;
ModeFBWA          mFBWA;
ModeAUTO          mAUTO;

