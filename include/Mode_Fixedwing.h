#pragma once

#include "AHRS.h"
#include "Arduino.h"
#include "Attitude.h"
#include "Actuator.h"
#include "ControlModes.h"
#include "FW_config.h"
#include "Navigation.h"
#include "Radio.h"
#include "TECS.h"
#include "gps.h"
#include "takeoff.h"
#include "bno055.h"
#include "buzzer.h"
#include "Mode_setup.h"

void exit_auto(){
    // Serial2.println("EXIT AUTO MODE");
    enterauto = false;
    auto_throttle_mode = false;
    auto_navigation_mode = false;
    // Serial2.print("flag_wp tersimpan: "); Serial2.println(flag_wp);
}

void init_auto(){
    // Serial2.println("INIT AUTO MODE");
    auto_throttle_mode = true;
    auto_navigation_mode = true;
    auto_loiter_mode = false;
    
    // Cek apakah ini first entry atau resume
    if (flag_wp == -1) {
        // First entry: mulai dari awal
        // Serial2.println("First entry - Start from WP 0 (HOME)");
        prev_WP_loc = current_loc = next_WP_loc;
        flag_wp = -1;  // akan jadi 0 di navigate()
        
        // Set HOME sebagai WP 0 (overwrite koordinat, wp_sum sudah ter-increment di main)
        waypoint[0].lat = gepees.latitude * 10000000;
        waypoint[0].lng = gepees.longitude * 10000000;
        waypoint[0].alt = 69 * M_TO_CM;
        
        // Serial2.print("HOME set - lat:"); Serial2.print(waypoint[0].lat);
        // Serial2.print(" lng:"); Serial2.print(waypoint[0].lng);
        // Serial2.print(" alt:"); Serial2.println(waypoint[0].alt);
    } 
    else {
        // Resume: lanjut dari WP terakhir
        // Serial2.print("Resume from WP "); Serial2.println(flag_wp);
        
        // Pastikan flag_wp masih valid
        if (flag_wp >= wp_sum) {
            flag_wp = -1;  // reset jika sudah selesai semua
            // Serial2.println("All WP completed - restart from WP 0 (HOME)");
            prev_WP_loc = current_loc = next_WP_loc;
            
            // Update HOME lagi dengan posisi terkini
            waypoint[0].lat = gepees.latitude * 10000000;
            waypoint[0].lng = gepees.longitude * 10000000;
            waypoint[0].alt = 69 * M_TO_CM;
        } else {
            // Set target ke WP saat ini
            auto_state.next_wp_crosstrack = false;
            target_altitude.amsl_cm = waypoint[flag_wp].alt;
            set_next_WP(waypoint[flag_wp]);
            
            // Serial2.print("Target WP - lat:"); Serial2.print(waypoint[flag_wp].lat);
            // Serial2.print(" lng:"); Serial2.print(waypoint[flag_wp].lng);
            // Serial2.print(" alt:"); Serial2.println(waypoint[flag_wp].alt);
        }
    }
    
    // Only set target altitude if we have a valid waypoint to go to
    // Let navigate() handle the actual target altitude update on first iteration
    if (flag_wp >= 0) {
        target_altitude.amsl_cm = waypoint[flag_wp].alt;
    }
    target_airspeed = AIRSPEED_CRUISE;
    enterauto = true;
    
    // Serial2.print("Total WP: "); Serial2.print(wp_sum);
    // Serial2.print(" | Current flag_wp: "); Serial2.println(flag_wp);
}

// ======================= MODE MANUAL (Fixed Wing) =========================
class ModeManual_FW : public ModeBase {
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
    nav_gps();
    
    mode_fbwa_plane = false;
    auto_navigation_mode = false;
    servos_out_manual();
    plane_motors_out();  // pusher.writeMicroseconds(988);
    // dual_motor_throttle_out_yaw();  // Kontrol throttle langsung ke motor (tanpa yaw diff)
    payload_control_unified();  // Unified payload control (manual + auto)
    mode_phase = 1;
    current = ModeId::MANU;
    // Serial.print("ch_roll:");
    // Serial.print(ch_roll);
    // Serial.print(" |ch_pitch:");
    // Serial.print(ch_pitch);
    // Serial.print(" |ch_thr:");
    // Serial.print(ch_throttle);
    // Serial.print(" |ch_yaw:");
    // Serial.print(ch_yaw);
  }

  void _exit() override {
    // cleanup if needed
  }
};

// ======================= MODE FBWA (Fixed Wing) =========================
class ModeFBWA_FW : public ModeBase {
public:
  ModeId id() const override { return ModeId::FBWA; }

protected:
  bool _enter() override {
    mode_fbwa_plane = true;
    auto_navigation_mode = false;
    return true;
  }

  void _update() override {
    ahrs.update_ahrs();
    nav_gps();
    
    mode_fbwa_plane = true;
    auto_navigation_mode = false;
    fw_control.updateFBWA_FW(ch_roll, ch_pitch, ch_yaw, ch_throttle,
                          imu.roll, imu.pitch, imu.yaw,
                          imu.gyro_y, imu.gyro_x, imu.gyro_z);
    fw_servos_out_fbwa();
    plane_motors_out();
    // dual_motor_throttle_out_yaw();
    payload_control_unified();  // Unified payload control (manual + auto)
    mode_phase = 2;
    current = ModeId::FBWA;
  }

  void _exit() override {
    // cleanup if needed
  }
};

// ======================= MODE AUTO (Fixed Wing) =========================
class ModeAuto_FW : public ModeBase {
public:
  ModeId id() const override { return ModeId::AUTO; }

protected:
  bool _enter() override {
    if (!enterauto) {
        init_auto();
    }
    return true;
  }

  void _update() override {
    ahrs.update_ahrs();
    nav_gps();
    
    mode_fbwa_plane = true;
    navigate();
    update_speed_height();
    update_alt();
    updateAuto_FW();
    
    // Cascaded AUTO controller derived from FBWA inner loop + L1 + TECS
    fw_control.updateAUTO_FW(imu.roll, imu.pitch, imu.yaw,
                             imu.gyro_y, imu.gyro_x, imu.gyro_z);
    fw_servos_out_fbwa();
    plane_motors_out_auto();
    payload_control_unified();  // Unified payload control (manual + auto)

    Serial.print("demanded_pitch:"); Serial.print(get_pitch_demand());
    Serial.print(" |nav_pitch_deg:"); Serial.print(nav_pitch_deg);
    Serial.print(" |demanded_roll:"); Serial.print(nav_roll_cd()/100.0f);
    Serial.print(" |nav_roll_deg:"); Serial.print(nav_roll_deg);
    Serial.print(" |auto_thr:"); Serial.println(fw_control.get_auto_throttle());
    mode_phase = 3;
    current = ModeId::AUTO;
  }

  void _exit() override {
    exit_auto();
  }
};

// ======================= MODE GUIDED (Fixed Wing) =========================
class ModeGuided_FW : public ModeBase {
public:
  ModeId id() const override { return ModeId::GUIDED; }

protected:
  bool _enter() override {
    auto_throttle_mode = true;
    auto_navigation_mode = true;
    mode_fbwa_plane = true;
    return true;
  }

  void _update() override {
    ahrs.update_ahrs();
    nav_gps();

    mode_fbwa_plane = true;
    auto_navigation_mode = true;

    navigate();
    update_speed_height();
    update_alt();
    updateAuto_FW();
    fw_control.updateAUTO_FW(imu.roll, imu.pitch, imu.yaw,
                             imu.gyro_y, imu.gyro_x, imu.gyro_z);
    fw_servos_out_fbwa();
    plane_motors_out_auto();
    payload_control_unified();
    mode_phase = 4;
    current = ModeId::GUIDED;
  }

  void _exit() override {
    auto_navigation_mode = false;
    auto_throttle_mode = false;
  }
};

// Create OOP instances
ModeManual_FW     mMANU_FW;
ModeFBWA_FW       mFBWA_FW;
ModeAuto_FW       mAUTO_FW;
ModeGuided_FW     mGUIDED_FW;