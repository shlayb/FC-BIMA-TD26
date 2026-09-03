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
#include "takeoff.h"

void exit_auto(){
    // Serial2.println("EXIT AUTO MODE");
    enterauto = false;
    auto_throttle_mode = false;
    auto_navigation_mode = false;
    launch_guard.on_exit_auto();
    // Serial2.print("flag_wp tersimpan: "); Serial2.println(flag_wp);
}

void init_auto(){
    auto_throttle_mode = true;
    auto_navigation_mode = true;
    auto_loiter_mode = false;
    launch_guard.on_enter_auto();
    
    // Cek apakah ini first entry atau resume
    if (flag_wp <= 0) {
        flag_wp = -1;
        prev_WP_loc = current_loc;
        
        // Update HOME dengan posisi GPS saat ini (jika valid)
        if (gepees.latitude != 0.0f && gepees.longitude != 0.0f) {
            waypoint[0].lat = (int32_t)(gepees.latitude * 10000000.0f);
            waypoint[0].lng = (int32_t)(gepees.longitude * 10000000.0f);
        }
    } else {
        // Resume: lanjut dari WP terakhir
        if (flag_wp >= wp_sum) {
            flag_wp = -1;  // reset jika selesai semua
        } else {
            auto_state.next_wp_crosstrack = false;
            target_altitude.amsl_cm = waypoint[flag_wp].alt;
            set_next_WP(waypoint[flag_wp]);
        }
    }
    
    target_airspeed = AIRSPEED_CRUISE;
    enterauto = true;
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
    
    // Cek jika seluruh waypoint misi telah selesai -> transisi ke RTL
    if (flag_wp >= wp_sum && wp_sum > 0) {
        mode_now = 5;
        modes.setMode(ModeId::RTL);
        active_mode = ModeId::RTL;
        current = ModeId::RTL;
        return;
    }

    mode_fbwa_plane = true;
    navigate();
    if (active_mode != ModeId::AUTO || current != ModeId::AUTO || mode_now == 5 || (flag_wp >= wp_sum && wp_sum > 0)) {
        return;
    }
    update_speed_height();
    update_alt();
    updateAuto_FW();
    
    // Launch Guard: Di darat motor dijamin OFF (988), aktif setelah dilempar / di-shake
    float safe_pitch_dem = nav_pitch_deg;
    float safe_roll_dem  = nav_roll_deg;
    float safe_thr_pwm   = 988.0f;

    launch_guard.update(safe_pitch_dem, safe_roll_dem, safe_thr_pwm);

    if (!launch_guard.is_launch_complete()) {
        nav_pitch_deg = safe_pitch_dem;
        nav_roll_deg  = safe_roll_dem;
    }

    // Cascaded AUTO controller derived from FBWA inner loop + L1 + TECS
    fw_control.updateAUTO_FW(imu.roll, imu.pitch, imu.yaw,
                             imu.gyro_y, imu.gyro_x, imu.gyro_z);
    fw_servos_out_fbwa();
    plane_motors_out_auto(safe_thr_pwm);
    payload_control_unified();  // Unified payload control (manual + auto)

#if !HITL_ENABLED
    Serial.print("demanded_pitch:"); Serial.print(get_pitch_demand());
    Serial.print(" |nav_pitch_deg:"); Serial.print(nav_pitch_deg);
    Serial.print(" |demanded_roll:"); Serial.print(nav_roll_cd()/100.0f);
    Serial.print(" |nav_roll_deg:"); Serial.print(nav_roll_deg);
    Serial.print(" |auto_thr:"); Serial.println(fw_control.get_auto_throttle());
#endif
    if (active_mode == ModeId::AUTO && current == ModeId::AUTO) {
        mode_phase = 3;
    }
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

// ======================= MODE RTL (Return To Launch) =========================
class ModeRTL_FW : public ModeBase {
public:
  ModeId id() const override { return ModeId::RTL; }

private:
  Locations rtl_start_loc{};

protected:
  bool _enter() override {
    auto_throttle_mode = true;
    auto_navigation_mode = false;
    auto_loiter_mode = true;
    mode_fbwa_plane = true;
    active_mode = ModeId::RTL;
    current = ModeId::RTL;
    mode_now = 5;
    
    // Simpan posisi saat RTL dimulai sebagai titik awal garis lintasan
    rtl_start_loc = current_loc;
    
    // Ketinggian aman RTL: rtl_altitude (default 60m) di atas HOME
    target_altitude.amsl_cm = (int32_t)(rtl_altitude * M_TO_CM);
    target_airspeed = AIRSPEED_CRUISE;
    Serial.println("[MODE] >>> ENTERING RTL MODE (Returning Home) <<<");
    return true;
  }

  void _update() override {
    ahrs.update_ahrs();
    nav_gps();

    mode_fbwa_plane = true;
    active_mode = ModeId::RTL;
    current = ModeId::RTL;
    mode_now = 5;

    // Koordinat HOME tersimpan di waypoint[0]
    Locations home_loc = waypoint[0];
    if (home_loc.lat == 0 && home_loc.lng == 0) {
        home_loc = home_wp_loc;
    }

    float dist_to_home = current_loc.get_distance(home_loc);

    if (dist_to_home > (LOITER_RADIUS_DEFAULT + 15.0f)) {
        // Fase 1: Terbang langsung menuju HOME sepanjang garis jalur start -> HOME
        update_waypoint(rtl_start_loc, home_loc, 0.0f);
    } else {
        // Fase 2: Sudah di area HOME -> Orbit Loiter melingkar 50m radius (Clockwise)
        update_loiter(home_loc, LOITER_RADIUS_DEFAULT, 1);
    }

    update_speed_height();
    update_alt();
    updateAuto_FW();
    fw_control.updateAUTO_FW(imu.roll, imu.pitch, imu.yaw,
                             imu.gyro_y, imu.gyro_x, imu.gyro_z);
    fw_servos_out_fbwa();
    plane_motors_out_auto();
    payload_control_unified();

    mode_phase = 5;
  }

  void _exit() override {
    auto_loiter_mode = false;
    auto_throttle_mode = false;
  }
};

// Create OOP instances
ModeManual_FW     mMANU_FW;
ModeFBWA_FW       mFBWA_FW;
ModeAuto_FW       mAUTO_FW;
ModeGuided_FW     mGUIDED_FW;
ModeRTL_FW        mRTL_FW;