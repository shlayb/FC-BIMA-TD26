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
    // launch_guard.on_exit_auto();
    // Serial2.print("flag_wp tersimpan: "); Serial2.println(flag_wp);
}

void init_auto(){
    auto_throttle_mode = true;
    auto_navigation_mode = true;
    auto_loiter_mode = false;
    // launch_guard.on_enter_auto();
    
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
    mode_fbwa_plane      = false;
    auto_navigation_mode = false;
    auto_throttle_mode   = false;
    auto_loiter_mode     = false;
    mode_phase           = 1;
    current              = ModeId::MANU;
    return true;
  }

  void _update() override {
    servos_out_manual();
    plane_motors_out();
    payload_control_unified();
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
    mode_fbwa_plane      = true;
    auto_navigation_mode = false;
    auto_throttle_mode   = false;
    auto_loiter_mode     = false;
    mode_phase           = 2;
    current              = ModeId::FBWA;
    return true;
  }

  void _update() override {
    fw_control.updateFBWA_FW(ch_roll, ch_pitch, ch_yaw, ch_throttle,
                             imu.roll, imu.pitch, imu.yaw,
                             imu.gyro_y, imu.gyro_x, imu.gyro_z);
    fw_servos_out_fbwa();
    plane_motors_out();
    payload_control_unified();
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
    mode_fbwa_plane      = true;
    auto_navigation_mode = true;
    auto_throttle_mode   = true;
    auto_loiter_mode     = false;
    mode_phase           = 3;
    current              = ModeId::AUTO;
    return true;
  }

  void _update() override {
    // 1. Navigation & guidance (L1 track following / waypoint sequencing)
    navigate();

    // 2. TECS energy & altitude controller
    update_tecs();

    // 3. Inner attitude PID loop
    fw_control.updateAUTO_FW(imu.roll, imu.pitch, imu.yaw,
                             imu.gyro_y, imu.gyro_x, imu.gyro_z);

    // 4. Actuator outputs
    payload_control_unified();
    fw_servos_out_fbwa();
    // plane_motors_out_auto(); // Full TECS auto throttle control
    plane_motors_out();

#if !HITL_ENABLED
    // Serial.print("demanded_pitch:"); Serial.print(get_pitch_demand());
    // Serial.print(" |nav_pitch_deg:"); Serial.print(nav_pitch_deg);
    // Serial.print(" |demanded_roll:"); Serial.print(nav_roll_cd()/100.0f);
    // Serial.print(" |nav_roll_deg:"); Serial.print(nav_roll_deg);
    // Serial.print(" |auto_thr:"); Serial.println(fw_control.get_auto_throttle());
#endif
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
    auto_throttle_mode   = true;
    auto_navigation_mode = true;
    auto_loiter_mode     = false;
    mode_fbwa_plane      = true;
    mode_phase           = 4;
    current              = ModeId::GUIDED;
    return true;
  }

  void _update() override {
    navigate();
    update_tecs();
    fw_control.updateAUTO_FW(imu.roll, imu.pitch, imu.yaw,
                             imu.gyro_y, imu.gyro_x, imu.gyro_z);
    fw_servos_out_fbwa();
    plane_motors_out_auto();
    payload_control_unified();
  }

  void _exit() override {
    auto_navigation_mode = false;
    auto_throttle_mode   = false;
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
    
    // Simpan posisi saat RTL dimulai sebagai titik awal garis lintasan
    rtl_start_loc = current_loc;
    
    // Ketinggian aman RTL: rtl_altitude (default 60m) di atas HOME
    target_altitude.amsl_cm = (int32_t)(rtl_altitude * M_TO_CM);
    target_airspeed = AIRSPEED_CRUISE;
    // Serial.println("[MODE] >>> ENTERING RTL MODE (Returning Home) <<<");
    return true;
  }

  void _update() override {
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
        // Fase 2: Sudah di area HOME -> Orbit Loiter melingkar sesuai WP_LOITER_RAD
        float rtl_rad = fabsf(loiter_mode_radius);
        if (rtl_rad < 20.0f) rtl_rad = LOITER_RADIUS_DEFAULT;
        int8_t rtl_dir = (loiter_mode_radius >= 0.0f) ? 1 : -1;
        update_loiter(home_loc, rtl_rad, rtl_dir);
    }

    update_tecs();
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

// =====================================================================
//  Loiter runtime variables (state/lap tracking)
//  Variabel konfigurasi (loiter_center_loc, loiter_mode_radius, dll)
//  berada di Auto_setup.h agar dapat diakses oleh Navigation.h & Params.h
// =====================================================================

/** Jumlah putaran penuh (lap) yang sudah diselesaikan */
uint16_t loiter_lap_count = 0;

/** Bearing terakhir ke center (untuk deteksi lap) */
int32_t loiter_last_bearing_cd = 0;

/** Akumulator sudut untuk deteksi lap (centi-degrees) */
int32_t loiter_total_angle_cd = 0;

/** Timestamp saat mode LOITER pertama kali diaktifkan (ms) */
uint32_t loiter_start_time_ms = 0;

/** Durasi loiter saat ini dalam detik */
float loiter_duration_sec = 0.0f;

/** Flag: apakah loiter sudah menangkap circle (dari L1 controller) */
bool loiter_circle_captured = false;

// =====================================================================
//  Helper: Hitung perubahan bearing dan akumulasi untuk lap counter
// =====================================================================
static void update_loiter_lap_counter()
{
    // Hitung bearing dari current position ke loiter center
    int32_t bearing_cd = current_loc.get_bearing_to(loiter_center_loc);
    
    // Hitung delta bearing dari update sebelumnya
    int32_t delta_cd = bearing_cd - loiter_last_bearing_cd;
    
    // Wrap ke range -18000 .. +18000 (centi-degrees)
    delta_cd = wrap_180_cd(delta_cd);
    
    int8_t dir = (loiter_mode_radius >= 0.0f) ? 1 : -1;
    loiter_mode_direction = dir;

    // Filter delta yang terlalu besar (GPS glitch)
    if (abs(delta_cd) < 3600) {  // max 36° per update = reasonable
        // Akumulasi sesuai arah loiter
        if (dir == 1) {
            // CW: bearing bertambah (positif)
            if (delta_cd > 0) {
                loiter_total_angle_cd += delta_cd;
            }
        } else {
            // CCW: bearing berkurang (negatif)
            if (delta_cd < 0) {
                loiter_total_angle_cd += abs(delta_cd);
            }
        }
        
        // 1 lap = 36000 centi-degrees = 360°
        if (loiter_total_angle_cd >= 36000) {
            loiter_lap_count++;
            loiter_total_angle_cd -= 36000;
        }
    }
    
    loiter_last_bearing_cd = bearing_cd;
}

// ======================= MODE LOITER (Fixed Wing) =========================
class ModeLoiter_FW : public ModeBase {
public:
    ModeId id() const override { return ModeId::LOITER; }

protected:
    // =================================================================
    //  _enter() — Dipanggil SEKALI saat masuk ke mode LOITER
    // =================================================================
    bool _enter() override {
        // --- Set kontrol flags ---
        auto_throttle_mode   = true;   // TECS mengontrol throttle
        auto_navigation_mode = false;  // BUKAN waypoint sequencer, kita handle sendiri
        auto_loiter_mode     = true;   // flag loiter aktif
        mode_fbwa_plane      = true;   // inner-loop attitude control aktif
        active_mode          = ModeId::LOITER;
        current              = ModeId::LOITER;

        // --- Handle target loiter: kustom (dari misi selesai) vs capture posisi GPS saat ini ---
        if (loiter_custom_target_set) {
            loiter_custom_target_set = false;
            target_altitude.amsl_cm = loiter_target_alt_cm;
        } else {
            ahrs.get_position(loiter_center_loc);
            loiter_target_alt_cm = current_loc.alt;
            target_altitude.amsl_cm = loiter_target_alt_cm;
        }
        
        // --- Set airspeed target ---
        target_airspeed = AIRSPEED_CRUISE;

        // --- Sinkronisasi radius dan arah loiter (ArduPilot WP_LOITER_RAD convention) ---
        float rad = fabsf(loiter_mode_radius);
        if (rad < 20.0f) rad = LOITER_RADIUS_DEFAULT;
        loiter_mode_direction = (loiter_mode_radius >= 0.0f) ? 1 : -1;
        
        // --- Reset lap counter ---
        loiter_lap_count = 0;
        loiter_total_angle_cd = 0;
        loiter_last_bearing_cd = current_loc.get_bearing_to(loiter_center_loc);
        
        // --- Catat waktu mulai ---
        loiter_start_time_ms = millis();
        loiter_duration_sec = 0.0f;
        
        // --- Reset L1 circle capture status ---
        loiter_circle_captured = false;

        // Serial.println("[LOITER] ====== MODE LOITER ACTIVATED ======");
        // Serial.printf("[LOITER] Center: lat=%ld lng=%ld\n", 
        //               loiter_center_loc.lat, loiter_center_loc.lng);
        // Serial.printf("[LOITER] Radius: %.0fm (WP_LOITER_RAD: %.0f, %s)\n", 
        //               rad, loiter_mode_radius,
        //               loiter_mode_direction == 1 ? "CW" : "CCW");
        // Serial.printf("[LOITER] Target Alt: %.1fm | Airspeed: %.1f m/s\n",
        //               loiter_target_alt_cm / 100.0f, target_airspeed);
        // Serial.printf("[LOITER] Bank limit: %.0f deg\n", loiter_bank_limit);

        return true;
    }

    // =================================================================
    //  _update() — Dipanggil setiap cycle selama LOITER aktif
    // =================================================================
    void _update() override {
        // ---- 1. L1 Navigation: Hitung lateral acceleration untuk circular path (ArduPilot convention) ----
        float rad = fabsf(loiter_mode_radius);
        if (rad < 20.0f) rad = LOITER_RADIUS_DEFAULT;
        int8_t dir = (loiter_mode_radius >= 0.0f) ? 1 : -1;
        loiter_mode_direction = dir;

        update_loiter(loiter_center_loc, 
                      rad, 
                      dir);

        // ---- 3. Update circle capture status ----
        loiter_circle_captured = reached_loiter_target();

        // ---- 4. TECS: Guidance & Energy Controller (pitch, throttle, roll) ----
        update_tecs();
        nav_roll_deg = constrain(nav_roll_deg, -loiter_bank_limit, loiter_bank_limit);

        // ---- 6. Inner-loop attitude controller ----
        fw_control.updateAUTO_FW(imu.roll, imu.pitch, imu.yaw,
                                 imu.gyro_y, imu.gyro_x, imu.gyro_z);

        // ---- 7. Output ke actuator ----
        fw_servos_out_fbwa();
        // plane_motors_out_auto();
        plane_motors_out();
        payload_control_unified();

        // ---- 8. Update lap counter & timer ----
        update_loiter_lap_counter();
        loiter_duration_sec = (millis() - loiter_start_time_ms) / 1000.0f;

        // ---- 9. Telemetry output ----
        mode_phase = 6;

        // Debug output periodik (500ms)
        static uint32_t last_print_ms = 0;
        if (millis() - last_print_ms > 500) {
            last_print_ms = millis();
            
            float dist_to_center = current_loc.get_distance(loiter_center_loc);
            
            // Serial.printf("[LOITER] circle:%s | dist:%.1fm | rad:%.0fm | "
            //              "roll:%.1f | pitch:%.1f | thr:%.0f%% | "
            //              "laps:%d | time:%.0fs\n",
            //              loiter_circle_captured ? "YES" : "NO",
            //              dist_to_center,
            //              rad,
            //              nav_roll_deg,
            //              nav_pitch_deg,
            //              (float)fw_control.get_auto_throttle(),
            //              loiter_lap_count,
            //              loiter_duration_sec);
        }
    }

    // =================================================================
    //  _exit() — Dipanggil SEKALI saat keluar dari mode LOITER
    // =================================================================
    void _exit() override {
        auto_throttle_mode = false;
        auto_loiter_mode   = false;

        // Serial.println("[LOITER] ====== MODE LOITER DEACTIVATED ======");
        // Serial.printf("[LOITER] Total laps: %d | Duration: %.1fs\n", 
        //               loiter_lap_count, loiter_duration_sec);
    }
};

// =====================================================================
//  Public API — Untuk mengubah parameter loiter secara runtime
//  Mengikuti konvensi ArduPilot WP_LOITER_RAD (+ = CW, - = CCW)
// =====================================================================

inline void loiter_set_radius(float radius_m) {
    float sgn = (loiter_mode_radius < 0.0f) ? -1.0f : 1.0f;
    loiter_mode_radius = sgn * constrain(fabsf(radius_m), 20.0f, 500.0f);
    // Serial.printf("[LOITER] Radius updated: %.0fm (%s, WP_LOITER_RAD: %.0f)\n", 
    //               fabsf(loiter_mode_radius), loiter_mode_radius >= 0 ? "CW" : "CCW", loiter_mode_radius);
}

inline void loiter_set_direction(int8_t dir) {
    float r = fabsf(loiter_mode_radius);
    if (r < 20.0f) r = LOITER_RADIUS_DEFAULT;
    loiter_mode_radius = (dir >= 0) ? r : -r;
    loiter_mode_direction = (dir >= 0) ? 1 : -1;
    loiter_lap_count = 0;
    loiter_total_angle_cd = 0;
    // Serial.printf("[LOITER] Direction updated: %s (WP_LOITER_RAD: %.0f)\n", 
    //               loiter_mode_radius >= 0 ? "CW" : "CCW", loiter_mode_radius);
}

// Create OOP instances
ModeManual_FW     mMANU_FW;
ModeFBWA_FW       mFBWA_FW;
ModeAuto_FW       mAUTO_FW;
ModeGuided_FW     mGUIDED_FW;
ModeRTL_FW        mRTL_FW;
ModeLoiter_FW     mLOITER_FW;