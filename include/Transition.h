#pragma once

#include "AHRS.h"
#include "Arduino.h"
#include "Attitude.h"
#include "ControlModes.h"
#include "FW_config.h"
#include "Mode.h"
#include "Navigation.h"
#include "Radio.h"
#include "TECS.h"
#include "gps.h"
#include "takeoff.h"
#include "bno055.h"
#include "buzzer.h"

float mode_phase, mode_fw;
float decrease = 0.0f;  // ignore
float true_aileron, true_elevator, true_rudder;
bool takeoff_is_done = false;
bool landing_is_done = false;
bool init_time = true;
bool start_landing = false;
bool break_start = false;
bool break_is_done = false;
bool transition_is_done = false;
uint8_t break_delay;
uint8_t time_flag;
float control_signal;
float init_pwm_tkff = 1100;
float init_pwm_land;
float k_p = 0.0;
float k_i = 0.0;
float k_d = 0.0;
bool enterauto = false;
int32_t temp_alt = 0;

int display_wp() {
    if (flag_wp == -1) {
        return -1;
    }
    return flag_wp - 1;
}

void add_waypoint(int32_t lat, int32_t lng, int32_t alt, int index) {
    waypoint[index].lat = lat;
    waypoint[index].lng = lng;
    waypoint[index].alt = alt;
    wp_sum++;
}

void navigate() {
    if (auto_navigation_mode) {
        if (flag_wp < 0) {
            flag_wp++;
            auto_state.next_wp_crosstrack = false;
            target_altitude.amsl_cm = waypoint[flag_wp].alt;
            set_next_WP(waypoint[flag_wp]);
        }
    }

    else if (auto_takeoff_mode) {
        if (flag_takeoff < 0) {
            flag_takeoff++;
            auto_state.next_wp_crosstrack = false;
            set_next_WP(_takeoff[flag_takeoff]);
        }
    }
    // get distance to next waypoint
    auto_state.distance_next_wp = current_loc.get_distance(next_WP_loc);
    auto_state.bearing = current_loc.get_bearing_to(next_WP_loc) / 100.0f;
    auto_state.wp_proportion = current_loc.line_path_proportion(prev_WP_loc, next_WP_loc);
    set_path_proportion(auto_state.wp_proportion);

    if (auto_navigation_mode) {
        const float wp_radius = WP_RADIUS_DEFAULT;
        acceptance_distance_m = turn_distance(wp_radius, auto_state.next_turn_angle);  //  > 20 ? turn_distance(wp_radius, auto_state.next_turn_angle) : 21;
    }

    if (auto_takeoff_mode) {
        const float wp_radius = 13.0f;
        acceptance_distance_m = turn_distance(wp_radius, auto_state.next_turn_angle);
    }

    if (auto_state.distance_next_wp <= acceptance_distance_m || current_loc.past_interval_finish_line(prev_WP_loc, next_WP_loc)) {
        if (auto_navigation_mode) {
            flag_wp++;
            if (flag_wp >= wp_sum) {
                flag_wp = -1;
                auto_navigation_mode = false;
                mode_fbwa = true;
            } else {
                auto_state.next_wp_crosstrack = true;
                target_altitude.amsl_cm = waypoint[flag_wp].alt;
                set_next_WP(waypoint[flag_wp]);
            }
        }
    }
    update_wp_nav();
}

void update_speed_height() {
    if (auto_throttle_mode) {
        update_50hz();
    }
}

void updateLoiter_FW() {
    calc_nav_roll();
    calc_nav_pitch();
    calc_throttle();
}

void updateAuto_FW() {
    calc_nav_roll();
    calc_nav_pitch();
    // calc_throttle();
}

void exit_auto(){
    Serial2.println("EXIT AUTO MODE");
    enterauto = false;
    auto_throttle_mode = false;
    auto_navigation_mode = false;
    Serial2.print("flag_wp tersimpan: "); Serial2.println(flag_wp);
}

void init_auto(){
    Serial2.println("INIT AUTO MODE");
    auto_throttle_mode = true;
    auto_navigation_mode = true;
    auto_loiter_mode = false;
    
    // Cek apakah ini first entry atau resume
    if (flag_wp == -1) {
        // First entry: mulai dari awal
        Serial2.println("First entry - Start from WP 0 (HOME)");
        prev_WP_loc = current_loc = next_WP_loc;
        flag_wp = -1;  // akan jadi 0 di navigate()
        
        // Set HOME sebagai WP 0 (overwrite koordinat, wp_sum sudah ter-increment di main)
        waypoint[0].lat = gepees.latitude * 10000000;
        waypoint[0].lng = gepees.longitude * 10000000;
        waypoint[0].alt = 69 * M_TO_CM;
        
        Serial2.print("HOME set - lat:"); Serial2.print(waypoint[0].lat);
        Serial2.print(" lng:"); Serial2.print(waypoint[0].lng);
        Serial2.print(" alt:"); Serial2.println(waypoint[0].alt);
    } 
    else {
        // Resume: lanjut dari WP terakhir
        Serial2.print("Resume from WP "); Serial2.println(flag_wp);
        
        // Pastikan flag_wp masih valid
        if (flag_wp >= wp_sum) {
            flag_wp = -1;  // reset jika sudah selesai semua
            Serial2.println("All WP completed - restart from WP 0 (HOME)");
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
            
            Serial2.print("Target WP - lat:"); Serial2.print(waypoint[flag_wp].lat);
            Serial2.print(" lng:"); Serial2.print(waypoint[flag_wp].lng);
            Serial2.print(" alt:"); Serial2.println(waypoint[flag_wp].alt);
        }
    }
    
    target_altitude.amsl_cm = waypoint[0].alt;
    target_airspeed = AIRSPEED_CRUISE;
    enterauto = true;
    
    Serial2.print("Total WP: "); Serial2.print(wp_sum);
    Serial2.print(" | Current flag_wp: "); Serial2.println(flag_wp);
}

// manual transistion fixed wing 
void Transition_sequence_manual() {
    static uint32_t last_wp_update = 0;

    // vertical_velocity = get_climb_rate();
    // parameter changer for manual/auto
    ahrs.update_ahrs();
    nav_gps();  // get_position() inside here
    
    // update next WP setiap 1 detik
    if(mode_now != 3 && millis() - last_wp_update > 1000){
        auto_state.distance_next_wp = current_loc.get_distance(next_WP_loc);
        last_wp_update = millis();
    }

    if (mode_now == 1) {
        if (prev_mode == 3) {
            exit_auto();
        }
        
        mode_fbwa_plane = false;
        auto_navigation_mode = false;
        servos_out_manual();
        plane_motors_out();  // pusher.writeMicroseconds(988);
        mode_phase = 1;
        MODE = manual->mode();
    }
    
    else if (mode_now == 2) {
        if (prev_mode == 3) {
            exit_auto();
        }
        
        mode_fbwa_plane = true;
        auto_navigation_mode = false;
        // updateFBWA_FW();
        // stabilize();
        // servos_out_fbwa();
        fw_fbwa.updateFBWA_FW(ch_roll, ch_pitch, ch_yaw, ch_throttle,
                            imu.roll, imu.pitch, imu.yaw,
                            imu.gyro_y, imu.gyro_x, imu.gyro_z);
        fw_servos_out_fbwa();
        plane_motors_out();  // TODO:crosschecking (sblmnya plane_motors_out_auto(1900))
        mode_phase = 2;
        MODE = fbwa->mode();  // pusher.writeMicroseconds(ch_throttle);
    }

    else if (mode_now == 3) {
        if (!enterauto) {
            init_auto();
        }
        
        mode_fbwa_plane = true;
        navigate();
        update_speed_height();
        update_alt();
        updateAuto_FW();
        stabilize();
        servos_out_fbwa();
        plane_motors_out();
        mode_phase = 3;
        MODE = automatic->mode();
    }
    prev_mode = mode_now;
}

// manual transistion flywing
void Transition_sequence_manual_old() {
    // vertical_velocity = get_climb_rate();
    // parameter changer for manual/auto
    ahrs.update_ahrs();
    nav_gps();  // get_position() inside here
    // param changer ends
    if (mode_now == 1) {
        mode_fbwa_plane = false;
        auto_navigation_mode = false;
        servos_out_manual();
        plane_motors_out();  // pusher.writeMicroseconds(988);
        mode_phase = 1;
        MODE = manual->mode();  // copter
        // auto_state.distance_next_wp = current_loc.get_distance(next_WP_loc);
    }
    
    else if (mode_now == 2) {
        mode_fbwa_plane = true;
        auto_navigation_mode = false;
        // updateFBWA_FW();
        // stabilize();
        // servos_out_fbwa();
        fw_fbwa.updateFBWA_FW(ch_roll, ch_pitch, ch_yaw, ch_throttle,
                            imu.roll, imu.pitch, imu.yaw,
                            imu.gyro_y, imu.gyro_x, imu.gyro_z);
        fw_servos_out_fbwa();
        plane_motors_out();  // TODO:crosschecking (sblmnya plane_motors_out_auto(1900))
        mode_phase = 2;
        MODE = fbwa->mode();  // pusher.writeMicroseconds(ch_throttle);
    }

    // FASE 3 TEST AUTO
    else if (mode_now == 3) {
        if (!enterauto) {
            auto_throttle_mode = true;
            auto_navigation_mode = true;
            auto_loiter_mode = false;
            prev_WP_loc = current_loc = next_WP_loc;
            flag_wp = -1;
            target_altitude.amsl_cm = 69.0f * M_TO_CM;  // cm
            target_airspeed = AIRSPEED_CRUISE;
            // Serial.println("enter auto done!");
            add_waypoint(gepees.latitude * 10000000, gepees.longitude * 10000000, 69, 0);
            // add_waypoint(latitude, longitude, 90);
            enterauto = true;  // Set flag menjadi true agar fungsi tidak dipanggil lagi
        }
        mode_fbwa_plane = true;
        // test_fuzzy_roll();
        navigate();
        update_speed_height();
        update_alt();
        updateAuto_FW();
        stabilize();
        servos_out_fbwa();
        plane_motors_out();
        // plane_motors_out_auto(988);
        mode_phase = 3;
        MODE = automatic->mode();
    }
}

extern FlywingControl flywingcontrol;