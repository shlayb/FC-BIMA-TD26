#pragma once

#include "FW_controller.h"
#include "Auto_setup.h"
#include "TECS.h"
#include "L1_Controller.h"
#include "Actuator.h"
#include "Mode_setup.h"
#include "Navigation.h"
#include "FW_ControlModes.h"
#include "Fuzzy_FW_Roll.h"
// #include "Radio.h"

// Forward declaration for waypoint notification
void notifyWaypointReached(uint16_t seq);

bool enable_throttle_nudge = true;
int16_t throttle_nudge;
uint32_t last_home_set_ms = 0;
float groundspeed_undershoot;

void calc_groundspeed_undershoot() {
    if (gepees.gps_status >= GPS_FIX_2D) {
        float gndSpdFwd = ahrs.groundspeed;
        groundspeed_undershoot = (min_gndspeed > 0) ? (min_gndspeed - gndSpdFwd) : 0;
    } else {
        groundspeed_undershoot = 0;
    }
}

void nav_gps() {
    uint32_t now = millis();
    if (!arming && now - last_home_set_ms >= 5000 && baro.altitude <= 10) {
        last_home_set_ms = now;
        ahrs.setHome(home_wp_loc);
    }

    ahrs.get_position(current_loc);
    ahrs.get_relative_position_D_home(relative_alt);
    ahrs.estimate_windspeed();
    relative_alt *= -1.0f;
    calc_groundspeed_undershoot();
}

void update_wp_nav()
{
    if (auto_state.crosstrack)
    {
        update_waypoint(prev_WP_loc, next_WP_loc, 0.0f);
    }
    else
    {
        update_waypoint(current_loc, next_WP_loc, 0.0f);
    }
}

float get_next_ground_course(float default_angle)
{
    Locations next_wp_after = waypoint[flag_wp+1];
    if (next_wp_after.lat==0 && next_wp_after.lng==0)
    {
        return default_angle;
    }
    return next_WP_loc.get_bearing_to(next_wp_after)/100.0f;
}

void setup_turn_angle()
{
    float next_ground_course = get_next_ground_course(-1);
    if (next_ground_course == -1)
    {
        auto_state.next_turn_angle = 90.0f;
    }
    else
    {
        float ground_course = prev_WP_loc.get_bearing_to(next_WP_loc) / 100.0f;
        auto_state.next_turn_angle = wrap_180(next_ground_course - ground_course);
    }
}

void set_next_WP(struct Locations &loc)
{
    if (auto_state.next_wp_crosstrack)
    {
        prev_WP_loc = next_WP_loc;
        auto_state.crosstrack = true;
    }
    else
    {
        prev_WP_loc = current_loc;
        auto_state.crosstrack = false;
        auto_state.next_wp_crosstrack = true;
    }
    next_WP_loc = loc;
    setup_turn_angle();
}

// update altitude
void update_alt()
{

    if (enable_throttle_nudge)
    {
        throttle_nudge = 0.5 * scaleToPercent(ch_throttle);
    }
    else
    {
        throttle_nudge = 0.0f;
    }

    //calculate sink rate (UNUSED)
    // float sink_rate = -get_climb_rate();
    if (auto_throttle_mode)
    {
        update_pitch_throttle(target_altitude.amsl_cm,
                                                  target_airspeed,
                                                  throttle_nudge,
                                                  tecs_hgt_afe(),
                                                  aerodynamic_load_factor);
    }
}

int autonav_state = 0;  /// variable to check auto-loop state, delete after testing
bool enterauto = false;

void update_speed_height() {
    if (auto_throttle_mode) {
        update_50hz();
    }
}

void updateAuto_FW() {
    calc_nav_roll();
    calc_nav_pitch();
    // calc_throttle();
}

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
    if (rpi_external_setpoint_active) {
        if (millis() - rpi_external_setpoint_last_ms > 1000) {
            rpi_external_setpoint_active = false;
            rpi_external_setpoint_yaw_valid = false;
        } else {
            prev_WP_loc = current_loc;
            next_WP_loc = rpi_external_setpoint_target;
            auto_state.next_wp_crosstrack = false;
            target_altitude.amsl_cm = rpi_external_setpoint_target.alt;
            auto_navigation_mode = true;
            update_wp_nav();
            return;
        }
    }

    if (current == ModeId::GUIDED) {
        prev_WP_loc = current_loc;
        next_WP_loc = current_loc;
        auto_state.next_wp_crosstrack = false;
        target_altitude.amsl_cm = current_loc.alt;
        auto_state.distance_next_wp = 0.0f;
        auto_state.bearing = 0.0f;
        auto_state.wp_proportion = 0.0f;
        update_wp_nav();
        return;
    }

    if (current == ModeId::RTL || !auto_navigation_mode) {
        return;
    }

    if (flag_wp <= 0) {
        flag_wp = 1;
        auto_state.next_wp_crosstrack = false;
        target_altitude.amsl_cm = waypoint[flag_wp].alt;
        set_next_WP(waypoint[flag_wp]);
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
        float td = turn_distance(wp_radius, auto_state.next_turn_angle);
        acceptance_distance_m = MAX(25.0f, td);
    }

    if (auto_takeoff_mode) {
        const float wp_radius = 13.0f;
        float td = turn_distance(wp_radius, auto_state.next_turn_angle);
        acceptance_distance_m = MAX(15.0f, td);
    }
    // Hit radius check
    bool hit_radius = (auto_state.distance_next_wp <= acceptance_distance_m);
    bool finish_line = current_loc.past_interval_finish_line(prev_WP_loc, next_WP_loc);
    
    // Syarat klaim waypoint: sudah masuk radius acceptance atau melewati garis batas (finish line)
    // Khusus waypoint terakhir (WP4), toleransi diperluas (<= 45m atau finish line)
    bool is_last_wp = (flag_wp >= (wp_sum - 1));
    bool should_advance = hit_radius || finish_line || (is_last_wp && auto_state.distance_next_wp <= 45.0f);

    if (should_advance) {
        if (auto_navigation_mode) {
            // Beritahu GCS seq WP saat ini telah dicapai
            notifyWaypointReached(flag_wp);
            
            if (is_last_wp) {
                // Semua Waypoint telah selesai -> Masuk Mode RTL (Return to Launch)
                // Matikan navigasi auto DULU agar navigate() tidak jalan lagi saat mode sedang transisi
                auto_navigation_mode = false;
                flag_wp = wp_sum;
                mode_now = 5;
                // Langsung set mode RTL via manager agar responsif.
                modes.setMode(ModeId::RTL);
                active_mode = ModeId::RTL;
                current = ModeId::RTL;
                return;
            } else {
                // Lanjut ke waypoint berikutnya
                flag_wp++;
                target_altitude.amsl_cm = waypoint[flag_wp].alt;
                set_next_WP(waypoint[flag_wp]);
            }
        }
    }
    update_wp_nav();
}

