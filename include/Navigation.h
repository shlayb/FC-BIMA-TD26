#pragma once

#include "FW_controller.h"
#include "AutoMode.h"
// #include "LQR_PF.h"
#include "TECS.h"
#include "L1_Controller.h"

bool enable_throttle_nudge = true;
int16_t throttle_nudge;
uint32_t last_home_set_ms = 0;
float groundspeed_undershoot;

void calc_groundspeed_undershoot() {
    if (gepees.gps_status >= GPS_FIX_2D) {
        float gndSpdFwd = ahrs.groundspeed;
        groundspeed_undershoot = (MIN_GNDSPEED > 0) ? (MIN_GNDSPEED - gndSpdFwd) : 0;
    } else {
        groundspeed_undershoot = 0;
    }
}

void nav_gps() {
    uint32_t now = millis();
    if (!arming && now - last_home_set_ms >= 5000) {
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