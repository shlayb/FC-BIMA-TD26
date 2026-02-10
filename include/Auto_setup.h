#pragma once
#include <Locations.h>
// #include "FW_config.h"
#include <AP_Math.h>
#include "FW_ControlModes.h"
// #include "Radio.h"

bool auto_throttle_mode     = false;
bool auto_navigation_mode   = false;
bool auto_landing_mode      = false;
bool auto_loiter_mode       = false;
bool auto_takeoff_mode      = false;
bool safety_mode            = false;
bool stabilize_lock         = false;
float target_airspeed;
float relative_alt;
int flag_wp                 = -1;
int flag_land               = -1;
int flag_takeoff            = -1;
int wp_sum                  =  0;
int land_sum                =  1;
int takeoff_sum             =  1;


Locations waypoint[100];
// Locations landing[1]; //conflic declaration
Locations _takeoff[1]; //conflic declaration
/** previous waypoint */
Locations prev_WP_loc{};
/** current plane location */
Locations current_loc{};
/** next waypoint */
Locations next_WP_loc{};
/* Home/Launch waypoint*/
Locations home_wp_loc{};
float acceptance_distance_m;
// struct
// {
//     /** constrain the roll */
//     float max_roll_limit = HEAD_MAX;
//     /** constrain the pitch */
//     float max_pitch_limit = PITCH_MAX;
//     float min_pitch_limit = PITCH_MIN;
// } fbwa_state;

int value = 0;
float value2 = 0;
float landing_pitch_degree;
float takeoff_pitch_degree;
struct
{
    int8_t climb_rate = FBWB_CLIMB_RATE;
    float airspeed = MIN_AIRSPEED;
} fbwb_state;
struct
{
    /** should we crosstrack on the next wp? */
    bool next_wp_crosstrack : 1;
    bool crosstrack : 1;
    float distance_next_wp;
    float next_wp_altitude;
    float bearing;
    float wp_proportion;
    float next_turn_angle;
    Vector3f forced_rpy;
    float baro_takeoff_alt;
} auto_state;

//* takeoff *//
struct
    {
        float highest_airspeed      = 0;
        int32_t target_altitude     = 29 * M_TO_CM;
        int8_t pitch_limit_takeoff  = 15;
        bool complete               = false;
        bool doing                  = false;
        int32_t throttle            = 0;
        uint32_t now;
        uint32_t dtime;
        float head_setpoint;
        float nav_head;
        uint32_t last_tkoff_arm_time;
        uint32_t last_check_ms;
        uint32_t rudder_takeoff_warn_ms;
        uint32_t last_report_ms;
        bool launchTimerStarted;
        uint8_t accel_event_counter;
        uint32_t accel_event_ms;
        uint32_t start_time_ms;
        bool waiting_for_rudder_neutral;
        bool ready_for_throttle = false;

 } auto_takeoff_state;
// float relative_altitude = 0.0f;
// Altitude control
struct
{
    // target altitude above sea level in cm. Used for barometric
    // altitude navigation
    float amsl_cm;
    // target altitude from ground level in cm. Used for Lidar
    float ground_alt_lidar_cm;
    // Altitude difference between previous and current waypoint in
    // centimeters. Used for glide slope handling
    int32_t offset_cm;
    // last input for FBWB/CRUISE height control
    float last_elevator_input;
    // last time we checked for pilot control of height
    uint32_t last_elev_check_us; 
} target_altitude{}; //target_altitude{}

/** desired attitude navigation */
// float nav_roll_deg;
// float nav_pitch_deg;
float max_yaw_angle = 45.0f;

struct {  
        Locations center;
        //center of loiter
       
      
        uint16_t wp_radius = WP_RADIUS_DEFAULT; 
        //loiter wp radius
        uint16_t in_loit_rad = wp_radius;//2*5*wp_radius/(2*M_PI);
        float loiter_radius ;
        //loiter radius
        
        Locations wp_loiter[100];
        int num_wp_loiter;
        int flag_wp=0;
        //wp loiter
        int32_t old_target_bearing_cd;
        // previous target bearing, used to update sum_cd
        int32_t sum_cd = 0;
        // total angle completed in the loiter so far
        int8_t direction = -1;
        // Direction for loiter. 1 for clockwise, -1 for counter-clockwise
        uint32_t start_time_ms;
        // start time of the loiter.  Milliseconds.
        uint32_t time_max_ms;
        // The amount of time we should stay in a loiter for the Loiter Time command.  Milliseconds.
        int wp_needed;

        int counter = 0;
} loiter;

    /** scaled roll limit based on pitch */
    float roll_limit_deg = HEAD_MAX;
    float pitch_limit_min = PITCH_MIN;
    float pitch_limit_max = PITCH_MAX;
    float airspeed_min = MIN_AIRSPEED;

float tecs_hgt_afe(void)
{
    /*
      pass the height above field elevation as the height above
      the ground when in landing, which means that TECS gets the
      rangefinder information and thus can know when the flare is
      coming.
    */
    float hgt_afe;
    // when in normal flight we pass the hgt_afe as relative
    // altitude to home
    hgt_afe = relative_alt;
    return hgt_afe;
}
