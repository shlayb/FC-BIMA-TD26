#ifndef FW_CONTROLMODES_H
#define FW_CONTROLMODES_H

#include <Arduino.h>
#include "Radio.h"
#include "AHRS.h"
#include "Control_data.h"
// #include "Servos_FW.h"
#include "FW_config.h"
#include <AP_Math.h>
#include "TECS.h" // untuk auto juga
#include "L1_Controller.h"
#include "Auto_setup.h" // untuk auto juga

// int value = 0;
float nav_roll_deg;
float nav_pitch_deg;
uint16_t servo_pwm_ail_L;
uint16_t servo_pwm_ail_R;
uint16_t servo_pwm_ele;
uint16_t servo_pwm_rud_R;
uint16_t servo_pwm_rud_L;
uint16_t servo_pwm_payload;
uint16_t motor_pwm;
float cmdroll;

// Flag untuk autopayload trigger dari mavlink (pesan "DROP" dari Raspberry Pi)
bool payload_drop_command = false;
float aerodynamic_load_factor = 1.0f; //ignore

/*
// currently ignore

class Mode {
    public:
        virtual const char *mode() const = 0;
};

class MANUAL : public Mode {
    public:
        const char *mode() const override {return"MANU";}
};

class FBWA : public Mode {
    public:
        const char *mode() const override {return"FBWA";}
};

Mode* manu = new MANUAL;
Mode* fbwa = new FBWA;
const char *MODE;
*/


void updateManual_FW()
{
    servo_pwm_ail_L = ch_roll;
    servo_pwm_ail_R = ch_roll;  
    servo_pwm_ele = ch_pitch;
    servo_pwm_rud_L = ch_yaw;
    servo_pwm_rud_R = ch_yaw;
    motor_pwm = ch_throttle;

    // update_manual_payload();
}

void updateFBWA_FW()
{
    nav_roll_deg = outputScaler(ch_roll) * HEAD_MAX; 
    //pwm 1000: 0, 2000:1 
    if (ch_throttle <= 1100){
        nav_roll_deg = constrain(nav_roll_deg, -55, 55);  //pas kecepatan kecil, rollnya toleransinya diperbesar lg
    }
    float pitch_input = outputScaler(ch_pitch);
    //makin pelan pesawat osilasinya makin besar jadi toleransinya harus diperbesar lg

    if(pitch_input > 0){
        nav_pitch_deg = pitch_input * PITCH_MAX;
    }
    else{
        nav_pitch_deg = -(pitch_input * PITCH_MIN);
    }
    nav_pitch_deg = constrain(nav_pitch_deg, PITCH_MIN, PITCH_MAX); //membatasi 

}

/** calculate roll from navigation controller */
void calc_nav_roll()
{
    float commanded_roll = (nav_roll_cd())/100; //kalau arm kedalam navroll nya mines
    // cmdroll = commanded_roll;
    nav_roll_deg = constrain(commanded_roll, -36, 36);
    // update_load_factor();
}

/** calculate pitch from speed and height controller in auto mode */
void calc_nav_pitch()
{
    // float demanded_pitch;
    // if (get_pitch_demand() < -20){
    //     demanded_pitch = 0;
    // }
    // else {
    //     demanded_pitch = get_pitch_demand();
    // }
    
    nav_pitch_deg = constrain(get_pitch_demand(), pitch_limit_min, pitch_limit_max);
}

void update_load_factor(void)
{
    float roll_limit_cd = HEAD_MAX;
    float demanded_roll = fabsf(nav_roll_deg);
    if (demanded_roll > 85) {
        // limit to 85 degrees to prevent numerical errors
        demanded_roll = 85;
    }
    aerodynamic_load_factor = 1.0f / safe_sqrt(cosf(radians(demanded_roll)));

    float max_load_factor = arspd.v_ms / airspeed_min;
    if (max_load_factor <= 1) {
        // our airspeed is below the minimum airspeed. Limit roll to
        // 25 degrees
        nav_roll_deg = constrain(nav_roll_deg, -25, 25);
        roll_limit_cd = MIN(roll_limit_cd, 25);
    } else if (max_load_factor < aerodynamic_load_factor) {
        // the demanded nav_roll would take us past the aerodynamic
        // load limit. Limit our roll to a bank angle that will keep
        // the load within what the airframe can handle. We always
        // allow at least 25 degrees of roll however, to ensure the
        // aircraft can be manoeuvred with a bad airspeed estimate. At
        // 25 degrees the load factor is 1.1 (10%)
        int32_t roll_limit = degrees(acosf(sq(1.0f / max_load_factor)));
        if (roll_limit < 25) {
            roll_limit = 25;
        }
        nav_roll_deg = constrain(nav_roll_deg, -roll_limit, roll_limit);
        roll_limit_cd = MIN(roll_limit_cd, roll_limit);
    }
    if (isnan(aerodynamic_load_factor) || isinf(aerodynamic_load_factor) || aerodynamic_load_factor < 1.0f) {
        aerodynamic_load_factor = 1.0f;
    }
}

// void update_manual_payload()
// {
//     if (ch_vehicle_mode > 1500) {
//         servo_pwm_payload = 2000; // payload drop
//     } else {
//         servo_pwm_payload = 1000; // payload hold
//     }
// }



// /**
//  * @brief funcction to get uav ground course
// */
// float get_next_ground_course(float default_angle)
// {
//     Locations next_wp_after = waypoint[flag_wp+1];
//     if (next_wp_after.lat==0 && next_wp_after.lng==0)
//     {
//         return default_angle;
//     }
//     return next_WP_loc.get_bearing_to(next_wp_after)/100.0f;
// }

// /**
//  * @brief funcction to setup next angle to turn
// */
// void setup_turn_angle()
// {
//     float next_ground_course = get_next_ground_course(-1);
//     if (next_ground_course == -1)
//     {
//         auto_state.next_turn_angle = 90.0f;
//     }
//     else
//     {
//         float ground_course = prev_WP_loc.get_bearing_to(next_WP_loc) / 100.0f;
//         auto_state.next_turn_angle = wrap_180(next_ground_course - ground_course);
//     }
// }

// /**
//  * @brief funcction to set next waypoint
// */
// void set_next_WP(struct Locations &loc)
// {
//     if (auto_state.next_wp_crosstrack)
//     {
//         prev_WP_loc = next_WP_loc;
//         auto_state.crosstrack = true;
//     }
//     else
//     {
//         prev_WP_loc = current_loc;
//         auto_state.crosstrack = false;
//         auto_state.next_wp_crosstrack = true;
//     }
//     next_WP_loc = loc;
//     setup_turn_angle();
// }



// /*
// //currently ignore

// void updateModes()
// {
//     if (mode_manual){
//         updateManual_FW();
//         MODE = manu->mode();
//     }       
//     else if (mode_fbwa_plane){
//         updateFBWA_FW();
//         MODE = fbwa->mode();
//     }
// }
// */

#endif