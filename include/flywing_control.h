#ifndef flywing_control_h
#define flywing_control_h
#include <Arduino.h>

#include "AHRS.h"
#include "ControlModes.h"
#include "Copter_Config.h"
#include "Radio.h"
#include "TECS.h"   
#include "L1_Controller.h"

// NYT (no yaw torque)
const double invers_matrix[2][2] = {{-1, -1},
                                       {-1, 1}};

class FlywingControl {
   public:
    float u2, u3;
    float servos[2];
    void flywing_getIntegral(int16_t ch_thr, float roll, float pitch, float yaw);
    void flywing_control_stabilize(int16_t ch_r, int16_t ch_p, int16_t ch_y, int16_t ch_thr, 
                                    float roll, float pitch, float yaw, float gy, float gx, float gz);
    void flywing_control_auto(int16_t ch_r, int16_t ch_p, int16_t ch_y, int16_t ch_thr, 
                                    float roll, float pitch, float yaw, float gy, float gx, float gz);
    struct Gains_flywing {
        float k_alt             = 2.0f;    
        float k_z_velocity      = 3.0f;     
        float k_roll            = 12.00f;   //241.5f; 324 474 474 234 274 ;245 250 246 238 239.9 //238 241 244.9
        float k_pitch           = 12.00f; // 6 //36.0f;   //175.0f;  334 434 484 223 ;182 188 190 193 //182 186 181 179.5 // 180 185.4 178.4 179.8 180.8 179 177.5 174.7 172.9 170.0 176.9 180 183 181.5
        float k_yaw             = 3.99f;   // 2400 2500 //950
        float k_z_vel           = 1.0f;     
        float k_roll_rate       = 1.1581f; //1.7f;        //3415.75f; 5987.8 6500 4500 4100 3790 3650 3610 3624.8 3628.8
        float k_pitch_rate      = 0.4681f;  //2.9374f; //2.8712f;  //2733.50f;  // 3174.8 3500 2400 2100 1790 1500 1200 1320 1298 1340 1480 1598 1550 2500 3000.8 3700 3580 3690 3250 3080.8 2860 2840 2788 2800
        float k_yaw_rate        = 0.0001f;     // 11 2980
        float k_i_roll          = 0.0f;     //
        float k_i_pitch         = 0.0f;   // 0.022
        float k_i_yaw           = 0.0f;    // 0.05
        // 15.05: 08.05
        int16_t roll_rmt;
        int16_t pitch_rmt;
        int16_t yaw_rmt;
    }; Gains_flywing gain_flywing;

    private:
    unsigned long flytnow, flytbefore;
    float flytdelta;
    float roll_i, pitch_i;
    unsigned long calc_time, last_calc_time;
    float delta_calc_time;
    float alt_ref, heading_now, last_alt, last_heading, alt_now;
    float alt_target, z_velocity;
    float roll_cmd, pitch_cmd, yaw_cmd;
    struct Limits {
        float flywing_min_roll = -45.0f;
        float flywing_max_roll = 45.0f;
        float flywing_min_pitch = -35.0f;
        float flywing_max_pitch = 35.0f;
        float demanded_pitch_min = -20.0f;
        float demanded_pitch_max = 20.0f;
        int PID_max_roll = 400;
        int PID_max_pitch = 400;
        int PID_max_yaw = 400;
        int PID_min_roll = -400;
        int PID_min_pitch = -400;
        int PID_min_yaw = -400;
        float trim_roll = 0.0f;  
        float trim_pitch = 0.0f;  
        float trim_yaw = 0.0f;
    }; Limits limits;
};

void FlywingControl::flywing_getIntegral(int16_t ch_thr, float roll, float pitch, float yaw) {
    this->flytbefore = this->flytnow;
    this->flytnow = millis();
    this->flytdelta = (this->flytnow - this->flytbefore) / 1000.0f;
    this->roll_i += this->gain_flywing.k_i_roll * roll * this->flytdelta;
    this->pitch_i += this->gain_flywing.k_i_pitch * pitch * this->flytdelta;
    this->roll_i = constrain(this->roll_i, -15, 15);
    this->pitch_i = constrain(this->pitch_i, -15, 15);
    // Reset the integrators
    if (ch_thr < 1100) {
        this->roll_i = 0.0f;
        this->pitch_i = 0.0f;
    }
}

void FlywingControl::flywing_control_stabilize(int16_t ch_r, int16_t ch_p, int16_t ch_y, int16_t ch_thr, float roll, float pitch, float yaw, float gy, float gx, float gz) {
    // last_calc_time = calc_time;
    // calc_time = micros();
    // delta_calc_time = (calc_time - last_calc_time) / 1000000.0f;
    // heading_now = yaw;
    // this->roll_cmd = 1.3*(map(ch_r - 1500, min_roll_corr, max_roll_corr, limits.flywing_min_roll, limits.flywing_max_roll));
    // alt_now = baro.altitude;
    flywing_getIntegral(ch_thr, roll, pitch, yaw);
    // copter_getIntegral(ch_thr, roll, pitch, yaw);

    // this->roll_cmd = -1.3*(map(ch_r - 1500, min_roll_corr, max_roll_corr, this->limits.flywing_min_roll, this->limits.flywing_max_roll));
    // this->pitch_cmd = 1.1*(map(ch_p - 1500, min_pitch_corr, max_pitch_corr, this->limits.flywing_min_pitch, this->limits.flywing_max_pitch));
    // this->u2 = ((-this->gain_flywing.k_roll * (roll + this->roll_i - (this->roll_cmd + trim_roll))) + (-this->gain_flywing.k_roll_rate * (gy)));
    // this->u3 = (-this->gain_flywing.k_pitch * ((-pitch) + this->pitch_i - (this->pitch_cmd + trim_pitch))) + (-this->gain_flywing.k_pitch_rate * (gx));
    // this->servos[0] = (invers_matrix[0][0] * this->u2 + invers_matrix[0][1] * this->u3);
    // this->servos[1] = (invers_matrix[1][0] * this->u2 + invers_matrix[1][1] * this->u3);
    // this->last_heading = this->heading_now;
    // last_alt = alt_setpoint;c
}

void FlywingControl::flywing_control_auto(int16_t ch_r, int16_t ch_p, int16_t ch_y, int16_t ch_thr, float roll, float pitch, float yaw, float gy, float gx, float gz) {
    this->last_calc_time = this->calc_time;
    this->calc_time = micros();
    this->delta_calc_time = (this->calc_time - this->last_calc_time) / 1000000.0f;
    this->heading_now = yaw;
    this->alt_ref = baro.altitude;
    this->alt_now = baro.altitude;
    flywing_getIntegral(ch_thr, roll, pitch, yaw);

    float demanded_pitch;

    demanded_pitch = constrain(get_pitch_demand(), this->limits.demanded_pitch_min, this->limits.demanded_pitch_max);

    if (get_pitch_demand() < -35){
        demanded_pitch = 0;
    }else{
        demanded_pitch = get_pitch_demand();
    }
    this->roll_cmd = 1.3*(constrain((nav_roll_cd())/100, this->limits.flywing_min_roll, this->limits.flywing_max_roll));
    this->pitch_cmd = 1.1*(constrain(demanded_pitch, this->limits.flywing_min_pitch, this->limits.flywing_max_pitch));
    this->u2 = ((-this->gain_flywing.k_roll * (roll + this->roll_i - (this->roll_cmd + this->limits.trim_roll))) + (-this->gain_flywing.k_roll_rate * (gy)));
    this->u3 = (-this->gain_flywing.k_pitch * ((-pitch) + this->pitch_i - (-this->pitch_cmd + this->limits.trim_pitch))) + (-this->gain_flywing.k_pitch_rate * (gx));
    this->servos[0] = (invers_matrix[0][0] * this->u2 + invers_matrix[0][1] * this->u3);
    this->servos[1] = (invers_matrix[1][0] * this->u2 + invers_matrix[1][1] * this->u3);
    this->last_heading = this->heading_now;
    // last_alt = alt_setpoint;
}

FlywingControl flywingcontrol;

#endif