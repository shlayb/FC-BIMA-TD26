#pragma once
#include <Arduino.h>
#include <AP_Math.h>
#include "FW_config.h"
#include "AHRS.h"
#include "FW_controller.h"
#include "Control_data.h"
// #include "airspeed.h"

//line 40, 42-44, 73-74, 77 need eas2tas (airspeed)

struct roll_gains{
    float roll_max_rate    = 70.0f;
    float roll_P           = 1.0f; //0.90 //0.80 0.60 0.50 0.40 0.35
    float roll_D           = 0.0f;//0.34f; //0.13 0.40 0.60
    float roll_tau         = 0.50f;
    float roll_I           = 0.0f;
    float roll_imax        = 30.0f;
    float _roll_ff         = 1.0f;
    float r_FF;
};
roll_gains r_gain;

float roll_control_bodyrate(const control_data &ctl_data)
{
    if((isnan(ctl_data.body_x_rate) &&
        isnan(ctl_data.roll_rate_setpoint)))
    {
        return _bodyrate_error;
    }
    uint32_t dt_micros = micros() - _time_last_run;
    bool lock_integrator = ctl_data.lock_integrator;
    if(dt_micros > 1000000 || _time_last_run == 0)
    {
        dt_micros = 0;
        lock_integrator = true;
    }
    _time_last_run = micros();
    float dt = (float)dt_micros*1e-6f;
    float desired_rate = _rate_setpoint;
    float achieved_rate = ctl_data.body_x_rate;
    float ki_rate = r_gain.roll_I*r_gain.roll_tau;
    float eas2tas = baro.getEAS2TAS(); //estimated airspeed to true airspeed
    float kp_ff = MAX((r_gain.roll_P - r_gain.roll_I*r_gain.roll_tau)*r_gain.roll_tau - r_gain.roll_D, 0)/eas2tas;
    float k_ff = r_gain.r_FF/eas2tas;
    //notes : butuh estimasi airspeed
    float aspeed;
    //if (is_airspeed sensor enbled()){
    //     aspeed = get_estimated_airspeed();
    // }
    // else{
        aspeed = 0;
    // }
    _rate_error = (desired_rate - achieved_rate)*ctl_data.speed_scaler;

    if(!lock_integrator && ki_rate > 0){
        if(dt > 0 && aspeed > MIN_AIRSPEED){
            float integrator_delta = _rate_error*ki_rate*dt*ctl_data.speed_scaler;
            if(_last_output < -45){
                integrator_delta = MAX(integrator_delta, 0);
            }
            else if(_last_output > 45){
                integrator_delta = MIN(integrator_delta, 0);
            }
            _integrator += integrator_delta;
        }
    }
    else{
        _integrator = 0;
    }
    float intLimit = r_gain.roll_imax;
    _integrator = constrain(_integrator, -intLimit, intLimit);

    float P_value = desired_rate*kp_ff*ctl_data.speed_scaler;
    float FF_value = desired_rate*k_ff*ctl_data.speed_scaler;
    float D_value = _rate_error*r_gain.roll_D*ctl_data.speed_scaler;

    _bodyrate_error = P_value + D_value + FF_value;
    _bodyrate_error += _integrator;
    _last_output = _bodyrate_error;
    float _u = constrain(_bodyrate_error, -45, 45);
    return _u;
}

float roll_control_attitude(const control_data &ctl_data)
{
    if((isnan(ctl_data.roll_setpoint) &&
        isnan(ctl_data.roll_data) &&
        isnan(ctl_data.airspeed)))
    {
        return _rate_setpoint;
    }
    float roll_error = ctl_data.roll_setpoint - ctl_data.roll_data;
    _rate_setpoint = roll_error/r_gain.roll_tau;
    return roll_control_bodyrate(ctl_data);

}

void reset_integrator_roll()
{
    _integrator = 0;
}

float get_gain_K_roll() {
    return r_gain.roll_P;
}

float get_gain_K_gyro_roll() {
    return r_gain.roll_D;
}

void set_gain_roll(float Kp_roll, float K_gyro_roll) {
    r_gain.roll_P = Kp_roll;
    r_gain.roll_D = K_gyro_roll;
}