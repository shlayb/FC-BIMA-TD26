#pragma once
//NOTE:

//85-90 need airspeed

#include <Arduino.h>
#include "AP_Math.h"
#include "AHRS.h"
#include "FW_config.h"
#include "FW_controller.h"
#include "Control_data.h"
#include "Radio.h"
// #include "airspeed.h"

struct pitch_gains{
    float pitch_max_rate    = 60.0f;
    float pitch_P           = 1.57f; //0.717 0.818 0.80 1.90
    float pitch_D           = 0.2f;
    float pitch_tau         = 0.45f;
    float pitch_I           = 0.0f;
    float pitch_imax        = 30.0f;
    float _roll_ff          = 1.0f;
    float p_FF;
};
pitch_gains p_gain;

float pitch_get_coordination_rate_offset(float &aspeed)
{
    float rate_offset;
    float bank_angle = imu.roll; //sudut wahana saat itu dlm posisi roll (bank ke kanan/kiri, didekatkan melalui sudut roll. sbnernya selisih bank sm roll itu dikit bgt)
    //knp pake roll? kondisi dmn pesawat ngepitch tp ngeroll jg
    if(fabsf(bank_angle) < 90.0f){
        bank_angle = constrain(bank_angle, -80.0f, 80.0f);
    }
    //check old program if airspeed is available
    aspeed = 0.5f*(float(MIN_AIRSPEED) + float(MAX_AIRSPEED));
    if(abs(imu.pitch) > 70){
        rate_offset = 0;
    }
    else{
        rate_offset = cosf(radians(imu.pitch))*fabsf(degrees((GRAVITY_MSS / MAX((aspeed * baro.getEAS2TAS()) , float(MIN_AIRSPEED))) * tanf(radians(bank_angle)) * sinf(radians(bank_angle)))) * p_gain._roll_ff; 
    }
    return rate_offset;
}

float pitch_control_bodyrate(const control_data &ctl_data)
//koordinat termasuk ketinggian, nah ini utk deteksi perubahan kecepatan pitch, utk mengendalikan ratenya
{
    if(isnan(ctl_data.body_y_rate) && isnan(ctl_data.pitch_rate_setpoint)){
        return _bodyrate_error;
    }
    uint32_t dt_micros = micros() - _time_last_run; //elapsed time
	bool _lock_integrator = ctl_data.lock_integrator;
	if (dt_micros > 1000000 || _time_last_run == 0)
	{
		_lock_integrator = true;
		dt_micros = 0;
	}
	_time_last_run = micros();
    float dt = (float)dt_micros * 1e-6f;
	float desired_rate = _rate_setpoint;
	float achieved_rate = ctl_data.body_y_rate; 
    float aspeed;
    //if (is_airspeed sensor enbled()){
    //     aspeed = get_estimated_airspeed();
    // }
    // else{
        aspeed = 0;
    // }

    _rate_error = (desired_rate - achieved_rate) * ctl_data.speed_scaler;
    if(!_lock_integrator && p_gain.pitch_I > 0){
        float k_I = p_gain.pitch_I;
        if (is_zero(p_gain.p_FF)){
            k_I = MAX(k_I, 0.15);
        }
        float ki_rate = k_I * p_gain.pitch_tau;
        if(dt > 0 && aspeed > 0.5f*MIN_AIRSPEED){
            float integrator_delta = _rate_error * ki_rate * dt * ctl_data.speed_scaler;
            if (_last_output < -45){
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
    float intLimit = p_gain.pitch_imax;
    _integrator = constrain(_integrator, -intLimit, intLimit);
    float eas2tas = baro.getEAS2TAS(); //airspeed
    float kp_ff = MAX((p_gain.pitch_P - p_gain.pitch_I*p_gain.pitch_tau)*p_gain.pitch_tau-p_gain.pitch_D,0)/eas2tas; //airspeed
    float k_ff = p_gain.p_FF/eas2tas; //airspeed

    float P_value = desired_rate * kp_ff * ctl_data.speed_scaler;
    float FF_value = desired_rate * k_ff * ctl_data.speed_scaler;
    float D_value = D_value * p_gain.pitch_D * ctl_data.speed_scaler;

    _bodyrate_error = P_value + D_value + FF_value;
    _bodyrate_error += _integrator;
    _last_output = _bodyrate_error;

    float _u = constrain(_bodyrate_error, -45, 45);
    return _u;
}

float pitch_control_attitude(const control_data &ctl_data)
//utk memperoleh data set point dan error
{
    if((isnan(ctl_data.pitch_setpoint)&&
        isnan(ctl_data.pitch_data)&&
        isnan(ctl_data.airspeed)))
    {
        return _rate_setpoint;
    }
    float pitch_error = ctl_data.pitch_setpoint - ctl_data.pitch_data;
    float aspeed;
    float rate_offset;
    rate_offset = pitch_get_coordination_rate_offset(aspeed);
    _rate_setpoint = pitch_error/p_gain.pitch_tau;
    _rate_setpoint += rate_offset;
    return (-(pitch_control_bodyrate(ctl_data)));
}

void reset_integrator_pitch()
{
    _integrator = 0;
}

float get_gain_K_pitch() {
    return p_gain.pitch_P;
}

float get_gain_K_gyro_pitch() {
    return p_gain.pitch_D;
}

void set_gain_pitch(float Kp_pitch, float K_gyro_pitch) {
    p_gain.pitch_P = Kp_pitch;
    p_gain.pitch_D = K_gyro_pitch;
}
