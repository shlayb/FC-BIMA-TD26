#include <Arduino.h>//lagi apa bg
// #include "MPU6050.h"
#include "FW_config.h"
// #include "Barometer.h"
#include "AP_Math.h"
#include "AHRS.h"
#include "FW_controller.h"
#include "Control_data.h"

float _last_out;
float _last_rate_hp_out;
float _last_rate_hp_in;
float _K_D_last;

struct yaw_gains{
    float yaw_max_rate    = 50.0f;
    float yaw_K_A         = 0.05f;
    float yaw_K_I         = 0.005f;
    float yaw_K_D         = 0.01f;
    float yaw_K_FF        = 1.0f;
    float yaw_imax        = 15.0f;
    float r_FF;

    float _last_out;
	float _last_rate_hp_out;
	float _last_rate_hp_in;
	float _K_D_last;
	float _imax;
};
yaw_gains y_gain;

float yaw_control_bodyrate(const control_data &ctl_data)
{
    if((isnan(ctl_data.body_z_rate) &&
        isnan(ctl_data.yaw_rate_setpoint)))
    {
        return _bodyrate_error;
    }
    uint32_t dt_micros = micros() - _time_last_run;
    bool lock_integrator = ctl_data.lock_integrator;
    if(dt_micros > 1000000 || _time_last_run == 0){
        dt_micros = 0;
        lock_integrator = true;
    }
    _time_last_run = micros();
    float dt = (float)dt_micros*1e-6f;

    _rate_error = _rate_setpoint - ctl_data.body_z_rate;

    float aspeed;
    float rate_offset;
    float bank_angle = imu.roll*DEG_TO_RAD;

    if(fabsf(bank_angle) < 1.5707964f){
        bank_angle = constrain(bank_angle, -1.3962634f, 1.3962634f); //dr tahun lalu
    }
    // if (!ahrs().is_airspeed_sensor_enabled())
	// {
		// If no airspeed available use average of min and max
	aspeed = 0.5f * (float(plane_control_t.airspeed_min) + float(plane_control_t.airspeed_max));
	// }
    // else
    // {
    //     aspeed = ahrs().get_airspeed_estimated();
    // }
    rate_offset = (GRAVITY_MSS/MAX(aspeed, float(plane_control_t.airspeed_min))) * sinf(bank_angle) * y_gain.yaw_K_FF;

    float omega_z = ctl_data.body_z_rate;
    float accel_y = imu._accel.y; //ayg
    float rate_hp_in = omega_z - (rate_offset*RAD_TO_DEG);

    float rate_hp_out = 0.9960080f * _last_rate_hp_out + rate_hp_in - _last_rate_hp_in;
    _last_rate_hp_out = rate_hp_out;
    _last_rate_hp_in = rate_hp_in;

    float integ_in = -y_gain.yaw_K_I * (y_gain.yaw_K_A * accel_y + rate_hp_out); 

    if (!lock_integrator && y_gain.yaw_K_D > .0f){
        if (aspeed > plane_control_t.airspeed_min){
            if(_last_output < -45){
                _integrator += MAX(integ_in * dt, 0);
            }
            else if (_last_output > 45){
                _integrator += MIN(integ_in * dt, 0);
            }
            else{
                _integrator += integ_in * dt;
            }
            
        }
    }
    else{
        _integrator = 0;
    }

    if (y_gain.yaw_K_D < 0.0001f)
    {
        return 0;
    }

    float intLimitScale = y_gain.yaw_imax /(y_gain.yaw_K_D * ctl_data.speed_scaler * ctl_data.speed_scaler);

    _integrator = constrain(_integrator, -intLimitScale, intLimitScale);

    if (y_gain.yaw_K_D > _K_D_last && y_gain.yaw_K_D > 0)
    {
        _integrator = _K_D_last/y_gain.yaw_K_D * _integrator;
    }
    _K_D_last = y_gain.yaw_K_D;

    float int_val = y_gain.yaw_K_D * _integrator * ctl_data.speed_scaler * ctl_data.speed_scaler;
	float derr_val = y_gain.yaw_K_D * (-rate_hp_out) * ctl_data.speed_scaler * ctl_data.speed_scaler;

    _bodyrate_error = derr_val + int_val;
    _last_output = _bodyrate_error;

    float _u = constrain(_bodyrate_error, -45, 45);
    return _u;
}

float yaw_control_attitude(const control_data &ctl_data)
{
    if((isnan(ctl_data.yaw_setpoint) &&
        isnan(ctl_data.yaw_data) &&
        isnan(ctl_data.roll_data) &&
        isnan(ctl_data.airspeed)))
    {
        return _rate_setpoint;
    }
    float yaw_error = plane_control_t.yaw_setpoint - plane_control_t.yaw_data;
    _rate_setpoint = yaw_error / 0.4;
    return yaw_control_bodyrate(ctl_data);
}

void reset_integrator_yaw()  //gadipake krn pake PD aja jd bs dihapus
{
    _integrator = 0;
}
    