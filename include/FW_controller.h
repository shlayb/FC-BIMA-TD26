#ifndef FW_CONTROLLER_H
#define FW_CONTROLLER_H

#include <Arduino.h>
#include <stdint.h>

float dev_roll;
float dev_pitch;
float dev_yaw;
float roll_lalu;
float pitch_lalu;
float yaw_lalu;
float dev_ail;
float dev_ele;
float dev_rudd;

uint32_t _time_last_run;
float _tc;
float _rate_setpoint;
float _rate_error;
float _bodyrate_error;
float _bodyrate_setpoint;
float _integrator;
float _max_integrator;
float _max_rate;
float _last_output;
float _forward_thrust;
float _speed_scaler;

// float nav_roll_deg;
// float nav_pitch_deg;


void reset_integrator()
{
    _integrator = 0.0f;
}

void set_rate_setpoint(float rate) //ngendaliin kecepatan kembali ke setpoint
{
    _rate_setpoint = constrain(rate,-_max_rate, _max_rate);
}

void set_max_rate(float max_rate) //ignore tp jgn dihapus
{
    _max_rate = max_rate;
}

void set_max_integrator(float max_integrator) //max toleransi integral
{
    _max_integrator = max_integrator;
}

//pure sudut
float get_rate_error(){ //kecepatan perubahan errornya
	return _rate_error;
}

float get_desired_rate(){ //kecepatan mencapai setpoint
	return _rate_setpoint;
}

//bodyrate: lebih ke gyro (rpy, setpoint turunan dr sudut)
float get_bodyrate_error(){ //error kecepatan, buat troubleshoot
	return _bodyrate_error;
}

float get_desired_bodyrate(){ //kecepatan perubahan bodyrate yg diinginkan 
	return _bodyrate_setpoint;
}

float get_integrator(){
	return _integrator;
}

#endif