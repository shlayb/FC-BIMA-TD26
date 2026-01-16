#ifndef MESSAGE_CONTROLDATA_H
#define MESSAGE_CONTROLDATA_H

#include <stdint.h>

/** cant get proper enum._. (ignore)*/
typedef enum control_status { 
    UNDERSHOOT   = 0,
    OVERSHOOT    = 1,
    UNDUMPED     = 1 << 1
} ctrl_status_t;

struct control_data{
	float speed_scaler;
	float roll_data;
	float pitch_data;
	float yaw_data;
	float body_x_rate; 			//gyro x
	float body_y_rate; 			//gyro y
	float body_z_rate; 			//gyro z
	float roll_setpoint;
	float pitch_setpoint;
	float yaw_setpoint;
	uint16_t plane_thr;
	float roll_rate_setpoint; 	//setpoint angular roll
	float pitch_rate_setpoint; 	//setpoint angular pitch
	float yaw_rate_setpoint; 	//setpoint angular yaw
	float airspeed_min;
	float airspeed_max;
	float airspeed;
	float groundspeed;
	float mixGain = 1.0f; 		//ignore
	bool lock_integrator; 		//FW
	float weight;
	bool status = false;
}plane_control_t;

typedef struct control_gain{
	float _kp;
	float _ki;
	float _kd;
}ctrl_gain_t;

typedef struct tune_data {
	ctrl_gain_t roll;
	ctrl_gain_t pitch;
	ctrl_gain_t yaw;
	float trim_roll;
	float trim_pitch;
	float trim_yaw;
} tune_data_t;





#endif