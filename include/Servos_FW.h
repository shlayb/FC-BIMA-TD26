// #pragma once
// #include <Arduino.h>
// #include <Servo.h>
// #include "FW_ControlModes.h"
// #include "Attitude.h"

// #define CALIB_MOTOR false
// #define SERVO_AIL_L 2
// #define SERVO_ELE   3
// #define THROTTLE    4
// #define SERVO_RUD_L 5
// #define SERVO_AIL_R 6
// #define SERVO_RUD_R 9

// Servo aileron_L;
// Servo aileron_R;
// Servo elevator;
// Servo rudder_L;
// Servo rudder_R;
// Servo pusher;

// float u_roll, u_pitch, u_yaw, u_yaw_cmd;

// // uint16_t servo_trim;
// float high_out = 90;
// uint16_t servo_max = 2012;
// uint16_t servo_min = 988;

// // scale pwm to -45 until 45
// float scaleToAngle(uint16_t ch)
// {
//     float scaledOutput = 0.0879*ch - 131.46;
//     return scaledOutput;
// }

// // scale pwm to 0 until 100
// float scaleToPercent(uint16_t ch)
// {
//     float scaledOutput = 0.0976*ch - 96.067;
//     return scaledOutput;
// }

// uint16_t angleToPwm(float angle/*, uint16_t servo_trim*/)
// {
//     uint16_t servo_out;
//     servo_out = (11.378*angle)+1500;
//     return servo_out;
// }

// uint16_t angleToPwm_reversed(float angle/*, uint16_t servo_trim*/)
// {
//     uint16_t servo_out;
//     servo_out = (11.378*(-angle))+1500;
//     return servo_out;
// }

// void servo_idle()
// {
//     servo_pwm_ail_L = 0;
//     servo_pwm_ail_R = 0;
//     servo_pwm_ele = 0;
//     servo_pwm_rud_L = 0;
//     servo_pwm_rud_R = 0;
// }

// // void angleToPwm_all()
// // {
// //     servo_pwm_ail_L = angleToPwm(u_roll,1500);
// //     servo_pwm_ail_R = angleToPwm(u_roll,1500);
// //     servo_pwm_ele   = angleToPwm(u_pitch,1500);
// //     servo_pwm_rud_L = angleToPwm(u_yaw_cmd,1500);
// //     servo_pwm_rud_R = angleToPwm(u_yaw_cmd,1500);
// // }

// void init_servo()
// {
//     pinMode(SERVO_AIL_L, OUTPUT);
//     pinMode(SERVO_ELE, OUTPUT);
//     pinMode(SERVO_RUD_L, OUTPUT);
//     pinMode(SERVO_AIL_R, OUTPUT);
//     pinMode(SERVO_RUD_R, OUTPUT);

//     aileron_L.attach(SERVO_AIL_L);
//     elevator.attach(SERVO_ELE);
//     rudder_L.attach(SERVO_RUD_L);
//     aileron_R.attach(SERVO_AIL_R);
//     rudder_R.attach(SERVO_RUD_R);

//     // servo_idle();    
// }

// void init_motor(){
//     pusher.attach(THROTTLE);
//     if(CALIB_MOTOR)
//     {
//         pusher.writeMicroseconds(2000);
//         delay(2000);
//         pusher.writeMicroseconds(1000);
//         delay(2000);
//     }
//     pusher.writeMicroseconds(1000);
//     delay(1000);
    
// }