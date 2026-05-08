//PWM
#define PWM_MIN 988
#define PWM1_MIN 1047
#define PWM2_MIN 1047
#define PWM3_MIN 1047
#define PWM4_MIN 1047
#define PWM_MAX 2012

//fbwastate
#define max_roll_limit = 75;
#define min_roll_limit = -75;
#define max_pitch_limit = 30;
#define min_pitch_limit = -30;
#define max_yaw_limit = 20;
#define min_yaw_limit = -20;

/** SERVO MAPPING */
#ifndef THROTTLE_MIN
#define THROTTLE_MIN 0 // percent
#endif
#ifndef THROTTLE_MAX
#define THROTTLE_MAX 100
#endif
#ifndef MIN_THROTTLE_PWM
#define MIN_THROTTLE_PWM 1000 //PWM
#endif
#ifndef MAX_THROTTLE_PWM
#define MAX_THROTTLE_PWM 2000
#endif

#define M_CONST 1.25

#define MOTOR_1 7 //rol kiri
#define MOTOR_2 8 //pitch depan
#define MOTOR_3 9 //roll kanan
#define MOTOR_4 10 //pitch belakang