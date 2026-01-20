#pragma once
// buat misi auto
#ifndef WP_RADIUS_DEFAULT
#define WP_RADIUS_DEFAULT 40
#endif

// loitering: wahana berputar2 di wp tertentu
#ifndef LOITER_RADIUS_DEFAULT
#define LOITER_RADIUS_DEFAULT 50
#endif

#ifndef ALT_HOLD_HOME
#define ALT_HOLD_HOME 100
#endif
#define ALT_HOLD_HOME_CM ALT_HOLD_HOME * 100

#ifndef USE_CURRENT_ALT
#define USE_CURRENT_ALT FALSE
#endif

/** SERVO MAPPING */
#ifndef THROTTLE_MIN
#define THROTTLE_MIN 0  // percent
#endif
#ifndef THROTTLE_CRUISE
#define THROTTLE_CRUISE 25  // mode auto (percent)
#endif
#ifndef THROTTLE_MAX
#define THROTTLE_MAX 100
#endif
#ifndef MIN_THROTTLE_PWM
#define MIN_THROTTLE_PWM 988  // PWM 1000-12
#endif
#ifndef MAX_THROTTLE_PWM
#define MAX_THROTTLE_PWM 2012  // PWM 2000+12
#endif
#ifndef THR_SLEW_RATE
#define THR_SLEW_RATE 100
#endif

/** TECS TUNE */
// total energy control system, ignore
#ifndef TECS_MAX_THROTTLE
#define TECS_MAX_THROTTLE 85
#endif
#ifndef TECS_CRIUSE_THROTTLE
#define TECS_CRUISE_THROTTLE 30
#endif
#ifndef TECS_MIN_THROTTLE
#define TECS_MIN_THROTTLE 30
#endif
#ifndef TECS_TIME_CONST
#define TESC_TIME_CONST 8   /// range 3-10, increment 0.2, semakin kecil semakin liar respon kejar altitude
#endif

/** FLY BY WIRE AIRSPEED */

// AIRSPEED_CRUISE
//
#ifndef AIRSPEED_CRUISE
#define AIRSPEED_CRUISE 12.0f  // 12 m/s
#endif
#ifndef MIN_AIRSPEED
#define MIN_AIRSPEED 9.0f  // m/s (yg jadi batasan buat dipake)
#endif
#ifndef MAX_AIRSPEED
#define MAX_AIRSPEED 22.0f
#endif
#ifndef AIRSPEED_STALL
#define AIRSPEED_STALL 7.f //5-75m/s
#endif

#ifndef SCALING_SPEED
#define SCALING_SPEED 20.0f
#endif
#ifndef FBWB_CLIMB_RATE
#define FBWB_CLIMB_RATE 2.0f  // kecepatan dr bawah sampe ketinggian yg diinginkan
#endif
#ifndef MAX_YAW_ANGLE
#define MAX_YAW_ANGLE 45.0f
#endif

// MIN_GNDSPEED
//
#ifndef MIN_GNDSPEED
#define MIN_GNDSPEED 17.0f  // m/s (0 disables)
// kecepatan wahana utk mencapai 1 wp ke wp lainnya
#endif
#define MIN_GNDSPEED_CM MIN_GNDSPEED * 100

// /** AUTOPILOT CONTROL LIMITS */
// #ifndef HEAD_MAX
// #define HEAD_MAX 55  // sudut max roll
// #endif
// #ifndef PITCH_MAX
// #define PITCH_MAX 55 //45
// #endif
// #ifndef PITCH_MIN
// #define PITCH_MIN -45
// #endif
// #ifndef RUDDER_MIX        // utk pesawat ketika belok, gacuma ael tapi rudder jg biar wahana ga turun ke bawah
// #define RUDDER_MIX -0.5f  // pesawat gerakannya cenderung ke bawah, rud mix untuk antisipasi itu, setengah ini brrti dr keseluruhan respons roll, setengahnya dipake sm ruddernya jg
// #endif

/** AUTOPILOT CONTROL LIMITS */
#ifndef HEAD_MAX
#define HEAD_MAX 36
#endif
#ifndef PITCH_MAX
#define PITCH_MAX 30 //42 35 fix 38 35
#endif
#ifndef PITCH_MIN
#define PITCH_MIN -30 //-20 -30 -40 fix -35 -30 -35
#endif
#ifndef RUDDER_MIX
#define RUDDER_MIX 0.30f
#endif

/** AUTOTAKEOFF CONTROL*/

// ignore
#ifndef ROLL_LIMIT_TKOFF
#define ROLL_LIMIT_TKOFF 15  // 50
#endif

//** AUTO LANDING*/
// ignore
#define USE_LIDAR false

// #define AUTO_WPLAND
//  #define AUTO_GENE_WPLAND
#ifndef AUTO_GENE_WPLAND
#define MANUAL_WPLAND
#endif
// buat landing FW
#define LAND_MAX_THR 75
#define LAND_MIN_THR 5
#define LAND_PITCH_MIN -35  // 2021-06-15 manual landing pitch
#define LAND_PITCH_MAX 25
#define MIN_LAND_ALT 40