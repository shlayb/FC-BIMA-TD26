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

/** TECS TUNE (Non-Airspeed Mode Defaults) */
inline float tecs_cruise_throttle = 45.0f; // % throttle saat terbang datar
inline float tecs_min_throttle    = 18.0f; // % throttle minimal saat menukik
inline float tecs_max_throttle    = 85.0f; // % throttle maksimal saat menanjak
inline float min_gndspeed         = 5.0f;  // m/s (safety floor angin sakal)

#ifndef TECS_MAX_THROTTLE
#define TECS_MAX_THROTTLE 85
#endif
#ifndef TECS_CRUISE_THROTTLE
#define TECS_CRUISE_THROTTLE 45
#endif
#ifndef TECS_MIN_THROTTLE
#define TECS_MIN_THROTTLE 18
#endif
#ifndef TECS_TIME_CONST
#define TECS_TIME_CONST 8
#endif

/** FLY BY WIRE AIRSPEED */
#ifndef AIRSPEED_CRUISE
#define AIRSPEED_CRUISE 14.0f  // 14 m/s nominal
#endif
#ifndef MIN_AIRSPEED
#define MIN_AIRSPEED 9.0f  // m/s
#endif
#ifndef MAX_AIRSPEED
#define MAX_AIRSPEED 22.0f
#endif
#ifndef AIRSPEED_STALL
#define AIRSPEED_STALL 7.0f
#endif

#ifndef SCALING_SPEED
#define SCALING_SPEED 20.0f
#endif
#ifndef FBWB_CLIMB_RATE
#define FBWB_CLIMB_RATE 2.0f
#endif
#ifndef MAX_YAW_ANGLE
#define MAX_YAW_ANGLE 45.0f
#endif

#ifndef MIN_GNDSPEED
#define MIN_GNDSPEED 5.0f  // m/s safety floor
#endif
#define MIN_GNDSPEED_CM (min_gndspeed * 100)

/** AUTOPILOT CONTROL LIMITS */
inline float roll_max_deg  = 35.0f; // sudut max roll auto / fbwa
inline float pitch_max_deg = 18.0f; // sudut max nanjak
inline float pitch_min_deg = -12.0f; // sudut max menukik

#ifndef HEAD_MAX
#define HEAD_MAX roll_max_deg
#endif
#ifndef PITCH_MAX
#define PITCH_MAX pitch_max_deg
#endif
#ifndef PITCH_MIN
#define PITCH_MIN pitch_min_deg
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