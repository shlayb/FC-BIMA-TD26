#pragma once

// ============================================================
//  HARDWARE-IN-THE-LOOP (HITL) CONFIG
//  Set to 1 to simulate via USB Python Bridge, 0 for real flight
// ============================================================
#ifndef HITL_ENABLED
#define HITL_ENABLED 0
#endif

// buat misi auto
#ifndef WP_RADIUS_DEFAULT
#define WP_RADIUS_DEFAULT 40
#endif

// loitering: wahana berputar2 di wp tertentu
#ifndef LOITER_RADIUS_DEFAULT
#define LOITER_RADIUS_DEFAULT 50
#endif

inline float rtl_altitude = 60.0f; // meter
#ifndef ALT_HOLD_HOME
#define ALT_HOLD_HOME rtl_altitude
#endif
#define ALT_HOLD_HOME_CM (rtl_altitude * 100.0f)

#ifndef USE_CURRENT_ALT
#define USE_CURRENT_ALT FALSE
#endif

/** SERVO MAPPING */
#ifndef THROTTLE_MIN
#define THROTTLE_MIN 0  // percent
#endif
#ifndef THROTTLE_CRUISE
#define THROTTLE_CRUISE 60  // mode auto (percent) — harus sinkron dengan tecs_cruise_throttle!
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
inline float tecs_cruise_throttle = 70.0f; // % throttle saat terbang datar
inline float tecs_min_throttle    = 30.0f; // % throttle minimal saat menukik
inline float tecs_max_throttle    = 85.0f; // % throttle maksimal saat menanjak
inline float min_gndspeed         = 17.0f; // m/s (safety floor angin sakal)

#ifndef TECS_MAX_THROTTLE
#define TECS_MAX_THROTTLE 85
#endif
#ifndef TECS_CRUISE_THROTTLE
#define TECS_CRUISE_THROTTLE 70
#endif
#ifndef TECS_MIN_THROTTLE
#define TECS_MIN_THROTTLE 60
#endif
#ifndef TECS_TIME_CONST
#define TECS_TIME_CONST 8
#endif

/** FLY BY WIRE AIRSPEED */
inline float AIRSPEED_CRUISE = 18.0f;  // kecepatan jelajah nominal (m/s)
inline float MIN_AIRSPEED    = 15.5f;  // margin aman ~20% di atas AIRSPEED_STALL (m/s)
inline float MAX_AIRSPEED    = 22.0f;  // batas atas struktural (m/s)
inline float AIRSPEED_STALL  = 13.0f;  // kecepatan stall airframe (m/s)

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
#define MIN_GNDSPEED 17.0f  // m/s safety floor
#endif
#define MIN_GNDSPEED_CM (min_gndspeed * 100)

/** AUTOPILOT CONTROL LIMITS */
inline float roll_max_deg  = 36.0f;  // sudut max roll auto / fbwa
inline float pitch_max_deg = 15.0f;  // sudut max nanjak
inline float pitch_min_deg = -10.0f; // sudut max menukik

#ifndef HEAD_MAX
#define HEAD_MAX 36
#endif
#ifndef PITCH_MAX
#define PITCH_MAX 15
#endif
#ifndef PITCH_MIN
#define PITCH_MIN -10
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