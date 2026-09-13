#pragma once
// #include <stdint.h>
#include <Barometer.h>
#include <FW_config.h>
#include <../lib/Math/AP_Math.h>
#include <AHRS.h>
#include <Barometer.h>
#include <../lib/Math/Filter/AverageFilter.h>
#include <bno055.h>
#include "quaternion.h"
#include <../lib/Math/Filter/LowPassFilter.h>

/**
 * @file    AP_TECS.h
 * @brief   Combined Total Energy Speed & Height Control. This is a instance of an AP_SpdHgtControl class
 * @addtogroup Modules
 */

/*
 *  Written by Paul Riseborough 2013 to provide:
 *  - Combined control of speed and height using throttle to control
 *    total energy and pitch angle to control exchange of energy between
 *    potential and kinetic.
 *    Selectable speed or height priority modes when calculating pitch angle
 *  - Fallback mode when no airspeed measurement is available that
 *    sets throttle based on height rate demand and switches pitch angle control to
 *    height priority
 *  - Underspeed protection that demands maximum throttle switches pitch angle control
 *    to speed priority mode
 *  - Relative ease of tuning through use of intuitive time constant, trim rate and damping parameters and the use
 *    of easy to measure aircraft performance data
 */

    int tecs_min_throtlle = 30;
    int tecs_max_throttle = 85;

    // Last time update_50Hz was called
    uint64_t _update_50hz_last_usec;

    // Last time update_speed was called
    uint64_t _update_speed_last_usec;

    // Last time update_pitch_throttle was called
    uint64_t _update_pitch_throttle_last_usec;
    
    // TECS tuning parameters
    float _hgtCompFiltOmega   = 5.0f;//3.5f;
    float _spdCompFiltOmega   = 2.0f;
    float _maxClimbRate       = 5.0f;
    float _minSinkRate        = 2.0f;
    float _maxSinkRate        = 5.0f;
    float _timeConst          = 5.0f;//3.275f; 
    float _landTimeConst      = 2.0f;
    float _ptchDamp           = 0.0f;
    float _land_pitch_damp    = 0;
    float _landDamp           = 0.5f;
    float _thrDamp            = 0.5f;
    float _land_throttle_damp = 0;
    float _integGain          = 0.1f;
    float _integGain_takeoff  = 0;
    float _integGain_land     = 0;
    float _vertAccLim         = 7.0f;
    float _rollComp           = 10.0f;
    float _spdWeight          = 1.0f;
    float _spdWeightLand      = -1.0f;
    float _landThrottle       = -1;
    float _landAirspeed       = -1;
    float _land_sink          = 0.25f;
    float _land_sink_rate_change;
    int8_t  _pitch_max      = 0;
    int8_t  _pitch_min      = 0;
    int8_t  _land_pitch_max = 10;
    float _maxSinkRate_approach = 0;

    // temporary _pitch_max_limit. Cleared on each loop. Clear when >= 90
    int8_t _pitch_max_limit = 90;
    
    // current height estimate (above field elevation)
    float _height;

    // throttle demand in the range from -1.0 to 1.0, usually positive unless reverse thrust is enabled via _THRminf < 0
    float _throttle_dem;

    // pitch angle demand in radians
    float _pitch_dem;

    // estimated climb rate (m/s)
    float _climb_rate;

    /*
      a filter to estimate climb rate if we don't have it from the EKF
     */
    struct {
        // height filter second derivative
        float dd_height;

        // height integration
        float height;
    } _height_filter;

    // Integrator state 4 - airspeed filter first derivative
    float _integDTAS_state;

    // Integrator state 5 - true airspeed
    float _TAS_state;

    // Integrator state 6 - throttle integrator
    float _integTHR_state;

    // Integrator state 6 - pitch integrator
    float _integSEB_state;

    // throttle demand rate limiter state
    float _last_throttle_dem;

    // pitch demand rate limiter state
    float _last_pitch_dem;

    // Rate of change of speed along X axis
    float _vel_dot;

    // Equivalent airspeed
    float _EAS;

    // True airspeed limits
    float _TASmax;
    float _TASmin;

    // Current true airspeed demand
    float _TAS_dem;

    // Equivalent airspeed demand
    float _EAS_dem;

    // height demands
    float _hgt_dem;
    float _hgt_dem_in_old;
    float _hgt_dem_adj;
    float _hgt_dem_adj_last;
    float _hgt_rate_dem;
    float _hgt_dem_prev;
    float _land_hgt_dem;

    // Speed demand after application of rate limiting
    // This is the demand tracked by the TECS control loops
    float _TAS_dem_adj;

    // Speed rate demand after application of rate limiting
    // This is the demand tracked by the TECS control loops
    float _TAS_rate_dem;

    // Total energy rate filter state
    float _STEdotErrLast;

    typedef struct {
        bool underspeed:1;
        bool badDescent:1;
        // true when a reset of airspeed and height states to current is performed on this frame
        bool reset:1;
    } flags;

    typedef union {
        flags _flags;
        uint8_t _flags_byte;
    } flags_union;

    flags_union my_flags;

    enum class clipStatus : int8_t {
        MIN  = -1,
        NONE =  0,
        MAX  =  1,
    };
    clipStatus _thr_clip_status;

    float _integKE;

    // energy balance error integral
    float _integSEBdot;
    
    // 1 when specific energy balance rate demand is clipping in the up direction
    // -1 when specific energy balance rate demand is clipping in the down direction
    // 0 when not clipping
    clipStatus _SEBdot_dem_clip;


    // time when underspeed started
    uint32_t _underspeed_start_ms;

    // pitch demand before limiting
    float _pitch_dem_unc;

    // Maximum and minimum specific total energy rate limits
    float _STEdot_max;
    float _STEdot_min;

    // Maximum and minimum floating point throttle limits
    float _THRmaxf;
    float _THRminf;

    // Maximum and minimum floating point pitch limits
    float _PITCHmaxf;
    float _PITCHminf;

    // Specific energy quantities
    float _SPE_dem;
    float _SKE_dem;
    float _SPEdot_dem;
    float _SKEdot_dem;
    float _SPE_est;
    float _SKE_est;
    float _SPEdot;
    float _SKEdot;

    // Specific energy error quantities
    float _STE_error;

    // Time since last update of main TECS loop (seconds)
    float _DT;

    // counter for demanded sink rate on land final
    uint8_t _flare_counter;

    // slew height demand lag filter value when transition to land
    float hgt_dem_lag_filter_slew;

    // percent traveled along the previous and next waypoints
    float _path_proportion;

    float _distance_beyond_land_wp;

    float _land_pitch_min = -90;

    float _SKE_weighting;

    // Speed rate demand after application of rate limiting
    // This is the demand tracked by the TECS control loops
    // float _TAS_rate_dem;
    float _TAS_rate_dem_lpf;

    // Rate of change of speed along X axis
    // float _vel_dot;
    float _vel_dot_lpf;

    // internal variables to be logged
    struct {
        float SKE_weighting;
        float SPE_error;
        float SKE_error;
        float SEB_delta;
    } logging;

     // aerodynamic load factor
    float _load_factor;

     // low pass filters used for crossover filter that combines demanded and measured pitch
    // when calculating a pitch to throttle mapping.
    LowPassFilterFloat _pitch_demand_lpf;
    LowPassFilterFloat _pitch_measured_lpf;

    int8_t _use_synthetic_airspeed = false;
    
    // use synthetic airspeed for next loop
    bool _use_synthetic_airspeed_once = false;
    
    // declares a 5point average filter using floats
    AverageFilterFloat_Size5 _vdot_filter;
    
    int32_t get_throttle_demand(void) {
        return int32_t(_throttle_dem * 100.0f);
    }

    // demanded pitch angle in centi-degrees
    // should return between -9000 to +9000
    float get_pitch_demand(void) {

        return _pitch_dem * 57.295781f; // 180/PI
        }
    
    

    // Rate of change of velocity along X body axis in m/s^2
    float get_VXdot(void) {
        return _vel_dot;
    }

    // return current target airspeed
    float get_target_airspeed(void) {
        return _TAS_dem / baro.getEAS2TAS();
    }

    // return maximum climb rate
    float get_max_climbrate(void) {
        return _maxClimbRate;
    }

    // added to let SoaringContoller reset pitch integrator to zero
    void reset_pitch_I(void) {
        _integSEB_state = 0.0f;
    }
    
    // return landing sink rate
    float get_land_sinkrate(void) {
        return _land_sink;
    }

    // return landing airspeed
    float get_land_airspeed(void) {
        return _landAirspeed;
    }

    // return height rate demand, in m/s
    float get_height_rate_demand(void) {
        return _hgt_rate_dem;
    }

    // set path_proportion
    void set_path_proportion(float path_proportion) {
        _path_proportion = constrain(path_proportion, 0.0f, 1.0f);
    }

    // set pitch max limit in degrees
    void set_pitch_max_limit(int8_t pitch_limit) {
        _pitch_max_limit = pitch_limit;
    }

    // force use of synthetic airspeed for one loop
    void use_synthetic_airspeed(void) {
        _use_synthetic_airspeed_once = true;
    }

    void set_tecs_min_throttle(int value){
        tecs_min_throtlle = value;
    }
    void set_tecs_max_throttle(int value){
        tecs_max_throttle = value;
    }
    int get_tecs_min_throttle(){
        return tecs_min_throtlle;
    }
    int get_tecs_max_throttle(){
        return tecs_max_throttle;
    }

float timeConstant(void)
{    
    if (_timeConst < 0.1f) {
        return 0.1f;
    }
    return _timeConst;
}

/*
  current time constant. It is lower in landing to try to give a precise approach
 */

void _update_speed(float DT)
{
    // calculate a low pass filtered _vel_dot
    if (my_flags._flags.reset) {
        _vdot_filter.reset();
        _vel_dot_lpf = _vel_dot;
    } else {
        // Get DCM
        const Matrix3f &rotMat = ahrs.get_rotation_body_to_ned();
        // Calculate speed rate of change
        float temp = rotMat.c.x * GRAVITY_MSS + imu._accel.x;
        // take 5 point moving average
        _vel_dot = _vdot_filter.apply(temp);
        float alpha = DT / (DT + timeConstant());
        _vel_dot_lpf = _vel_dot_lpf * (1.0f - alpha) + _vel_dot * alpha;
    }

    bool use_airspeed = _use_synthetic_airspeed_once || ahrs.is_airspeed_sensor_enabled;

    // Convert equivalent airspeeds to true airspeeds and harmonise limits

    float EAS2TAS = baro.getEAS2TAS();
    _TAS_dem = _EAS_dem * EAS2TAS;
    if (my_flags._flags.reset || !use_airspeed) {
        _TASmax = MAX_AIRSPEED * EAS2TAS;
    } else if (_thr_clip_status == clipStatus::MAX) {
        // wind down airspeed upper limit  to prevent a situation where the aircraft can't climb
        // at the maximum speed
        const float velRateMin = 0.5f * _STEdot_min / MAX(_TAS_state, MIN_AIRSPEED * EAS2TAS);
        _TASmax += _DT * velRateMin;
        _TASmax = MAX(_TASmax, AIRSPEED_CRUISE * EAS2TAS);
    } else {
        // wind airspeed upper limit back to parameter defined value
        const float velRateMax = 0.5f * _STEdot_max / MAX(_TAS_state, MIN_AIRSPEED * EAS2TAS);
        _TASmax += _DT * velRateMax;
    }
    _TASmax   = MIN(_TASmax, MAX_AIRSPEED * EAS2TAS);
    _TASmin   = MIN_AIRSPEED * EAS2TAS;

    // if (aparm.stall_prevention) {
    //     // when stall prevention is active we raise the minimum
    //     // airspeed based on aerodynamic load factor
    //     if (is_positive(AIRSPEED_STALL)) {
    //         _TASmin = MAX(_TASmin, AIRSPEED_STALL*EAS2TAS*_load_factor);
    //     } else {
    //         _TASmin *= _load_factor;
    //     }
    // }

    if (_TASmax < _TASmin) {
        _TASmax = _TASmin;
    }

    // Get measured airspeed or default to trim speed and constrain to range between min and max if
    // airspeed sensor data cannot be used
    if (!ahrs.is_airspeed_sensor_enabled) {
        // If no airspeed available use average of min and max
        _EAS = constrain(AIRSPEED_CRUISE, MIN_AIRSPEED, MAX_AIRSPEED);
    }

    // limit the airspeed to a minimum of 3 m/s
    float min_airspeed = 3.0;

    // Reset states of time since last update is too large
    if (my_flags._flags.reset) {
        _TAS_state = (_EAS * EAS2TAS);
        _TAS_state = MAX(_TAS_state, MIN_AIRSPEED);
        _integDTAS_state = 0.0f;
        return;
    }

    // Implement a second order complementary filter to obtain a
    // smoothed airspeed estimate
    // airspeed estimate is held in _TAS_state
    float aspdErr = (_EAS * EAS2TAS) - _TAS_state;
    float integDTAS_input = aspdErr * _spdCompFiltOmega * _spdCompFiltOmega;
    // Prevent state from winding up
    if (_TAS_state < 3.1f) {
        integDTAS_input = MAX(integDTAS_input, 0.0f);
    }
    _integDTAS_state = _integDTAS_state + integDTAS_input * DT;
    float TAS_input = _integDTAS_state + _vel_dot + aspdErr * _spdCompFiltOmega * 1.4142f;
    _TAS_state = _TAS_state + TAS_input * DT;
    _TAS_state = MAX(_TAS_state, min_airspeed);
}

void update_50hz(void)
{
    // Implement third order complementary filter for height and height rate
    // estimated height rate = _climb_rate
    // estimated height above field elevation  = _height
    // Reference Paper :
    // Optimizing the Gains of the Baro-Inertial Vertical Channel
    // Widnall W.S, Sinha P.K,
    // AIAA Journal of Guidance and Control, 78-1307R

    /*
      if we have a vertical position estimate from the EKF then use
      it, otherwise use barometric altitude
     */
    ahrs.get_relative_position_D_home(_height);
    _height *= -1.0f;

    // Calculate time in seconds since last update
    uint64_t now = micros();
    float DT = (now - _update_50hz_last_usec) * 1.0e-6f;
    my_flags._flags.reset = DT > 1.0f;
    if (my_flags._flags.reset) {
        _climb_rate = 0.0f;
        _height_filter.dd_height = 0.0f;
        DT = 0.02f; // when first starting TECS, use most likely time constant
        _vdot_filter.reset();
    }
    _update_50hz_last_usec = now;

    // Use inertial nav verical velocity and height if available
    Vector3f velned = ahrs.get_velocity_ned();
    if (true){
        // if possible use the EKF vertical velocity
        _climb_rate = -velned.z;
    } else {
        /*
          use a complimentary filter to calculate climb_rate. This is
          designed to minimise lag
         */
        float baro_alt = baro.altitude;
        // Get height acceleration
        float hgt_ddot_mea = -(ahrs.get_accel_ef().z + GRAVITY_MSS);
        // Perform filter calculation using backwards Euler integration
        // Coefficients selected to place all three filter poles at omega
        float omega2 = _hgtCompFiltOmega*_hgtCompFiltOmega;
        float hgt_err = baro_alt - _height_filter.height;
        float integ1_input = hgt_err * omega2 * _hgtCompFiltOmega;

        _height_filter.dd_height += integ1_input * DT;

        float integ2_input = _height_filter.dd_height + hgt_ddot_mea + hgt_err * omega2 * 3.0f;

        _climb_rate += integ2_input * DT;

        float integ3_input = _climb_rate + hgt_err * _hgtCompFiltOmega * 3.0f;
        // If more than 1 second has elapsed since last update then reset the integrator state
        // to the measured height
        if (my_flags._flags.reset) {
            _height_filter.height = _height;
        } else {
            _height_filter.height += integ3_input*DT;
        }
    }

    // Update the speed estimate using a 2nd order complementary filter
    _update_speed(DT);
}

void _update_speed_demand(void)
{
    // Set the airspeed demand to the minimum value if an underspeed condition exists
    // or a bad descent condition exists
    // This will minimise the rate of descent resulting from an engine failure,
    // enable the maximum climb rate to be achieved and prevent continued full power descent
    // into the ground due to an unachievable airspeed value
    if ((my_flags._flags.badDescent) || (my_flags._flags.underspeed))
    {
        _TAS_dem = _TASmin;
    }

    // Constrain speed demand, taking into account the load factor
    _TAS_dem = constrain(_TAS_dem, _TASmin, _TASmax);

    // calculate velocity rate limits based on physical performance limits
    // provision to use a different rate limit if bad descent or underspeed condition exists
    // Use 50% of maximum energy rate to allow margin for total energy contgroller
    const float velRateMax = 0.5f * _STEdot_max / _TAS_state;
    const float velRateMin = 0.5f * _STEdot_min / _TAS_state;
    const float TAS_dem_previous = _TAS_dem_adj;

    // assume fixed 10Hz call rate
    const float dt = 0.1;

    // Apply rate limit
    if ((_TAS_dem - TAS_dem_previous) > (velRateMax * dt))
    {
        _TAS_dem_adj = TAS_dem_previous + velRateMax * dt;
        _TAS_rate_dem = velRateMax;
    }
    else if ((_TAS_dem - TAS_dem_previous) < (velRateMin * dt))
    {
        _TAS_dem_adj = TAS_dem_previous + velRateMin * dt;
        _TAS_rate_dem = velRateMin;
    }
    else
    {
        _TAS_rate_dem = (_TAS_dem - TAS_dem_previous) / dt;
        _TAS_dem_adj = _TAS_dem;
    }
    // Constrain speed demand again to protect against bad values on initialisation.
    _TAS_dem_adj = constrain(_TAS_dem_adj, _TASmin, _TASmax);
}

void _update_height_demand()
{
    // Apply 2 point moving average to demanded height
    _hgt_dem = 0.5f * (_hgt_dem + _hgt_dem_in_old);
    _hgt_dem_in_old = _hgt_dem;

    float max_sink_rate = _maxSinkRate; 

    // Limit height rate of change
    if ((_hgt_dem - _hgt_dem_prev) > (_maxClimbRate * 0.1f))
    {
        _hgt_dem = _hgt_dem_prev + _maxClimbRate * 0.1f;
    }
    else if ((_hgt_dem - _hgt_dem_prev) < (-max_sink_rate * 0.1f))
    {
        _hgt_dem = _hgt_dem_prev - max_sink_rate * 0.1f;
    }
    _hgt_dem_prev = _hgt_dem;

    // Apply first order lag to height demand
    _hgt_dem_adj = 0.05f * _hgt_dem + 0.95f * _hgt_dem_adj_last;

    // when flaring force height rate demand to the
    // configured sink rate and adjust the demanded height to
    // be kinematically consistent with the height rate.

    // for landing approach we will predict ahead by the time constant
    // plus the lag produced by the first order filter. This avoids a
    // lagged height demand while constantly descending which causes
    // us to consistently be above the desired glide slope. This will
    // be replaced with a better zero-lag filter in the future.
    float new_hgt_dem = _hgt_dem_adj;
    hgt_dem_lag_filter_slew = 0;
    _hgt_dem_adj_last = _hgt_dem_adj;
    _hgt_dem_adj = new_hgt_dem;
}

void _detect_underspeed(void)
{
    // see if we can clear a previous underspeed condition. We clear
    // it if we are now more than 15% above min speed, and haven't
    // been below min speed for at least 3 seconds.
    if (my_flags._flags.underspeed &&
        _TAS_state >= _TASmin * 1.15f &&
        millis() - _underspeed_start_ms > 3000U)
    {
        my_flags._flags.underspeed = false;
    }

    if (((_TAS_state < _TASmin * 0.9f) &&
         (_throttle_dem >= _THRmaxf * 0.95f)) ||
        ((_height < _hgt_dem_adj) && my_flags._flags.underspeed))
    {
        my_flags._flags.underspeed = true;
        if (_TAS_state < _TASmin * 0.9f)
        {
            // reset start time as we are still underspeed
            _underspeed_start_ms = millis();
        }
    }
    else
    {
        // this clears underspeed if we reach our demanded height and
        // we are either below 95% throttle or we above 90% of min
        // airspeed
        my_flags._flags.underspeed = false;
    }
}

void _update_energies(void){
    // Calculate specific energy demands
    _SPE_dem = _hgt_dem * GRAVITY_MSS;
    _SKE_dem = 0.5f * _TAS_dem_adj * _TAS_dem_adj;

    // Calculate specific energy rate demands and high pass filter demanded airspeed
    // rate of change to match the filtering applied to the measurement
    _SKEdot_dem = _TAS_state * (_TAS_rate_dem - _TAS_rate_dem_lpf);

    // Calculate specific energy
    _SPE_est = _height * GRAVITY_MSS;
    _SKE_est = 0.5f * _TAS_state * _TAS_state;

    // Calculate specific energy rate and high pass filter airspeed rate of change
    // to remove effect of complementary filter induced bias errors
    _SPEdot = _climb_rate * GRAVITY_MSS;
    _SKEdot = _TAS_state * (_vel_dot - _vel_dot_lpf);
}

float _get_i_gain(void)
{
    float i_gain = _integGain;
    return i_gain;
}
/*
  calculate throttle demand - airspeed enabled case
 */
void _update_throttle_with_airspeed(void)
{
    // Calculate limits to be applied to potential energy error to prevent over or underspeed occurring due to large height errors
    float SPE_err_max = 0.5f * _TASmax * _TASmax - _SKE_dem;
    float SPE_err_min = 0.5f * _TASmin * _TASmin - _SKE_dem;

    // Calculate total energy error
    _STE_error = constrain((_SPE_dem - _SPE_est), SPE_err_min, SPE_err_max) + _SKE_dem - _SKE_est;
    float STEdot_dem = constrain((_SPEdot_dem + _SKEdot_dem), _STEdot_min, _STEdot_max);
    float STEdot_error = STEdot_dem - _SPEdot - _SKEdot;

    // Apply 0.5 second first order filter to STEdot_error
    // This is required to remove accelerometer noise from the  measurement
    STEdot_error = 0.2f * STEdot_error + 0.8f * _STEdotErrLast;
    _STEdotErrLast = STEdot_error;

    // Calculate throttle demand
    // If underspeed condition is set, then demand full throttle
    if (my_flags._flags.underspeed)
    {
        _throttle_dem = 1.0f;
    }
    else
    {
        // Calculate gain scaler from specific energy error to throttle
        // (_STEdot_max - _STEdot_min) / (_THRmaxf - _THRminf) is the derivative of STEdot wrt throttle measured across the max allowed throttle range.
        float K_STE2Thr = 1 / (timeConstant() * (_STEdot_max - _STEdot_min) / (_THRmaxf - _THRminf));

        // Calculate feed-forward throttle
        float ff_throttle = 0;
        float nomThr = TECS_CRUISE_THROTTLE * 0.01f;
        const Matrix3f &rotMat = ahrs.get_rotation_body_to_ned();
        // Use the demanded rate of change of total energy as the feed-forward demand, but add
        // additional component which scales with (1/cos(bank angle) - 1) to compensate for induced
        // drag increase during turns.
        float cosPhi = sqrtf((rotMat.a.y * rotMat.a.y) + (rotMat.b.y * rotMat.b.y));
        STEdot_dem = STEdot_dem + _rollComp * (1.0f / constrain(cosPhi * cosPhi, 0.1f, 1.0f) - 1.0f);
        ff_throttle = nomThr + STEdot_dem / (_STEdot_max - _STEdot_min) * (_THRmaxf - _THRminf);

        // Calculate PD + FF throttle
        float throttle_damp = _thrDamp;
        _throttle_dem = (_STE_error + STEdot_error * throttle_damp) * K_STE2Thr + ff_throttle;

        // Constrain throttle demand
        _throttle_dem = constrain(_throttle_dem, _THRminf, _THRmaxf);

        float THRminf_clipped_to_zero = constrain(_THRminf, 0, _THRmaxf);

        // Rate limit PD + FF throttle
        // Calculate the throttle increment from the specified slew time
        if (THR_SLEW_RATE != 0)
        {
            float thrRateIncr = _DT * (_THRmaxf - THRminf_clipped_to_zero) * THR_SLEW_RATE * 0.01f;

            _throttle_dem = constrain(_throttle_dem,
                                            _last_throttle_dem - thrRateIncr,
                                            _last_throttle_dem + thrRateIncr);
            _last_throttle_dem = _throttle_dem;
        }

        // Calculate integrator state upper and lower limits
        // Set to a value that will allow 0.1 (10%) throttle saturation to allow for noise on the demand
        // Additionally constrain the integrator state amplitude so that the integrator comes off limits faster.
        float maxAmp = 0.5f * (_THRmaxf - THRminf_clipped_to_zero);
        float integ_max = constrain((_THRmaxf - _throttle_dem + 0.1f), -maxAmp, maxAmp);
        float integ_min = constrain((_THRminf - _throttle_dem - 0.1f), -maxAmp, maxAmp);

        // Calculate integrator state, constraining state
        // Set integrator to a max throttle value during climbout
        _integTHR_state = _integTHR_state + (_STE_error * _get_i_gain()) * _DT * K_STE2Thr;
        _integTHR_state = constrain(_integTHR_state, integ_min, integ_max);

        // Sum the components.
        _throttle_dem = _throttle_dem + _integTHR_state;
    }

    // Constrain throttle demand
    _throttle_dem = constrain(_throttle_dem, _THRminf, _THRmaxf);
}

/*
  calculate throttle, non-airspeed case
 */
void _update_throttle_without_airspeed(int16_t throttle_nudge)
{
    // Calculate throttle demand by interpolating between pitch and throttle limits
    float nomThr;
    //If landing and we don't have an airspeed sensor and we have a non-zero
    //TECS_LAND_THR param then use it
    nomThr = (TECS_CRUISE_THROTTLE + throttle_nudge) * 0.01f;

    if (_pitch_dem > 0.0f && _PITCHmaxf > 0.0f)
    {
        _throttle_dem = nomThr + (_THRmaxf - nomThr) * _pitch_dem / _PITCHmaxf;
    }
    else if (_pitch_dem < 0.0f && _PITCHminf < 0.0f)
    {
        _throttle_dem = nomThr + (_THRminf - nomThr) * _pitch_dem / _PITCHminf;
    }
    else
    {
        _throttle_dem = nomThr;
    }

    // Calculate additional throttle for turn drag compensation including throttle nudging
    const Matrix3f &rotMat = ahrs.get_rotation_body_to_ned();
    // Use the demanded rate of change of total energy as the feed-forward demand, but add
    // additional component which scales with (1/cos(bank angle) - 1) to compensate for induced
    // drag increase during turns.
    float cosPhi = sqrtf((rotMat.a.y * rotMat.a.y) + (rotMat.b.y * rotMat.b.y));
    float STEdot_dem = _rollComp * (1.0f / constrain(cosPhi * cosPhi, 0.1f, 1.0f) - 1.0f);
    _throttle_dem = _throttle_dem + STEdot_dem / (_STEdot_max - _STEdot_min) * (_THRmaxf - _THRminf);
}

void _detect_bad_descent(void)
{
    // Detect a demanded airspeed too high for the aircraft to achieve. This will be
    // evident by the the following conditions:
    // 1) Underspeed protection not active
    // 2) Specific total energy error > 200 (greater than ~20m height error)
    // 3) Specific total energy reducing
    // 4) throttle demand > 90%
    // If these four conditions exist simultaneously, then the protection
    // mode will be activated.
    // Once active, the following condition are required to stay in the mode
    // 1) Underspeed protection not active
    // 2) Specific total energy error > 0
    // This mode will produce an undulating speed and height response as it cuts in and out but will prevent the aircraft from descending into the ground if an unachievable speed demand is set
    float STEdot = _SPEdot + _SKEdot;
    if ((!my_flags._flags.underspeed && (_STE_error > 200.0f) && (STEdot < 0.0f) && (_throttle_dem >= _THRmaxf * 0.9f)) || (my_flags._flags.badDescent && !my_flags._flags.underspeed && (_STE_error > 0.0f)))
    {
        my_flags._flags.badDescent = true;
    }
    else
    {
        my_flags._flags.badDescent = false;
    }
}

void _update_pitch(void)
{
    // Calculate Speed/Height Control Weighting
    // This is used to determine how the pitch control prioritises speed and height control
    // A weighting of 1 provides equal priority (this is the normal mode of operation)
    // A SKE_weighting of 0 provides 100% priority to height control. This is used when no airspeed measurement is available
    // A SKE_weighting of 2 provides 100% priority to speed control. This is used when an underspeed condition is detected. In this instance, if airspeed
    // rises above the demanded value, the pitch angle will be increased by the TECS controller.
    _SKE_weighting = constrain(_spdWeight, 0.0f, 2.0f);
    if (!ahrs.is_airspeed_sensor_enabled || _use_synthetic_airspeed) {
        _SKE_weighting = 0.0f;
    } 

    float SPE_weighting = 2.0f - _SKE_weighting;

    // either weight can fade to 0, but don't go above 1 to prevent instability if tuned at a speed weight of 1 and wieghting is varied to end points in flight.
    SPE_weighting = MIN(SPE_weighting, 1.0f);
    _SKE_weighting = MIN(_SKE_weighting, 1.0f);

    // Calculate demanded specific energy balance and error
    float SEB_dem = _SPE_dem * SPE_weighting - _SKE_dem * _SKE_weighting;
    float SEB_est = _SPE_est * SPE_weighting - _SKE_est * _SKE_weighting;
    float SEB_error = SEB_dem - SEB_est;

    // track demanded height using the specified time constant
    float SEBdot_dem = _hgt_rate_dem * GRAVITY_MSS * SPE_weighting + SEB_error / timeConstant();
    float SEBdot_dem_min = - _maxSinkRate * GRAVITY_MSS;
    float SEBdot_dem_max = _maxClimbRate * GRAVITY_MSS;
    if (SEBdot_dem < SEBdot_dem_min) {
        SEBdot_dem = SEBdot_dem_min;
        _SEBdot_dem_clip = clipStatus::MIN;
    } else if (SEBdot_dem > SEBdot_dem_max) {
        SEBdot_dem = SEBdot_dem_max;
        _SEBdot_dem_clip = clipStatus::MAX;
    } else {
        _SEBdot_dem_clip = clipStatus::NONE;
    }

    // calculate specific energy balance rate error
    float SEBdot_est = _SPEdot * SPE_weighting - _SKEdot * _SKE_weighting;
    float SEBdot_error = SEBdot_dem - SEBdot_est;

    // sum predicted plus damping correction
    // integral correction is added later
    // During flare a different damping gain is used
    float pitch_damp = _ptchDamp;
    
    float SEBdot_dem_total = SEBdot_dem + SEBdot_error * pitch_damp;

    // inverse of gain from SEB to pitch angle
    float gainInv = (_TAS_state * GRAVITY_MSS);

    // don't allow the integrator to rise by more than 20% of its full
    // Calculate max and min values for integrator state that will allow for no more than
    // 5deg of saturation. This allows for some pitch variation due to gusts before the
    // integrator is clipped. Otherwise the effectiveness of the integrator will be reduced in turbulence
    float integSEBdot_min = (gainInv * (_PITCHminf - radians(5.0f))) - SEBdot_dem_total;
    float integSEBdot_max = (gainInv * (_PITCHmaxf + radians(5.0f))) - SEBdot_dem_total;

    // Calculate integrator state, constraining input if pitch limits are exceeded
    // don't allow the integrator to rise by more than 10% of its full
    // range in one step. This prevents single value glitches from
    // causing massive integrator changes. See Issue#4066
    float integSEB_range = integSEBdot_max - integSEBdot_min;
    float integSEB_delta = constrain(SEBdot_error * _get_i_gain() * _DT, -integSEB_range*0.1f, integSEB_range*0.1f);

    // predict what pitch will be with uncontrained integration
    _pitch_dem_unc = (SEBdot_dem_total + _integSEBdot + integSEB_delta + _integKE) / gainInv;

    // integrate SEB rate error and apply integrator state limits
    bool inhibit_integrator = ((_pitch_dem_unc > _PITCHmaxf) && integSEB_delta > 0.0f) ||
                                    ((_pitch_dem_unc < _PITCHminf) && integSEB_delta < 0.0f);
    if (!inhibit_integrator) {
        _integSEBdot += integSEB_delta;
        _integKE += (_SKE_est - _SKE_dem) * _SKE_weighting * _DT / timeConstant();
    } else {
        // fade out integrator if saturating
        float coef = 1.0f - _DT / (_DT + timeConstant());
        _integSEBdot *= coef;
        _integKE *= coef;
    }
    _integSEBdot = constrain(_integSEBdot, integSEBdot_min, integSEBdot_max);
    float KE_integ_limit = 0.25f * (_PITCHmaxf - _PITCHminf) * gainInv; // allow speed trim integrator to access 505 of pitch range
    _integKE = constrain(_integKE, - KE_integ_limit, KE_integ_limit);

    // Calculate pitch demand from specific energy balance signals
    _pitch_dem_unc = (SEBdot_dem_total + _integSEBdot + _integKE) / gainInv;

    // Add a feedforward term from demanded airspeed to pitch
    // if (_flags.is_gliding) {
    //     _pitch_dem_unc += (_TAS_dem_adj - _pitch_ff_v0) * _pitch_ff_k;
    // }

    // Constrain pitch demand
    _pitch_dem = constrain(_pitch_dem_unc, _PITCHminf, _PITCHmaxf);

    // Rate limit the pitch demand to comply with specified vertical
    // acceleration limit
    float ptchRateIncr = _DT * _vertAccLim / _TAS_state;

    if ((_pitch_dem - _last_pitch_dem) > ptchRateIncr) {
        _pitch_dem = _last_pitch_dem + ptchRateIncr;
    } else if ((_pitch_dem - _last_pitch_dem) < -ptchRateIncr) {
        _pitch_dem = _last_pitch_dem - ptchRateIncr;
    }

    _last_pitch_dem = _pitch_dem;
}

void _initialise_states(float hgt_afe)
{
    // Initialise states and variables if DT > 1 second or in climbout
    if (_DT > 1.0f)
    {
        _integTHR_state = 0.0f;
        _integSEB_state = 0.0f;
        _last_throttle_dem = THROTTLE_CRUISE * 0.01f;
        _last_pitch_dem = imu.pitch;
        _hgt_dem_adj_last = hgt_afe;
        _hgt_dem_adj = _hgt_dem_adj_last;
        _hgt_dem_prev = _hgt_dem_adj_last;
        _hgt_dem_in_old = _hgt_dem_adj_last;
        _TAS_dem_adj = _TAS_dem;
        my_flags._flags.underspeed = false;
        my_flags._flags.badDescent = false;
        _DT = 0.2f; // when first starting TECS, use a
        // small time constant
    }
}

void _update_STE_rate_lim(void)
{
    // Calculate Specific Total Energy Rate Limits
    // This is a trivial calculation at the moment but will get bigger once we start adding altitude effects
    _STEdot_max = _maxClimbRate * GRAVITY_MSS;
    _STEdot_min = -_minSinkRate * GRAVITY_MSS;
}

void update_pitch_throttle( int32_t hgt_dem_cm,
                            float EAS_dem_cm,
                            int16_t throttle_nudge,
                            float hgt_afe,
                            float load_factor)
{
    // Calculate time in seconds since last update
    uint64_t now = micros();
    _DT = (now - _update_pitch_throttle_last_usec) * 1.0e-6f;
    _update_pitch_throttle_last_usec = now;

    // Convert inputs
    _hgt_dem = hgt_dem_cm * 0.01; // konversi ke meter
    _EAS_dem = EAS_dem_cm; // equivalent airspeed

    // Update the speed estimate using a 2nd order complementary filter
    _update_speed(load_factor);

    _THRmaxf = tecs_max_throttle * 0.01f; // 0.85
    _THRminf = tecs_min_throtlle * 0.01f; // 0.30

    // min of 1% throttle range to prevent a numerical error
    _THRmaxf = MAX(_THRmaxf, _THRminf + 0.01);

    // work out the maximum and minimum pitch
    // if TECS_PITCH_{MAX,MIN} isn't set then use
    // LIM_PITCH_{MAX,MIN}. Don't allow TECS_PITCH_{MAX,MIN} to be
    // larger than LIM_PITCH_{MAX,MIN}
    if (_pitch_max == 0)
    {
        _PITCHmaxf = PITCH_MAX;
    }
    else
    {
        _PITCHmaxf = MIN(_pitch_max, PITCH_MAX);
    }

    if (_pitch_min >= 0)
    {
        _PITCHminf = PITCH_MIN;
    }
    else
    {
        _PITCHminf = MAX(_pitch_min, PITCH_MIN);
    }

    // apply temporary pitch limit and clear
    if (_pitch_max_limit < 90)
    {
        _PITCHmaxf = constrain(_PITCHmaxf, -90, _pitch_max_limit);
        _PITCHminf = constrain(_PITCHminf, -_pitch_max_limit, _PITCHmaxf);
        _pitch_max_limit = 90;
    }

    // convert to radians
    _PITCHmaxf = radians(_PITCHmaxf);
    _PITCHminf = radians(_PITCHminf);

    // don't allow max pitch to go below min pitch
    _PITCHmaxf = MAX(_PITCHmaxf, _PITCHminf);

    // initialise selected states and variables if DT > 1 second or in climbout
    _initialise_states(hgt_afe);

    // Calculate Specific Total Energy Rate Limits
    _update_STE_rate_lim();

    // Calculate the speed demand
    _update_speed_demand();

    // Calculate the height demand
    _update_height_demand();

    // Detect underspeed condition
    _detect_underspeed();

    // Calculate specific energy quantitiues
    _update_energies();

    // Calculate throttle demand - use simple pitch to throttle if no
    // airspeed sensor.
    // Note that caller can demand the use of
    // synthetic airspeed for one loop if needed. This is required
    // during QuadPlane transition when pitch is constrained
    if ((ahrs.is_airspeed_sensor_enabled && _use_synthetic_airspeed) || _use_synthetic_airspeed_once)
    {
        _update_throttle_with_airspeed();
        _use_synthetic_airspeed_once = false;
    }
    else
    {
        _update_throttle_without_airspeed(throttle_nudge);
    }

    // Detect bad descent due to demanded airspeed being too high
    _detect_bad_descent();

    // Calculate pitch demand
    _update_pitch(); // ngehasilin nilai pitch
}

