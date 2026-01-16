// #pragma once
// #include "Arduino.h"
// #include "AHRS.h"
// #include <AP_Math.h>
// #include <../lib/Common/Locations.h>
// #include <../lib/Common/Common.h>
// #include <definitions.h>
// #include <gps.h>
// // #include "drivers/hmc5883l.h"
// #include "AutoMode.h"
// #include <bno055.h>
// #include <../lib/Math/vector2.h>
// // #include <TECS.h>

//     // L1 tracking distance in meters which is dynamically updated
//     float _L1_dist;

//     // Status which is true when the vehicle has started circling the WP
//     bool _WPcircle;

//     // bearing angle (radians) to L1 point
//     float _nav_bearing;

//     // bearing error angle (radians) +ve to left of track
//     float _bearing_error;

//     // crosstrack error in meters
//     float _crosstrack_error;

//     // target bearing in centi-degrees from last update
//     float _target_bearing_deg;

//     // L1 tracking loop period (sec)
//     float _L1_period = 14;//25 september 2019
//     // L1 tracking loop damping ratio
//     float _L1_damping = 0.75f;

//     // previous value of cross-track velocity
//     float _last_Nu;
//     float _latAccDem;

//     // integral feedback to correct crosstrack error. Used to ensure xtrack converges to zero.
//     // For tuning purposes it's helpful to clear the integrator when it changes so a _prev is used
//     float _L1_xtrack_i = 0;
//     float _L1_xtrack_i_gain = 0.02f;
//     float _L1_xtrack_i_gain_prev = 0;
//     uint32_t _last_update_waypoint_us;
//     bool _data_is_stale = true;

//     float _loiter_bank_limit;

//     bool _reverse = false;

// //dari LQR_PF
// float _lat_accel;

// //dari WPNav
// // const float R = 6378100.0;
// uint32_t last_update_wp;
// uint32_t delta_time;
// Vector2f _distance_NE;
// float _distance;
// float _heading;
// float _bearing_to_end;
// float _bearing_to_current;
// float _error_bearing;
// // float _crosstrack_error;
// float _distance_NE_lenght;
// float _distance_plane_lenght;
// // float _lat_accel;

// float get_yaw(){
//     if (_reverse){
//         return wrap_PI(M_PI + radians(heading));
//     }
//     return radians(heading);
// }

// float get_yaw_sensor(){
//     return heading;
// }

// // prevent indecision in waypoint tracking
// void _prevent_indecision(float &Nu)
// {
//     const float Nu_limit = 0.9f*M_PI;
//     if (fabsf(Nu) > Nu_limit &&
//         fabsf(_last_Nu) > Nu_limit &&
//         fabsf(wrap_180(_target_bearing_deg - get_yaw_sensor())) > 120 &&
//         Nu * _last_Nu < 0.0f) {
//         // we are moving away from the target waypoint and pointing
//         // away from the waypoint (not flying backwards). The sign
//         // of Nu has also changed, which means we are
//         // oscillating in our decision about which way to go
//         Nu = _last_Nu;
//     }
// }





// //L1_Controller program untuk loiter START
// // void update_loiter(const struct Locations &center_WP, float radius, int8_t loiter_direction)
// // {
// //     struct Locations current_loc;

// //     // scale loiter radius with square of EAS2TAS to allow us to stay
// //     // stable at high altitude
// //     radius = (fabsf(radius));//loiter_radius(fabsf(radius));

// //     // Calculate guidance gains used by PD loop (used during circle tracking)
// //     float omega = (6.2832f / _L1_period); // (2phi / periode)
// //     float Kx = omega * omega;
// //     float Kv = 2.0f * _L1_damping * omega;

// //     // Calculate L1 gain required for specified damping (used during waypoint capture)
// //     float K_L1 = 4.0f * _L1_damping * _L1_damping;
 
// // /*
// // komen 
// // Kx      = gain untuk nilai error 
// // Kv      = gain derivative
// // K_L1    = 
// // */
// //     //Get current position and velocity
// //     if (get_position(current_loc) == false) {
// //         // if no GPS loc available, maintain last nav/target_bearing
// //         _data_is_stale = true;
// //         return;
// //     }


// //     Vector2f _groundspeed_vector =  {get_velocity_ned().x, get_velocity_ned().y};  


// //     //Calculate groundspeed
// //     float groundSpeed = MAX(_groundspeed_vector.length() , 1.0f); //menentukan _groundspeed_vector.length() atau 1 f
// //                                                                   //return berupa value terbesar dan berupa float


// //     // update _target_bearing_cd
// //     _target_bearing_deg = current_loc.get_bearing_to(center_WP); //update derajad "centi derajad" menuju target center loiter


// //     // Calculate time varying control parameters
// //     // Hitung parameter kontrol waktu yang bervariasi
// //     // Calculate the L1 length required for specified period
// //     // Hitung panjang L1 yang diperlukan untuk periode tertentu
// //     // 0.3183099 = 1/pi
// //     _L1_dist = 0.3183099f * _L1_damping * _L1_period * groundSpeed; 

// //     //Calculate the NE position of the aircraft relative to WP A
// //     // Hitung posisi NE pesawat relatif terhadap WP A
// //     // NE = north / east
// //     const Vector2f A_air = center_WP.get_distance_NE(current_loc);

// //     // Calculate the unit vector from WP A to aircraft
// //     // protect against being on the waypoint and having zero velocity
// //     // if too close to the waypoint, use the velocity vector
// //     // if the velocity vector is too small, use the heading vector
// //     Vector2f A_air_unit;
// //     if (A_air.length() > 0.1f) {        //jika jarak dari center WP dan lokasi saat ini masih > 0.1f
// //         A_air_unit = A_air.normalized();    //value A_air_unit = (A_a.x/length),(A_a.y/length)
// //     } else {
// //         if (_groundspeed_vector.length() < 0.1f) { 
// //             //jika kecepatan ground speed <0.1 data A_air unit yang dipakai di ambil berdasar heading pesawat
// //             //(x,y) = (cos(heading),sin(heading))
// //             A_air_unit = Vector2f(cosf(heading), sinf(heading));




            
// //         } else {
// //             A_air_unit = _groundspeed_vector.normalized();
// //         }
// //     }

// //     //Calculate Nu to capture center_WP
// //     float xtrackVelCap = A_air_unit % _groundspeed_vector; // Velocity across line - perpendicular to radial inbound to WP
// //     float ltrackVelCap = - (_groundspeed_vector * A_air_unit); // Velocity along line - radial inbound to WP
// //     float Nu = atan2f(xtrackVelCap,ltrackVelCap);
// //     //Nu = sudut dari value kecepatan Y/x
// //     _prevent_indecision(Nu);
// //     _last_Nu = Nu;

// //     Nu = constrain(Nu, -M_PI_2, M_PI_2); //Limit Nu to +- Pi/2

// //     //Calculate lat accln demand to capture center_WP (use L1 guidance law)
// //     float latAccDemCap = K_L1 * groundSpeed * groundSpeed / _L1_dist * sinf(Nu);

// //     //Calculate radial position and velocity errors
// //     float xtrackVelCirc = -ltrackVelCap; // Radial outbound velocity - reuse previous radial inbound velocity
// //     float xtrackErrCirc = A_air.length() - radius; // Radial distance from the loiter circle //jarak dari pesawat ke radius loiter

// //     // keep crosstrack error for reporting
// //     _crosstrack_error = xtrackErrCirc;

// //     //Calculate PD control correction to circle waypoint_ahrs.roll
// //     float latAccDemCircPD = (xtrackErrCirc * Kx + xtrackVelCirc * Kv);

// //     //Calculate tangential velocity
// //     float velTangent = xtrackVelCap * float(loiter_direction);

// //     //Prevent PD demand from turning the wrong way by limiting the command when flying the wrong way
// //     if (ltrackVelCap < 0.0f && velTangent < 0.0f) {
// //         latAccDemCircPD =  MAX(latAccDemCircPD, 0.0f);
// //     }

// //     // Calculate centripetal acceleration demand
// //     float latAccDemCircCtr = velTangent * velTangent / MAX((0.5f * radius), (radius + xtrackErrCirc));

// //     //Sum PD control and centripetal acceleration to calculate lateral manoeuvre demand
// //     float latAccDemCirc = loiter_direction * (latAccDemCircPD + latAccDemCircCtr);

// //     // Perform switchover between 'capture' and 'circle' modes at the
// //     // point where the commands cross over to achieve a seamless transfer
// //     // Only fly 'capture' mode if outside the circle
// //     if (xtrackErrCirc > 0.0f && loiter_direction * latAccDemCap < loiter_direction * latAccDemCirc) {
// //         _latAccDem = latAccDemCap;
// //         _WPcircle = false;
// //         _bearing_error = Nu; // angle between demanded and achieved velocity vector, +ve to left of track
// //         _nav_bearing = atan2f(-A_air_unit.y , -A_air_unit.x); // bearing (radians) from AC to L1 point
// //     } else { 
// //         _latAccDem = latAccDemCirc;
// //         _WPcircle = true;
// //         _bearing_error = 0.0f; // bearing error (radians), +ve to left of track
// //         _nav_bearing = atan2f(-A_air_unit.y , -A_air_unit.x); // bearing (radians)from AC to L1 point
// //     }

// //     _data_is_stale = false; // status are correctly updated with current waypoint data
// // }

// // //L1_Controller program untuk loiter radius START
// // float loiter_radius(const float radius) 
// // {
// //     // prevent an insane loiter bank limit
// //     float sanitized_bank_limit = constrain(_loiter_bank_limit, 0.0f, 89.0f);
// //     float lateral_accel_sea_level = tanf(radians(sanitized_bank_limit)) * GRAVITY_MSS;

// //     float nominal_velocity_sea_level;
// //     if(is_airspeed_sensor_enabled) {
// //         nominal_velocity_sea_level = get_target_airspeed();
// //     } else {
// //         nominal_velocity_sea_level = 0.0f; 
// //     }

// //     float eas2tas_sq = sq(getEAS2TAS());

// //     if (is_zero(sanitized_bank_limit) || is_zero(nominal_velocity_sea_level) ||
// //         is_zero(lateral_accel_sea_level)) {
// //         // Missing a sane input for calculating the limit, or the user has
// //         // requested a straight scaling with altitude. This will always vary
// //         // with the current altitude, but will at least protect the airframe
// //         return radius * eas2tas_sq;
// //     } else {
// //         float sea_level_radius = sq(nominal_velocity_sea_level) / lateral_accel_sea_level;
// //         if (sea_level_radius > radius) {
// //             // If we've told the plane that its sea level radius is unachievable fallback to
// //             // straight altitude scaling
// //             return radius * eas2tas_sq;
// //         } else {
// //             // select the requested radius, or the required altitude scale, whichever is safer
// //             return MAX(sea_level_radius * eas2tas_sq, radius);
// //         }
// //     }
// // }
// // //L1_Controller program loiter radius END

// // return the desired roll angle in centi-degrees to move towards
// // the target waypoint
// float nav_roll() 
// {
//     float ret; 
//     ret = cosf(radians(pitch))*degrees(atanf(_latAccDem * 0.101972f) * 1.0f); // 0.101972 = 1/9.81
//     ret = constrain(ret, -90, 90);
//     return ret;
// }

// float lateral_acceleration(void) 
// {
//     return _latAccDem;
// }

// /*
//   this is the turn distance assuming a 90 degree turn
//  */
// float turn_distance(float wp_radius) 
// {
//     wp_radius *= sq(getEAS2TAS());
//     return MIN(wp_radius, _crosstrack_error);
// }

// /*
//   this approximates the turn distance for a given turn angle. If the
//   turn_angle is > 90 then a 90 degree turn distance is used, otherwise
//   the turn distance is reduced linearly.
//   This function allows straight ahead mission legs to avoid thinking
//   they have reached the waypoint early, which makes things like camera
//   trigger and ball drop at exact positions under mission control much easier
//  */
// float turn_distance(float wp_radius, float turn_angle)
// {
//     float distance_90 = turn_distance(wp_radius);
//     turn_angle = fabsf(turn_angle);
//     if (turn_angle >= 90) {
//         return distance_90;
//     }
//     return distance_90 * turn_angle / 90.0f;
// }

// void update_waypoint(struct Locations &prev_WP,struct Locations &next_WP, float dist_min)
// {

//     Locations current_loc;
//     float Nu;
//     float xtrackVel;
//     float ltrackVel;

//     uint32_t now = micros();
//     float dt = (now - _last_update_waypoint_us) * 1.0e-6f;
//     if (dt > 0.1) {
//         dt = 0.1;
//         _L1_xtrack_i = 0.0f;
//     }
//     _last_update_waypoint_us = now;

//     // Calculate L1 gain required for specified damping
//     float K_L1 = 4.0f * _L1_damping * _L1_damping;

//     // Get current position and velocity
//     if (get_position(current_loc) == false) {
//         // if no GPS loc available, maintain last nav/target_bearing
//         _data_is_stale = true;
//         return;
//     }

//     Vector2f _groundspeed_vector(get_velocity_ned().x, get_velocity_ned().y);

//     // update _target_bearing_cd
//     _target_bearing_deg = current_loc.get_bearing_to(next_WP)/100.0f;

//     //Calculate groundspeed
//     float groundSpeed = _groundspeed_vector.length();
//     if (groundSpeed < 0.1f) {
//         // use a small ground speed vector in the right direction,
//         // allowing us to use the compass heading at zero GPS velocity
//         groundSpeed = 0.1f;
//         _groundspeed_vector = Vector2f(cosf(get_yaw()), sinf(get_yaw())) * groundSpeed;
//     }

//     // Calculate time varying control parameters
//     // Calculate the L1 length required for specified period
//     // 0.3183099 = 1/pi
//     _L1_dist = MAX(0.3183099f * _L1_damping * _L1_period * groundSpeed, dist_min);

//     // Calculate the NE position of WP B relative to WP A
//     Vector2f AB = prev_WP.get_distance_NE(next_WP);
//     float AB_length = AB.length();

//     // Check for AB zero length and track directly to the destination
//     // if too small
//     if (AB.length() < 1.0e-6f) {
//         AB = current_loc.get_distance_NE(next_WP);
//         if (AB.length() < 1.0e-6f) {
//             AB = Vector2f(cosf(get_yaw()), sinf(get_yaw()));
//         }
//     }
//     AB.normalize();

//     // Calculate the NE position of the aircraft relative to WP A
//     const Vector2f A_air = prev_WP.get_distance_NE(current_loc);

//     // calculate distance to target track, for reporting
//     _crosstrack_error = A_air % AB;

//     //Determine if the aircraft is behind a +-135 degree degree arc centred on WP A
//     //and further than L1 distance from WP A. Then use WP A as the L1 reference point
//     //Otherwise do normal L1 guidance
//     float WP_A_dist = A_air.length();
//     float alongTrackDist = A_air * AB;
//     if (WP_A_dist > _L1_dist && alongTrackDist/MAX(WP_A_dist, 1.0f) < -0.7071f)
//     {
//         //Calc Nu to fly To WP A
//         Vector2f A_air_unit = (A_air).normalized(); // Unit vector from WP A to aircraft
//         xtrackVel = _groundspeed_vector % (-A_air_unit); // Velocity across line
//         ltrackVel = _groundspeed_vector * (-A_air_unit); // Velocity along line
//         Nu = atan2f(xtrackVel,ltrackVel);
//         _nav_bearing = atan2f(-A_air_unit.y , -A_air_unit.x); // bearing (radians) from AC to L1 point
//     } else if (alongTrackDist > AB_length + groundSpeed*3) {
//         // we have passed point B by 3 seconds. Head towards B
//         // Calc Nu to fly To WP B
//         const Vector2f B_air = next_WP.get_distance_NE(current_loc);
//         Vector2f B_air_unit = (B_air).normalized(); // Unit vector from WP B to aircraft
//         xtrackVel = _groundspeed_vector % (-B_air_unit); // Velocity across line
//         ltrackVel = _groundspeed_vector * (-B_air_unit); // Velocity along line
//         Nu = atan2f(xtrackVel,ltrackVel);
//         _nav_bearing = atan2f(-B_air_unit.y , -B_air_unit.x); // bearing (radians) from AC to L1 point
//     } else { //Calc Nu to fly along AB line

//         //Calculate Nu2 angle (angle of velocity vector relative to line connecting waypoints)
//         xtrackVel = _groundspeed_vector % AB; // Velocity cross track
//         ltrackVel = _groundspeed_vector * AB; // Velocity along track
//         float Nu2 = atan2f(xtrackVel,ltrackVel);
//         //Calculate Nu1 angle (Angle to L1 reference point)
//         float sine_Nu1 = _crosstrack_error/MAX(_L1_dist, 0.1f);
//         //Limit sine of Nu1 to provide a controlled track capture angle of 45 deg
//         sine_Nu1 = constrain(sine_Nu1, -0.7071f, 0.7071f);
//         float Nu1 = asinf(sine_Nu1);

//         // compute integral error component to converge to a crosstrack of zero when traveling
//         // straight but reset it when disabled or if it changes. That allows for much easier
//         // tuning by having it re-converge each time it changes.
//         if (_L1_xtrack_i_gain <= 0 || !is_equal(_L1_xtrack_i_gain, _L1_xtrack_i_gain_prev)) {
//             _L1_xtrack_i = 0;
//             _L1_xtrack_i_gain_prev = _L1_xtrack_i_gain;
//         } else if (fabsf(Nu1) < radians(5)) {
//             _L1_xtrack_i += Nu1 * _L1_xtrack_i_gain * dt;

//             // an AHRS_TRIM_X=0.1 will drift to about 0.08 so 0.1 is a good worst-case to clip at
//             _L1_xtrack_i = constrain(_L1_xtrack_i, -0.1f, 0.1f);
//         }

//         // to converge to zero we must push Nu1 harder
//         Nu1 += _L1_xtrack_i;

//         Nu = Nu1 + Nu2;
//         _nav_bearing = atan2f(AB.y, AB.x) + Nu1; // bearing (radians) from AC to L1 point
//     }

//     _prevent_indecision(Nu);
//     _last_Nu = Nu;

//     //Limit Nu to +-(pi/2)
//     Nu = constrain(Nu, -1.5708f, +1.5708f);
//     _latAccDem = K_L1 * groundspeed * groundspeed / _L1_dist * sinf(Nu);

//     // Waypoint capture status is always false during waypoint following
//     _WPcircle = false;

//     _bearing_error = Nu; // bearing error angle (radians), +ve to left of track

//     _data_is_stale = false; // status are correctly updated with current waypoint data
// }

// //update L1 control for heading hold navigation
// void update_heading_hold(float navigation_heading_deg)
// {
//     // Calculate normalised frequency for tracking loop
//     const float omegaA = 4.4428f/_L1_period; // sqrt(2)*pi/period
//     // Calculate additional damping gain

//     int32_t Nu_cd;
//     float Nu;

//     // copy to _target_bearing_cd and _nav_bearing
//     _target_bearing_deg = wrap_180(navigation_heading_deg);
//     _nav_bearing = radians(navigation_heading_deg);

//     Nu_cd = _target_bearing_deg - wrap_180(heading);
//     Nu_cd = wrap_180(Nu_cd);
//     Nu = radians(Nu_cd);

//     Vector2f _groundspeed_vector(get_velocity_ned().x,get_velocity_ned().y);

//     //Calculate groundspeed
//     float groundSpeed = _groundspeed_vector.length();

//     // Calculate time varying control parameters
//     _L1_dist = groundSpeed / omegaA; // L1 distance is adjusted to maintain a constant tracking loop frequency
//     float VomegaA = groundSpeed * omegaA;

//     // Waypoint capture status is always false during heading hold
//     _WPcircle = false;

//     _crosstrack_error = 0;

//     _bearing_error = Nu; // bearing error angle (radians), +ve to left of track

//     // Limit Nu to +-pi
//     Nu = constrain(Nu, -M_PI_2, M_PI_2);
//     _latAccDem = 2.0f*sinf(Nu)*VomegaA;

//     _data_is_stale = false; // status are correctly updated with current waypoint data
// }

// float nav_bearing() {
//     return wrap_180(degrees(_nav_bearing));
// }

// float bearing_error() {
//     return degrees(_bearing_error);
// }

// float target_bearing() {
//     return wrap_180(_target_bearing_deg);
// }