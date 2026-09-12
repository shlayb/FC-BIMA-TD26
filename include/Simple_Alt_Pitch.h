#pragma once
#include <Arduino.h>

// ===================================================================
//  Simple Altitude Hold Controller (PID altitude -> pitch)
//  Alternatif TECS untuk kondisi tanpa airspeed sensor (GPS-only)
// ===================================================================

extern float _maxClimbRate;
extern float _maxSinkRate;
extern float _climb_rate;

#ifndef GET_CLIMB_RATE_DEFINED
#define GET_CLIMB_RATE_DEFINED
inline float get_climb_rate() { return _climb_rate; }
#endif

// --- Tuning parameters ---
inline float alt_kP        = 0.06f;   // derajat pitch per meter error altitude
inline float alt_kI        = 0.005f;  // integral gain, kecil untuk hilangkan steady-state error
inline float alt_kD        = 0.15f;   // derajat pitch per (m/s) climb rate error -> ini yang meredam osilasi
inline float alt_pitch_ff  = 0.0f;   // feedforward pitch untuk level flight (biasanya 0, tp bisa disesuaikan trim)

inline float alt_integrator     = 0.0f;
inline float alt_integrator_max = 8.0f;   // batas integrator dalam satuan derajat, cegah windup

// Batas pitch demand (derajat)
inline float simple_pitch_max_deg = 15.0f;
inline float simple_pitch_min_deg = -15.0f;

// Batas rate perubahan pitch demand per detik (derajat/detik) -- untuk kehalusan
inline float simple_pitch_rate_limit = 8.0f;
inline float _last_simple_pitch_dem  = 0.0f;

// Toggle untuk memilih apakah memakai Simple Alt Pitch atau TECS
inline float use_simple_alt_pitch = 1.0f; // 1.0 = aktifkan Simple Pitch, 0.0 = pakai TECS

inline void reset_altitude_pitch_controller() {
    alt_integrator = 0.0f;
    _last_simple_pitch_dem = 0.0f;
}

inline float update_altitude_pitch_controller(float target_alt_m, float current_alt_m,
                                              float climb_rate_mps, float dt)
{
    // Error altitude (positif = kurang tinggi, perlu naik)
    float alt_error = target_alt_m - current_alt_m;

    // Target climb rate proporsional terhadap error, dibatasi oleh max climb/sink rate
    // supaya tidak minta climb rate yang mustahil dicapai airframe
    float desired_climb_rate = constrain(alt_error * 0.3f, -_maxSinkRate, _maxClimbRate);

    // Error climb rate (ini komponen D, meredam overshoot & oscillation)
    float climb_rate_error = desired_climb_rate - climb_rate_mps;

    // Proportional term dari altitude error langsung
    float p_term = alt_kP * alt_error;

    // Derivative-like term dari climb rate error (paling penting untuk redam osilasi)
    float d_term = alt_kD * climb_rate_error;

    // Integral term -- akumulasi pelan, dengan anti-windup
    // Hanya integrasi kalau pitch demand belum mentok limit (dicek di bawah)
    float pitch_dem_unc = alt_pitch_ff + p_term + d_term + alt_integrator;

    bool would_saturate_high = (pitch_dem_unc > simple_pitch_max_deg);
    bool would_saturate_low  = (pitch_dem_unc < simple_pitch_min_deg);

    if (!(would_saturate_high && alt_error > 0) && !(would_saturate_low && alt_error < 0)) {
        alt_integrator += alt_error * alt_kI * dt;
        alt_integrator = constrain(alt_integrator, -alt_integrator_max, alt_integrator_max);
    }
    // kalau saturasi searah dengan error, integrator TIDAK diupdate sama sekali
    // (bukan di-fade-out seperti TECS -- ini yang mencegah limit-cycle)

    // Hitung ulang pitch demand final setelah update integrator
    float pitch_dem = alt_pitch_ff + p_term + d_term + alt_integrator;

    // Constrain ke limit pitch
    pitch_dem = constrain(pitch_dem, simple_pitch_min_deg, simple_pitch_max_deg);

    // Rate limit supaya perubahan pitch demand halus, tidak tersentak
    float max_step = simple_pitch_rate_limit * dt;
    if ((pitch_dem - _last_simple_pitch_dem) > max_step) {
        pitch_dem = _last_simple_pitch_dem + max_step;
    } else if ((pitch_dem - _last_simple_pitch_dem) < -max_step) {
        pitch_dem = _last_simple_pitch_dem - max_step;
    }
    _last_simple_pitch_dem = pitch_dem;

    return pitch_dem;   // derajat, siap dipakai sebagai nav_pitch_deg
}
