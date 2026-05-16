#pragma once
#include "Arduino.h"
#include "AHRS.h"
#include "Radio.h"
#include "Mode_setup.h"
#include "Mode_Flywing.h"
#include "Mode_Copter.h"
#include "Mode_Fixedwing.h"
#include "Mode_Transisi.h"

enum UAVModelType {
    MODEL_UAV_FIXEDWING = 0,
    MODEL_UAV_VTOL      = 1
};

// Nilai ini harus berasal dari param yang kamu tambahkan sebelumnya
extern float model_uav;

void setup_mode() {
    ahrs.update_ahrs();
    nav_gps();
    modes.update();

    int model = (int)(model_uav + 0.5f);

    // ============================================================
    //  FULL FIXED WING
    // ============================================================
    if (model == MODEL_UAV_FIXEDWING) {

        if (mode_now == 1) {
            if (active_mode != ModeId::MANU) {
                modes.setMode(ModeId::MANU);
                active_mode = ModeId::MANU;
                current = ModeId::MANU;
            }
        }
        else if (mode_now == 2) {
            if (active_mode != ModeId::FBWA) {
                modes.setMode(ModeId::FBWA);
                active_mode = ModeId::FBWA;
                current = ModeId::FBWA;
            }
        }
        else if (mode_now == 3) {
            if (active_mode != ModeId::GUIDED) {
                modes.setMode(ModeId::GUIDED);
                active_mode = ModeId::GUIDED;
                current = ModeId::GUIDED;
            }
        }
        else if (mode_now == 4) {
            if (active_mode != ModeId::AUTO) {
                modes.setMode(ModeId::AUTO);
                active_mode = ModeId::AUTO;
                current = ModeId::AUTO;
            }
        }

        return;
    }

    // ============================================================
    //  VTOL PLANE
    // ============================================================
    else if (model == MODEL_UAV_VTOL) {

        if (is_transitioning) {
            update_transition();
            return;
        }

        ModeId want = active_mode;

        // mode_now:
        // 1 = Multicopter / COPT
        // 2 = Fixed Wing / FBWA
        // 3 = AUTO
        // 4 = GUIDED

        if (mode_now == 1) {

            // FW -> MC
            if (active_mode == ModeId::FBWA) {
                modes.suspend();
                start_transition_to_mc();
                return;
            }

            want = ModeId::COPT;
        }

        else if (mode_now == 2) {

            // MC -> FW
            if (active_mode != ModeId::FBWA &&
                active_mode != ModeId::AUTO &&
                active_mode != ModeId::GUIDED) {

                modes.suspend();
                start_transition_to_fw();
                return;
            }

            want = ModeId::FBWA;
        }

        else if (mode_now == 3) {
            want = ModeId::AUTO;
        }

        else if (mode_now == 4) {
            want = ModeId::GUIDED;
        }

        else {
            return;
        }

        if (want != active_mode) {
            if (modes.setMode(want)) {
                active_mode = want;
                current = want;
            }
        }

        return;
    }

    // ============================================================
    //  FALLBACK kalau nilai MODEL_UAV aneh
    // ============================================================
    else {
        if (active_mode != ModeId::MANU) {
            modes.setMode(ModeId::MANU);
            active_mode = ModeId::MANU;
            current = ModeId::MANU;
        }

        return;
    }
}