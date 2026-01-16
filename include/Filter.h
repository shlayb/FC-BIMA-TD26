#pragma once

float Xt, Xt_update, Xt_prev;
float Pt, Pt_update, Pt_prev;
float Kt;
float R, Q;

// void initializeKalman() {
//     R = 5;
//     Q = 0.2;
//     Pt_prev = 1;
// }

// float kalmanFilter(float data) {
//     // Predict
//     Xt_update = Xt_prev;
//     Pt_update = Pt_prev + Q;
//     // Update
//     Kt = Pt_update / (Pt_update + R);
//     Xt = Xt_update + (Kt * (data - Xt_update));
//     Pt = (1 - Kt) * Pt_update;

//     Xt_prev = Xt;
//     Pt_prev = Pt;

//     return Xt;
// }

class KalmanFilter {
    public:
    void initializeKalman(float R_value, float Q_value, float Pt_value);
    float kalmanFilter(float data);

    private:
    float Xt, Xt_update, Xt_prev;
    float Pt, Pt_update, Pt_prev;
    float Kt;
    float R, Q;

};

void KalmanFilter::initializeKalman(float R_value, float Q_value, float Pt_value) {
    R = R_value;
    Q = Q_value;
    Pt_prev = Pt_value;
}

float KalmanFilter::kalmanFilter(float data) {
    // Predict
    Xt_update = Xt_prev;
    Pt_update = Pt_prev + Q;
    // Update
    Kt = Pt_update / (Pt_update + R);
    Xt = Xt_update + (Kt * (data - Xt_update));
    Pt = (1 - Kt) * Pt_update;

    Xt_prev = Xt;
    Pt_prev = Pt;

    return Xt;
}