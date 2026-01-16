/*
 * TinyEKF: Extended Kalman Filter for Arduino and TeensyBoard.
 *
 * Copyright (C) 2015 Simon D. Levy
 *
 * MIT License
 */
#pragma once

#include <stdio.h>
#include <stdlib.h>

#define NstaEKF2 4 // 4 state estimated (x,vx,y,vy)
#define MobsEKF2 4 // 4 state observed (x,vx,y,vy)

typedef struct {

    int n; /* number of state values */
    int m; /* number of observables */

    double x[NstaEKF2]; /* state vector */

    double P[NstaEKF2][NstaEKF2]; /* prediction error covariance */
    double Q[NstaEKF2][NstaEKF2]; /* process noise covariance */
    double R[MobsEKF2][MobsEKF2]; /* measurement error covariance */

    double G[NstaEKF2][MobsEKF2]; /* Kalman gain; a.k.a. K */

    double F[NstaEKF2][NstaEKF2]; /* Jacobian of process model */
    double H[MobsEKF2][NstaEKF2]; /* Jacobian of measurement model */

    double Ht[NstaEKF2][MobsEKF2]; /* transpose of measurement Jacobian */
    double Ft[NstaEKF2][NstaEKF2]; /* transpose of process Jacobian */
    double Pp[NstaEKF2][NstaEKF2]; /* P, post-prediction, pre-update */

    double fx[NstaEKF2]; /* output of user defined f() state-transition function */
    double hx[MobsEKF2]; /* output of user defined h() measurement function */

    /* temporary storage */
    double tmp0[NstaEKF2][NstaEKF2];
    double tmp1[NstaEKF2][MobsEKF2];
    double tmp2[MobsEKF2][NstaEKF2];
    double tmp3[MobsEKF2][MobsEKF2];
    double tmp4[MobsEKF2][MobsEKF2];
    double tmp5[MobsEKF2];

} ekf_t;


// Support both Arduino and command-line versions
#ifndef MAIN
extern "C" {
#endif
    void ekf_init(void *, int, int); //deklarasi fungsi inisialisai ekf, dengan parameter () , jumlah state, jumlah observe state
    int ekf_step(void *, double *); // dua f ini aada di tiny_ekf
#ifndef MAIN
}
#endif

/**
 * A header-only class for the Extended Kalman Filter.  Your implementing class should #define the constant N and
 * and then #include <TinyEKF.h>  You will also need to implement a model() method for your application.
 */
class TinyEKF {

private:

    ekf_t ekf;      // instansiasi objek ekf dari struct ekf_t

protected:

    /**
      * The current state.
      */
    double * x;     //membuat pointer naama x tipe double

    /**
     * Initializes a TinyEKF object.
     */
    TinyEKF() {     // ini konstruktor
        ekf_init(&this->ekf, NstaEKF2, MobsEKF2);
        this->x = this->ekf.x;
    }

    /**
     * Deallocates memory for a TinyEKF object.
     */
    ~TinyEKF() { }

    /**
     * Implement this function for your EKF model.
     * @param fx gets output of state-transition function <i>f(x<sub>0 .. n-1</sub>)</i>
     * @param F gets <i>n &times; n</i> Jacobian of <i>f(x)</i>
     * @param hx gets output of observation function <i>h(x<sub>0 .. n-1</sub>)</i>
     * @param H gets <i>m &times; n</i> Jacobian of <i>h(x)</i>
     */
    virtual void model(double fx[NstaEKF2], double F[NstaEKF2][NstaEKF2], double hx[MobsEKF2], double H[MobsEKF2][NstaEKF2]) = 0;

    /**
     * Sets the specified value of the prediction error covariance. <i>P<sub>i,j</sub> = value</i>
     * @param i row index
     * @param j column index
     * @param value value to set
     */
    void setP(int i, int j, double value) { //method setter untuk nilai matriks p
        this->ekf.P[i][j] = value;
    }

    /**
     * Sets the specified value of the process noise covariance. <i>Q<sub>i,j</sub> = value</i>
     * @param i row index
     * @param j column index
     * @param value value to set
     */
    void setQ(int i, int j, double value) { //method setter untk nilai matrix Q
        this->ekf.Q[i][j] = value;
    }

    /**
     * Sets the specified value of the observation noise covariance. <i>R<sub>i,j</sub> = value</i>
     * @param i row index
     * @param j column index
     * @param value value to set
     */
    void setR(int i, int j, double value) {
        this->ekf.R[i][j] = value;
    }

public:

    /**
     * Returns the state element at a given index.
     * @param i the index (at least 0 and less than <i>n</i>
     * @return state value at index
     */
    double getX(int i) {
        return this->ekf.x[i];
    }

    /**
     * Sets the state element at a given index.
     * @param i the index (at least 0 and less than <i>n</i>
     * @param value value to set
     */
    void setX(int i, double value) { // memasukan nilai pada matriks state
        this->ekf.x[i] = value;
    }

    /**
      Performs one step of the prediction and update.
     * @param z observation vector, length <i>m</i>
     * @return true on success, false on failure caused by non-positive-definite matrix.
     */
    bool step(double * z) {
        this->model(this->ekf.fx, this->ekf.F, this->ekf.hx, this->ekf.H);
        return ekf_step(&this->ekf, z) ? false : true;
    }
};
