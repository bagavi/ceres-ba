#pragma once

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include "c_api.h"

// Bundle adjustment has edges, where each edge connects a landmark and a keyframe
// Each node could be fixed or free to be optimized

struct BAVarKfVarLmErr {  // both lm point and kf pose are variables
    BAVarKfVarLmErr(double* bearing, double* t_fl_fx) : bearing_(bearing), t_fl_fx_(t_fl_fx) {}

    template <typename T>
    bool operator()(const T* const kf_t_g_ifl, const T* const lm, T* residuals) const {
        // 1.a Transform form global  to left
        // kf_t_g_ifl[0,1,2] are the angle-axis rotation.
        T ifl_lm[POINT_DIM];
        ceres::AngleAxisRotatePoint(kf_t_g_ifl, lm, ifl_lm);
        // kf_t_g_ifl[3,4,5] are the translation.
        ifl_lm[0] += kf_t_g_ifl[3];
        ifl_lm[1] += kf_t_g_ifl[4];
        ifl_lm[2] += kf_t_g_ifl[5];

        //1.b Transform left to `x` frame
        T t_fl_fx[AA_POSE_DIM];
        t_fl_fx[0] = T(t_fl_fx_[0]);
        t_fl_fx[1] = T(t_fl_fx_[1]);
        t_fl_fx[2] = T(t_fl_fx_[2]);

        // transform from front left camera frame to front right camera frame
        T ifx_lm[POINT_DIM];
        // t_fl_fx [1,2, 3] encodes axis angle rotation
        ceres::AngleAxisRotatePoint(t_fl_fx, ifl_lm, ifx_lm);
        // t_fl_fx_ [3,4,5] are the translation.
        ifx_lm[0] += t_fl_fx_[3];
        ifx_lm[1] += t_fl_fx_[4];
        ifx_lm[2] += t_fl_fx_[5];

        //2. Project
        T est_bearing[BEARING_DIM];

        est_bearing[0] = ifx_lm[0] / ifx_lm[2];
        est_bearing[1] = ifx_lm[1] / ifx_lm[2];

        // The error is the difference between the predicted and observed position.
        residuals[0] = est_bearing[0] - bearing_[0];
        residuals[1] = est_bearing[1] - bearing_[1];

        return true;
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create(double* bearing, double* t_fl_fx) {
        return (new ceres::AutoDiffCostFunction<BAVarKfVarLmErr, BEARING_DIM, AA_POSE_DIM, POINT_DIM>(
            new BAVarKfVarLmErr(bearing, t_fl_fx)));
    }

    // fixed data
    double* bearing_;
    double* t_fl_fx_;  // transforms lm from front left to front `x` camera frame
};

struct BAFixedKfVarLmErr {  //  lm point is variable and kf pose is fixed
    BAFixedKfVarLmErr(double* bearing, double* t_fl_fx, double* kf_t_g_ifl) : bearing_(bearing), t_fl_fx_(t_fl_fx), kf_t_g_ifl_(kf_t_g_ifl) {}

    template <typename T>
    bool operator()(const T* const lm, T* residuals) const {
        //1.a Transform global to left frame
        // kf_t_g_ifl[0,1,2] are the angle-axis rotation.
        T kf_t_g_ifl[POINT_DIM];
        kf_t_g_ifl[0] = T(kf_t_g_ifl_[0]);
        kf_t_g_ifl[1] = T(kf_t_g_ifl_[1]);
        kf_t_g_ifl[2] = T(kf_t_g_ifl_[2]);

        T ifl_lm[POINT_DIM];
        ceres::AngleAxisRotatePoint(kf_t_g_ifl, lm, ifl_lm);
        // kf_t_g_ifl[3,4,5] are the translation.
        ifl_lm[0] += kf_t_g_ifl_[3];
        ifl_lm[1] += kf_t_g_ifl_[4];
        ifl_lm[2] += kf_t_g_ifl_[5];

        //1.b Transform left frame to `x` frame
        T t_fl_fx[AA_POSE_DIM];
        t_fl_fx[0] = T(t_fl_fx_[0]);
        t_fl_fx[1] = T(t_fl_fx_[1]);
        t_fl_fx[2] = T(t_fl_fx_[2]);

        // transform from front left camera frame to front right camera frame
        T ifx_lm[POINT_DIM];
        ceres::AngleAxisRotatePoint(t_fl_fx, ifl_lm, ifx_lm);  // t_fl_fx [1,2, 3] encodes axis angle rotation. //todo t_l_r does not have rotation!
        // t_fl_fx_ [3,4,5] are the translation.
        ifx_lm[0] += t_fl_fx_[3];
        ifx_lm[1] += t_fl_fx_[4];
        ifx_lm[2] += t_fl_fx_[5];

        // 2. Project
        T est_bearing[BEARING_DIM];

        est_bearing[0] = ifx_lm[0] / ifx_lm[2];
        est_bearing[1] = ifx_lm[1] / ifx_lm[2];

        // The error is the difference between the predicted and observed position.
        residuals[0] = est_bearing[0] - bearing_[0];
        residuals[1] = est_bearing[1] - bearing_[1];

        return true;
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create(double* bearing, double* t_fl_fx, double* kf_t_g_ifl) {
        return (new ceres::AutoDiffCostFunction<BAFixedKfVarLmErr, BEARING_DIM, POINT_DIM>(
            new BAFixedKfVarLmErr(bearing, t_fl_fx, kf_t_g_ifl)));
    }

    // fixed data
    double* bearing_;
    double* t_fl_fx_;  // transforms lm from front left to front `x` camera frame
    double* kf_t_g_ifl_;
};
