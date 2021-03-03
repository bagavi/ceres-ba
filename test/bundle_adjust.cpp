#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include <ostream>
#include <random>

#include "../src/c_api.h"
#include "../src/read_write.h"
#include "ceres/rotation.h"
#include "utils.h"

using std::cout;
using std::endl;
using std::pair;

// calculates the bearing error
void calc_bearing_error(CameraFrames &camera_frames, BALmData &lm_data,
                        BAKfData &kf_data, BACorresp &corresp, double *lm_err,
                        double *kf_err) {
    auto lms = lm_data.lms_noisy;
    auto kfs = kf_data.kfs_noisy;

    double lm_total_error[lm_data.num_lms], kf_total_error[kf_data.num_kfs];
    uint lm_obs_count[lm_data.num_lms], kf_obs_count[kf_data.num_kfs];

    for (uint i = 0; i < lm_data.num_lms; i++) {
        lm_total_error[i] = 0;
        lm_obs_count[i] = 0;
    }
    for (uint i = 0; i < kf_data.num_kfs; i++) {
        kf_total_error[i] = 0;
        kf_obs_count[i] = 0;
    }

    for (uint i = 0; i < corresp.num_corresp; i++) {
        auto cam_id = corresp.cam_index[i];
        auto lm_id = corresp.lm_index[i];
        auto kf_id = corresp.kf_index[i];
        auto lm = &lms[lm_id * POINT_DIM];
        auto kf_t_g_ifl = &kfs[kf_id * AA_POSE_DIM];
        auto t_fl_fx = &camera_frames.t_fl_fx[cam_id * AA_POSE_DIM];
        auto bearing = &corresp.bearings[i * BEARING_DIM];

        double ifl_lm[POINT_DIM];
        transform_lm_aa(lm, kf_t_g_ifl, ifl_lm);
        double ifx_lm[POINT_DIM];
        transform_lm_aa(ifl_lm, t_fl_fx, ifx_lm);

        double est_bearing[BEARING_DIM];
        project_lm_to_bearing(ifx_lm, est_bearing);
        auto err = get_bearing_diff(bearing, est_bearing);
        // update err
        lm_total_error[lm_id] += err;
        kf_total_error[kf_id] += err;
        lm_obs_count[lm_id]++;
        kf_obs_count[kf_id]++;
    }
    for (uint kf_id = 0; kf_id < kf_data.num_kfs; kf_id++) {
        kf_err[kf_id] = sqrt(kf_total_error[kf_id] / kf_obs_count[kf_id]);
    }
    uint zero_obs_lms = 0;
    for (uint lm_id = 0; lm_id < lm_data.num_lms; lm_id++) {
        if (lm_obs_count[lm_id] > 0) {
            lm_err[lm_id] = sqrt(lm_total_error[lm_id] / lm_obs_count[lm_id]);
        }
        if (lm_obs_count[lm_id] == 0) {
            zero_obs_lms++;
        }
    }
}

double calc_lm_error(BALmData &lm_data, double lm_noise) {
    double sum_err_sq = 0;
    unsigned int count = 0;
    for (uint lm_id = 0; lm_id < lm_data.num_lms; lm_id++) {
        auto lm_gt = &lm_data.lms_gt[lm_id * POINT_DIM];
        auto lm_noisy = &lm_data.lms_noisy[lm_id * POINT_DIM];
        auto trans_err_sq = get_position_err_sq(lm_gt, lm_noisy);
        if (trans_err_sq < lm_noise * 10) {
            sum_err_sq += trans_err_sq;
            count++;
        } else if (lm_noise > 0) {
            cout << "lm_id:" << lm_noise << "\tErr: " << trans_err_sq << endl;
        }
    }
    return sqrt(sum_err_sq / count);
}

pair<double, double> calc_kf_error(BAKfData &kf_data, double kf_noise) {
    double sum_trans_err_sq = 0;
    double sum_rot_err_sq = 0;
    unsigned int count = 0;
    for (uint kf_id = 0; kf_id < kf_data.num_kfs; kf_id++) {
        auto kf_gt = &kf_data.kfs_gt[kf_id * AA_POSE_DIM];
        auto kf_noisy = &kf_data.kfs_noisy[kf_id * AA_POSE_DIM];
        auto trans_err_sq = get_trans_err_sq_aa(kf_gt, kf_noisy);
        auto rot_err_sq = get_rot_err_sq_aa(kf_gt, kf_noisy);
        if (trans_err_sq < kf_noise * 10) {
            sum_trans_err_sq += trans_err_sq;
            sum_rot_err_sq += rot_err_sq;
            count++;
        } else if (kf_noise > 0) {
            cout << "1kf_id:" << kf_id << "\tErr: " << trans_err_sq << endl;
        }
    }
    return std::make_pair(sqrt(sum_trans_err_sq / count),
                          sqrt(sum_rot_err_sq / count));
}

OptSummary run_ba(const char *filename, double time_in_sec, int num_iterations) {
    std::ifstream file(filename);

    //1. camera
    unsigned int num_cams = read_uint(file);
    double t_fx_fl[AA_POSE_DIM * num_cams];
    CameraFrames camera_frames = {num_cams, t_fx_fl};
    read_camera(camera_frames, file);

    /// 2. lms
    unsigned int num_lms = read_uint(file);
    double lms_gt[POINT_DIM * num_lms];
    double lms_noisy[POINT_DIM * num_lms];
    BALmData lm_data = {num_lms, lms_noisy, lms_gt};
    read_lm_data(lm_data, file);

    /// 3. variable kfs
    unsigned int num_var_kfs = read_uint(file);
    double var_kfs[AA_POSE_DIM * num_var_kfs];
    double var_kfs_noisy[AA_POSE_DIM * num_var_kfs];
    BAKfData var_kf_data = {
        num_var_kfs,
        var_kfs_noisy,
        var_kfs,
    };
    read_kf_data(var_kf_data, file);

    /// 4. varible correspondences
    unsigned int num_var_corresp = read_uint(file);
    double var_bearings[BEARING_DIM * num_var_corresp];
    unsigned int var_cam_index[num_var_corresp];
    unsigned int var_kf_index[num_var_corresp];
    unsigned int var_lm_index[num_var_corresp];
    BACorresp var_corresp = {num_var_corresp, var_bearings, var_cam_index, var_kf_index,
                             var_lm_index};
    read_corresp_data(var_corresp, file);

    /// 5. Fix kfs
    unsigned int num_fix_kfs = read_uint(file);
    double fix_kfs[AA_POSE_DIM * num_fix_kfs];
    double fix_kfs_noisy[AA_POSE_DIM * num_fix_kfs];
    BAKfData fix_kf_data = {num_fix_kfs, fix_kfs_noisy, fix_kfs};
    read_kf_data(fix_kf_data, file);

    /// 6. Var corresp
    unsigned int num_fix_corresp = read_uint(file);
    double fix_bearings[BEARING_DIM * num_fix_corresp];
    unsigned int fix_cam_index[num_fix_corresp];
    unsigned int fix_kf_index[num_fix_corresp];
    unsigned int fix_lm_index[num_fix_corresp];
    BACorresp fix_corresp = {num_fix_corresp, fix_bearings, fix_cam_index, fix_kf_index,
                             fix_lm_index};
    read_corresp_data(fix_corresp, file);

    BAData ba_data = {
        &camera_frames,
        &lm_data,
        &var_kf_data,
        &var_corresp,
        &fix_kf_data,
        &fix_corresp,
        time_in_sec,
        300,
    };
    double bearing_noise = read_double(file);
    double kf_noise = read_double(file);
    double lm_noise = read_double(file);

    double pre_opt_kf_err[var_kf_data.num_kfs],
        post_opt_kf_err[var_kf_data.num_kfs];
    double pre_opt_lm_err[lm_data.num_lms], post_opt_lm_err[lm_data.num_lms];

    double init_lm_trans_err = calc_lm_error(lm_data, lm_noise);
    auto init_kf_pose_err = calc_kf_error(var_kf_data, kf_noise);
    calc_bearing_error(camera_frames, lm_data, var_kf_data, var_corresp,
                       pre_opt_lm_err, pre_opt_kf_err);

    auto ceres_options = get_default();
    auto ba_prob = construct_ba(&ba_data, ceres_options);
    options_set_num_threads(ceres_options, 1);
    options_set_max_solve_time(ceres_options, time_in_sec);
    options_set_num_iterations(ceres_options, num_iterations + 1);

    auto summary = solve_ba(ba_prob, ceres_options, 1);
    // auto summary2 = solve_ba(ba_prob, ceres_options, 1);
    calc_bearing_error(camera_frames, lm_data, var_kf_data, var_corresp,
                       post_opt_lm_err, post_opt_kf_err);

    double final_lm_trans_err = calc_lm_error(lm_data, lm_noise);
    auto final_kf_pose_err = calc_kf_error(var_kf_data, kf_noise);

    uint kf_proj_err_outliers = 0;
    for (uint i = 0; i < var_kf_data.num_kfs; i++) {
        if (post_opt_kf_err[i] > 1.5 * bearing_noise) {
            cout << "BA: Kf id:" << i << "Bearing Err:" << post_opt_kf_err[i] << "|"
                 << pre_opt_kf_err[i]
                 << ". Ratio: " << pre_opt_kf_err[i] / post_opt_kf_err[i] << endl;
            kf_proj_err_outliers++;
        }
    }

    uint lm_proj_err_outliers = 0;
    for (uint i = 0; i < lm_data.num_lms; i++) {
        auto ratio = post_opt_lm_err[i] / pre_opt_lm_err[i];
        if (post_opt_lm_err[i] > 1.5 * bearing_noise) {
            // cout << "BA: Lm id:" << i << "Bearing Err:" << post_opt_lm_err[i] <<
            // "|" << pre_opt_lm_err[i]
            //      << ". Ratio: " << ratio << endl;
            lm_proj_err_outliers++;
        }
    }

    uint variables = num_lms * POINT_DIM + num_var_kfs * AA_POSE_DIM;
    uint constraints =
        (var_corresp.num_corresp + fix_corresp.num_corresp) * BEARING_DIM;
    cout << "BA: Number of lm_data: " << num_lms << endl;
    cout << "BA: Number of var_kfs_noisy: " << num_var_kfs << endl;
    cout << "BA: Number of fix_kfs_noisy: " << num_fix_kfs << endl;
    cout << "BA: Number of variables: " << variables << endl;
    cout << "BA: Number of obs (constrains): " << constraints << endl;
    cout << "BA: Bearing noise: " << bearing_noise << endl;
    cout << "BA: KF noise: " << kf_noise << endl;
    cout << "BA: LM noise: " << lm_noise << endl;
    cout << "BA: Lm outliers: " << lm_proj_err_outliers << "|" << num_lms << endl;
    cout << "BA: KF outliers: " << kf_proj_err_outliers << "|" << num_var_kfs
         << endl;
    cout << "BA: Lm Avg Bearing error"
         << average(post_opt_lm_err, lm_data.num_lms) << "|"
         << average(pre_opt_lm_err, lm_data.num_lms) << endl;
    cout << "BA: Kf Avg Bearing error"
         << average(post_opt_kf_err, var_kf_data.num_kfs) << "|"
         << average(pre_opt_kf_err, var_kf_data.num_kfs) << endl;

    cout << "BA: Lm avg trans error\t" << final_lm_trans_err << "|"
         << init_lm_trans_err << endl;
    cout << "BA: Kf avg trans error\t" << final_kf_pose_err.first << "|"
         << init_kf_pose_err.first << endl;
    cout << "BA: Kf avg rot error\t" << final_kf_pose_err.second << "|"
         << init_kf_pose_err.second << endl;

    return summary;
}

int main() {
    const char *filename = "../data/data.csv";

    double time_in_sec = 2;
    int num_iterations = 100;
    auto summary = run_ba(filename, time_in_sec, num_iterations);
    cout << "BA: Initial cost:" << summary.initial_cost
         << "\tFinal cost:" << summary.final_cost
         << "\tRatio cost:" << summary.initial_cost / summary.final_cost
         << "\tTime: " << summary.total_time_in_seconds << endl;
}
