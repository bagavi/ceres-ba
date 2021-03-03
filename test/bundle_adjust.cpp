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
    };

    auto ceres_options = get_default();
    auto ba_prob = construct_ba(&ba_data, ceres_options);
    options_set_num_threads(ceres_options, 1);
    options_set_max_solve_time(ceres_options, time_in_sec);
    options_set_num_iterations(ceres_options, num_iterations + 1);

    auto summary = solve_ba(ba_prob, ceres_options, 1);

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
