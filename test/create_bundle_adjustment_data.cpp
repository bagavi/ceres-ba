#include <cassert>
#include <cmath>
#include <filesystem>
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

#define NUM_LMS 4000
#define NUM_FIX_KFS 30
#define NUM_VAR_KFS 25

// fills camera frame poses
void fill_cam_data(CameraFrames &camera_frames) {
    auto t_fx_fl = camera_frames.t_fl_fx;

    // front left
    memset(t_fx_fl, 0, sizeof(t_fx_fl[0]) * AA_POSE_DIM);

    // front right
    uint index = 1 * AA_POSE_DIM;  // filled random data
    // rotation
    t_fx_fl[index] = 0.3;
    t_fx_fl[index + 1] = 0.1;
    t_fx_fl[index + 2] = 0.2;
    // translation
    t_fx_fl[index + 3] = -.0605;
    t_fx_fl[index + 4] = .01;
    t_fx_fl[index + 5] = 0;
}

// generates random `num_lms` and stores them in `lms_gt` and their noisy
// version in `lms_noisy`
void fill_lm_data(double lm_noise, BALmData &lm_data) {
    auto lms_gt = lm_data.lms_gt;
    auto lms_noisy = lm_data.lms_noisy;
    for (uint lm_id = 0; lm_id < lm_data.num_lms; lm_id++) {
        auto lm = &lms_gt[POINT_DIM * lm_id];
        create_random_lm(lm);
    }
    deep_copy(lms_noisy, lms_gt, POINT_DIM * lm_data.num_lms);

    for (uint lm_id = 0; lm_id < lm_data.num_lms; lm_id++) {
        auto lm_noisy = &lms_noisy[POINT_DIM * lm_id];
        add_noise_lm(lm_noise, lm_noisy);
    }
}

void fill_kf_data(BAKfData &kf_data, double kf_noise, bool no_noise_first_kf) {
    for (uint kf_id = 0; kf_id < kf_data.num_kfs; kf_id++) {
        auto kf_t_g_ifl = &kf_data.kfs_gt[AA_POSE_DIM * kf_id];
        create_random_kf_aa(kf_t_g_ifl);
    }
    deep_copy(kf_data.kfs_noisy, kf_data.kfs_gt, AA_POSE_DIM * kf_data.num_kfs);

    for (uint kf_id = 0; kf_id < kf_data.num_kfs; kf_id++) {
        if (kf_id == 0 && no_noise_first_kf) {
            continue;  // dont add noise for the first frame
        }
        auto kf_t_g_ifl_noisy = &kf_data.kfs_noisy[AA_POSE_DIM * kf_id];
        add_noise_kf_aa(kf_noise, kf_t_g_ifl_noisy);
    }
}

void fill_corresp(CameraFrames &camera_frames, BALmData &lm_data,
                  BAKfData &kf_data, BACorresp &corresp, double bearing_noise) {
    double camera_count[camera_frames.num_cams];
    for (uint kf_id = 0; kf_id < kf_data.num_kfs; kf_id++) {
        auto kf_t_g_ifl = &kf_data.kfs_gt[AA_POSE_DIM * kf_id];

        for (uint lm_id = 0; lm_id < lm_data.num_lms; lm_id++) {
            if (rand_double() < 0.5) {  // using a fraction of lms
                continue;
            }

            auto cam_id = rand_int(camera_frames.num_cams);
            auto t_fl_fx = &camera_frames.t_fl_fx[AA_POSE_DIM * cam_id];

            auto lm = &lm_data.lms_gt[POINT_DIM * lm_id];
            double ifl_lm[POINT_DIM];
            transform_lm_aa(lm, kf_t_g_ifl, ifl_lm);

            double ifx_lm[POINT_DIM];
            transform_lm_aa(ifl_lm, t_fl_fx, ifx_lm);

            auto bearing = &corresp.bearings[BEARING_DIM * corresp.num_corresp];
            project_lm_to_bearing(ifx_lm, bearing);

            add_noise_bearing(bearing_noise, bearing);

            if (abs(bearing[0]) < BEARING_WINDOW_LEN &&
                abs(bearing[1]) < BEARING_WINDOW_LEN) {
                corresp.cam_index[corresp.num_corresp] = cam_id;
                corresp.kf_index[corresp.num_corresp] = kf_id;
                corresp.lm_index[corresp.num_corresp] = lm_id;
                corresp.num_corresp++;
                camera_count[cam_id]++;
            }
        }
    }
}

void create_ba_data(double lm_noise, double kf_noise, double bearing_noise, const char *filename) {
    srand(time(NULL));

    /// 1. camera_frames
    uint num_cams = 2;
    double t_fx_fl[AA_POSE_DIM * num_cams];
    CameraFrames camera_frames = {num_cams, t_fx_fl};
    fill_cam_data(camera_frames);

    /// 2. lms
    uint num_lms = NUM_LMS;
    double lms_gt[POINT_DIM * num_lms];
    double lms_noisy[POINT_DIM * num_lms];
    BALmData lm_data = {num_lms, lms_noisy, lms_gt};
    fill_lm_data(lm_noise, lm_data);

    /// 3. kfs
    // variable
    uint num_var_kfs = NUM_FIX_KFS;
    double var_kfs[AA_POSE_DIM * num_var_kfs];
    double var_kfs_noisy[AA_POSE_DIM * num_var_kfs];
    BAKfData var_kf_data = {
        num_var_kfs,
        var_kfs_noisy,
        var_kfs,
    };
    fill_kf_data(var_kf_data, kf_noise, false);
    // fixed
    uint num_fix_kfs = NUM_VAR_KFS;
    double fix_kfs[AA_POSE_DIM * num_fix_kfs];
    double fix_kfs_noisy[AA_POSE_DIM * num_fix_kfs];
    BAKfData fix_kf_data = {num_fix_kfs, fix_kfs_noisy, fix_kfs};
    fill_kf_data(fix_kf_data, kf_noise / 5., true);

    /// 4. correspondences
    // variable
    uint max_var_corresp = num_lms * num_var_kfs;
    double var_bearings[BEARING_DIM * max_var_corresp];
    unsigned int var_cam_index[max_var_corresp];
    unsigned int var_kf_index[max_var_corresp];
    unsigned int var_lm_index[max_var_corresp];
    BACorresp var_corresp = {0, var_bearings, var_cam_index, var_kf_index,
                             var_lm_index};
    fill_corresp(camera_frames, lm_data, var_kf_data, var_corresp, bearing_noise);

    // fixed
    uint max_fix_corresp = num_lms * num_fix_kfs;
    double fix_bearings[BEARING_DIM * max_fix_corresp];
    unsigned int fix_cam_index[max_fix_corresp];
    unsigned int fix_kf_index[max_fix_corresp];
    unsigned int fix_lm_index[max_fix_corresp];
    BACorresp fix_corresp = {0, fix_bearings, fix_cam_index, fix_kf_index,
                             fix_lm_index};
    fill_corresp(camera_frames, lm_data, fix_kf_data, fix_corresp, bearing_noise);

    BAData ba_data = {
        &camera_frames,
        &lm_data,
        &var_kf_data,
        &var_corresp,
        &fix_kf_data,
        &fix_corresp,
        0,
        0,
    };

    write_data(filename, ba_data, bearing_noise, lm_noise, kf_noise);
}

int main() {
    const char *filename = "../data/data.csv";
    double lm_noise = 0.1;         // mts
    double kf_noise = 0.1;         // mts
    double bearing_noise = 0.015;  // 3 pixels

    create_ba_data(lm_noise, kf_noise, bearing_noise, filename);
}
