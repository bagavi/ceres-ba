#pragma once

#define BEARING_DIM 2
#define POINT_DIM 3
#define AA_ROT_DIM 3
#define AA_POSE_DIM 6
#define QUAT_ROT_DIM 4
#define QUAT_POSE_DIM 7
#define BEARING_WINDOW_LEN 1.0
#define LM_SEED_START 10000

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _OptSummary {
    double initial_cost;
    double final_cost;
    double total_time_in_seconds;
    int success;

} OptSummary;

// BA: Camera poses wrt to the a fixed frame
typedef struct _CameraFrames {
    unsigned int num_cams;
    // pose of `x` camera w.r.t front left camera frame
    double* t_fl_fx;  //size: POSE_DIM * num_cams
} CameraFrames;

// BA: lm_data with their initial point estimate
typedef struct _BALmData {
    unsigned int num_lms;
    double* lms_noisy;  // size: POINT_DIM * num_lms // position of the lm
    double* lms_gt;     // same as above. used only for debugging
} BALmData;

// BA: kfs_noisy with their initial pose estimate
typedef struct _BAKfData {
    unsigned int num_kfs;
    double* kfs_noisy;  // size: POSE_DIM*num_kfs // pose of the kf
    double* kfs_gt;     // same as above. used only for debugging
} BAKfData;

// BA: correspondences between kf and lm ids
typedef struct _BACorresp {
    // lm-kf edges
    unsigned int num_corresp;
    double* bearings;         // size: BEARING_DIM*num_corresp // bearings co-ordinates
    unsigned int* cam_index;  // size: num_corresp
    unsigned int* kf_index;   // size: num_corresp
    unsigned int* lm_index;   // size: num_corresp
} BACorresp;

typedef struct _BAData {
    CameraFrames* camera_frames;
    BALmData* lm_data;
    // kfs pose are variable (to be optimized)
    BAKfData* var_kf_data;
    BACorresp* var_corresp;
    // kfs pose are fixed (not to be optimized)
    BAKfData* fix_kf_data;
    BACorresp* fix_corresp;
    // Solver options
    double max_solver_time_in_seconds;
    double huber_loss_thresh;
} BAData;

// wrapper on ceres data structure
typedef struct _CeresOptionWarpper CeresOptionWrapper;    // Forward declaration, defined below. Used by c code
typedef struct _CeresProblemWrapper CeresProblemWrapper;  // Forward declaration, defined below. Used by c code

// Bundle adjustment
// a. set optmization options/paramaters
CeresOptionWrapper* get_default();
void options_set_num_threads(CeresOptionWrapper* ceres_opts, int num_threads);
void options_set_max_solve_time(CeresOptionWrapper* ceres_opts, double max_solver_time_in_secs);
void options_set_num_iterations(CeresOptionWrapper* ceres_opts, int max_num_iterations);

// b. construst and solve ba
CeresProblemWrapper* construct_ba(BAData* data, CeresOptionWrapper* ceres_opts, const unsigned int* new_kf);
OptSummary solve_ba(CeresProblemWrapper* ceres_prob, CeresOptionWrapper* ceres_opts, int print_summary);

// test code
void test_binding_add_one(int* a);

#ifdef __cplusplus
}
#endif

// C++ code use this
#if defined(__cplusplus)
#include <ceres/ceres.h>

extern "C" {

struct _CeresOptionWarpper {
    ceres::Solver::Options opts;
};

struct _CeresProblemWrapper {
    ceres::Problem ceres_prob;
};
}

#endif /* __cplusplus */
