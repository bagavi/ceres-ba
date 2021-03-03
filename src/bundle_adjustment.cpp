#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <fstream>
#include <set>

#include "ba_reproj_error_aa.h"
#include "c_api.h"

#ifdef __cplusplus
extern "C" {
#endif

using std::cout;
using std::endl;
// Constructs the ceres problem from `data` and adds appropriate ordering in Options.
CeresProblemWrapper *construct_ba(BAData *data, CeresOptionWrapper *ceres_opts) {
    CeresProblemWrapper *problem = new CeresProblemWrapper();
    auto &ceres_prob = problem->ceres_prob;

    CameraFrames *camera_frames = data->camera_frames;
    BALmData *lm_data = data->lm_data;

    // 1. Adding correspondences with variable kfs
    BAKfData *var_kf_data = data->var_kf_data;
    BACorresp *var_corresp = data->var_corresp;

    std::set<unsigned int> obs_lms_ids;  // lm ids with >0 observation
    std::set<unsigned int> obs_kfs_ids;  // kf ids with >0 observation
    for (uint i = 0; i < var_corresp->num_corresp; i++) {
        auto cam_id = var_corresp->cam_index[i];
        auto t_fl_fx = &camera_frames->t_fl_fx[AA_POSE_DIM * cam_id];

        unsigned int kf_id = var_corresp->kf_index[i];
        auto kf_t_g_ifl = &var_kf_data->kfs_noisy[AA_POSE_DIM * kf_id];
        obs_kfs_ids.insert(kf_id);

        unsigned int lm_id = *(var_corresp->lm_index + i);
        auto lm = &lm_data->lms_noisy[POINT_DIM * lm_id];
        obs_lms_ids.insert(lm_id);

        auto bearing = &var_corresp->bearings[BEARING_DIM * i];
        ceres::LossFunction *loss_function =
            NULL;  // new ceres::HuberLoss(data->huber_loss_thresh);
        ceres::CostFunction *residue_func =
            VarKfVarLmErr::Create(bearing, t_fl_fx);  // fixed data
        ceres_prob.AddResidualBlock(residue_func, loss_function, kf_t_g_ifl,
                                    lm);  // var data
    }

    // 2. Adding correspondences with fixed kfs
    BAKfData *fix_kf_data = data->fix_kf_data;
    BACorresp *fix_corresp = data->fix_corresp;

    for (uint i = 0; i < fix_corresp->num_corresp; i++) {
        auto cam_id = fix_corresp->cam_index[i];
        auto t_fl_fx = &camera_frames->t_fl_fx[AA_POSE_DIM * cam_id];

        unsigned int kf_id = fix_corresp->kf_index[i];
        auto kf_t_g_ifl = &fix_kf_data->kfs_noisy[AA_POSE_DIM * kf_id];

        unsigned int lm_id = fix_corresp->lm_index[i];
        auto lm = &lm_data->lms_noisy[POINT_DIM * lm_id];
        obs_lms_ids.insert(lm_id);

        auto bearing = &fix_corresp->bearings[BEARING_DIM * i];
        ceres::LossFunction *loss_function = NULL;
        ceres::CostFunction *residue_func =
            FixedKfVarLmErr::Create(bearing, t_fl_fx, kf_t_g_ifl);     // fixed data
        ceres_prob.AddResidualBlock(residue_func, loss_function, lm);  // var data
    }

    // 3. Ordering the parameters.
    /*
        Idea:
        a. Two variables are "independent" if they dont share a common equations
        Thus by def, cameras for an independent set and so does landmark
        b. We group all the landmarks into group 0 and cameras into group 1
        c. While solving the linear equation group 0 elements are eliminated first and then group 1.
    */
    ceres_opts->opts.linear_solver_ordering.reset(
        new ceres::ParameterBlockOrdering);

    // 3a. Adding points
    for (uint lm_id = 0; lm_id < lm_data->num_lms; lm_id++) {
        if (obs_lms_ids.find(lm_id) != obs_lms_ids.end()) {
            auto lm = &lm_data->lms_noisy[POINT_DIM * lm_id];
            ceres_opts->opts.linear_solver_ordering->AddElementToGroup(
                lm, 0);  // group 0
        }
    }
    // 3b. Adding kfs
    for (uint kf_id = 0; kf_id < var_kf_data->num_kfs; kf_id++) {
        if (obs_kfs_ids.find(kf_id) != obs_kfs_ids.end()) {
            auto kf = &var_kf_data->kfs_noisy[AA_POSE_DIM * kf_id];
            ceres_opts->opts.linear_solver_ordering->AddElementToGroup(
                kf, 1);  // group 1
        }
    }

    return problem;
}

OptSummary solve_ba(CeresProblemWrapper *ceres_prob, CeresOptionWrapper *ceres_opts, int print_summary) {
    ceres::Solver::Summary summary;

    ceres::Solve(ceres_opts->opts, &ceres_prob->ceres_prob, &summary);
    if (print_summary) {
        cout << summary.FullReport() << endl;
    }

    return OptSummary{summary.initial_cost, summary.final_cost,
                      summary.total_time_in_seconds,
                      summary.termination_type != ceres::FAILURE};
}

#ifdef __cplusplus
}
#endif
