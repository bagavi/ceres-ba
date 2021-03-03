#include <ceres/ceres.h>

#include "c_api.h"

//sets default options used by ceres::Solver
CeresOptionWrapper *get_default() {
    CeresOptionWrapper *ceres_opts = new CeresOptionWrapper();
    auto &options = ceres_opts->opts;

    options.linear_solver_type = ceres::DENSE_SCHUR;
    // options.linear_solver_type = ceres::SPARSE_SCHUR;

    options.dense_linear_algebra_library_type = ceres::LAPACK;
    // options.dense_linear_algebra_library_type = ceres::EIGEN;

    options.sparse_linear_algebra_library_type = ceres::CX_SPARSE;
    // options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;

    return ceres_opts;
}

void options_set_num_iterations(CeresOptionWrapper *ceres_opts, int max_num_iterations) {
    ceres_opts->opts.max_num_iterations = max_num_iterations;
}

void options_set_num_threads(CeresOptionWrapper *ceres_opts, int num_threads) {
    ceres_opts->opts.num_threads = num_threads;
}

void options_set_max_solve_time(CeresOptionWrapper *ceres_opts,
                                double max_solver_time_in_seconds) {
    ceres_opts->opts.max_solver_time_in_seconds = max_solver_time_in_seconds;
}
