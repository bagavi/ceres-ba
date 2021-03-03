#include <ceres/ceres.h>

#include "c_api.h"

// This function is used to stop ceres::Solver if the mapper thread receives a new kf.
class NewKfCallback : public ceres::IterationCallback {
   public:
    explicit NewKfCallback(const unsigned int* new_kf) : new_kf_(new_kf) {}

    ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) {
        // terminate if *new_kf_ = 1
        if (*new_kf_) {
            std::cout << "*new_kf_: " << *new_kf_ << ". Terminating at " << summary.iteration << " because new kf received" << std::endl;
            return ceres::CallbackReturnType::SOLVER_TERMINATE_SUCCESSFULLY;
        }
        return ceres::CallbackReturnType::SOLVER_CONTINUE;
    }

   private:
    const unsigned int* new_kf_;  // the new_kf is controlled from external code/agent
};