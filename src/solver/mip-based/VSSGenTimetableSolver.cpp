#include "solver/mip-based/VSSGenTimetableSolver.hpp"
#include "gurobi_c++.h"
#include "MultiArray.hpp"

cda_rail::solver::mip_based::VSSGenTimetableSolver::VSSGenTimetableSolver(
        const cda_rail::instances::VSSGenerationTimetable &instance) : instance(instance) {}

cda_rail::solver::mip_based::VSSGenTimetableSolver::VSSGenTimetableSolver(const std::filesystem::path &instance_path) {
    instance = cda_rail::instances::VSSGenerationTimetable::import_instance(instance_path);
}

cda_rail::solver::mip_based::VSSGenTimetableSolver::VSSGenTimetableSolver(const std::string &instance_path) {
    instance = cda_rail::instances::VSSGenerationTimetable::import_instance(instance_path);
}

cda_rail::solver::mip_based::VSSGenTimetableSolver::VSSGenTimetableSolver(const char *instance_path) {
    instance = cda_rail::instances::VSSGenerationTimetable::import_instance(instance_path);
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::solve(int delta_t) {
    /**
     * Solve the instance using Gurobi
     */

    // Save passed variables
    dt = delta_t;
    num_t = instance.maxT() / dt + 1;
    num_tr = instance.get_train_list().size();

    // Create environment and model
    GRBEnv env(true);
    env.start();
    GRBModel model(env);

    create_fixed_routes_variables();
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::create_fixed_routes_variables() {
    /**
     * Creates variables connected to the fixed route version of the problem
     */

    vars["lda"] = MultiArray<GRBVar>(num_tr, num_t);
    vars["mu"] = MultiArray<GRBVar>(num_tr, num_t);
}
