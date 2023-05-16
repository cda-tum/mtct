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

    // Discretize
    instance.discretize();

    // Save passed variables
    dt = delta_t;
    num_t = instance.maxT() / dt + 1;
    num_tr = instance.get_train_list().size();
    num_edges = instance.n().number_of_edges();
    num_vertices = instance.n().number_of_vertices();

    // Create environment and model
    env.emplace(true);
    env->start();
    model.emplace(env.value());

    create_fixed_routes_variables();
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::create_fixed_routes_variables() {
    /**
     * Creates variables connected to the fixed route version of the problem
     */

    // Create MultiArrays
    vars["lda"] = MultiArray<GRBVar>(num_tr, num_t);
    vars["mu"] = MultiArray<GRBVar>(num_tr, num_t);
    vars["x_lda"] = MultiArray<GRBVar>(num_tr, num_t, num_edges);
    vars["x_mu"] = MultiArray<GRBVar>(num_tr, num_t, num_edges);

    auto train_list = instance.get_train_list();
    for (int i = 0; i < num_tr; ++i) {
        auto tr_name = train_list.get_train(i).name;
        auto r_len = instance.route_length(tr_name);
        auto r_size = instance.get_route(tr_name).size();
        auto initial_speed = instance.get_schedule(tr_name).v_0;
        auto final_speed = instance.get_schedule(tr_name).v_n;
        auto tr_len = instance.get_train_list().get_train(tr_name).length;
        for (int t_steps = 0; t_steps < num_t; ++t_steps) {
            auto t = t_steps * dt;
            vars["lda"](i, t_steps) = model->addVar(0, r_len + tr_len + dt * final_speed, 0, GRB_CONTINUOUS, "lda_" + tr_name + "_" + std::to_string(t));
            vars["mu"](i, t_steps) = model->addVar(- tr_len - dt*initial_speed, r_len, 0, GRB_CONTINUOUS, "mu_" + tr_name + "_" + std::to_string(t));
            for (int j = 0; j < r_size; ++j) {
                auto edge_id = instance.get_route(tr_name).get_edge(j);
                vars["x_lda"](i, t_steps, edge_id) = model->addVar(0, 1, 0, GRB_BINARY, "x_lda_" + tr_name + "_" + std::to_string(t) + "_" + std::to_string(edge_id));
                vars["x_mu"](i, t_steps, edge_id) = model->addVar(0, 1, 0, GRB_BINARY, "x_mu_" + tr_name + "_" + std::to_string(t) + "_" + std::to_string(edge_id));
            }
        }
    }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::create_general_variables() {
    /**
     * Creates general variables that are independent of the fixed route
     */

    // Create MultiArrays
    vars["v"] = MultiArray<GRBVar>(num_tr, num_t + 1);
    //vars["x_in"] = MultiArray<GRBVar>(num_tr, num_t);
    //vars["x_out"] = MultiArray<GRBVar>(num_tr, num_t);
    vars["x"] = MultiArray<GRBVar>(num_tr, num_t, num_edges);

    auto train_list = instance.get_train_list();
    for (int i = 0; i < num_tr; ++i) {
        auto max_speed = instance.get_train_list().get_train(i).max_speed;
        auto tr_name = train_list.get_train(i).name;
        auto r_size = instance.get_route(tr_name).size();
        for (int t = 0; t < num_t + 1; ++t) {
            vars["v"](i, t) = model->addVar(0, max_speed, 0, GRB_CONTINUOUS, "v_" + tr_name + "_" + std::to_string(t));
        }
        for (int t = 0; t < num_t; ++t) {
            for (int j = 0; j < r_size; ++j) {
                auto edge_id = instance.get_route(tr_name).get_edge(j);
                vars["x_lda"](i, t, edge_id) = model->addVar(0, 1, 0, GRB_BINARY, "x_lda_" + tr_name + "_" + std::to_string(t) + "_" + std::to_string(edge_id));
                vars["x_mu"](i, t, edge_id) = model->addVar(0, 1, 0, GRB_BINARY, "x_mu_" + tr_name + "_" + std::to_string(t) + "_" + std::to_string(edge_id));
            }
        }
    }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::create_vss_variables() {
    /**
     * Creates variables connected to the VSS decisions of the problem
     */

    // Create MultiArrays

}
