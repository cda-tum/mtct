#include "solver/mip-based/VSSGenTimetableSolver.hpp"
#include "gurobi_c++.h"
#include "MultiArray.hpp"

void cda_rail::solver::mip_based::VSSGenTimetableSolver::create_free_routes_variables() {
    /**
     * This method creates the variables needed if the routes are not fixed.
     */

    // Create MultiArrays
    vars["overlap"] = MultiArray<GRBVar>(num_tr, num_t - 1, num_edges);
    vars["x_v"] = MultiArray<GRBVar>(num_tr, num_t, num_vertices);
    vars["len_in"] = MultiArray<GRBVar>(num_tr, num_t);
    vars["len_out"] = MultiArray<GRBVar>(num_tr, num_t);
    vars["e_lda"] = MultiArray<GRBVar>(num_tr, num_t, num_edges);
    vars["e_mu"] = MultiArray<GRBVar>(num_tr, num_t, num_edges);

    // Iterate over all trains
    const auto& train_list = instance.get_train_list();
    for (int tr = 0; tr < num_tr; ++tr) {
        const auto& tr_name = train_list.get_train(tr).name;
        const auto& tr_len = instance.get_train_list().get_train(tr_name).length;
        for (int t = train_interval[tr].first; t <= train_interval[tr].second; ++t) {
            for (int e = 0; e < num_edges; ++e) {
                if (t < train_interval[tr].second) {
                    vars["overlap"](tr, t, e) = model->addVar(0, instance.n().get_edge(e).length, 0, GRB_CONTINUOUS,
                                                              "overlap_" + tr_name + "_" + std::to_string(t) + "_" +
                                                              std::to_string(e));
                }
                vars["e_lda"](tr, t, e) = model->addVar(0, instance.n().get_edge(e).length, 0, GRB_CONTINUOUS,
                                                        "e_lda_" + tr_name + "_" + std::to_string(t) + "_" + std::to_string(e));
                vars["e_mu"](tr, t, e) = model->addVar(0, instance.n().get_edge(e).length, 0, GRB_CONTINUOUS,
                                                       "e_mu_" + tr_name + "_" + std::to_string(t) + "_" + std::to_string(e));
            }
            for (int v = 0; v < num_vertices; ++v) {
                vars["x_v"](tr, t, v) = model->addVar(0, 1, 0, GRB_BINARY,
                                                      "x_v_" + tr_name + "_" + std::to_string(t) + "_" + std::to_string(v));
            }
            vars["len_in"](tr, t) = model->addVar(0, tr_len, 0, GRB_CONTINUOUS,
                                                  "len_in_" + tr_name + "_" + std::to_string(t));
            vars["len_out"](tr, t) = model->addVar(0, tr_len, 0, GRB_CONTINUOUS,
                                                   "len_out_" + tr_name + "_" + std::to_string(t));
        }
    }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::create_free_routes_constraints() {

}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::create_non_discretized_free_route_constraints() {

}