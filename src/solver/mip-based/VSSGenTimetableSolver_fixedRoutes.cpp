#include "solver/mip-based/VSSGenTimetableSolver.hpp"
#include "gurobi_c++.h"
#include "MultiArray.hpp"

void cda_rail::solver::mip_based::VSSGenTimetableSolver::create_fixed_routes_variables() {
    /**
     * Creates variables connected to the fixed route version of the problem
     */

    // Create MultiArrays
    vars["lda"] = MultiArray<GRBVar>(num_tr, num_t);
    vars["mu"] = MultiArray<GRBVar>(num_tr, num_t);
    vars["x_lda"] = MultiArray<GRBVar>(num_tr, num_t, num_edges);
    vars["x_mu"] = MultiArray<GRBVar>(num_tr, num_t, num_edges);

    const auto& train_list = instance.get_train_list();
    for (int i = 0; i < num_tr; ++i) {
        const auto tr_name = train_list.get_train(i).name;
        const auto r_len = instance.route_length(tr_name);
        const auto r_size = instance.get_route(tr_name).size();
        const auto tr_len = instance.get_train_list().get_train(tr_name).length;
        for (int t_steps = train_interval[i].first; t_steps <= train_interval[i].second; ++t_steps) {
            auto t = t_steps * dt;
            vars["mu"](i, t_steps) = model->addVar(0, r_len + tr_len, 0, GRB_CONTINUOUS, "mu_" + tr_name + "_" + std::to_string(t));
            vars["lda"](i, t_steps) = model->addVar(- tr_len, r_len, 0, GRB_CONTINUOUS, "lda_" + tr_name + "_" + std::to_string(t));
            for (int edge_id : instance.edges_used_by_train(tr_name, fix_routes)) {
                vars["x_lda"](i, t_steps, edge_id) = model->addVar(0, 1, 0, GRB_BINARY, "x_lda_" + tr_name + "_" + std::to_string(t) + "_" + std::to_string(edge_id));
                vars["x_mu"](i, t_steps, edge_id) = model->addVar(0, 1, 0, GRB_BINARY, "x_mu_" + tr_name + "_" + std::to_string(t) + "_" + std::to_string(edge_id));
            }
        }
    }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::create_fixed_routes_position_constraints() {
    /**
     * Creates constraints that ensure that the trains move according to their fixed routes.
     */

    auto train_list = instance.get_train_list();
    for (int i = 0; i < num_tr; ++i) {
        auto tr_name = train_list.get_train(i).name;
        auto tr_len = instance.get_train_list().get_train(tr_name).length;
        for (int t = train_interval[i].first; t <= train_interval[i].second - 1; ++t) {
            // full pos: mu - lda = len + (v(t) + v(t+1))/2 * dt
            model->addConstr(vars["mu"](i, t) - vars["lda"](i, t) == tr_len + (vars["v"](i, t) + vars["v"](i, t + 1)) * dt / 2, "full_pos_" + tr_name + "_" + std::to_string(t));
            // overlap: mu(t) - lda(t+1) = len
            model->addConstr(vars["mu"](i, t) - vars["lda"](i, t + 1) == tr_len, "overlap_" + tr_name + "_" + std::to_string(t));
            // mu increasing: mu(t+1) >= mu(t)
            model->addConstr(vars["mu"](i, t + 1) >= vars["mu"](i, t), "mu_increasing_" + tr_name + "_" + std::to_string(t));
            // lda increasing: lda(t+1) >= lda(t)
            model->addConstr(vars["lda"](i, t + 1) >= vars["lda"](i, t), "lda_increasing_" + tr_name + "_" + std::to_string(t));
        }
        // full pos also holds for t = train_interval[i].second
        auto t = train_interval[i].second;
        model->addConstr(vars["mu"](i, t) - vars["lda"](i, t) == tr_len + (vars["v"](i, t) + vars["v"](i, t + 1)) * dt / 2, "full_pos_" + tr_name + "_" + std::to_string(t));
    }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::create_boundary_fixed_routes_constraints() {
    /**
     * Create boundary conditions for the fixed routes of the trains
     */

    auto train_list = instance.get_train_list();
    for (int i = 0; i < num_tr; ++i) {
        auto tr_name = train_list.get_train(i).name;
        auto r_len = instance.route_length(tr_name);
        auto initial_speed = instance.get_schedule(tr_name).v_0;
        auto final_speed = instance.get_schedule(tr_name).v_n;
        auto tr_len = instance.get_train_list().get_train(tr_name).length;
        // initial_lda: lda(train_interval[i].first) = - tr_len
        model->addConstr(vars["lda"](i, train_interval[i].first) == -tr_len, "initial_lda_" + tr_name);
        // final_mu: mu(train_interval[i].second) = r_len + tr_len
        model->addConstr(vars["mu"](i, train_interval[i].second) == r_len + tr_len, "final_mu_" + tr_name);
        // initial_speed: v(train_interval[i].first) = initial_speed
        model->addConstr(vars["v"](i, train_interval[i].first) == initial_speed, "initial_speed_" + tr_name);
        // final_speed: v(train_interval[i].second) = final_speed
        model->addConstr(vars["v"](i, train_interval[i].second) == final_speed, "final_speed_" + tr_name);
    }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::create_fixed_routes_occupation_constraints() {
    /**
     * Create constraints for edge occupation of trains with fixed routes.
     */

    // Iterate over all trains
    const auto& train_list = instance.get_train_list();
    for (int tr = 0; tr < train_list.size(); ++tr) {
        const auto tr_name = train_list.get_train(tr).name;
        const auto r_len = instance.route_length(tr_name);
        const auto& tr_route = instance.get_route(tr_name);
        const auto r_size = tr_route.size();
        const auto tr_len = instance.get_train_list().get_train(tr_name).length;

        // Iterate over all edges
        for (int j = 0; j < r_size; ++j) {
            const auto edge_id = tr_route.get_edge(j);
            const auto edge_pos = instance.route_edge_pos(tr_name, edge_id);
            // Iterate over possible time steps
            for (int t = train_interval[tr].first; t <= train_interval[tr].second; ++t) {
                // x_mu(tr, t, edge_id) = 1 if, and only if, mu(tr,t) > edge_pos.first
                model->addConstr((r_len + tr_len) * vars["x_mu"](tr, t, edge_id) >= (vars["mu"](tr, t) - edge_pos.first), "x_mu_if_" + tr_name + "_" + std::to_string(t) + "_" + std::to_string(edge_id));
                model->addConstr(r_len * vars["x_mu"](tr, t, edge_id) <= r_len + vars["mu"](tr, t) - edge_pos.first, "x_mu_only_if_" + tr_name + "_" + std::to_string(t) + "_" + std::to_string(edge_id));

                // x_lda = 1 if, and only if, lda < edge_pos.second
                model->addConstr((r_len + tr_len) * vars["x_lda"](tr, t, edge_id) >= edge_pos.second - vars["lda"](tr, t), "x_lda_if_" + tr_name + "_" + std::to_string(t) + "_" + std::to_string(edge_id));
                model->addConstr(r_len * vars["x_lda"](tr, t, edge_id) <= r_len + edge_pos.second - vars["lda"](tr, t), "x_lda_only_if_" + tr_name + "_" + std::to_string(t) + "_" + std::to_string(edge_id));

                // x = x_lda AND x_mu
                GRBVar clause[2];
                clause[0] = vars["x_lda"](tr, t, edge_id);
                clause[1] = vars["x_mu"](tr, t, edge_id);
                model->addGenConstrAnd(vars["x"](tr, t, edge_id), clause, 2, "x_" + tr_name + "_" + std::to_string(t) + "_" + std::to_string(edge_id));
            }
        }
    }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::create_fixed_route_schedule_constraints() {
    /**
     * Constrain lambda and mu for fixed routes in stations.
     */

    // Iterate over all trains
    const auto& train_list = instance.get_train_list();
    for (int tr = 0; tr < train_list.size(); ++tr) {
        const auto tr_name = train_list.get_train(tr).name;
        const auto& tr_schedule = instance.get_schedule(tr_name);
        const auto& tr_route = instance.get_route(tr_name);
        const auto& tr_edges = tr_route.get_edges();
        for (const auto& tr_stop : tr_schedule.stops) {
            const auto t0 = tr_stop.begin / dt;
            const auto t1 = std::ceil(static_cast<double>(tr_stop.end) / dt);
            const auto& stop_edges = instance.get_station_list().get_station(tr_stop.station).tracks;
            const auto& stop_pos = instance.route_edge_pos(tr_name, stop_edges);
            for (int t = t0; t <= t1; ++t) {
                model->addConstr(vars["mu"](tr, t) >= stop_pos.first, "mu_station_min_" + tr_name + "_" + std::to_string(t));
                model->addConstr(vars["mu"](tr, t) <= stop_pos.second, "mu_station_max_" + tr_name + "_" + std::to_string(t));
                model->addConstr(vars["lda"](tr, t) >= stop_pos.first, "lda_station_min_" + tr_name + "_" + std::to_string(t));
                model->addConstr(vars["lda"](tr, t) <= stop_pos.second, "lda_station_max_" + tr_name + "_" + std::to_string(t));
            }
        }
    }
}