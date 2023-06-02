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

void cda_rail::solver::mip_based::VSSGenTimetableSolver::solve(int delta_t, bool fix_routes, bool discretize, bool include_acceleration_deceleration, bool include_breaking_distances) {
    /**
     * Solve the instance using Gurobi
     */

    // Save relevant variables
    dt = delta_t;
    this->fix_routes = fix_routes;
    this->discretize = discretize;
    this->include_acceleration_deceleration = include_acceleration_deceleration;
    this->include_breaking_distances = include_breaking_distances;

    // Discretize if applicable
    if (this->discretize) {
        instance.discretize();
    }

    // Initialize other relevant variables
    num_t = instance.maxT() / dt + 1;
    num_tr = instance.get_train_list().size();
    num_edges = instance.n().number_of_edges();
    num_vertices = instance.n().number_of_vertices();
    unbreakable_sections = instance.n().unbreakable_sections();
    if (this->discretize) {
        no_border_vss_sections = instance.n().no_border_vss_sections();
        num_breakable_sections = no_border_vss_sections.size();
        no_border_vss_vertices = instance.n().get_vertices_by_type(cda_rail::VertexType::NO_BORDER_VSS);
    } else {
        breakable_sections = instance.n().combine_reverse_edges(instance.n().breakable_edges());
        num_breakable_sections = breakable_sections.size();
    }



    for (int i = 0; i < num_tr; ++i) {
        train_interval.emplace_back(instance.time_interval(i));
        train_interval.back().first /= dt;
        train_interval.back().second /= dt;
    }

    // Create environment and model
    env.emplace(true);
    env->start();
    model.emplace(env.value());

    // Create variables
    create_general_variables();
    create_discretized_variables();
    create_fixed_routes_variables();

    // Set objective
    set_objective();

    // Create constraints
    create_acceleration_constraints(); // ERROR
    create_fixed_routes_train_movement_constraints();
    create_boundary_fixed_routes_constraints();
    create_vss_discretized_constraints();
    create_unbreakable_sections_constraints();
    create_fixed_routes_train_occupation_constraints();
    create_general_station_constraints();
    create_fixed_route_station_constraints();

    // Breaklen: https://www.gurobi.com/documentation/10.0/refman/constraints.html#subsubsection:GenConstrFunction


    model->write("model.lp");

    // Optimize
    model->optimize();

    // Print solution of all trains
    // Iterate over all times
    for (int t = 0; t < num_t; ++t) {
        std::cout << "t = " << t*dt << "; ";
        // Iterate over all trains
        const auto tr_at_t = instance.trains_at_t(t * dt);
        for (const auto &tr : tr_at_t) {
            std::cout << tr << " at [" << vars["lda"](tr, t).get(GRB_DoubleAttr_X) << ", " << vars["mu"](tr, t).get(GRB_DoubleAttr_X) << "]; ";
        }
        std::cout << std::endl;
    }
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
    vars["x"] = MultiArray<GRBVar>(num_tr, num_t, num_edges);
    vars["x_sec"] = MultiArray<GRBVar>(num_tr, num_t, unbreakable_sections.size());
    vars["y_sec_fwd"] = MultiArray<GRBVar>(num_t, num_breakable_sections);
    vars["y_sec_bwd"] = MultiArray<GRBVar>(num_t, num_breakable_sections);

    auto train_list = instance.get_train_list();
    for (int i = 0; i < num_tr; ++i) {
        auto max_speed = instance.get_train_list().get_train(i).max_speed;
        auto tr_name = train_list.get_train(i).name;
        auto r_size = instance.get_route(tr_name).size();
        for (int t = train_interval[i].first; t <= train_interval[i].second + 1; ++t) {
            vars["v"](i, t) = model->addVar(0, max_speed, 0, GRB_CONTINUOUS, "v_" + tr_name + "_" + std::to_string(t));
        }
        for (int t = train_interval[i].first; t <= train_interval[i].second; ++t) {
            for (int j = 0; j < r_size; ++j) {
                auto edge_id = instance.get_route(tr_name).get_edge(j);
                vars["x"](i, t, edge_id) = model->addVar(0, 1, 0, GRB_BINARY, "x_" + tr_name + "_" + std::to_string(t) + "_" + std::to_string(edge_id));
            }
            for (const auto& sec : unbreakable_section_indices(i)) {
                vars["x_sec"](i, t, sec) = model->addVar(0, 1, 0, GRB_BINARY, "x_sec_" + tr_name + "_" + std::to_string(t) + "_" + std::to_string(sec));
            }
        }
    }
    for (int t = 0; t < num_t; ++t) {
        for (int i = 0; i < num_breakable_sections; ++i) {
            vars["y_sec_fwd"](t, i) = model->addVar(0, 1, 0, GRB_BINARY, "y_sec_fwd_" + std::to_string(t) + "_" + std::to_string(i));
            vars["y_sec_bwd"](t, i) = model->addVar(0, 1, 0, GRB_BINARY, "y_sec_bwd_" + std::to_string(t) + "_" + std::to_string(i));
        }
    }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::create_discretized_variables() {
    /**
     * Creates variables connected to the VSS decisions of the problem
     */

    // Create MultiArrays
    vars["b"] = MultiArray<GRBVar>(no_border_vss_vertices.size());

    // Create variables
    for (int i = 0; i < no_border_vss_vertices.size(); ++i) {
        vars["b"](i) = model->addVar(0, 1, 0, GRB_BINARY, "b_" + std::to_string(no_border_vss_vertices[i]));
    }

}

std::vector<int>
cda_rail::solver::mip_based::VSSGenTimetableSolver::unbreakable_section_indices(int train_index) const {
    /**
     * This function returns the indices of the unbreakable sections that are traversed by the train with index train_index
     * @param train_index index of the train
     * @return vector of indices
     */

    std::vector<int> indices;
    const auto& tr_name = instance.get_train_list().get_train(train_index).name;
    const auto& tr_route = instance.get_route(tr_name).get_edges();
    for (int i = 0; i < unbreakable_sections.size(); ++i) {
        bool edge_found = false;
        // If unbreakable_section[i] (of type vector) and tr_route (of type vector) overlap (have a common element), add i to indices
        for (int j0 = 0; j0 < unbreakable_sections[i].size() && !edge_found; ++j0) {
            for (int j1 = 0; j1 < tr_route.size() && !edge_found; ++j1) {
                if (unbreakable_sections[i][j0] == tr_route[j1]) {
                    indices.push_back(i);
                    edge_found = true;
                }
            }
        }
    }

    return indices;
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::create_fixed_routes_train_movement_constraints() {
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

void cda_rail::solver::mip_based::VSSGenTimetableSolver::set_objective() {
    /**
     * Sets the objective function of the problem
     */

    // sum over all b_i as in no_border_vss_vertices
    GRBLinExpr obj = 0;
    for (int i = 0; i < no_border_vss_vertices.size(); ++i) {
        obj += vars["b"](i);
    }
    model->setObjective(obj, GRB_MINIMIZE);
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::create_vss_discretized_constraints() {
    /**
     * Creates VSS constraints, i.e., on NO_BORDER_VSS sections two trains must be separated by a chosen vertex.
     */

    // Iterate over all non-border VSS sections
    for (const auto& no_border_vss_section : no_border_vss_sections) {
        const auto tr_on_section = instance.trains_in_section(no_border_vss_section);
        const auto no_border_vss_section_sorted = instance.n().combine_reverse_edges(no_border_vss_section, true);
        // Iterate over all pairs of trains on the section
        for (int i = 0; i < tr_on_section.size(); ++i) {
            const auto& tr1 = tr_on_section[i];
            const auto& tr1_interval = train_interval[tr1];
            const auto& tr1_name = instance.get_train_list().get_train(tr1).name;
            const auto& tr1_route = instance.get_route(tr1_name);
            for (int j = i + 1; j < tr_on_section.size(); ++j) {
                const auto& tr2 = tr_on_section[j];
                const auto& tr2_interval = train_interval[tr2];
                const auto& tr2_name = instance.get_train_list().get_train(tr2).name;
                const auto& tr2_route = instance.get_route(tr2_name);
                std::pair<int, int> t_interval = {std::max(tr1_interval.first, tr2_interval.first),
                                                  std::min(tr1_interval.second, tr2_interval.second)};
                // Iterate over all time steps where both trains can potentially be on the section
                for (int t = t_interval.first; t <= t_interval.second; ++t) {
                    // Iterate over all pairs of edges in the section
                    for (int e1 = 0; e1 < no_border_vss_section_sorted.size(); ++e1) {
                        for (int e2 = 0; e2 < no_border_vss_section_sorted.size(); ++e2) {
                            if (e1 == e2) {
                                continue;
                            }
                            GRBLinExpr lhs = 2;
                            if (tr1_route.contains_edge(no_border_vss_section_sorted[e1].first)) {
                                lhs -= vars["x"](tr1, t, no_border_vss_section_sorted[e1].first);
                            }
                            if (tr1_route.contains_edge(no_border_vss_section_sorted[e1].second)) {
                                lhs -= vars["x"](tr1, t, no_border_vss_section_sorted[e1].second);
                            }
                            if (tr2_route.contains_edge(no_border_vss_section_sorted[e2].first)) {
                                lhs -= vars["x"](tr2, t, no_border_vss_section_sorted[e2].first);
                            }
                            if (tr2_route.contains_edge(no_border_vss_section_sorted[e2].second)) {
                                lhs -= vars["x"](tr2, t, no_border_vss_section_sorted[e2].second);
                            }

                            // Overlapping vertices
                            for (int e_overlap = std::min(e1,e2); e_overlap < std::max(e1, e2); ++e_overlap) {
                                const auto& v_overlap = instance.n().common_vertex(no_border_vss_section_sorted[e_overlap],
                                                                                   no_border_vss_section_sorted[e_overlap + 1]);
                                if (!v_overlap.has_value()) {
                                    throw std::runtime_error("No common vertex found, this should not have happened");
                                }
                                // Find index of v_overlap in no_border_vss_vertices
                                int v_overlap_index = std::find(no_border_vss_vertices.begin(), no_border_vss_vertices.end(), v_overlap.value()) - no_border_vss_vertices.begin();
                                if (v_overlap_index >= no_border_vss_vertices.size()) {
                                    throw std::runtime_error("Vertex not found in no_border_vss_vertices, this should not have happened");
                                }
                                lhs += vars["b"](v_overlap_index);
                            }

                            model->addConstr(lhs >= 1, "vss_" + tr1_name + "_" + tr2_name + "_" + std::to_string(t) + "_" + std::to_string(e1) + "_" + std::to_string(e2));
                        }
                    }
                }
            }
        }
    }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::create_unbreakable_sections_constraints() {
    /**
     * Creates constraints for unbreakable sections, i.e., only one train can be on an unbreakable section at a time.
     */

    // Iterate over all unbreakable sections
    for (int sec_index = 0; sec_index < unbreakable_sections.size(); ++sec_index) {
        const auto& sec = unbreakable_sections[sec_index];
        const auto& tr_on_sec = instance.trains_in_section(sec);
        // tr is on section if it occupies at least one edge of the section
        for (int tr : tr_on_sec) {
            const auto& tr_interval = train_interval[tr];
            const auto& tr_name = instance.get_train_list().get_train(tr).name;
            const auto& tr_route = instance.get_route(tr_name);
            for (int t = tr_interval.first; t <= tr_interval.second; ++t) {
                GRBLinExpr lhs = 0;
                int count = 0;
                for (int e_index : sec) {
                    if (tr_route.contains_edge(e_index)) {
                        lhs += vars["x"](tr, t, e_index);
                        count++;
                    }
                }
                model->addConstr(lhs >= vars["x_sec"](tr, t, sec_index), "unbreakable_section_only_" + tr_name + "_" + std::to_string(t) + "_" + std::to_string(sec_index));
                model->addConstr(lhs <= count * vars["x_sec"](tr, t, sec_index), "unbreakable_section_if_" + tr_name + "_" + std::to_string(t) + "_" + std::to_string(sec_index));
            }
        }

        for (int t = 0; t <= num_t; ++t) {
            const auto tr_to_consider = instance.trains_at_t(t*dt, tr_on_sec);
            GRBLinExpr lhs = 0;
            for (int tr : tr_to_consider) {
                lhs += vars["x_sec"](tr, t, sec_index);
            }
            model->addConstr(lhs <= 1, "unbreakable_section"+ std::to_string(sec_index) +"_at_most_one_" + std::to_string(t));
        }
    }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::create_fixed_routes_train_occupation_constraints() {
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

void cda_rail::solver::mip_based::VSSGenTimetableSolver::create_general_station_constraints() {
    /**
     * Creates constraints for general stations, i.e., if a train is in a station:
     * - all other x variables are 0
     * - the speed is 0
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
            const auto inverse_stop_edges = instance.n().inverse_edges(stop_edges, tr_edges);
            for (int t = t0; t <= t1; ++t) {
                model->addConstr(vars["v"](tr, t) == 0, "station_speed_" + tr_name + "_" + std::to_string(t));
                for (int e : inverse_stop_edges) {
                    model->addConstr(vars["x"](tr, t, e) == 0, "station_x_" + tr_name + "_" + std::to_string(t) + "_" + std::to_string(e));
                }
                // At least on station edge must be occupied
                GRBLinExpr lhs = 0;
                for (int e : stop_edges) {
                    if (tr_route.contains_edge(e)) {
                        lhs += vars["x"](tr, t, e);
                    }
                }
                model->addConstr(lhs >= 1, "station_occupancy_" + tr_name + "_" + std::to_string(t));
            }
        }
    }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::create_fixed_route_station_constraints() {
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

void cda_rail::solver::mip_based::VSSGenTimetableSolver::create_acceleration_constraints() {
    /**
     * This method adds constraints connected to acceleration and deceleration of the trains.
     */

    // Iterate over all trains
    const auto& train_list = instance.get_train_list();
    for (int tr = 0; tr < train_list.size(); ++tr) {
        // Iterate over all time steps
        const auto& tr_object = train_list.get_train(tr);
        for (int t = train_interval[tr].first; t < train_interval[tr].second; ++t) {
            // v(t+1) - v(t) <= acceleration * dt
            model->addConstr(vars["v"](tr, t + 1) - vars["v"](tr, t) <= tr_object.acceleration * dt, "acceleration_" + tr_object.name + "_" + std::to_string(t));
            // v(t) - v(t+1) <= deceleration * dt
            model->addConstr(vars["v"](tr, t) - vars["v"](tr, t + 1) <= tr_object.deceleration * dt, "deceleration_" + tr_object.name + "_" + std::to_string(t));
        }
    }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::create_breaklen_variables() {
    /**
     * This method creates the variables corresponding to breaking distances.
     */

    // Create MultiArrays
    vars["breaklen"] = MultiArray<GRBVar>(num_tr, num_t);

    // Iterate over all trains
    for (int tr = 0; tr < num_tr; ++tr) {
        for (int t = train_interval[tr].first; t <= train_interval[tr].second; ++t) {
            vars["breaklen"](tr, t) = model->addVar(0, GRB_INFINITY, 0, GRB_CONTINUOUS, "breaklen_" + std::to_string(tr) + "_" + std::to_string(t));
        }
    }
}

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
