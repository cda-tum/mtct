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

    if (this->fix_routes && !instance.has_route_for_every_train()) {
        throw std::runtime_error("Instance does not have a route for every train");
    }

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
        breakable_edges = instance.n().breakable_edges();
        for (int i = 0; i < breakable_edges.size(); ++i) {
            breakable_edge_indices[breakable_edges[i]] = i;
        }
        breakable_edges_pairs = instance.n().combine_reverse_edges(breakable_edges);
        num_breakable_sections = breakable_edges.size();
        relevant_edges = instance.n().relevant_breakable_edges();
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
    if (this->fix_routes) {
        create_fixed_routes_variables();
    } else {
        create_free_routes_variables();
    }
    if (this->discretize) {
        create_discretized_variables();
    } else {
        create_non_discretized_variables();
    }
    if (this->include_breaking_distances) {
        create_breaklen_variables();
    }

    // Set objective
    set_objective();

    // Create constraints
    create_general_constraints();
    if (this->fix_routes) {
        create_fixed_routes_constraints();
    } else {
        create_free_routes_constraints();
    }
    if (this->discretize) {
        create_discretized_constraints();
    } else {
        create_non_discretized_constraints();
    }
    if (this->include_acceleration_deceleration) {
        create_acceleration_constraints();
    }
    if (this->include_breaking_distances) {
        create_breaklen_constraints();
    }

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
            for (int edge_id : instance.edges_used_by_train(tr_name, fix_routes)) {
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

void cda_rail::solver::mip_based::VSSGenTimetableSolver::create_non_discretized_variables() {
    /**
     * This method creates the variables needed if the graph is not discretized.
     */

    int max_vss = 0;
    for (const auto& e : breakable_edges) {
        max_vss = std::max(max_vss, instance.n().max_vss_on_edge(e));
    }

    // Create MultiArrays
    vars["b_pos"] = MultiArray<GRBVar>(num_breakable_sections, max_vss);
    vars["b_front"] = MultiArray<GRBVar>(num_tr, num_t, num_breakable_sections, max_vss);
    vars["b_rear"] = MultiArray<GRBVar>(num_tr, num_t, num_breakable_sections, max_vss);
    vars["b_used"] = MultiArray<GRBVar>(relevant_edges.size(), max_vss);

    for (int i = 0; i < breakable_edges.size(); ++i) {
        const auto& e = breakable_edges[i];
        const auto vss_number_e = instance.n().max_vss_on_edge(e);
        const auto& edge_len = instance.n().get_edge(e).length;
        for (int vss = 0; vss < vss_number_e; ++vss) {
            vars["b_pos"](e, vss) = model->addVar(0, edge_len, 0, GRB_CONTINUOUS, "b_pos_" + std::to_string(e) + "_" + std::to_string(vss));
            for (int tr = 0; tr < num_tr; ++tr) {
                for (int t = train_interval[tr].first; t <= train_interval[tr].second; ++t) {
                    vars["b_front"](tr, t, i, vss) = model->addVar(0, 1, 0, GRB_BINARY,
                                                                   "b_front_" + std::to_string(tr) + "_" + std::to_string(t) + "_" + std::to_string(i) + "_" + std::to_string(vss));
                    vars["b_rear"](tr, t, i, vss) = model->addVar(0, 1, 0, GRB_BINARY,
                                                                  "b_rear_" + std::to_string(tr) + "_" + std::to_string(t) + "_" + std::to_string(i) + "_" + std::to_string(vss));
                }
            }
        }
    }

    for (int i = 0; i < relevant_edges.size(); ++i) {
        const auto& e = relevant_edges[i];
        const auto vss_number_e = instance.n().max_vss_on_edge(e);
        for (int vss = 0; vss < vss_number_e; ++vss) {
            vars["b_used"](i, vss) = model->addVar(0, 1, 0, GRB_BINARY, "b_used_" + std::to_string(e) + "_" + std::to_string(vss));
        }
    }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::set_objective() {
    /**
     * Sets the objective function of the problem
     */

    // sum over all b_i as in no_border_vss_vertices
    GRBLinExpr obj = 0;
    if (discretize) {
        for (int i = 0; i < no_border_vss_vertices.size(); ++i) {
            obj += vars["b"](i);
        }
    } else {
        for (int i = 0; i < relevant_edges.size(); ++i) {
            const auto& e = relevant_edges[i];
            const auto vss_number_e = instance.n().max_vss_on_edge(e);
            for (int vss = 0; vss < vss_number_e; ++vss) {
                obj += vars["b_used"](i, vss);
            }
        }
    }
    model->setObjective(obj, GRB_MINIMIZE);
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::create_discretized_constraints() {
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



void cda_rail::solver::mip_based::VSSGenTimetableSolver::create_general_schedule_constraints() {
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

void cda_rail::solver::mip_based::VSSGenTimetableSolver::create_general_constraints() {
    /**
     * These constraints appear in all variants
     */

    create_general_schedule_constraints();
    create_unbreakable_sections_constraints();
    create_general_speed_constraints();
    create_reverse_occupation_constraints();
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::create_fixed_routes_constraints() {
    /**
     * These constraints appear only when routes are fixed
     */

    create_fixed_routes_position_constraints();
    create_boundary_fixed_routes_constraints();
    create_fixed_routes_occupation_constraints();
    create_fixed_route_schedule_constraints();
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::create_non_discretized_constraints() {
    /**
     * These constraints appear only when the graph is not discretized
     */

    create_non_discretized_general_constraints();
    create_non_discretized_position_constraints();
    if (this->fix_routes) {
        create_non_discretized_fixed_route_constraints();
    } else {
        create_non_discretized_free_route_constraints();
    }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::create_non_discretized_general_constraints() {
    // VSS can only be used if it is non-zero
    for (int i = 0; i < relevant_edges.size(); ++i) {
        const auto& e = relevant_edges[i];
        const auto& e_index = breakable_edge_indices[e];
        const auto vss_number_e = instance.n().max_vss_on_edge(e);
        const auto& e_len = instance.n().get_edge(e).length;
        const auto& min_block_len_e = instance.n().get_edge(e).min_block_length;
        for (int vss = 0; vss < vss_number_e; ++vss) {
            model->addConstr(e_len * vars["b_used"](i, vss), GRB_GREATER_EQUAL, vars["b_pos"](e_index, vss), "b_used_" + std::to_string(e) + "_" + std::to_string(vss));
            // Also remove redundant solutions
            if (vss < vss_number_e - 1) {
                model->addConstr(vars["b_pos"](e_index, vss), GRB_GREATER_EQUAL, vars["b_pos"](e_index, vss + 1) + vars["b_used"](i, vss + 1) * min_block_len_e,
                                 "b_used_decreasing_" + std::to_string(e) + "_" + std::to_string(vss));
            }
        }
    }

    // Connect position of reverse edges
    for (const auto& e_pair : breakable_edges_pairs) {
        if (e_pair.second < 0) {
            continue;
        }
        const auto vss_number_e = instance.n().max_vss_on_edge(e_pair.first);
        if (instance.n().max_vss_on_edge(e_pair.second) != vss_number_e) {
            throw std::runtime_error("VSS number of edges " + std::to_string(e_pair.first) + " and " + std::to_string(e_pair.second) + " do not match");
        }
        const auto& e_len = instance.n().get_edge(e_pair.first).length;
        for (int vss = 0; vss < vss_number_e; ++vss) {
            model->addConstr(vars["b_pos"](breakable_edge_indices[e_pair.first], vss) + vars["b_pos"](breakable_edge_indices[e_pair.second], vss), GRB_EQUAL, e_len, "b_pos_reverse_" + std::to_string(e_pair.first) + "_" + std::to_string(vss) + "_" + std::to_string(e_pair.second) + "_" + std::to_string(vss));
        }
    }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::create_non_discretized_position_constraints() {
    /**
     * Creates the position constraints related to non-discretized VSS blocks
     */

    // Border only usable by a train if it is on the edge
    for (int tr = 0; tr < num_tr; ++tr) {
        for (const auto& e : instance.edges_used_by_train(tr, this->fix_routes)) {
            const auto& e_index = breakable_edge_indices[e];
            const auto vss_number_e = instance.n().max_vss_on_edge(e);
            for (int t = train_interval[tr].first; t <= train_interval[tr].second; ++t) {
                for (int vss = 0; vss < vss_number_e; ++vss) {
                    // x(tr,t,e) >= b_front(tr,t,e_index,vss)
                    model->addConstr(vars["x"](tr, t, e), GRB_GREATER_EQUAL, vars["b_front"](tr, t, e_index, vss), "x_b_front_" + std::to_string(tr) + "_" + std::to_string(t) + "_" + std::to_string(e) + "_" + std::to_string(vss));
                    // x(tr,t,e) >= b_rear(tr,t,e_index,vss)
                    model->addConstr(vars["x"](tr, t, e), GRB_GREATER_EQUAL, vars["b_rear"](tr, t, e_index, vss), "x_b_rear_" + std::to_string(tr) + "_" + std::to_string(t) + "_" + std::to_string(e) + "_" + std::to_string(vss));
                }
            }
        }
    }

    // Correct number of borders
    for (int e_index = 0; e_index < breakable_edges.size(); ++e_index) {
        const auto& e = breakable_edges[e_index];
        const auto vss_number_e = instance.n().max_vss_on_edge(e);
        const auto& tr_on_e = instance.trains_on_edge(e, this->fix_routes);
        for (int t = 0; t < num_t; ++t) {
            // sum_(tr,vss) b_front(tr, t, e_index, vss) = sum_(tr) x(tr, t, e) - 1
            // sum_(tr,vss) b_rear(tr, t, e_index, vss) = sum_(tr) x(tr, t, e) - 1
            GRBLinExpr lhs_front = 0;
            GRBLinExpr lhs_rear = 0;
            GRBLinExpr rhs = -1;
            for (const auto& tr : instance.trains_at_t(t, tr_on_e)) {
                for (int vss = 0; vss < vss_number_e; ++vss) {
                    lhs_front += vars["b_front"](tr, t, e_index, vss);
                    lhs_rear += vars["b_rear"](tr, t, e_index, vss);
                }
                rhs += vars["x"](tr, t, e);
            }
            model->addConstr(lhs_front, GRB_EQUAL, rhs, "b_front_correct_number_" + std::to_string(t) + "_" + std::to_string(e) + "_" + std::to_string(e_index));
            model->addConstr(lhs_rear, GRB_EQUAL, rhs, "b_rear_correct_number_" + std::to_string(t) + "_" + std::to_string(e) + "_" + std::to_string(e_index));
        }
    }

    // At most one border used per train
    for (int tr = 0; tr < num_tr; ++tr) {
        for (int t = train_interval[tr].first; t < train_interval[tr].second; ++t) {
            // sum_(e,vss) b_front(tr, t, e_index, vss) <= 1
            // sum_(e,vss) b_rear(tr, t, e_index, vss) <= 1
            GRBLinExpr lhs_front = 0;
            GRBLinExpr lhs_rear = 0;
            for (const auto& e : instance.edges_used_by_train(tr, this->fix_routes)) {
                const auto& e_index = breakable_edge_indices[e];
                const auto vss_number_e = instance.n().max_vss_on_edge(e);
                for (int vss = 0; vss < vss_number_e; ++vss) {
                    lhs_front += vars["b_front"](tr, t, e_index, vss);
                    lhs_rear += vars["b_rear"](tr, t, e_index, vss);
                }
            }
            model->addConstr(lhs_front, GRB_LESS_EQUAL, 1, "b_front_at_most_one_" + std::to_string(tr) + "_" + std::to_string(t));
            model->addConstr(lhs_rear, GRB_LESS_EQUAL, 1, "b_rear_at_most_one_" + std::to_string(tr) + "_" + std::to_string(t));
        }
    }

    // A border must be both front and rear or nothing
    for (int e_index = 0; e_index < breakable_edges.size(); ++ e_index) {
        const auto& e = breakable_edges[e_index];
        const auto tr_on_e = instance.trains_on_edge(e, this->fix_routes);
        const auto vss_number_e = instance.n().max_vss_on_edge(e);
        for (int t = 0; t < num_t; ++t) {
            for (int vss = 0; vss < vss_number_e; ++vss) {
                // sum_tr b_front(tr, t, e_index, vss) = sum_tr b_rear(tr, t, e_index, vss) <= 1
                GRBLinExpr lhs = 0;
                GRBLinExpr rhs = 0;
                for (const auto& tr : instance.trains_at_t(t, tr_on_e)) {
                    lhs += vars["b_front"](tr, t, e_index, vss);
                    rhs += vars["b_rear"](tr, t, e_index, vss);
                }
                model->addConstr(lhs, GRB_EQUAL, rhs, "b_front_rear_" + std::to_string(t) + "_" + std::to_string(e) + "_" + std::to_string(vss));
                model->addConstr(rhs, GRB_LESS_EQUAL, 1, "b_front_rear_limit_" + std::to_string(t) + "_" + std::to_string(e) + "_" + std::to_string(vss));
            }
        }
    }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::create_breaklen_constraints() {

}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::create_general_speed_constraints() {
    /**
     * Train does not exceed maximum speed on edges
     */

    for (int tr = 0; tr < num_tr; ++tr) {
        const auto& tr_speed = instance.get_train_list().get_train(tr).max_speed;
        for (const auto e: instance.edges_used_by_train(tr, this->fix_routes)) {
            const auto& max_speed = instance.n().get_edge(e).max_speed;
            if (max_speed < tr_speed) {
                for (int t = train_interval[tr].first; t <= train_interval[tr].second; ++t) {
                    // v(tr,t) <= max_speed + (tr_speed - max_speed) * (1 - x(tr,t,e))
                    model->addConstr(vars["v"](tr, t), GRB_LESS_EQUAL, max_speed + (tr_speed - max_speed) * (1 - vars["x"](tr, t, e)), "v_max_speed_" + std::to_string(tr) + "_" + std::to_string(t) + "_" + std::to_string(e));
                }
            }
        }
    }
}
