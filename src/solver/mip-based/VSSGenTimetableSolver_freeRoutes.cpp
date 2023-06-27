#include "solver/mip-based/VSSGenTimetableSolver.hpp"
#include "gurobi_c++.h"
#include "MultiArray.hpp"
#include <exception>

void cda_rail::solver::mip_based::VSSGenTimetableSolver::create_free_routes_variables() {
    /**
     * This method creates the variables needed if the routes are not fixed.
     */

    // Create MultiArrays
    vars["overlap"] = MultiArray<GRBVar>(num_tr, num_t - 1, num_edges);
    vars["x_v"] = MultiArray<GRBVar>(num_tr, num_t, num_vertices);
    vars["len_in"] = MultiArray<GRBVar>(num_tr, num_t);
    vars["x_in"] = MultiArray<GRBVar>(num_tr, num_t);
    vars["len_out"] = MultiArray<GRBVar>(num_tr, num_t);
    vars["x_out"] = MultiArray<GRBVar>(num_tr, num_t);
    vars["e_lda"] = MultiArray<GRBVar>(num_tr, num_t, num_edges);
    vars["e_mu"] = MultiArray<GRBVar>(num_tr, num_t, num_edges);

    // Iterate over all trains
    const auto& train_list = instance.get_train_list();
    for (int tr = 0; tr < num_tr; ++tr) {
        const auto& tr_name = train_list.get_train(tr).name;
        const auto& tr_len = instance.get_train_list().get_train(tr_name).length;
        double len_out_ub = tr_len;
        if (this->include_breaking_distances) {
            len_out_ub += get_max_brakelen(tr);
        }
        for (int t = train_interval[tr].first; t <= train_interval[tr].second; ++t) {
            for (int e = 0; e < num_edges; ++e) {
                const auto& edge = instance.n().get_edge(e);
                const auto& edge_name = "[" + instance.n().get_vertex(edge.source).name + "," + instance.n().get_vertex(edge.target).name + "]";
                if (t < train_interval[tr].second) {
                    vars["overlap"](tr, t, e) = model->addVar(0, instance.n().get_edge(e).length, 0, GRB_CONTINUOUS,
                                                              "overlap_" + tr_name + "_" + std::to_string(t*dt) + "_" +
                                                                      edge_name);
                }
                vars["e_lda"](tr, t, e) = model->addVar(0, instance.n().get_edge(e).length, 0, GRB_CONTINUOUS,
                                                        "e_lda_" + tr_name + "_" + std::to_string(t*dt) + "_" + edge_name);
                vars["e_mu"](tr, t, e) = model->addVar(0, instance.n().get_edge(e).length, 0, GRB_CONTINUOUS,
                                                       "e_mu_" + tr_name + "_" + std::to_string(t*dt) + "_" + edge_name);
            }
            for (int v = 0; v < num_vertices; ++v) {
                const auto& v_name = instance.n().get_vertex(v).name;
                vars["x_v"](tr, t, v) = model->addVar(0, 1, 0, GRB_BINARY,
                                                      "x_v_" + tr_name + "_" + std::to_string(t*dt) + "_" + v_name);
            }
            vars["len_in"](tr, t) = model->addVar(0, tr_len, 0, GRB_CONTINUOUS,
                                                  "len_in_" + tr_name + "_" + std::to_string(t*dt));
            vars["x_in"](tr, t) = model->addVar(0, 1, 0, GRB_BINARY,
                                                "x_in_" + tr_name + "_" + std::to_string(t*dt));
            vars["len_out"](tr, t) = model->addVar(0, len_out_ub, 0, GRB_CONTINUOUS,
                                                   "len_out_" + tr_name + "_" + std::to_string(t*dt));
            vars["x_out"](tr, t) = model->addVar(0, 1, 0, GRB_BINARY,
                                                    "x_out_" + tr_name + "_" + std::to_string(t*dt));
        }
    }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::create_free_routes_constraints() {
    create_free_routes_position_constraints();
    create_free_routes_overlap_constraints();
    create_boundary_free_routes_constraints();
    create_free_routes_occupation_constraints();
    create_free_routes_no_overlap_entry_exit_constraints();
    if (this->use_cuts) {
        create_free_routes_impossibility_cuts();
    }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::create_free_routes_position_constraints() {
    /**
     * Creates constraints connected to positioning of trains.
     */

    auto train_list = instance.get_train_list();
    for (int tr = 0; tr < num_tr; ++tr) {
        const auto& tr_name = train_list.get_train(tr).name;
        const auto& tr_len = instance.get_train_list().get_train(tr_name).length;
        const auto& entry = instance.get_schedule(tr).entry;
        const auto& exit = instance.get_schedule(tr).exit;
        for (int t = train_interval[tr].first; t <= train_interval[tr].second; ++t) {
            // Train position has the correct length
            // full pos: sum_e (e_mu - e_lda) + len_in + len_out = len + (v(t) + v(t+1))/2 * dt + brakelen (if applicable)
            GRBLinExpr lhs = vars["len_in"](tr, t) + vars["len_out"](tr, t);
            for (int e = 0; e < num_edges; ++e) {
                lhs += vars["e_mu"](tr, t, e) - vars["e_lda"](tr, t, e);
            }
            GRBLinExpr rhs = tr_len + (vars["v"](tr, t) + vars["v"](tr, t + 1)) * dt / 2;
            if (this->include_breaking_distances) {
                rhs += vars["brakelen"](tr, t);
            }
            model->addConstr(lhs, GRB_EQUAL, rhs, "train_pos_len_" + tr_name + "_" + std::to_string(t));

            // Train position is a simple connected path, i.e.,
            // x_v <= sum_(e in delta_v) x_e
            // x_v >= sum_(e in delta_in_v) x_e
            // x_v >= sum_(e in delta_out_v) x_e
            for (int v = 0; v < num_vertices; ++v) {
                const auto out_edges = instance.n().out_edges(v);
                const auto in_edges = instance.n().in_edges(v);
                GRBLinExpr lhs = vars["x_v"](tr, t, v);
                GRBLinExpr rhs_in = 0;
                GRBLinExpr rhs_out = 0;
                for (const auto& e : out_edges) {
                    rhs_out += vars["x"](tr, t, e);
                }
                for (const auto& e : in_edges) {
                    rhs_in += vars["x"](tr, t, e);
                }
                if (v == exit) {
                    rhs_out += vars["x_out"](tr, t);
                }
                if (v == entry) {
                    rhs_in += vars["x_in"](tr, t);
                }
                model->addConstr(lhs, GRB_LESS_EQUAL, rhs_out + rhs_in, "train_pos_x_v_" + tr_name + "_" + std::to_string(t) + "_" + std::to_string(v));
                model->addConstr(lhs, GRB_GREATER_EQUAL, rhs_out, "train_pos_x_v_out_" + tr_name + "_" + std::to_string(t) + "_" + std::to_string(v));
                model->addConstr(lhs, GRB_GREATER_EQUAL, rhs_in, "train_pos_x_v_in_" + tr_name + "_" + std::to_string(t) + "_" + std::to_string(v));
            }
            // and sum_e x_e = sum_v x_v - 1
            // add x_in and x_out on both lhs and rhs cancel out
            lhs = 0;
            rhs = -1;
            for (int e = 0; e < num_edges; ++e) {
                lhs += vars["x"](tr, t, e);
            }
            for (int v = 0; v < num_vertices; ++v) {
                rhs += vars["x_v"](tr, t, v);
            }
            model->addConstr(lhs, GRB_EQUAL, rhs, "train_pos_simple_connected_path_" + tr_name + "_" + std::to_string(t));

            // Switches are obeyed, i.e., illegal movements prohibited
            // And train does not go backwards
            for (int e1 = 0; e1 < num_edges; ++e1) {
                const auto& v = instance.n().get_edge(e1).target;
                const auto& out_edges = instance.n().out_edges(v);
                const auto& e_len = instance.n().get_edge(e1).length;
                for (const auto& e2 : out_edges) {
                    if (t < train_interval[tr].second && instance.n().is_valid_successor(e1, e2)) {
                        // Prohibit train going backwards
                        // x_e1(t+1) <= x_e1(t) + (1-x_e2(t))
                        model->addConstr(vars["x"](tr, t + 1, e1), GRB_LESS_EQUAL,
                                         vars["x"](tr, t, e1) + (1 - vars["x"](tr, t, e2)),
                                         "train_pos_no_backwards_" + tr_name + "_" + std::to_string(t) + "_" + std::to_string(e1) + "_" + std::to_string(e2));
                    } else if (!instance.n().is_valid_successor(e1, e2)) {
                        // Prohibit illegal movement
                        // x_e1 + x_e2 <= 1
                        model->addConstr(vars["x"](tr, t, e1) + vars["x"](tr, t, e2), GRB_LESS_EQUAL, 1,
                                         "train_pos_switches_" + tr_name + "_" + std::to_string(t) + "_" + std::to_string(e1) + "_" + std::to_string(e2));
                    }
                }

                // Only going forward on edge
                if (t < train_interval[tr].second) {
                    // e_lda(t) <= e_lda(t+1) + e_len * (1 - x_e(t+1))
                    // e_mu(t) <= e_mu(t+1) + e_len * (1 - x_e(t+1))
                    model->addConstr(vars["e_lda"](tr, t, e1), GRB_LESS_EQUAL,
                                     vars["e_lda"](tr, t + 1, e1) + e_len * (1 - vars["x"](tr, t + 1, e1)),
                                     "train_pos_e_lda_" + tr_name + "_" + std::to_string(t) + "_" + std::to_string(e1));
                    model->addConstr(vars["e_mu"](tr, t, e1), GRB_LESS_EQUAL,
                                        vars["e_mu"](tr, t + 1, e1) + e_len * (1 - vars["x"](tr, t + 1, e1)),
                                        "train_pos_e_mu_" + tr_name + "_" + std::to_string(t) + "_" + std::to_string(e1));
                }
            }
            if (t < train_interval[tr].second) {
                // Also for in and out position, i.e.,
                // len_in is decreasing, len_out is increasing
                model->addConstr(vars["len_in"](tr, t+1), GRB_LESS_EQUAL, vars["len_in"](tr, t), "train_pos_len_in_" + tr_name + "_" + std::to_string(t));
                model->addConstr(vars["len_out"](tr, t+1), GRB_GREATER_EQUAL, vars["len_out"](tr, t), "train_pos_len_out_" + tr_name + "_" + std::to_string(t));
            }
        }
    }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::create_free_routes_overlap_constraints() {
    /**
     * Creates the constraints to ensure the correct overlap when using free routes
     */

    const auto train_list = instance.get_train_list();
    for (int tr = 0; tr < num_tr; ++tr) {
        const auto &tr_name = train_list.get_train(tr).name;
        const auto &tr_len = train_list.get_train(tr_name).length;
        const auto &entry = instance.get_schedule(tr).entry;
        const auto &exit = instance.get_schedule(tr).exit;
        for (int t = train_interval[tr].first; t <= train_interval[tr].second - 1; ++t) {
            // Train cannot be solely on the exit edge
            GRBLinExpr lhs = vars["x_in"](tr, t);
            for (int e = 0; e < num_edges; ++e) {
                lhs += vars["x"](tr, t, e);
            }
            // lhs >= 1
            model->addConstr(lhs, GRB_GREATER_EQUAL, 1, "train_not_left_" + tr_name + "_" + std::to_string(t*dt));

            // Correct overlap length
            lhs = vars["len_in"](tr, t + 1) + vars["len_out"](tr, t);
            for (int e = 0; e < num_edges; ++e) {
                lhs += vars["overlap"](tr, t, e);
            }
            GRBLinExpr rhs = tr_len;
            if (this->include_breaking_distances) {
                rhs += vars["brakelen"](tr, t);
            }
            model->addConstr(lhs, GRB_EQUAL, rhs, "train_pos_overlap_len_" + tr_name + "_" + std::to_string(t));

            // Determine overlap value per edge
            for (int e = 0; e < num_edges; ++e) {
                const auto& e_v0 = instance.n().get_edge(e).source;
                const auto& e_v1 = instance.n().get_edge(e).target;
                const auto& out_edges = instance.n().out_edges(e_v1);
                const auto& e_len = instance.n().get_edge(e).length;

                // overlap >= e_mu(t) - e_lda(t+1) if e is occupied at t+1, i.e.,
                // overlap_e + e_len * (1 - x_e(t+1)) >= e_mu(t) - e_lda(t+1)
                model->addConstr(vars["overlap"](tr, t, e) + e_len * (1 - vars["x"](tr, t + 1, e)), GRB_GREATER_EQUAL,
                                 vars["e_mu"](tr, t, e) - vars["e_lda"](tr, t + 1, e),
                                 "train_pos_overlap_e_lb_" + tr_name + "_" + std::to_string(t) + "_" + std::to_string(e));
                // overlap <= e_mu(t) - e_lda(t+1)
                model->addConstr(vars["overlap"](tr, t, e), GRB_LESS_EQUAL,
                                 vars["e_mu"](tr, t, e) - vars["e_lda"](tr, t + 1, e),
                                 "train_pos_overlap_e_ub_" + tr_name + "_" + std::to_string(t) + "_" + std::to_string(e));

                // overlap <= e_len * x_e(t)
                // overlap <= e_len * x_e(t+1)
                model->addConstr(vars["overlap"](tr, t, e), GRB_LESS_EQUAL, e_len * vars["x"](tr, t, e),
                                 "train_pos_overlap_e_t_" + tr_name + "_" + std::to_string(t) + "_" +
                                 std::to_string(e));
                model->addConstr(vars["overlap"](tr, t, e), GRB_LESS_EQUAL, e_len * vars["x"](tr, t + 1, e),
                                 "train_pos_overlap_e_tp1_" + tr_name + "_" + std::to_string(t) + "_" +
                                 std::to_string(e));

                // Overlap is only at front
                for (const auto& e2 : out_edges) {
                    if (instance.n().is_valid_successor(e, e2)) {
                        // overlap_e <= e_len * overlap_e2 + e_len * (1 - x_e2)
                        model->addConstr(vars["overlap"](tr, t, e), GRB_LESS_EQUAL,
                                         e_len * vars["overlap"](tr, t, e2) + e_len * (1 - vars["x"](tr, t, e2)),
                                         "train_pos_overlap_at_front_" + tr_name + "_" + std::to_string(t) + "_" +
                                         std::to_string(e) + "_" + std::to_string(e2));
                    }
                }
                if (e_v0 == entry) {
                    // len_in <= tr_len * overlap_e + tr_len * (1 - x_e)
                    model->addConstr(vars["len_in"](tr, t), GRB_LESS_EQUAL,
                                     tr_len * vars["overlap"](tr, t, e) + tr_len * (1 - vars["x"](tr, t, e)),
                                     "train_pos_overlap_at_front_" + tr_name + "_" + std::to_string(t) + "_len_in" + std::to_string(e));
                }
                if (e_v1 == exit) {
                    // overlap_e <= e_len * len_out + e_len * (1 - x_out)
                    model->addConstr(vars["overlap"](tr, t, e), GRB_LESS_EQUAL,
                                     e_len * vars["len_out"](tr, t) + e_len * (1 - vars["x_out"](tr, t)),
                                     "train_pos_overlap_at_front_" + tr_name + "_" + std::to_string(t) + "_len_out" + std::to_string(e));
                }
            }
        }
    }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::create_boundary_free_routes_constraints() {
    /**
     * Boundary conditions in case of no fixed routes
     */

    const auto train_list = instance.get_train_list();
    for (int tr = 0; tr < num_tr; ++tr) {
        const auto &tr_name = train_list.get_train(tr).name;
        const auto &tr_len = train_list.get_train(tr_name).length;
        const auto& t0 = train_interval[tr].first;
        const auto& tn = train_interval[tr].second;
        // len_in(t0) = tr_len
        model->addConstr(vars["len_in"](tr, t0), GRB_EQUAL, tr_len, "train_boundary_len_in_" + tr_name + "_" + std::to_string(t0));
        // len_out(tn) = tr_len + brakelen(tn) (if apllicable)
        GRBLinExpr rhs = tr_len;
        if (this->include_breaking_distances) {
            rhs += vars["brakelen"](tr, tn);
        }
        model->addConstr(vars["len_out"](tr, tn), GRB_EQUAL, rhs,
                         "train_boundary_len_out_" + tr_name + "_" + std::to_string(tn));
    }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::create_free_routes_occupation_constraints() {
    /**
     * Connects trains position and occupation variables if routes are not fixed
     */

    const auto train_list = instance.get_train_list();
    for (int tr = 0; tr < num_tr; ++tr) {
        const auto &tr_name = train_list.get_train(tr).name;
        const auto &entry = instance.get_schedule(tr).entry;
        const auto &exit = instance.get_schedule(tr).exit;
        for (int e = 0; e < num_edges; ++e) {
            const auto& e_v0 = instance.n().get_edge(e).source;
            const auto& e_v1 = instance.n().get_edge(e).target;
            const auto& in_edges = instance.n().in_edges(e_v0);
            const auto& out_edges = instance.n().out_edges(e_v1);
            const auto& e_len = instance.n().get_edge(e).length;
            for (int t = train_interval[tr].first; t <= train_interval[tr].second; ++t) {
                // e_lda <= e_mu
                model->addConstr(vars["e_lda"](tr, t, e), GRB_LESS_EQUAL, vars["e_mu"](tr, t, e),
                                 "train_occupation_free_routes_mu_lda_" + tr_name + "_" + std::to_string(t) + "_" + std::to_string(e));
                // e_mu <= e_len * x
                model->addConstr(vars["e_mu"](tr, t, e), GRB_LESS_EQUAL, e_len * vars["x"](tr, t, e),
                                 "train_occupation_free_routes_mu_x_" + tr_name + "_" + std::to_string(t) + "_" + std::to_string(e));

                // e_mu = e_len if not last edge, i.e.,
                // e_mu + e_len*(1-x) >= e_len * sum_outedges x
                GRBLinExpr rhs = 0;
                for (const auto& e2 : out_edges) {
                    rhs += vars["x"](tr, t, e2);
                }
                if (e_v1 == exit) {
                    // exit is an out-edge of the last edge
                    rhs += vars["x_out"](tr, t);
                }
                rhs *= e_len;
                model->addConstr(vars["e_mu"](tr, t, e) + e_len * (1 - vars["x"](tr, t, e)), GRB_GREATER_EQUAL, rhs,
                                 "train_occupation_free_routes_mu_1_if_not_last_edge_" + tr_name + "_" + std::to_string(t) + "_" + std::to_string(e));

                // e_lda = 0 if not first edge, i.e.,
                // e_lda <= e_len * (1 - sum_inedges x) + e_len * (1-x)
                rhs = 2 - vars["x"](tr, t, e);
                for (const auto& e2 : in_edges) {
                    rhs -= vars["x"](tr, t, e2);
                }
                if (e_v0 == entry) {
                    // entry is an in-edge of the first edge
                    rhs -= vars["x_in"](tr, t);
                }
                rhs *= e_len;
                model->addConstr(vars["e_lda"](tr, t, e), GRB_LESS_EQUAL, rhs,
                                 "train_occupation_free_routes_lda_0_if_not_first_edge_" + tr_name + "_" + std::to_string(t) + "_" + std::to_string(e));

                // x = 0 if mu=lda, i.e.,
                // x <= e_mu - e_lda
                model->addConstr(vars["x"](tr, t, e), GRB_LESS_EQUAL, vars["e_mu"](tr, t, e) - vars["e_lda"](tr, t, e),
                                 "train_occupation_free_routes_x_0_if_mu_lda_" + tr_name + "_" + std::to_string(t) + "_" + std::to_string(e));
            }
        }

        // x_in and x_out
        const auto& tr_len = train_list.get_train(tr_name).length;
        double len_out_ub = tr_len;
        if (this->include_breaking_distances) {
            len_out_ub += get_max_brakelen(tr);
        }
        for (int t = train_interval[tr].first; t <= train_interval[tr].second; ++t) {
            // x_in = 1 if, and only if, len_in > 0, i.e.,
            // x_in <= len_in, tr_len * x_in >= len_in
            model->addConstr(vars["x_in"](tr, t), GRB_LESS_EQUAL, vars["len_in"](tr, t),
                             "train_occupation_free_routes_x_in_1_only_if_" + tr_name + "_" + std::to_string(t));
            model->addConstr(tr_len * vars["x_in"](tr, t), GRB_GREATER_EQUAL, vars["len_in"](tr, t),
                                "train_occupation_free_routes_x_in_1_if_" + tr_name + "_" + std::to_string(t));

            // x_out = 1 if, and only if, len_out > 0, i.e.,
            // x_out <= len_out, len_out_ub * x_out >= len_out
            model->addConstr(vars["x_out"](tr, t), GRB_LESS_EQUAL, vars["len_out"](tr, t),
                             "train_occupation_free_routes_x_out_1_only_if_" + tr_name + "_" + std::to_string(t));
            model->addConstr(len_out_ub * vars["x_out"](tr, t), GRB_GREATER_EQUAL, vars["len_out"](tr, t),
                                "train_occupation_free_routes_x_out_1_if_" + tr_name + "_" + std::to_string(t));
        }
    }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::create_free_routes_impossibility_cuts() {
    /**
     * Impossible positions cut off due to schedule.
     */

    const auto apsp = instance.n().all_edge_pairs_shortest_paths();

    // Iterate over all trains
    const auto& train_list = instance.get_train_list();
    for (int tr = 0; tr < train_list.size(); ++tr) {
        const auto tr_name = train_list.get_train(tr).name;
        for (int t = train_interval[tr].first; t <= train_interval[tr].second; ++t) {
            const auto before_after_struct = get_temporary_impossibility_struct(tr, t);
            if (!before_after_struct.to_use) {
                continue;
            }

            // maximum_dist_travelled
            const auto t_steps_before = t - before_after_struct.t_before + 1;
            const auto dist_travelled_before = max_distance_travelled(tr, t_steps_before, before_after_struct.v_before, train_list.get_train(tr).acceleration, this->include_breaking_distances);
            const auto t_steps_after = before_after_struct.t_after - t;
            const auto dist_travelled_after = max_distance_travelled(tr, t_steps_after, before_after_struct.v_after, train_list.get_train(tr).deceleration, false);

            // Iterate over all edges
            for (int e = 0; e < num_edges; ++e) {
                const auto& e_len = instance.n().get_edge(e).length;

                double dist_before, dist_after;

                // Constraint inferred from before position
                if (before_after_struct.t_before <= train_interval[tr].first) {
                    const auto e_before = instance.n().out_edges(instance.get_schedule(tr).entry)[0];
                    const auto& e_len_before = instance.n().get_edge(e_before).length;
                    dist_before = apsp.at(e_before, e) + e_len_before - e_len;
                } else {
                    dist_before = INF;
                    for (const auto& e_tmp : before_after_struct.edges_before) {
                        const auto tmp_val = apsp.at(e_tmp, e) - e_len;
                        if (tmp_val < dist_before) {
                            dist_before = tmp_val;
                        }
                    }
                }

                if (dist_travelled_before < dist_before) {
                    // Edge cannot be reached, i.e. x = 0
                    model->addConstr(vars["x"](tr, t, e), GRB_EQUAL, 0,
                                     "train_occupation_free_routes_impossibility_before_var1_" + tr_name + "_" + std::to_string(t) + "_" + std::to_string(e));
                } else if (dist_travelled_before < dist_before + e_len) {
                    // Edge can be reached, but not fully, i.e.
                    // e_mu <= dist_travelled_before - dist_before
                    model->addConstr(vars["e_mu"](tr, t, e), GRB_LESS_EQUAL, dist_travelled_before - dist_before,
                                     "train_occupation_free_routes_impossibility_before_var2_" + tr_name + "_" + std::to_string(t) + "_" + std::to_string(e));
                }
                // Otherwise no constraint can be inferred

                // Constraint inferred from after position
                if (before_after_struct.t_after >= train_interval[tr].second) {
                    const auto e_after = instance.n().in_edges(instance.get_schedule(tr).exit)[0];
                    dist_after = apsp.at(e, e_after);
                } else {
                    dist_after = INF;
                    for (const auto& e_tmp : before_after_struct.edges_after) {
                        const auto tmp_val = apsp.at(e, e_tmp) - instance.n().get_edge(e_tmp).length;
                        if (tmp_val < dist_after) {
                            dist_after = tmp_val;
                        }
                    }
                }

                if (dist_travelled_after < dist_after) {
                    // Destination is unreachable from edge, hence not possible and x = 0
                    model->addConstr(vars["x"](tr, t, e), GRB_EQUAL, 0,
                                     "train_occupation_free_routes_impossibility_after_var1_" + tr_name + "_" + std::to_string(t) + "_" + std::to_string(e));
                } else if (dist_travelled_after < dist_after + e_len) {
                    // Destination is reachable, but not from full edge, i.e.,
                    // e_lda >= (e_len - (dist_travelled_after - dist_after))*x
                    model->addConstr(vars["e_lda"](tr, t, e), GRB_GREATER_EQUAL, (e_len - (dist_travelled_after - dist_after))*vars["x"](tr, t, e),
                                     "train_occupation_free_routes_impossibility_after_var2_" + tr_name + "_" + std::to_string(t) + "_" + std::to_string(e));
                }
            }

        }
    }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::create_non_discretized_free_route_constraints() {
    /**
     * VSS constraints for free routes
     */

    const auto train_list = instance.get_train_list();
    for (int tr = 0; tr < num_tr; ++tr) {
        const auto &tr_name = train_list.get_train(tr).name;
        for (int e_index = 0; e_index < breakable_edges.size(); ++e_index) {
            const auto& e = breakable_edges[e_index];
            const auto &e_len = instance.n().get_edge(e).length;
            const auto vss_number_e = instance.n().max_vss_on_edge(e);
            for (int t = train_interval[tr].first; t <= train_interval[tr].second; ++t) {
                for (int vss = 0; vss < vss_number_e; ++vss) {
                    // e_mu(e) <= b_pos(e_index) + e_len * (1 - b_front(e_index))
                    model->addConstr(vars["e_mu"](tr, t, e),
                                     GRB_LESS_EQUAL, vars["b_pos"](e_index, vss) + e_len * (1 - vars["b_front"](tr, t, e_index, vss)),
                                     "train_occupation_free_routes_vss_lda_b_pos_b_front_" + tr_name + "_" + std::to_string(t) + "_" + std::to_string(e) + "_" + std::to_string(vss));
                    // b_pos(e_index) <= e_lda(e) + e_len * (1 - b_rear(e_index))
                    model->addConstr(vars["b_pos"](e_index, vss),
                                     GRB_LESS_EQUAL, vars["e_lda"](tr, t, e) + e_len * (1 - vars["b_rear"](tr, t, e_index, vss)),
                                     "train_occupation_free_routes_vss_b_pos_mu_b_rear_" + tr_name + "_" + std::to_string(t) + "_" + std::to_string(e) + "_" + std::to_string(vss));
                }
            }
        }
    }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::create_free_routes_no_overlap_entry_exit_constraints() {
    /**
     * Create constraints on common entry and exit points.
     */

    const auto train_entry_exit_pairs = common_entry_exit_vertices();

    // If two trains share an entry vertex, then the first train must have left before the second train enters
    for (const auto& tr_list : train_entry_exit_pairs.first) {
        for (int i = 0; i < tr_list.size() - 1; ++i) {
            const auto& tr1_entry = train_interval[tr_list[i]].first;
            const auto& tr2_entry = train_interval[tr_list[i + 1]].first;
            if (tr1_entry >= tr2_entry) {
                throw std::runtime_error("Something went wrong with trains " + std::to_string(tr_list[i]) + " and " + std::to_string(tr_list[i + 1]) + " at common entry");
            }
            for (int t = tr2_entry; t < train_interval[tr_list[i]].second; ++t) {
                // len_in(tr1, t) = 0 AND x_in(tr1, t) = 0
                model->addConstr(vars["len_in"](tr_list[i], t), GRB_EQUAL, 0,
                                 "train_occupation_free_routes_common_entry_len_in_" + std::to_string(tr_list[i]) + "_" + std::to_string(tr_list[i+1]) + "_" + std::to_string(t));
                model->addConstr(vars["x_in"](tr_list[i], t), GRB_EQUAL, 0,
                                    "train_occupation_free_routes_common_entry_x_in_" + std::to_string(tr_list[i]) + "_" + std::to_string(tr_list[i+1]) + "_" + std::to_string(t));
            }
        }
    }

    // If two trains share an exit vertex, then the first train must have left before the second train enters
    for (const auto& tr_list : train_entry_exit_pairs.second) {
        for (int i = 0; i < tr_list.size() - 1; ++i) {
            const auto& tr1_exit = train_interval[tr_list[i]].second;
            const auto& tr2_exit = train_interval[tr_list[i + 1]].second;
            if (tr1_exit <= tr2_exit) {
                throw std::runtime_error("Something went wrong with trains " + std::to_string(tr_list[i]) + " and " + std::to_string(tr_list[i + 1]) + " at common exit");
            }
            for (int t = train_interval[tr_list[i]].first; t <= tr2_exit; ++t) {
                // len_out(tr1, t) = 0 AND x_out(tr1, t) = 0
                model->addConstr(vars["len_out"](tr_list[i], t), GRB_EQUAL, 0,
                                 "train_occupation_free_routes_common_exit_len_out_" + std::to_string(tr_list[i]) + "_" + std::to_string(tr_list[i+1]) + "_" + std::to_string(t));
                model->addConstr(vars["x_out"](tr_list[i], t), GRB_EQUAL, 0,
                                    "train_occupation_free_routes_common_exit_x_out_" + std::to_string(tr_list[i]) + "_" + std::to_string(tr_list[i+1]) + "_" + std::to_string(t));
            }
        }
    }
}
