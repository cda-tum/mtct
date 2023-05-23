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

    // Save relevant variables
    dt = delta_t;
    num_t = instance.maxT() / dt + 1;
    num_tr = instance.get_train_list().size();
    num_edges = instance.n().number_of_edges();
    num_vertices = instance.n().number_of_vertices();
    unbreakable_sections = instance.n().unbreakable_sections();
    no_border_vss_sections = instance.n().no_border_vss_sections();
    no_border_vss_vertices = instance.n().get_vertices_by_type(cda_rail::VertexType::NO_BORDER_VSS);

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
    create_vss_variables();
    create_fixed_routes_variables();

    // Set objective
    set_objective();

    // Create constraints
    create_fixed_routes_train_movement_constraints();
    create_boundary_fixed_routes_constraints();
    create_vss_discretized_constraints();
    create_unbreakable_sections_constraints();

    //model->write("model.lp");

    // Optimize
    model->optimize();

    // Print solution of the first train
    const auto& interval = train_interval[0];
    for (int t = interval.first; t <= interval.second; ++t) {
        std::cout << "t = " << t << " at [" << vars["lda"](0, t).get(GRB_DoubleAttr_X) << ", " << vars["mu"](0,t).get(GRB_DoubleAttr_X) << "]" <<  std::endl;
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

    auto train_list = instance.get_train_list();
    for (int i = 0; i < num_tr; ++i) {
        auto tr_name = train_list.get_train(i).name;
        auto r_len = instance.route_length(tr_name);
        auto r_size = instance.get_route(tr_name).size();
        auto initial_speed = instance.get_schedule(tr_name).v_0;
        auto final_speed = instance.get_schedule(tr_name).v_n;
        auto tr_len = instance.get_train_list().get_train(tr_name).length;
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
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::create_vss_variables() {
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
        // If unbreakable_section[i] (of type vector) and tr_route (of type vector) overlap (have a common element), add i to indices
        for (int j0 = 0; j0 < unbreakable_sections[i].size(); ++j0) {
            for (int j1 = 0; j1 < tr_route.size(); ++j1) {
                if (unbreakable_sections[i][j0] == tr_route[j1]) {
                    indices.push_back(i);
                    break;
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
                model->addConstr(lhs >= vars["x_sec"](tr, t, sec_index), "unbreakable_section_only_" + tr_name + "_" + std::to_string(t));
                model->addConstr(lhs <= count * vars["x_sec"](tr, t, sec_index), "unbreakable_section_if_" + tr_name + "_" + std::to_string(t));
            }
        }
        //TODO: sum of x_sec <= 1 for given time
        
    }
}
