#include "CustomExceptions.hpp"
#include "MultiArray.hpp"
#include "gurobi_c++.h"
#include "solver/mip-based/VSSGenTimetableSolver.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <plog/Appenders/ColorConsoleAppender.h>
#include <plog/Formatters/TxtFormatter.h>
#include <plog/Initializers/ConsoleInitializer.h>
#include <plog/Log.h>
#include <utility>

// NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay)

// NOLINTBEGIN(performance-inefficient-string-concatenation)

cda_rail::solver::mip_based::VSSGenTimetableSolver::VSSGenTimetableSolver(
    const instances::VSSGenerationTimetable& instance)
    : GeneralMIPSolver<instances::VSSGenerationTimetable,
                       instances::SolVSSGenerationTimetable>(instance) {
  if (plog::get() == nullptr) {
    static plog::ColorConsoleAppender<plog::TxtFormatter> console_appender;
    plog::init(plog::debug, &console_appender);
  }
}

cda_rail::solver::mip_based::VSSGenTimetableSolver::VSSGenTimetableSolver(
    const std::filesystem::path& instance_path) {
  instance = instances::VSSGenerationTimetable::import_instance(instance_path);
  if (plog::get() == nullptr) {
    static plog::ColorConsoleAppender<plog::TxtFormatter> console_appender;
    plog::init(plog::debug, &console_appender);
  }
}

cda_rail::solver::mip_based::VSSGenTimetableSolver::VSSGenTimetableSolver(
    const std::string& instance_path) {
  instance = instances::VSSGenerationTimetable::import_instance(instance_path);
  if (plog::get() == nullptr) {
    static plog::ColorConsoleAppender<plog::TxtFormatter> console_appender;
    plog::init(plog::debug, &console_appender);
  }
}

cda_rail::solver::mip_based::VSSGenTimetableSolver::VSSGenTimetableSolver(
    const char* instance_path) {
  instance = instances::VSSGenerationTimetable::import_instance(instance_path);
  if (plog::get() == nullptr) {
    static plog::ColorConsoleAppender<plog::TxtFormatter> console_appender;
    plog::init(plog::debug, &console_appender);
  }
}

cda_rail::instances::SolVSSGenerationTimetable
cda_rail::solver::mip_based::VSSGenTimetableSolver::solve(
    const ModelDetail& model_detail, const ModelSettings& model_settings,
    const SolverStrategy&   solver_strategy,
    const SolutionSettings& solution_settings, int time_limit,
    bool debug_input) {
  /**
   * Solves initiated VSSGenerationTimetable instance using Gurobi and a
   * flexible MILP formulation. The level of detail can be controlled using the
   * parameters.
   *
   * @param model_detail: Contains information on the model detail, namely
   * - delta_t: Length of discretized time intervals in seconds. Default: 15
   * - fix_routes: If true, the routes are fixed to the ones given in the
   * - train_dynamic: If true, the train dynamics (i.e., limited acceleration
   * and deceleration) are included in the model. Default: true
   * - braking_curves: If true, the braking curves (i.e., the braking distance
   * depending on the current speed has to be cleared) are included in the
   * model. Default: true
   *
   * @param model_settings: Contains information on the model settings, namely
   * - model_type: Denotes, how the VSS borders are modelled in the solution
   * process. Default uses VSSModel::Continuous
   * - use_pwl: If true, the braking distances are approximated by piecewise
   * linear functions with a fixed maximal error. Otherwise, they are modeled as
   * quadratic functions and Gurobi's ability to solve these using spatial
   * branching is used. Only relevant if include_braking_curves_input is true.
   * Default: false
   * - use_schedule_cuts: If true, the formulation is strengthened using cuts
   * implied by the schedule. Default: true
   *
   * @param solver_strategy: Specify information on the algorithm's strategy to
   * use, namely
   * - iterative_approach: If true, the VSS is iterated to optimality. Default:
   * false
   * - optimality_strategy: Specify the optimality strategy to use. Default:
   * Optimal
   * - update_strategy: Specify the update strategy to use. Only relevant if
   * iterative approach is used. Default: Fixed
   * - initial_value: Specify the initial value or fraction to use. Only
   * relevant if iterative approach is used. In case of fixed update, the value
   * has to be an integer. Otherwise between 0 and 1. Default: 1
   * - update_value: Specify the update value or fraction to use. Only relevant
   * if iterative approach is used. In case of fixed update, the value has to be
   * greater than 1, otherwise between 0 and 1. Default: 2
   *
   * @param solution_settings: Specify information on the solution, namely
   * - postprocess: If true, the solution is postprocessed to remove potentially
   * unused VSS. Default: false
   * - export_option: Denotes if the solution and/or Gurobi model is exported.
   * Default: NoExport
   * - name: Name of the file (without extension) to which the model is
   * exported. Default: "model"
   * - path: Path to which the model is exported. Default: "", i.e., the current
   * working directory
   *
   * @param time_limit: Time limit in seconds. No limit if negative. Default: -1
   *
   * @param debug: If true, (more detailed) debug output is printed. Default:
   * false
   *
   * @return Solution object containing status, objective value, and solution
   */

  auto old_instance =
      initialize_variables(model_detail, model_settings, solver_strategy,
                           solution_settings, time_limit, debug_input);

  create_variables();
  set_objective();
  create_constraints();

  set_timeout(time_limit);

  const auto sol_object = optimize(old_instance, time_limit);

  export_lp_if_applicable(solution_settings);

  if (old_instance.has_value()) {
    instance = old_instance.value();
  }

  export_solution_if_applicable(sol_object, solution_settings);

  cleanup();

  return sol_object.value();
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    create_general_variables() {
  /**
   * Creates general variables that are independent of the fixed route
   */

  vars["v"] = MultiArray<GRBVar>(num_tr, num_t + 1);
  vars["x"] = MultiArray<GRBVar>(num_tr, num_t, num_edges);
  vars["x_sec"] =
      MultiArray<GRBVar>(num_tr, num_t, unbreakable_sections.size());
  vars["y_sec_fwd"] = MultiArray<GRBVar>(num_t, fwd_bwd_sections.size());
  vars["y_sec_bwd"] = MultiArray<GRBVar>(num_t, fwd_bwd_sections.size());

  if (vss_model.get_only_stop_at_vss()) {
    vars["stopped"] = MultiArray<GRBVar>(num_tr, num_t);
  }

  auto train_list = instance.get_train_list();
  for (size_t i = 0; i < num_tr; ++i) {
    auto max_speed = instance.get_train_list().get_train(i).max_speed;
    auto tr_name   = train_list.get_train(i).name;
    for (size_t t = train_interval[i].first; t <= train_interval[i].second + 1;
         ++t) {
      vars["v"](i, t) =
          model->addVar(0, max_speed, 0, GRB_CONTINUOUS,
                        "v_" + tr_name + "_" + std::to_string(t * dt));
    }
    for (size_t t = train_interval[i].first; t <= train_interval[i].second;
         ++t) {
      for (auto const edge_id :
           instance.edges_used_by_train(tr_name, fix_routes)) {
        const auto& edge = instance.n().get_edge(edge_id);
        const auto& edge_name =
            "[" + instance.n().get_vertex(edge.source).name + "," +
            instance.n().get_vertex(edge.target).name + "]";
        vars["x"](i, t, edge_id) = model->addVar(
            0, 1, 0, GRB_BINARY,
            "x_" + tr_name + "_" + std::to_string(t * dt) + "_" + edge_name);
      }
      for (const auto& sec : unbreakable_section_indices(i)) {
        vars["x_sec"](i, t, sec) =
            model->addVar(0, 1, 0, GRB_BINARY,
                          "x_sec_" + tr_name + "_" + std::to_string(t * dt) +
                              "_" + std::to_string(sec));
      }
    }
  }
  for (size_t t = 0; t < num_t; ++t) {
    for (size_t i = 0; i < fwd_bwd_sections.size(); ++i) {
      vars["y_sec_fwd"](t, i) = model->addVar(
          0, 1, 0, GRB_BINARY,
          "y_sec_fwd_" + std::to_string(t * dt) + "_" + std::to_string(i));
      vars["y_sec_bwd"](t, i) = model->addVar(
          0, 1, 0, GRB_BINARY,
          "y_sec_bwd_" + std::to_string(t * dt) + "_" + std::to_string(i));
    }
  }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    create_discretized_variables() {
  /**
   * Creates variables connected to the VSS decisions of the problem
   */

  vars["b"] = MultiArray<GRBVar>(no_border_vss_vertices.size());

  for (size_t i = 0; i < no_border_vss_vertices.size(); ++i) {
    const auto& v_name =
        instance.n().get_vertex(no_border_vss_vertices[i]).name;
    vars["b"](i) = model->addVar(0, 1, 0, GRB_BINARY, "b_" + v_name);
  }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    create_non_discretized_variables() {
  /**
   * This method creates the variables needed if the graph is not discretized.
   */

  int max_vss = 0;
  for (const auto& e : breakable_edges) {
    max_vss = std::max(max_vss, instance.n().max_vss_on_edge(e));
  }

  vars["b_pos"] = MultiArray<GRBVar>(num_breakable_sections, max_vss);
  vars["b_front"] =
      MultiArray<GRBVar>(num_tr, num_t, num_breakable_sections, max_vss);
  vars["b_rear"] =
      MultiArray<GRBVar>(num_tr, num_t, num_breakable_sections, max_vss);

  if (this->vss_model.get_model_type() == vss::ModelType::Inferred) {
    vars["num_vss_segments"]  = MultiArray<GRBVar>(relevant_edges.size());
    vars["frac_vss_segments"] = MultiArray<GRBVar>(
        relevant_edges.size(),
        this->vss_model.get_separation_functions().size(), max_vss);
    vars["edge_type"] =
        MultiArray<GRBVar>(relevant_edges.size(),
                           this->vss_model.get_separation_functions().size());
    vars["frac_type"] = MultiArray<GRBVar>(
        relevant_edges.size(),
        this->vss_model.get_separation_functions().size(), max_vss);
  } else if (this->vss_model.get_model_type() == vss::ModelType::Continuous) {
    vars["b_used"] = MultiArray<GRBVar>(relevant_edges.size(), max_vss);
  } else if (this->vss_model.get_model_type() == vss::ModelType::InferredAlt) {
    vars["type_num_vss_segments"] = MultiArray<GRBVar>(
        relevant_edges.size(),
        this->vss_model.get_separation_functions().size(), max_vss);
  } else {
    throw exceptions::ConsistencyException(
        "Model type not supported for non-discretized graph");
  }

  for (size_t i = 0; i < breakable_edges.size(); ++i) {
    const auto& e            = breakable_edges[i];
    const auto  vss_number_e = instance.n().max_vss_on_edge(e);
    const auto& edge         = instance.n().get_edge(e);
    const auto& edge_len     = edge.length;
    const auto& edge_name    = "[" + instance.n().get_vertex(edge.source).name +
                            "," + instance.n().get_vertex(edge.target).name +
                            "]";
    for (size_t vss = 0; vss < vss_number_e; ++vss) {
      const auto& lb = 0;
      const auto& ub = edge_len;
      vars["b_pos"](i, vss) =
          model->addVar(lb, ub, 0, GRB_CONTINUOUS,
                        "b_pos_" + edge_name + "_" + std::to_string(vss));
      for (size_t tr : instance.trains_on_edge(e, this->fix_routes)) {
        for (size_t t = train_interval[tr].first;
             t <= train_interval[tr].second; ++t) {
          vars["b_front"](tr, t, i, vss) = model->addVar(
              0, 1, 0, GRB_BINARY,
              "b_front_" + std::to_string(tr) + "_" + std::to_string(t * dt) +
                  "_" + edge_name + "_" + std::to_string(vss));
          if (instance.get_train_list().get_train(tr).tim) {
            vars["b_rear"](tr, t, i, vss) = model->addVar(
                0, 1, 0, GRB_BINARY,
                "b_rear_" + std::to_string(tr) + "_" + std::to_string(t * dt) +
                    "_" + edge_name + "_" + std::to_string(vss));
          }
        }
      }
    }
  }

  for (size_t i = 0; i < relevant_edges.size(); ++i) {
    const auto& e            = relevant_edges[i];
    const auto  vss_number_e = instance.n().max_vss_on_edge(e);
    const auto& edge         = instance.n().get_edge(e);
    const auto& edge_name    = "[" + instance.n().get_vertex(edge.source).name +
                            "," + instance.n().get_vertex(edge.target).name +
                            "]";

    if (this->vss_model.get_model_type() == vss::ModelType::Inferred) {
      vars["num_vss_segments"](i) = model->addVar(
          1, vss_number_e + 1, 0, GRB_INTEGER, "num_vss_segments_" + edge_name);

      if (iterative_vss &&
          vss_number_e + 1 > max_vss_per_edge_in_iteration.at(i)) {
        vars["num_vss_segments"](i).set(
            GRB_DoubleAttr_UB,
            static_cast<double>(max_vss_per_edge_in_iteration.at(i)) + 1);
      }

      for (size_t sep_type = 0;
           sep_type < this->vss_model.get_separation_functions().size();
           ++sep_type) {
        vars["edge_type"](i, sep_type) = model->addVar(
            0, 1, 0, GRB_BINARY,
            "edge_type_" + edge_name + "_" + std::to_string(sep_type));
        for (size_t vss = 0; vss < vss_number_e; ++vss) {
          const auto& lb                              = 0.0;
          const auto& ub                              = 1.0;
          vars["frac_vss_segments"](i, sep_type, vss) = model->addVar(
              lb, ub, 0, GRB_CONTINUOUS,
              "frac_vss_segments_" + edge_name + "_" +
                  std::to_string(sep_type) + "_" + std::to_string(vss));
          vars["frac_type"](i, sep_type, vss) = model->addVar(
              lb, ub, 0, GRB_CONTINUOUS,
              "frac_type_" + edge_name + "_" + std::to_string(sep_type) + "_" +
                  std::to_string(vss));
        }
      }
    } else if (this->vss_model.get_model_type() == vss::ModelType::Continuous) {
      for (size_t vss = 0; vss < vss_number_e; ++vss) {
        vars["b_used"](i, vss) =
            model->addVar(0, 1, 0, GRB_BINARY,
                          "b_used_" + edge_name + "_" + std::to_string(vss));
        if (iterative_vss && vss >= max_vss_per_edge_in_iteration.at(i)) {
          vars["b_used"](i, vss).set(GRB_DoubleAttr_UB, 0);
        }
      }
    } else if (this->vss_model.get_model_type() ==
               vss::ModelType::InferredAlt) {
      for (size_t sep_type = 0;
           sep_type < this->vss_model.get_separation_functions().size();
           ++sep_type) {
        for (size_t vss = 0; vss < vss_number_e; ++vss) {
          vars["type_num_vss_segments"](i, sep_type, vss) = model->addVar(
              0, 1, 0, GRB_BINARY,
              "type_num_vss_segments_" + edge_name + "_" +
                  std::to_string(sep_type) + "_" + std::to_string(vss));

          if (iterative_vss && vss >= max_vss_per_edge_in_iteration.at(i)) {
            vars["type_num_vss_segments"](i, sep_type, vss)
                .set(GRB_DoubleAttr_UB, 0);
          }
        }
      }
    }
  }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    create_non_discretized_only_stop_at_vss_variables() {
  int max_vss = 0;
  for (const auto& e : breakable_edges) {
    max_vss = std::max(max_vss, instance.n().max_vss_on_edge(e));
  }

  vars["b_tight"] =
      MultiArray<GRBVar>(num_tr, num_t, num_breakable_sections, max_vss);
  vars["e_tight"] = MultiArray<GRBVar>(num_tr, num_t, num_edges);

  for (size_t i = 0; i < breakable_edges.size(); ++i) {
    const auto& e            = breakable_edges[i];
    const auto  vss_number_e = instance.n().max_vss_on_edge(e);
    const auto& edge         = instance.n().get_edge(e);
    const auto& edge_name    = "[" + instance.n().get_vertex(edge.source).name +
                            "," + instance.n().get_vertex(edge.target).name +
                            "]";
    for (size_t vss = 0; vss < vss_number_e; ++vss) {
      for (size_t tr : instance.trains_on_edge(e, this->fix_routes)) {
        const auto& tr_name = instance.get_train_list().get_train(tr).name;
        for (size_t t = train_interval[tr].first + 2;
             t <= train_interval[tr].second; ++t) {
          vars["b_tight"](tr, t, i, vss) = model->addVar(
              0, 1, 0, GRB_BINARY,
              "b_tight_" + tr_name + "_" + std::to_string(t * dt) + "_" +
                  edge_name + "_" + std::to_string(vss));
        }
      }
    }
  }

  for (size_t e = 0; e < num_edges; ++e) {
    const auto& edge      = instance.n().get_edge(e);
    const auto& edge_name = "[" + instance.n().get_vertex(edge.source).name +
                            "," + instance.n().get_vertex(edge.target).name +
                            "]";
    for (size_t tr : instance.trains_on_edge(e, this->fix_routes)) {
      const auto& tr_name = instance.get_train_list().get_train(tr).name;
      for (size_t t = train_interval[tr].first + 2;
           t <= train_interval[tr].second; ++t) {
        vars["e_tight"](tr, t, e) =
            model->addVar(0, 1, 0, GRB_BINARY,
                          "e_tight_" + tr_name + "_" + std::to_string(t * dt) +
                              "_" + edge_name);
      }
    }
  }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::set_objective() {
  /**
   * Sets the objective function of the problem
   */

  PLOGD << "Set objective";

  // sum over all b_i as in no_border_vss_vertices
  objective_expr = 0;
  if (vss_model.get_model_type() == vss::ModelType::Discrete) {
    for (size_t i = 0; i < no_border_vss_vertices.size(); ++i) {
      objective_expr += vars["b"](i);
    }
  } else if (vss_model.get_model_type() == vss::ModelType::Continuous) {
    for (size_t i = 0; i < relevant_edges.size(); ++i) {
      const auto& e            = relevant_edges[i];
      const auto  vss_number_e = instance.n().max_vss_on_edge(e);
      for (size_t vss = 0; vss < vss_number_e; ++vss) {
        objective_expr += vars["b_used"](i, vss);
      }
    }
  } else if (vss_model.get_model_type() == vss::ModelType::Inferred) {
    for (size_t i = 0; i < relevant_edges.size(); ++i) {
      objective_expr += (vars["num_vss_segments"](i) - 1);
    }
  } else if (vss_model.get_model_type() == vss::ModelType::InferredAlt) {
    for (size_t i = 0; i < relevant_edges.size(); ++i) {
      const auto& e            = relevant_edges[i];
      const auto  vss_number_e = instance.n().max_vss_on_edge(e);
      for (size_t vss = 0; vss < vss_number_e; ++vss) {
        for (size_t sep_type = 0;
             sep_type < this->vss_model.get_separation_functions().size();
             ++sep_type) {
          objective_expr += (static_cast<double>(vss) + 1) *
                            vars["type_num_vss_segments"](i, sep_type, vss);
        }
      }
    }
  } else {
    throw std::logic_error("Objective for vss model type not implemented");
  }
  model->setObjective(objective_expr, GRB_MINIMIZE);
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    create_discretized_constraints() {
  /**
   * Creates VSS constraints, i.e., on NoBorderVSS sections two trains must be
   * separated by a chosen vertex.
   */

  for (const auto& no_border_vss_section : no_border_vss_sections) {
    const auto tr_on_section =
        instance.trains_in_section(no_border_vss_section);
    const auto no_border_vss_section_sorted =
        instance.n().combine_reverse_edges(no_border_vss_section, true);
    for (size_t i = 0; i < tr_on_section.size(); ++i) {
      const auto& tr1          = tr_on_section[i];
      const auto& tr1_interval = train_interval[tr1];
      const auto& tr1_name     = instance.get_train_list().get_train(tr1).name;
      const auto& tr1_route    = instance.get_route(tr1_name);
      for (size_t j = i + 1; j < tr_on_section.size(); ++j) {
        const auto& tr2          = tr_on_section[j];
        const auto& tr2_interval = train_interval[tr2];
        const auto& tr2_name  = instance.get_train_list().get_train(tr2).name;
        const auto& tr2_route = instance.get_route(tr2_name);
        std::pair<size_t, size_t> const t_interval = {
            std::max(tr1_interval.first, tr2_interval.first),
            std::min(tr1_interval.second, tr2_interval.second)};
        for (size_t t = t_interval.first; t <= t_interval.second; ++t) {
          for (size_t e1 = 0; e1 < no_border_vss_section_sorted.size(); ++e1) {
            for (size_t e2 = 0; e2 < no_border_vss_section_sorted.size();
                 ++e2) {
              if (e1 == e2) {
                continue;
              }
              GRBLinExpr lhs        = 2;
              GRBLinExpr lhs_first  = 0;
              GRBLinExpr lhs_second = 0;
              if (tr1_route.contains_edge(
                      no_border_vss_section_sorted[e1].first)) {
                lhs -= vars["x"](
                    tr1, t, no_border_vss_section_sorted[e1].first.value());
                lhs_first += vars["x"](
                    tr1, t, no_border_vss_section_sorted[e1].first.value());
              }
              if (tr1_route.contains_edge(
                      no_border_vss_section_sorted[e1].second)) {
                lhs -= vars["x"](
                    tr1, t, no_border_vss_section_sorted[e1].second.value());
                lhs_second += vars["x"](
                    tr1, t, no_border_vss_section_sorted[e1].second.value());
              }
              if (tr2_route.contains_edge(
                      no_border_vss_section_sorted[e2].first)) {
                lhs -= vars["x"](
                    tr2, t, no_border_vss_section_sorted[e2].first.value());
                lhs_first += vars["x"](
                    tr2, t, no_border_vss_section_sorted[e2].first.value());
              }
              if (tr2_route.contains_edge(
                      no_border_vss_section_sorted[e2].second)) {
                lhs -= vars["x"](
                    tr2, t, no_border_vss_section_sorted[e2].second.value());
                lhs_second += vars["x"](
                    tr2, t, no_border_vss_section_sorted[e2].second.value());
              }

              for (size_t e_overlap = std::min(e1, e2);
                   e_overlap < std::max(e1, e2); ++e_overlap) {
                const auto& v_overlap = instance.n().common_vertex(
                    no_border_vss_section_sorted[e_overlap],
                    no_border_vss_section_sorted[e_overlap + 1]);
                if (!v_overlap.has_value()) {
                  throw exceptions::ConsistencyException(
                      "No common vertex found, this should not have happened");
                }

                auto const v_overlap_index =
                    std::find(no_border_vss_vertices.begin(),
                              no_border_vss_vertices.end(), v_overlap.value()) -
                    no_border_vss_vertices.begin();
                if (v_overlap_index >= no_border_vss_vertices.size()) {
                  throw exceptions::ConsistencyException(
                      "Vertex not found in no_border_vss_vertices, this should "
                      "not have happened");
                }
                lhs += vars["b"](v_overlap_index);
              }

              model->addConstr(
                  lhs >= 1,
                  "vss_" + tr1_name + "_" + tr2_name + "_" + std::to_string(t) +
                      "_" +
                      std::to_string(
                          no_border_vss_section_sorted[e1].first.value()) +
                      "_" +
                      std::to_string(
                          no_border_vss_section_sorted[e2].first.value()));

              if ((!instance.get_train_list().get_train(tr1).tim &&
                   (e1 > e2)) ||
                  (!instance.get_train_list().get_train(tr2).tim &&
                   (e2 > e1))) {
                // lhs_first <= 1
                model->addConstr(
                    lhs_first <= 1,
                    "vss_tim_first_" + tr1_name + "_" + tr2_name + "_" +
                        std::to_string(t) + "_" +
                        std::to_string(
                            no_border_vss_section_sorted[e1].first.value()) +
                        "_" +
                        std::to_string(
                            no_border_vss_section_sorted[e2].first.value()) +
                        "_first");
              }
              if ((!instance.get_train_list().get_train(tr2).tim &&
                   (e1 > e2)) ||
                  (!instance.get_train_list().get_train(tr1).tim &&
                   (e2 > e1))) {
                // lhs_second <= 1
                model->addConstr(
                    lhs_second <= 1,
                    "vss_tim_second_" + tr1_name + "_" + tr2_name + "_" +
                        std::to_string(t) + "_" +
                        std::to_string(
                            no_border_vss_section_sorted[e1].first.value()) +
                        "_" +
                        std::to_string(
                            no_border_vss_section_sorted[e2].first.value()) +
                        "_first");
              }
            }
          }
        }
      }
    }
  }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    create_unbreakable_sections_constraints() {
  /**
   * Creates constraints for unbreakable sections, i.e., only one train can be
   * on an unbreakable section at a time.
   */

  for (size_t sec_index = 0; sec_index < unbreakable_sections.size();
       ++sec_index) {
    const auto& sec       = unbreakable_sections[sec_index];
    const auto& tr_on_sec = instance.trains_in_section(sec);
    // tr is on section if it occupies at least one edge of the section
    for (auto const tr : tr_on_sec) {
      const auto& tr_interval = train_interval[tr];
      const auto& tr_name     = instance.get_train_list().get_train(tr).name;
      const auto& tr_route    = instance.get_route(tr_name);
      for (size_t t = tr_interval.first; t <= tr_interval.second; ++t) {
        GRBLinExpr lhs   = 0;
        int        count = 0;
        for (auto const e_index : sec) {
          if (tr_route.contains_edge(e_index)) {
            lhs += vars["x"](tr, t, e_index);
            count++;
          }
        }
        model->addConstr(lhs >= vars["x_sec"](tr, t, sec_index),
                         "unbreakable_section_only_" + tr_name + "_" +
                             std::to_string(t) + "_" +
                             std::to_string(sec_index));
        model->addConstr(lhs <= count * vars["x_sec"](tr, t, sec_index),
                         "unbreakable_section_if_" + tr_name + "_" +
                             std::to_string(t) + "_" +
                             std::to_string(sec_index));
      }
    }

    for (size_t t = 0; t <= num_t; ++t) {
      const auto tr_to_consider =
          instance.trains_at_t(static_cast<int>(t) * dt, tr_on_sec);
      GRBLinExpr lhs = 0;
      for (auto const tr : tr_to_consider) {
        lhs += vars["x_sec"](tr, t, sec_index);
      }
      model->addConstr(lhs <= 1, "unbreakable_section" +
                                     std::to_string(sec_index) +
                                     "_at_most_one_" + std::to_string(t));
    }
  }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    create_general_schedule_constraints() {
  /**
   * Creates constraints for general stations, i.e., if a train is in a station:
   * - all other x variables are 0
   * - the speed is 0
   */

  const auto& train_list = instance.get_train_list();
  for (size_t tr = 0; tr < train_list.size(); ++tr) {
    const auto  tr_name     = train_list.get_train(tr).name;
    const auto& tr_schedule = instance.get_schedule(tr_name);
    const auto& tr_edges = instance.edges_used_by_train(tr, this->fix_routes);
    for (const auto& tr_stop : tr_schedule.get_stops()) {
      const auto t0 = static_cast<size_t>(tr_stop.arrival() / dt);
      const auto t1 = static_cast<size_t>(
          std::ceil(static_cast<double>(tr_stop.departure()) / dt));
      const auto& stop_edges = instance.get_station_list()
                                   .get_station(tr_stop.get_station_name())
                                   .tracks;
      const auto inverse_stop_edges =
          instance.n().inverse_edges(stop_edges, tr_edges);
      for (size_t t = t0 - 1; t <= t1; ++t) {
        if (t >= t0) {
          model->addConstr(vars["v"](tr, t) == 0, "station_speed_" + tr_name +
                                                      "_" + std::to_string(t));
        }
        if (t >= t0 && t < t1) { // because otherwise the front corresponds to
                                 // t1+dt which is allowed outside
          for (auto const e : inverse_stop_edges) {
            model->addConstr(vars["x"](tr, t, e) == 0,
                             "station_x_" + tr_name + "_" + std::to_string(t) +
                                 "_" + std::to_string(e));
          }
        }
        // At least on station edge must be occupied, this also holds for the
        // leaving and entering time interval
        GRBLinExpr lhs = 0;
        for (auto const e : stop_edges) {
          // If e in tr_edges
          if (std::find(tr_edges.begin(), tr_edges.end(), e) !=
              tr_edges.end()) {
            lhs += vars["x"](tr, t, e);
          }
        }
        model->addConstr(lhs >= 1, "station_occupancy_" + tr_name + "_" +
                                       std::to_string(t));
      }
    }
  }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    create_acceleration_constraints() {
  /**
   * This method adds constraints connected to acceleration and deceleration of
   * the trains.
   */

  const auto& train_list = instance.get_train_list();
  for (size_t tr = 0; tr < train_list.size(); ++tr) {
    // Iterate over all time steps
    const auto& tr_object = train_list.get_train(tr);
    for (size_t t = train_interval[tr].first; t <= train_interval[tr].second;
         ++t) {
      // v(t+1) - v(t) <= acceleration * dt
      model->addConstr(vars["v"](tr, t + 1) - vars["v"](tr, t) <=
                           tr_object.acceleration * dt,
                       "acceleration_" + tr_object.name + "_" +
                           std::to_string(t));
      // v(t) - v(t+1) <= deceleration * dt
      model->addConstr(vars["v"](tr, t) - vars["v"](tr, t + 1) <=
                           tr_object.deceleration * dt,
                       "deceleration_" + tr_object.name + "_" +
                           std::to_string(t));
    }
  }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    create_brakelen_variables() {
  /**
   * This method creates the variables corresponding to breaking distances.
   */

  vars["brakelen"] = MultiArray<GRBVar>(num_tr, num_t);
  for (size_t tr = 0; tr < num_tr; ++tr) {
    const auto  max_break_len = get_max_brakelen(tr);
    const auto& tr_name       = instance.get_train_list().get_train(tr).name;
    for (size_t t = train_interval[tr].first; t <= train_interval[tr].second;
         ++t) {
      vars["brakelen"](tr, t) =
          model->addVar(0, max_break_len, 0, GRB_CONTINUOUS,
                        "brakelen_" + tr_name + "_" + std::to_string(t * dt));
    }
  }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    create_general_constraints() {
  /**
   * These constraints appear in all variants
   */

  create_general_schedule_constraints();
  create_unbreakable_sections_constraints();
  create_general_speed_constraints();
  create_reverse_occupation_constraints();
  create_general_boundary_constraints();

  if (vss_model.get_only_stop_at_vss()) {
    for (size_t tr = 0; tr < num_tr; ++tr) {
      const auto& tr_speed = instance.get_train_list().get_train(tr).max_speed;
      for (size_t t = train_interval[tr].first; t <= train_interval[tr].second;
           ++t) {
        // v(tr,t) = 0 iff stopped(tr,t) = 0 otherwise v(tr,t) >= V_MIN
        model->addConstr(
            vars["v"](tr, t), GRB_GREATER_EQUAL, V_MIN * vars["stopped"](tr, t),
            "v_min_" + std::to_string(tr) + "_" + std::to_string(t * dt));
        model->addConstr(
            vars["v"](tr, t), GRB_LESS_EQUAL, tr_speed * vars["stopped"](tr, t),
            "v_max_" + std::to_string(tr) + "_" + std::to_string(t * dt));
      }
    }
  }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    create_non_discretized_constraints() {
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
  if (vss_model.get_model_type() == vss::ModelType::Inferred) {
    create_non_discretized_fraction_constraints();
  } else if (vss_model.get_model_type() == vss::ModelType::InferredAlt) {
    create_non_discretized_alt_fraction_constraints();
  }
  if (vss_model.get_only_stop_at_vss()) {
    create_non_discretized_general_only_stop_at_vss_constraints();
  }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    create_non_discretized_general_constraints() {
  /**
   * These constraints appear only when the graph is not discretized, but are
   * general enough to appear in all model variants.
   */
  // VSS can only be used if it is non-zero
  if (vss_model.get_model_type() == vss::ModelType::Continuous) {
    for (size_t i = 0; i < relevant_edges.size(); ++i) {
      const auto& e               = relevant_edges[i];
      const auto& e_index         = breakable_edge_indices[e];
      const auto  vss_number_e    = instance.n().max_vss_on_edge(e);
      const auto& e_len           = instance.n().get_edge(e).length;
      const auto& min_block_len_e = instance.n().get_edge(e).min_block_length;
      for (size_t vss = 0; vss < vss_number_e; ++vss) {
        model->addConstr(e_len * vars["b_used"](i, vss), GRB_GREATER_EQUAL,
                         vars["b_pos"](e_index, vss),
                         "b_used_" + std::to_string(e) + "_" +
                             std::to_string(vss));
        model->addConstr(vars["b_pos"](e_index, vss), GRB_GREATER_EQUAL,
                         vars["b_used"](i, vss) * min_block_len_e,
                         "b_used_min_value_if_used_" + std::to_string(e) + "_" +
                             std::to_string(vss));
        // Also remove redundant solutions
        if (vss < vss_number_e - 1) {
          model->addConstr(vars["b_pos"](e_index, vss), GRB_GREATER_EQUAL,
                           vars["b_pos"](e_index, vss + 1) +
                               vars["b_used"](i, vss + 1) * min_block_len_e,
                           "b_used_decreasing_" + std::to_string(e) + "_" +
                               std::to_string(vss));
        }
      }
    }
  }

  // Connect position of reverse edges
  for (const auto& e_pair : breakable_edges_pairs) {
    if (!e_pair.second.has_value() || !e_pair.first.has_value()) {
      continue;
    }
    const auto vss_number_e =
        instance.n().max_vss_on_edge(e_pair.first.value());
    if (instance.n().max_vss_on_edge(e_pair.second.value()) != vss_number_e) {
      throw exceptions::ConsistencyException(
          "VSS number of edges " + std::to_string(e_pair.first.value()) +
          " and " + std::to_string(e_pair.second.value()) + " do not match");
    }
    const auto& e_len = instance.n().get_edge(e_pair.first.value()).length;
    for (size_t vss = 0; vss < vss_number_e; ++vss) {
      model->addConstr(
          vars["b_pos"](breakable_edge_indices[e_pair.first.value()], vss) +
              vars["b_pos"](breakable_edge_indices[e_pair.second.value()], vss),
          GRB_EQUAL, e_len,
          "b_pos_reverse_" + std::to_string(e_pair.first.value()) + "_" +
              std::to_string(vss) + "_" +
              std::to_string(e_pair.second.value()) + "_" +
              std::to_string(vss));
    }
  }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    create_non_discretized_position_constraints() {
  /**
   * Creates the position constraints related to non-discretized VSS blocks
   */

  // Border only usable by a train if it is on the edge
  for (size_t e_index = 0; e_index < breakable_edges.size(); ++e_index) {
    const auto& e = breakable_edges[e_index];
    for (const auto& tr : instance.trains_on_edge(e, this->fix_routes)) {
      const auto vss_number_e = instance.n().max_vss_on_edge(e);
      for (size_t t = train_interval[tr].first; t <= train_interval[tr].second;
           ++t) {
        for (size_t vss = 0; vss < vss_number_e; ++vss) {
          // x(tr,t,e) >= b_front(tr,t,e_index,vss)
          model->addConstr(vars["x"](tr, t, e), GRB_GREATER_EQUAL,
                           vars["b_front"](tr, t, e_index, vss),
                           "x_b_front_" + std::to_string(tr) + "_" +
                               std::to_string(t) + "_" + std::to_string(e) +
                               "_" + std::to_string(vss));
          // x(tr,t,e) >= b_rear(tr,t,e_index,vss)
          if (instance.get_train_list().get_train(tr).tim) {
            model->addConstr(vars["x"](tr, t, e), GRB_GREATER_EQUAL,
                             vars["b_rear"](tr, t, e_index, vss),
                             "x_b_rear_" + std::to_string(tr) + "_" +
                                 std::to_string(t) + "_" + std::to_string(e) +
                                 "_" + std::to_string(vss));
          }
        }
      }
    }
  }

  // Correct number of borders
  for (size_t e_index = 0; e_index < breakable_edges.size(); ++e_index) {
    const auto& e            = breakable_edges[e_index];
    const auto  vss_number_e = instance.n().max_vss_on_edge(e);
    const auto& tr_on_e      = instance.trains_on_edge(e, this->fix_routes);
    for (size_t t = 0; t < num_t; ++t) {
      // sum_(tr,vss) b_front(tr, t, e_index, vss) >= sum_(tr) x(tr, t, e) - 1
      // sum_(tr,vss) b_rear(tr, t, e_index, vss) >= sum_(tr) x(tr, t, e) - 1
      GRBLinExpr lhs_front         = 0;
      GRBLinExpr lhs_rear          = 0;
      GRBLinExpr rhs               = -1;
      bool       create_constraint = false;
      for (const auto& tr :
           instance.trains_at_t(static_cast<int>(t) * dt, tr_on_e)) {
        create_constraint = true;
        for (size_t vss = 0; vss < vss_number_e; ++vss) {
          lhs_front += vars["b_front"](tr, t, e_index, vss);
          if (instance.get_train_list().get_train(tr).tim) {
            lhs_rear += vars["b_rear"](tr, t, e_index, vss);
          }
        }
        rhs += vars["x"](tr, t, e);
      }
      if (create_constraint) {
        model->addConstr(lhs_front, GRB_GREATER_EQUAL, rhs,
                         "b_front_correct_number_" + std::to_string(t) + "_" +
                             std::to_string(e) + "_" + std::to_string(e_index));
        model->addConstr(lhs_rear, GRB_GREATER_EQUAL, rhs,
                         "b_rear_correct_number_" + std::to_string(t) + "_" +
                             std::to_string(e) + "_" + std::to_string(e_index));
        // lhs_front = lhs_rear
        model->addConstr(lhs_front, GRB_EQUAL, lhs_rear,
                         "b_front_rear_correct_number_equal_" +
                             std::to_string(t) + "_" + std::to_string(e) + "_" +
                             std::to_string(e_index));
      }
    }
  }

  // At most one border used per train
  for (size_t tr = 0; tr < num_tr; ++tr) {
    for (size_t t = train_interval[tr].first; t < train_interval[tr].second;
         ++t) {
      // sum_(e,vss) b_front(tr, t, e_index, vss) <= 1
      // sum_(e,vss) b_rear(tr, t, e_index, vss) <= 1
      GRBLinExpr lhs_front = 0;
      GRBLinExpr lhs_rear  = 0;
      for (const auto& e : instance.edges_used_by_train(tr, this->fix_routes)) {
        const auto& e_index      = breakable_edge_indices[e];
        const auto  vss_number_e = instance.n().max_vss_on_edge(e);
        for (size_t vss = 0; vss < vss_number_e; ++vss) {
          lhs_front += vars["b_front"](tr, t, e_index, vss);
          if (instance.get_train_list().get_train(tr).tim) {
            lhs_rear += vars["b_rear"](tr, t, e_index, vss);
          }
        }
      }
      model->addConstr(lhs_front, GRB_LESS_EQUAL, 1,
                       "b_front_at_most_one_" + std::to_string(tr) + "_" +
                           std::to_string(t));
      model->addConstr(lhs_rear, GRB_LESS_EQUAL, 1,
                       "b_rear_at_most_one_" + std::to_string(tr) + "_" +
                           std::to_string(t));
    }
  }

  // A border must be both front and rear or nothing
  for (size_t e_index = 0; e_index < breakable_edges.size(); ++e_index) {
    const auto& e            = breakable_edges[e_index];
    const auto  tr_on_e      = instance.trains_on_edge(e, this->fix_routes);
    const auto  vss_number_e = instance.n().max_vss_on_edge(e);
    for (size_t t = 0; t < num_t; ++t) {
      for (size_t vss = 0; vss < vss_number_e; ++vss) {
        // sum_tr b_front(tr, t, e_index, vss) = sum_tr b_rear(tr, t, e_index,
        // vss) <= 1
        GRBLinExpr lhs = 0;
        GRBLinExpr rhs = 0;
        for (const auto& tr :
             instance.trains_at_t(static_cast<int>(t) * dt, tr_on_e)) {
          lhs += vars["b_front"](tr, t, e_index, vss);
          if (instance.get_train_list().get_train(tr).tim) {
            rhs += vars["b_rear"](tr, t, e_index, vss);
          }
        }
        model->addConstr(lhs, GRB_EQUAL, rhs,
                         "b_front_rear_" + std::to_string(t) + "_" +
                             std::to_string(e) + "_" + std::to_string(vss));
        model->addConstr(rhs, GRB_LESS_EQUAL, 1,
                         "b_front_rear_limit_" + std::to_string(t) + "_" +
                             std::to_string(e) + "_" + std::to_string(vss));
      }
    }
  }

  // A border is only usable if the VSS is used
  for (size_t e_index = 0; e_index < breakable_edges.size(); ++e_index) {
    const auto& e = breakable_edges[e_index];
    for (const auto& tr : instance.trains_on_edge(e, this->fix_routes)) {
      const auto vss_number_e = instance.n().max_vss_on_edge(e);
      // Get index of e in relevant_edges array
      const auto find_index =
          std::find(relevant_edges.begin(), relevant_edges.end(), e);
      auto e_index_relevant = find_index - relevant_edges.begin();
      // If edge not found check reverse edge
      if (find_index == relevant_edges.end()) {
        const auto reverse_e = instance.n().get_reverse_edge_index(e).value();
        const auto find_index_reverse =
            std::find(relevant_edges.begin(), relevant_edges.end(), reverse_e);
        if (find_index_reverse == relevant_edges.end()) {
          throw exceptions::ConsistencyException(
              "Edge " + std::to_string(e) + " and its reverse edge " +
              std::to_string(reverse_e) + " not found in relevant_edges");
        }
        e_index_relevant = find_index_reverse - relevant_edges.begin();
      }

      for (size_t t = train_interval[tr].first; t <= train_interval[tr].second;
           ++t) {
        for (size_t vss = 0; vss < vss_number_e; ++vss) {
          if (vss_model.get_model_type() == vss::ModelType::Continuous) {
            // b_front(tr, t, e_index, vss) <= b_used(e_index_relevant, vss)
            model->addConstr(vars["b_front"](tr, t, e_index, vss),
                             GRB_LESS_EQUAL,
                             vars["b_used"](e_index_relevant, vss),
                             "b_front_b_used_" + std::to_string(tr) + "_" +
                                 std::to_string(t) + "_" + std::to_string(e) +
                                 "_" + std::to_string(vss));
            // b_rear(tr, t, e_index, vss) <= b_used(e_index_relevant, vss)
            if (instance.get_train_list().get_train(tr).tim) {
              model->addConstr(vars["b_rear"](tr, t, e_index, vss),
                               GRB_LESS_EQUAL,
                               vars["b_used"](e_index_relevant, vss),
                               "b_rear_b_used_" + std::to_string(tr) + "_" +
                                   std::to_string(t) + "_" + std::to_string(e) +
                                   "_" + std::to_string(vss));
            }
          } else if (vss_model.get_model_type() == vss::ModelType::Inferred) {
            // b_front(tr, t, e_index, vss) <=
            // (num_vss_segments(e_index_relevant) - 1) / (vss + 1)
            model->addConstr(vars["b_front"](tr, t, e_index, vss),
                             GRB_LESS_EQUAL,
                             (vars["num_vss_segments"](e_index_relevant) - 1) /
                                 (static_cast<double>(vss) + 1),
                             "b_front_num_vss_segments_" + std::to_string(tr) +
                                 "_" + std::to_string(t) + "_" +
                                 std::to_string(e) + "_" + std::to_string(vss));
            // b_rear(tr, t, e_index, vss) <=
            // (num_vss_segments(e_index_relevant) - 1) / (vss + 1)
            if (instance.get_train_list().get_train(tr).tim) {
              model->addConstr(
                  vars["b_rear"](tr, t, e_index, vss), GRB_LESS_EQUAL,
                  (vars["num_vss_segments"](e_index_relevant) - 1) /
                      (static_cast<double>(vss) + 1),
                  "b_rear_num_vss_segments_" + std::to_string(tr) + "_" +
                      std::to_string(t) + "_" + std::to_string(e) + "_" +
                      std::to_string(vss));
            }
          } else if (vss_model.get_model_type() ==
                     vss::ModelType::InferredAlt) {
            // b_front(tr, t, e_index, vss) <= sum
            // type_num_vss_segments(e_index_relevant, *, <= vss)
            GRBLinExpr rhs = 0;
            for (size_t sep_type_index = 0;
                 sep_type_index < vss_model.get_separation_functions().size();
                 ++sep_type_index) {
              for (size_t vss2 = 0; vss2 <= vss; ++vss2) {
                rhs += vars["type_num_vss_segments"](e_index_relevant,
                                                     sep_type_index, vss2);
              }
            }
            model->addConstr(vars["b_front"](tr, t, e_index, vss),
                             GRB_LESS_EQUAL, rhs,
                             "b_front_num_vss_segments_" + std::to_string(tr) +
                                 "_" + std::to_string(t) + "_" +
                                 std::to_string(e) + "_" + std::to_string(vss));
            // b_rear(tr, t, e_index, vss) <= sum
            // type_num_vss_segments(e_index_relevant, *, <= vss)
            if (instance.get_train_list().get_train(tr).tim) {
              model->addConstr(
                  vars["b_rear"](tr, t, e_index, vss), GRB_LESS_EQUAL, rhs,
                  "b_rear_num_vss_segments_" + std::to_string(tr) + "_" +
                      std::to_string(t) + "_" + std::to_string(e) + "_" +
                      std::to_string(vss));
            }
          }
        }
      }
    }
  }

  // At most one non-tim train can be on any breakable edge
  for (const auto& e : breakable_edges) {
    const auto  tr_on_e = instance.trains_on_edge(e, this->fix_routes);
    const auto& edge    = instance.n().get_edge(e);
    const auto& v0      = instance.n().get_vertex(edge.source);
    const auto& v1      = instance.n().get_vertex(edge.target);
    const auto  e_name  = "[" + v0.name + "," + v1.name + "]";
    for (size_t t = 0; t < num_t; ++t) {
      GRBLinExpr lhs = 0;
      for (const auto& tr :
           instance.trains_at_t(static_cast<int>(t) * dt, tr_on_e)) {
        if (!instance.get_train_list().get_train(tr).tim) {
          lhs += vars["x"](tr, t, e);
        }
      }
      model->addConstr(lhs, GRB_LESS_EQUAL, 1,
                       "non_tim_train_on_edge_" + e_name + "_" +
                           std::to_string(static_cast<int>(t) * dt));
    }
  }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    create_non_discretized_fraction_constraints() {
  for (size_t i = 0; i < relevant_edges.size(); ++i) {
    const auto& e            = relevant_edges[i];
    const auto  vss_number_e = instance.n().max_vss_on_edge(e);
    const auto& edge         = instance.n().get_edge(e);
    const auto& edge_name    = "[" + instance.n().get_vertex(edge.source).name +
                            "," + instance.n().get_vertex(edge.target).name +
                            "]";
    const auto& breakable_e_index = breakable_edge_indices.at(e);
    const auto& e_len             = instance.n().get_edge(e).length;

    if (vss_model.get_model_type() == vss::ModelType::Inferred) {
      // sum edge_type(i,*) = 1
      GRBLinExpr lhs_sum_edge_type            = 0;
      bool       add_constraint_sum_edge_type = false;
      for (size_t sep_type_index = 0;
           sep_type_index < vss_model.get_separation_functions().size();
           ++sep_type_index) {
        lhs_sum_edge_type += vars["edge_type"](i, sep_type_index);
        add_constraint_sum_edge_type = true;
        const auto& sep_func =
            vss_model.get_separation_functions().at(sep_type_index);
        for (size_t vss = 0; vss < vss_number_e; ++vss) {
          // NOLINTBEGIN(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays)
          auto const xpts = std::make_unique<double[]>(vss_number_e + 1);
          auto const ypts = std::make_unique<double[]>(vss_number_e + 1);
          // NOLINTEND(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays)
          for (size_t x = 0; x < vss_number_e + 1; ++x) {
            xpts[x] = static_cast<double>(x) + 1;
            ypts[x] = sep_func(vss, x + 1);
          }
          model->addGenConstrPWL(
              vars["num_vss_segments"](i),
              vars["frac_vss_segments"](i, sep_type_index, vss),
              vss_number_e + 1, xpts.get(), ypts.get(),
              "frac_vss_segments_value_constraint_" + edge_name + "_" +
                  std::to_string(sep_type_index) + "_" + std::to_string(vss));
        }
      }
      if (add_constraint_sum_edge_type) {
        model->addConstr(lhs_sum_edge_type, GRB_EQUAL, 1,
                         "sum_edge_type_" + edge_name);
      }

      for (size_t vss = 0; vss < vss_number_e; ++vss) {
        // b_pos(breakable_e_index, vss) = e_len * sum_{separation_types}
        // frac_type(i, sep_type_index, vss)
        GRBLinExpr lhs = 0;
        for (size_t sep_type_index = 0;
             sep_type_index < vss_model.get_separation_functions().size();
             ++sep_type_index) {
          lhs += vars["frac_type"](i, sep_type_index, vss);

          // Make sure that frac_type(i, sep_type_index, vss) =
          // frac_vss_segments(i, sep_type_index, vss) * edge_type(i,
          // sep_type_index) by standard linearization
          const double lb = 0;
          const double ub = 1;
          // frac_type = 0 if edge_type = 0
          model->addConstr(
              lb * vars["edge_type"](i, sep_type_index), GRB_LESS_EQUAL,
              vars["frac_type"](i, sep_type_index, vss),
              "frac_type_0_lb_" + edge_name + "_" +
                  std::to_string(sep_type_index) + "_" + std::to_string(vss));
          model->addConstr(
              vars["frac_type"](i, sep_type_index, vss), GRB_LESS_EQUAL,
              ub * vars["edge_type"](i, sep_type_index),
              "frac_type_0_ub_" + edge_name + "_" +
                  std::to_string(sep_type_index) + "_" + std::to_string(vss));
          // frac_type = frac_vss_segments if edge_type = 1
          model->addConstr(
              (lb - ub) * (1 - vars["edge_type"](i, sep_type_index)),
              GRB_LESS_EQUAL,
              vars["frac_type"](i, sep_type_index, vss) -
                  vars["frac_vss_segments"](i, sep_type_index, vss),
              "frac_type_prod_lb_" + edge_name + "_" +
                  std::to_string(sep_type_index) + "_" + std::to_string(vss));
          model->addConstr(
              vars["frac_type"](i, sep_type_index, vss) -
                  vars["frac_vss_segments"](i, sep_type_index, vss),
              GRB_LESS_EQUAL,
              (ub - lb) * (1 - vars["edge_type"](i, sep_type_index)),
              "frac_type_prod_ub_" + edge_name + "_" +
                  std::to_string(sep_type_index) + "_" + std::to_string(vss));
        }
        lhs *= e_len;
        model->addConstr(lhs, GRB_EQUAL, vars["b_pos"](breakable_e_index, vss),
                         "b_pos_limited_" + edge_name + "_" +
                             std::to_string(vss));
      }
    }
  }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    create_non_discretized_alt_fraction_constraints() {
  if (vss_model.get_model_type() != vss::ModelType::InferredAlt) {
    return;
  }

  for (size_t i = 0; i < relevant_edges.size(); ++i) {
    const auto& e            = relevant_edges[i];
    const auto  vss_number_e = instance.n().max_vss_on_edge(e);
    const auto& edge         = instance.n().get_edge(e);
    const auto& edge_name    = "[" + instance.n().get_vertex(edge.source).name +
                            "," + instance.n().get_vertex(edge.target).name +
                            "]";
    const auto& breakable_e_index = breakable_edge_indices.at(e);
    const auto& e_len             = instance.n().get_edge(e).length;

    // Only choose one edge type and number per edge
    GRBLinExpr lhs_sum_edge_type = 0;
    for (size_t sep_type_index = 0;
         sep_type_index < vss_model.get_separation_functions().size();
         ++sep_type_index) {
      for (size_t vss = 0; vss < vss_number_e; ++vss) {
        lhs_sum_edge_type +=
            vars["type_num_vss_segments"](i, sep_type_index, vss);
      }
    }
    model->addConstr(lhs_sum_edge_type, GRB_LESS_EQUAL, 1,
                     "sum_edge_vss_type_" + edge_name);

    // Set b_pos accordingly
    for (size_t vss = 0; vss < vss_number_e; ++vss) {
      GRBLinExpr rhs = 0;
      for (size_t sep_type_index = 0;
           sep_type_index < vss_model.get_separation_functions().size();
           ++sep_type_index) {
        const auto& sep_func =
            vss_model.get_separation_functions().at(sep_type_index);
        for (size_t num_vss = 1; num_vss <= vss_number_e; ++num_vss) {
          rhs += vars["type_num_vss_segments"](i, sep_type_index, num_vss - 1) *
                 e_len * sep_func(vss, num_vss + 1);
        }
      }
      model->addConstr(vars["b_pos"](breakable_e_index, vss), GRB_EQUAL, rhs,
                       "b_pos_alt_limited_" + edge_name + "_" +
                           std::to_string(vss));
    }
  }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    create_brakelen_constraints() {
  /**
   * Creates the constraints related to braking distances.
   */
  // break_len(tr, t) = v(tr, t+1)^2 / (2*tr_deceleration)
  for (size_t tr = 0; tr < num_tr; ++tr) {
    const auto& tr_deceleration =
        instance.get_train_list().get_train(tr).deceleration;
    const auto& tr_max_speed =
        instance.get_train_list().get_train(tr).max_speed;
    if (this->use_pwl) {
      const int n = std::ceil(
          tr_max_speed / (2 * std::sqrt(2 * tr_deceleration * ABS_PWL_ERROR)));
      // NOLINTBEGIN(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays)
      auto const xpts = std::make_unique<double[]>(n + 1);
      auto const ypts = std::make_unique<double[]>(n + 1);
      // NOLINTEND(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays)
      for (size_t i = 0; i <= n; ++i) {
        xpts[i] = static_cast<double>(i) * tr_max_speed / n;
        ypts[i] = xpts[i] * xpts[i] / (2 * tr_deceleration);
      }
      for (size_t t = train_interval[tr].first; t <= train_interval[tr].second;
           ++t) {
        model->addGenConstrPWL(vars["v"](tr, t + 1), vars["brakelen"](tr, t),
                               n + 1, xpts.get(), ypts.get(),
                               "brakelen_" + std::to_string(tr) + "_" +
                                   std::to_string(t));
      }
    } else {
      for (size_t t = train_interval[tr].first; t <= train_interval[tr].second;
           ++t) {
        model->addQConstr(vars["brakelen"](tr, t), GRB_EQUAL,
                          (1 / (2 * tr_deceleration)) * vars["v"](tr, t + 1) *
                              vars["v"](tr, t + 1),
                          "brakelen_" + std::to_string(tr) + "_" +
                              std::to_string(t));
      }
    }
  }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    create_general_speed_constraints() {
  /**
   * Train does not exceed maximum speed on edges
   */

  for (size_t tr = 0; tr < num_tr; ++tr) {
    const auto& tr_speed = instance.get_train_list().get_train(tr).max_speed;
    for (const auto e : instance.edges_used_by_train(tr, this->fix_routes)) {
      const auto& max_speed = instance.n().get_edge(e).max_speed;
      if (max_speed < tr_speed) {
        for (size_t t = train_interval[tr].first;
             t <= train_interval[tr].second; ++t) {
          // v(tr,t+1) <= max_speed + (tr_speed - max_speed) * (1 - x(tr,t,e))
          model->addConstr(
              vars["v"](tr, t + 1), GRB_LESS_EQUAL,
              max_speed + (tr_speed - max_speed) * (1 - vars["x"](tr, t, e)),
              "v_max_speed_" + std::to_string(tr) + "_" +
                  std::to_string((t + 1) * dt) + "_" + std::to_string(e));
          // If brakelens are included the speed is reduced before entering an
          // edge, otherwise also include v(tr,t) <= max_speed + (tr_speed -
          // max_speed) * (1 - x(tr,t,e))
          if (!this->include_braking_curves) {
            model->addConstr(
                vars["v"](tr, t), GRB_LESS_EQUAL,
                max_speed + (tr_speed - max_speed) * (1 - vars["x"](tr, t, e)),
                "v_max_speed2_" + std::to_string(tr) + "_" +
                    std::to_string(t * dt) + "_" + std::to_string(e));
          }
        }
      }
    }
  }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    create_reverse_occupation_constraints() {
  /**
   * A breakable section can only be occupied in one direction at a time. This
   * prevents trains from blocking each other, since reversing trains is not
   * modelled.
   */

  // Connect y_sec and x
  for (size_t t = 0; t < num_t; ++t) {
    const auto tr_at_t = instance.trains_at_t(static_cast<int>(t) * dt);
    for (size_t i = 0; i < fwd_bwd_sections.size(); ++i) {
      // y_sec_fwd(t,i) >= x(tr, t, e) for all e in fwd_bwd_sections[i].first
      // and applicable trains y_sec_fwd(t,i) <= sum x(tr, t, e)
      GRBLinExpr rhs = 0;
      for (const auto& e : fwd_bwd_sections[i].first) {
        const auto tr_on_edge =
            instance.trains_on_edge(e, this->fix_routes, tr_at_t);
        for (const auto& tr : tr_on_edge) {
          rhs += vars["x"](tr, t, e);
          model->addConstr(vars["y_sec_fwd"](t, i), GRB_GREATER_EQUAL,
                           vars["x"](tr, t, e),
                           "y_sec_fwd_linker_1_" + std::to_string(t) + "_" +
                               std::to_string(i) + "_" + std::to_string(tr) +
                               "_" + std::to_string(e));
        }
      }
      model->addConstr(vars["y_sec_fwd"](t, i), GRB_LESS_EQUAL, rhs,
                       "y_sec_fwd_linker_2_" + std::to_string(t) + "_" +
                           std::to_string(i));

      // y_sec_bwd(t,i) >= x(tr, t, e) for all e in fwd_bwd_sections[i].second
      // and applicable trains y_sec_bwd(t,i) <= sum x(tr, t, e)
      rhs = 0;
      for (const auto& e : fwd_bwd_sections[i].second) {
        const auto tr_on_edge =
            instance.trains_on_edge(e, this->fix_routes, tr_at_t);
        for (const auto& tr : tr_on_edge) {
          rhs += vars["x"](tr, t, e);
          model->addConstr(vars["y_sec_bwd"](t, i), GRB_GREATER_EQUAL,
                           vars["x"](tr, t, e),
                           "y_sec_bwd_linker_1_" + std::to_string(t) + "_" +
                               std::to_string(i) + "_" + std::to_string(tr) +
                               "_" + std::to_string(e));
        }
      }
      model->addConstr(vars["y_sec_bwd"](t, i), GRB_LESS_EQUAL, rhs,
                       "y_sec_bwd_linker_2_" + std::to_string(t) + "_" +
                           std::to_string(i));
    }
  }

  // Only one direction occupied
  for (size_t t = 0; t < num_t; ++t) {
    for (size_t i = 0; i < fwd_bwd_sections.size(); ++i) {
      // y_sec_fwd(t,i) + y_sec_bwd(t, i) <= 1
      model->addConstr(
          vars["y_sec_fwd"](t, i) + vars["y_sec_bwd"](t, i), GRB_LESS_EQUAL, 1,
          "y_sec_fwd_bwd_" + std::to_string(t) + "_" + std::to_string(i));
    }
  }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    calculate_fwd_bwd_sections() {
  /**
   * Calculate the forward and backward sections for each breakable section
   */

  if (this->vss_model.get_model_type() == vss::ModelType::Discrete) {
    calculate_fwd_bwd_sections_discretized();
  } else {
    calculate_fwd_bwd_sections_non_discretized();
  }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    calculate_fwd_bwd_sections_discretized() {
  /**
   * For every section, cluster forward and backward edges.
   */
  for (const auto& vss_section : no_border_vss_sections) {
    const auto vss_section_sorted =
        instance.n().combine_reverse_edges(vss_section, true);
    bool fwd_found = false;
    bool bwd_found = false;
    for (size_t i = 0;
         i < vss_section_sorted.size() && !fwd_found && !bwd_found; ++i) {
      if (vss_section_sorted[i].first.has_value()) {
        fwd_found = true;
      }
      if (vss_section_sorted[i].second.has_value()) {
        bwd_found = true;
      }
    }
    if (!fwd_found || !bwd_found) {
      continue;
    }
    fwd_bwd_sections.emplace_back();
    for (const auto& e : vss_section_sorted) {
      if (e.first.has_value()) {
        fwd_bwd_sections.back().first.emplace_back(e.first.value());
      }
      if (e.second.has_value()) {
        fwd_bwd_sections.back().second.emplace_back(e.second.value());
      }
    }
  }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    calculate_fwd_bwd_sections_non_discretized() {
  /**
   * For every section, cluster forward and backward edges.
   */
  for (const auto& edge_pair : breakable_edges_pairs) {
    if (!edge_pair.first.has_value() || !edge_pair.second.has_value()) {
      continue;
    }
    fwd_bwd_sections.emplace_back();
    fwd_bwd_sections.back().first.emplace_back(edge_pair.first.value());
    fwd_bwd_sections.back().second.emplace_back(edge_pair.second.value());
  }
}

double cda_rail::solver::mip_based::VSSGenTimetableSolver::get_max_brakelen(
    const size_t& tr) const {
  /**
   * Returns the maximum braking distance of a train.
   */
  const auto& tr_deceleration =
      instance.get_train_list().get_train(tr).deceleration;
  const auto& tr_max_speed = instance.get_train_list().get_train(tr).max_speed;
  return tr_max_speed * tr_max_speed / (2 * tr_deceleration);
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    create_general_boundary_constraints() {
  /**
   * General boundary conditions, i.e., speed
   */
  auto train_list = instance.get_train_list();
  for (size_t i = 0; i < num_tr; ++i) {
    auto tr_name       = train_list.get_train(i).name;
    auto initial_speed = instance.get_schedule(tr_name).get_v_0();
    auto final_speed   = instance.get_schedule(tr_name).get_v_n();
    // initial_speed: v(train_interval[i].first) = initial_speed
    model->addConstr(vars["v"](i, train_interval[i].first) == initial_speed,
                     "initial_speed_" + tr_name);
    // final_speed: v(train_interval[i].second) = final_speed
    model->addConstr(vars["v"](i, train_interval[i].second + 1) == final_speed,
                     "final_speed_" + tr_name);
  }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    create_only_stop_at_vss_variables() {
  vars["stopped"] = MultiArray<GRBVar>(num_tr, num_t);

  for (size_t tr = 0; tr < num_tr; ++tr) {
    const auto& tr_name = instance.get_train_list().get_train(tr).name;
    for (size_t t = train_interval[tr].first; t <= train_interval[tr].second;
         ++t) {
      vars["stopped"](tr, t) =
          model->addVar(0, 1, 0, GRB_BINARY,
                        "stopped_" + tr_name + "_" + std::to_string(t * dt));
    }
  }

  if (vss_model.get_model_type() != vss::ModelType::Discrete) {
    create_non_discretized_only_stop_at_vss_variables();
  } else {
    throw exceptions::ConsistencyException(
        "Only stop at VSS variables are not supported for discretized VSS "
        "models");
  }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::
    create_non_discretized_general_only_stop_at_vss_constraints() {
  // At most one b_tight can be true per train and time
  for (size_t tr = 0; tr < num_tr; ++tr) {
    const auto& tr_name = instance.get_train_list().get_train(tr).name;
    for (size_t t = train_interval[tr].first + 2;
         t <= train_interval[tr].second; ++t) {
      GRBLinExpr lhs = 0;
      for (const auto& e : instance.edges_used_by_train(tr, this->fix_routes)) {
        if (!instance.const_n().get_edge(e).breakable) {
          continue;
        }
        const auto& vss_e     = instance.const_n().max_vss_on_edge(e);
        const auto& e_b_index = breakable_edge_indices.at(e);
        for (size_t vss = 0; vss < vss_e; ++vss) {
          lhs += vars["b_tight"](tr, t, e_b_index, vss);
        }
      }
      model->addConstr(lhs, GRB_LESS_EQUAL, 1,
                       "b_tight_max_one_" + tr_name + "_" +
                           std::to_string(t * dt));
    }
  }

  // On every breakable edge at most one b_tight or e_tight can be one per train
  // and time
  for (size_t i = 0; i < breakable_edges.size(); ++i) {
    const auto& e     = breakable_edges[i];
    const auto& vss_e = instance.const_n().max_vss_on_edge(e);
    const auto& edge  = instance.const_n().get_edge(e);
    const auto& edge_name =
        "[" + instance.const_n().get_vertex(edge.source).name + "," +
        instance.const_n().get_vertex(edge.target).name + "]";
    for (size_t tr : instance.trains_on_edge(e, this->fix_routes)) {
      const auto& tr_name = instance.get_train_list().get_train(tr).name;
      for (size_t t = train_interval[tr].first + 2;
           t <= train_interval[tr].second; ++t) {
        GRBLinExpr lhs = vars["e_tight"](tr, t, e);
        for (size_t vss = 0; vss < vss_e; ++vss) {
          lhs += vars["b_tight"](tr, t, i, vss);
        }
        model->addConstr(lhs, GRB_LESS_EQUAL, 1,
                         "b_tight_e_tight_max_one_" + tr_name + "_" +
                             std::to_string(t * dt) + "_" + edge_name);
      }
    }
  }

  // On every edge at least one b_tight or e_tight must be one if train is
  // present and speed is 0 per train, time, and edge
  for (size_t e = 0; e < num_edges; ++e) {
    const auto& edge = instance.const_n().get_edge(e);
    const auto& edge_name =
        "[" + instance.const_n().get_vertex(edge.source).name + "," +
        instance.const_n().get_vertex(edge.target).name + "]";
    std::optional<size_t> breakable_e_index;
    std::optional<size_t> vss_e;
    if (edge.breakable) {
      breakable_e_index = breakable_edge_indices.at(e);
      vss_e             = instance.const_n().max_vss_on_edge(e);
    }

    for (size_t tr : instance.trains_on_edge(e, this->fix_routes)) {
      const auto& tr_name = instance.get_train_list().get_train(tr).name;
      for (size_t t = train_interval[tr].first + 2;
           t <= train_interval[tr].second; ++t) {
        GRBLinExpr lhs = vars["e_tight"](tr, t, e);
        if (breakable_e_index.has_value()) {
          for (size_t vss = 0; vss < vss_e.value(); ++vss) {
            lhs += vars["b_tight"](tr, t, breakable_e_index.value(), vss);
          }
        }
        model->addConstr(lhs, GRB_GREATER_EQUAL,
                         vars["x"](tr, t - 1, e) - vars["stopped"](tr, t),
                         "b_tight_e_tight_min_one_" + tr_name + "_" +
                             std::to_string(t * dt) + "_" + edge_name);
      }
    }
  }

  // On every edge that is not breakable and does not end with a border at least
  // one out edge has to be used if it is used and v = 0
  for (size_t tr = 0; tr < num_tr; ++tr) {
    const auto& tr_name = instance.get_train_list().get_train(tr).name;
    const auto& edge_used_tr =
        instance.edges_used_by_train(tr, this->fix_routes);
    for (size_t e : edge_used_tr) {
      const auto& edge = instance.const_n().get_edge(e);
      const auto& edge_name =
          "[" + instance.const_n().get_vertex(edge.source).name + "," +
          instance.const_n().get_vertex(edge.target).name + "]";
      if (edge.breakable || instance.const_n().get_vertex(edge.target).type !=
                                VertexType::NoBorder) {
        continue;
      }

      const auto&         delta_out = instance.const_n().get_successors(e);
      std::vector<size_t> delta_out_tr;
      for (const auto& e_out : delta_out) {
        if (std::find(edge_used_tr.begin(), edge_used_tr.end(), e_out) !=
            edge_used_tr.end()) {
          delta_out_tr.emplace_back(e_out);
        }
      }

      for (size_t t = train_interval[tr].first + 2;
           t <= train_interval[tr].second; ++t) {
        GRBLinExpr lhs = 0;
        for (const auto& e_out : delta_out_tr) {
          lhs += vars["x"](tr, t - 1, e_out);
        }
        model->addConstr(lhs, GRB_GREATER_EQUAL,
                         vars["x"](tr, t - 1, e) - vars["stopped"](tr, t),
                         "no_stop_on_non-border_edge_ending_" + tr_name + "_" +
                             std::to_string(t * dt) + "_" + edge_name);
      }
    }
  }

  // b cannot be tight if it is not front. If v = 0 then it has to be
  for (size_t i = 0; i < breakable_edges.size(); ++i) {
    const auto& e    = breakable_edges[i];
    const auto& edge = instance.const_n().get_edge(e);
    const auto& edge_name =
        "[" + instance.const_n().get_vertex(edge.source).name + "," +
        instance.const_n().get_vertex(edge.target).name + "]";
    const auto& vss_e = instance.const_n().max_vss_on_edge(e);
    for (size_t tr : instance.trains_on_edge(e, this->fix_routes)) {
      const auto& tr_name = instance.get_train_list().get_train(tr).name;
      for (size_t t = train_interval[tr].first + 2;
           t <= train_interval[tr].second; ++t) {
        for (size_t vss = 0; vss < vss_e; ++vss) {
          model->addConstr(vars["b_tight"](tr, t, i, vss), GRB_LESS_EQUAL,
                           vars["b_front"](tr, t, i, vss),
                           "b_tight_not_front_1_" + tr_name + "_" +
                               std::to_string(t * dt) + "_" + edge_name + "_" +
                               std::to_string(vss));
          model->addConstr(
              vars["b_tight"](tr, t, i, vss), GRB_GREATER_EQUAL,
              vars["b_front"](tr, t, i, vss) - vars["stopped"](tr, t),
              "b_tight_not_front_2_" + tr_name + "_" + std::to_string(t * dt) +
                  "_" + edge_name + "_" + std::to_string(vss));
        }
      }
    }
  }

  // At least any one tight if speed is 0
  for (size_t tr = 0; tr < num_tr; ++tr) {
    const auto& tr_name = instance.get_train_list().get_train(tr).name;
    const auto& edge_used_tr =
        instance.edges_used_by_train(tr, this->fix_routes);
    for (size_t t = train_interval[tr].first + 2;
         t <= train_interval[tr].second; ++t) {
      GRBLinExpr lhs = 0;
      for (size_t e : edge_used_tr) {
        lhs += vars["e_tight"](tr, t, e);
        const auto& edge = instance.const_n().get_edge(e);
        if (!edge.breakable) {
          continue;
        }
        const auto& vss_e = instance.const_n().max_vss_on_edge(e);
        for (size_t vss = 0; vss < vss_e; ++vss) {
          lhs += vars["b_tight"](tr, t, breakable_edge_indices.at(e), vss);
        }
      }
      model->addConstr(lhs, GRB_GREATER_EQUAL, 1 - vars["stopped"](tr, t),
                       "at_least_one_tight_if_stopped_" + tr_name + "_" +
                           std::to_string(t * dt));
    }
  }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::create_variables() {
  PLOGD << "Create general variables";
  create_general_variables();
  if (this->fix_routes) {
    PLOGD << "Create fixed routes variables";
    create_fixed_routes_variables();
  } else {
    PLOGD << "Create free routes variables";
    create_free_routes_variables();
  }
  if (this->vss_model.get_model_type() == vss::ModelType::Discrete) {
    PLOGD << "Create discretized VSS variables";
    create_discretized_variables();
  } else {
    PLOGD << "Create non-discretized VSS variables";
    create_non_discretized_variables();
  }
  if (this->include_braking_curves) {
    PLOGD << "Create braking distance variables";
    create_brakelen_variables();
  }
  if (vss_model.get_only_stop_at_vss()) {
    PLOGD << "Create only stop at VSS variables";
    create_only_stop_at_vss_variables();
  }
}

void cda_rail::solver::mip_based::VSSGenTimetableSolver::create_constraints() {
  PLOGD << "Create general constraints";
  create_general_constraints();
  if (this->fix_routes) {
    PLOGD << "Create fixed routes constraints";
    create_fixed_routes_constraints();
  } else {
    PLOGD << "Create free routes constraints";
    create_free_routes_constraints();
  }
  if (this->vss_model.get_model_type() == vss::ModelType::Discrete) {
    PLOGD << "Create discretized VSS constraints";
    create_discretized_constraints();
  } else {
    PLOGD << "Create non-discretized VSS constraints";
    create_non_discretized_constraints();
  }
  if (this->include_train_dynamics) {
    PLOGD << "Create train dynamic constraints";
    create_acceleration_constraints();
  }
  if (this->include_braking_curves) {
    PLOGD << "Create braking distance constraints";
    create_brakelen_constraints();
  }
}

// NOLINTEND(performance-inefficient-string-concatenation)

// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay)
