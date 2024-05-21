#include "solver/mip-based/GenPOMovingBlockMIPSolver.hpp"

#include "EOMHelper.hpp"
#include "MultiArray.hpp"
#include "gurobi_c++.h"
#include "solver/mip-based/GeneralMIPSolver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

// NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay,performance-inefficient-string-concatenation)

cda_rail::instances::SolGeneralPerformanceOptimizationInstance
cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::solve(
    const ModelDetail&                 model_detail_input,
    const SolverStrategyMovingBlock&   solver_strategy_input,
    const SolutionSettingsMovingBlock& solution_settings_input, int time_limit,
    bool debug_input) {
  /**
   * Solves initiated performance optimization problem with moving block
   * signaling/routing. Only breakable edges can use moving block. On all
   * others, only one train is allowed (in practice Flankenschutz can be
   * included this way). Trains are only routed if no route is specified.
   *
   * @param time_limit: time limit for the solver in seconds. If -1, no time
   * limit is set.
   * @param debug_input: if true, the debug output is enabled.
   *
   * @return: respective solution object
   */

  std::optional<LazyCallback> cb;
  if (solver_strategy_input.use_lazy_constraints) {
    cb = LazyCallback(this);
    this->solve_init_general_mip(time_limit, debug_input, &(cb.value()));
  } else {
    this->solve_init_general_mip(time_limit, debug_input);
  }

  if (!instance.n().is_consistent_for_transformation()) {
    PLOGE << "Instance is not consistent for transformation.";
    throw exceptions::ConsistencyException();
  }

  PLOGI << "Create model";

  const instances::GeneralPerformanceOptimizationInstance old_instance =
      instance;
  this->instance.discretize_stops();

  this->initialize_variables(solution_settings_input, solver_strategy_input,
                             model_detail_input);

  PLOGD << "Create variables";
  create_variables();
  PLOGD << "Set objective";
  set_objective();
  PLOGD << "Create constraints";
  create_constraints();

  model->update();

  PLOGD << "Fix numerical issues with small coefficients";
  // TODO: Can we prevent this from being necessary by rounding at the source.
  // On the other hand, this does not take long to fix.
  // NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  size_t       num_fixed   = 0;
  const double integer_tol = model->getEnv().get(GRB_DoubleParam_IntFeasTol);
  auto*        vars_tmp    = model->getVars();
  const int    num_vars    = model->get(GRB_IntAttr_NumVars);
  for (size_t i = 0; i < num_vars; i++) {
    auto col_v = model->getCol(vars_tmp[i]);
    for (size_t j = 0; j < col_v.size(); j++) {
      if (std::abs(col_v.getCoeff(j)) < integer_tol && col_v.getCoeff(j) != 0) {
        auto c = col_v.getConstr(j);
        model->chgCoeff(c, vars_tmp[i], 0);
        num_fixed++;
      }
    }
  }
  // NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  PLOGD << "Fixed " << num_fixed << " coefficients";

  PLOGI << "Model created. Optimize.";
  if (plog::get()->checkSeverity(plog::debug) || time_limit > 0) {
    model_created = std::chrono::high_resolution_clock::now();
    create_time   = std::chrono::duration_cast<std::chrono::milliseconds>(
                      model_created - start)
                      .count();

    auto time_left = time_limit - create_time / 1000;
    if (time_left < 0 && time_limit > 0) {
      time_left = 1;
    }
    if (time_limit > 0) {
      model->set(GRB_DoubleParam_TimeLimit, static_cast<double>(time_left));
    }
    PLOGD << "Model created in " << (static_cast<double>(create_time) / 1000.0)
          << " s";
    if (time_limit > 0) {
      PLOGD << "Time left: " << time_left << " s";
    } else {
      PLOGD << "Time left: "
            << "No Limit";
    }
  }

  if (solver_strategy.use_lazy_constraints) {
    model->set(GRB_IntParam_LazyConstraints, 1);
  }

  model->optimize();

  IF_PLOG(plog::debug) {
    model_solved = std::chrono::high_resolution_clock::now();
    solve_time   = std::chrono::duration_cast<std::chrono::milliseconds>(
                     model_solved - model_created)
                     .count();
    PLOGD << "Model created in " << (static_cast<double>(create_time) / 1000.0)
          << " s";
    PLOGD << "Model solved in " << (static_cast<double>(solve_time) / 1000.0)
          << " s";
    PLOGD << "Total time "
          << (static_cast<double>(create_time + solve_time) / 1000.0) << " s";
  }

  instances::SolGeneralPerformanceOptimizationInstance solution(old_instance);
  extract_solution(solution);

  if (solution_settings.export_option == ExportOption::ExportLP ||
      solution_settings.export_option == ExportOption::ExportSolutionAndLP ||
      solution_settings.export_option ==
          ExportOption::ExportSolutionWithInstanceAndLP) {
    PLOGD << "Add " << lazy_constraints.size() << " lazy constraints";
    for (size_t i = 0; i < lazy_constraints.size(); i++) {
      model->addConstr(lazy_constraints.at(i), "Lazy" + std::to_string(i));
    }
    model->update();

    PLOGI << "Saving model and solution";
    std::filesystem::path path = solution_settings.path;

    if (!is_directory_and_create(path)) {
      PLOGE << "Could not create directory " << path.string();
      throw exceptions::ExportException("Could not create directory " +
                                        path.string());
    }

    model->write((path / (solution_settings.name + ".mps")).string());
    if (model->get(GRB_IntAttr_SolCount) > 0) {
      model->write((path / (solution_settings.name + ".sol")).string());
    }
  }

  if (solution_settings.export_option == ExportOption::ExportSolution ||
      solution_settings.export_option ==
          ExportOption::ExportSolutionWithInstance ||
      solution_settings.export_option == ExportOption::ExportSolutionAndLP ||
      solution_settings.export_option ==
          ExportOption::ExportSolutionWithInstanceAndLP) {
    const bool export_instance =
        (solution_settings.export_option ==
             ExportOption::ExportSolutionWithInstance ||
         solution_settings.export_option ==
             ExportOption::ExportSolutionWithInstanceAndLP);
    PLOGI << "Saving solution";
    std::filesystem::path path = solution_settings.path;
    path /= solution_settings.name;
    solution.export_solution(path, export_instance);
  }

  this->instance = old_instance;

  return solution;
}

void cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::
    create_variables() {
  create_timing_variables();
  create_general_edge_variables();
  create_velocity_extended_variables();
  create_stop_variables();
  create_reverse_edge_variables();
}

void cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::
    create_timing_variables() {
  vars["t_front_arrival"]   = MultiArray<GRBVar>(num_tr, num_vertices);
  vars["t_front_departure"] = MultiArray<GRBVar>(num_tr, num_vertices);
  vars["t_rear_departure"]  = MultiArray<GRBVar>(num_tr, num_vertices);
  vars["t_ttd_departure"]   = MultiArray<GRBVar>(num_tr, num_ttd);

  for (size_t tr = 0; tr < num_tr; tr++) {
    const double ub_timing_dept = ub_timing_variable(tr);
    const auto&  tr_name        = instance.get_train_list().get_train(tr).name;
    for (const auto v :
         instance.vertices_used_by_train(tr, model_detail.fix_routes, false)) {
      const auto& v_name = instance.const_n().get_vertex(v).name;
      vars["t_front_arrival"](tr, v) =
          model->addVar(0.0, ub_timing_dept, 0.0, GRB_CONTINUOUS,
                        "t_front_arrival_" + tr_name + "_" + v_name);
      vars["t_front_departure"](tr, v) =
          model->addVar(0.0, ub_timing_dept, 0.0, GRB_CONTINUOUS,
                        "t_front_departure_" + tr_name + "_" + v_name);
      vars["t_rear_departure"](tr, v) =
          model->addVar(0.0, ub_timing_dept, 0.0, GRB_CONTINUOUS,
                        "t_rear_departure_" + tr_name + "_" + v_name);
    }
    for (const auto& ttd : instance.sections_used_by_train(
             tr, ttd_sections, model_detail.fix_routes, false)) {
      vars["t_ttd_departure"](tr, ttd) = model->addVar(
          0.0, ub_timing_dept, 0.0, GRB_CONTINUOUS,
          "t_ttd_departure_" + tr_name + "_" + std::to_string(ttd));
    }
  }
}

void cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::
    create_general_edge_variables() {
  vars["x"]         = MultiArray<GRBVar>(num_tr, num_edges);
  vars["order"]     = MultiArray<GRBVar>(num_tr, num_tr, num_edges);
  vars["x_ttd"]     = MultiArray<GRBVar>(num_tr, num_ttd);
  vars["order_ttd"] = MultiArray<GRBVar>(num_tr, num_tr, num_ttd);

  for (size_t tr = 0; tr < num_tr; tr++) {
    const auto& tr_name = instance.get_train_list().get_train(tr).name;
    for (const auto e :
         instance.edges_used_by_train(tr, model_detail.fix_routes, false)) {
      vars["x"](tr, e) = model->addVar(0.0, 1.0, 0.0, GRB_BINARY,
                                       "x_" + tr_name + "_" +
                                           instance.const_n().get_edge_name(e));
    }
    for (const auto& ttd : instance.sections_used_by_train(
             tr, ttd_sections, model_detail.fix_routes, false)) {
      vars["x_ttd"](tr, ttd) =
          model->addVar(0.0, 1.0, 0.0, GRB_BINARY,
                        "x_ttd_" + tr_name + "_" + std::to_string(ttd));
    }
  }
  for (size_t e = 0; e < num_edges; e++) {
    const auto tr_on_e = instance.trains_on_edge_mixed_routing(
        e, model_detail.fix_routes, false);
    const auto& e_name = instance.const_n().get_edge_name(e);
    for (const auto& tr1 : tr_on_e) {
      const auto& tr1_name = instance.get_train_list().get_train(tr1).name;
      for (const auto& tr2 : tr_on_e) {
        if (tr1 != tr2) {
          const auto& tr2_name = instance.get_train_list().get_train(tr2).name;
          vars["order"](tr1, tr2, e) = model->addVar(
              0.0, 1.0, 0.0, GRB_BINARY,
              "order_" + tr1_name + "_" + tr2_name + "_" + e_name);
        }
      }
    }
  }
  for (size_t ttd = 0; ttd < num_ttd; ttd++) {
    const auto tr_on_ttd = instance.trains_in_section(
        ttd_sections.at(ttd), model_detail.fix_routes, false);
    for (const auto& tr1 : tr_on_ttd) {
      const auto& tr1_name = instance.get_train_list().get_train(tr1).name;
      for (const auto& tr2 : tr_on_ttd) {
        if (tr1 != tr2) {
          const auto& tr2_name = instance.get_train_list().get_train(tr2).name;
          vars["order_ttd"](tr1, tr2, ttd) =
              model->addVar(0.0, 1.0, 0.0, GRB_BINARY,
                            "order_ttd_" + tr1_name + "_" + tr2_name + "_" +
                                std::to_string(ttd));
        }
      }
    }
  }
}

void cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::
    create_stop_variables() {
  size_t max_num_stops = 0;
  for (size_t tr = 0; tr < num_tr; tr++) {
    max_num_stops =
        std::max(max_num_stops, instance.get_schedule(tr).get_stops().size());
  }
  vars["stop"] = MultiArray<GRBVar>(num_tr, max_num_stops, num_vertices);

  for (size_t tr = 0; tr < num_tr; tr++) {
    const auto& tr_name = instance.get_train_list().get_train(tr).name;
    for (size_t stop = 0; stop < instance.get_schedule(tr).get_stops().size();
         stop++) {
      const auto& stop_name =
          instance.get_schedule(tr).get_stops().at(stop).get_station_name();
      const auto& stop_data = tr_stop_data.at(tr).at(stop);
      for (const auto& [v, edges] : stop_data) {
        vars["stop"](tr, stop, v) =
            model->addVar(0.0, 1.0, 0.0, GRB_BINARY,
                          "stop_" + tr_name + "_" + stop_name + "_" +
                              instance.const_n().get_vertex(v).name);
      }
    }
  }
}

void cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::
    create_velocity_extended_variables() {
  const auto max_velocity_extension_size =
      get_maximal_velocity_extension_size();
  vars["y"] = MultiArray<GRBVar>(num_tr, num_edges, max_velocity_extension_size,
                                 max_velocity_extension_size);

  for (size_t tr = 0; tr < num_tr; tr++) {
    const auto& train = instance.get_train_list().get_train(tr);
    for (const auto e :
         instance.edges_used_by_train(tr, model_detail.fix_routes, false)) {
      const auto& edge = instance.const_n().get_edge(e);
      const auto& edge_name =
          instance.const_n().get_edge_name(edge.source, edge.target);
      const auto& v_1           = velocity_extensions.at(tr).at(edge.source);
      const auto& v_2           = velocity_extensions.at(tr).at(edge.target);
      const auto  tmp_max_speed = std::min(train.max_speed, edge.max_speed);
      for (size_t i = 0; i < v_1.size(); i++) {
        if (v_1.at(i) > tmp_max_speed) {
          continue;
        }
        for (size_t j = 0; j < v_2.size(); j++) {
          if (v_2.at(j) > tmp_max_speed) {
            continue;
          }
          if (cda_rail::possible_by_eom(v_1.at(i), v_2.at(j),
                                        train.acceleration, train.deceleration,
                                        edge.length)) {
            vars["y"](tr, e, i, j) =
                model->addVar(0.0, 1.0, 0.0, GRB_BINARY,
                              "y_" + train.name + "_" + edge_name + "_" +
                                  std::to_string(v_1.at(i)) + "_" +
                                  std::to_string(v_2.at(j)));
          }
        }
      }
    }
  }
}

void cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::
    create_reverse_edge_variables() {
  /**
   * In order to prevent collisions of trains traveling in opposite directions,
   * we need additional variables.
   */
  vars["reverse_order"] =
      MultiArray<GRBVar>(num_tr, num_tr, relevant_reverse_edges.size());

  for (size_t idx = 0; idx < relevant_reverse_edges.size(); idx++) {
    const auto& [e1, e2] = relevant_reverse_edges.at(idx);
    const auto tr_list =
        instance.trains_in_section({e1, e2}, model_detail.fix_routes, false);
    const auto  e_obj   = instance.const_n().get_edge(e1);
    const auto& v1_name = instance.const_n().get_vertex(e_obj.source).name;
    const auto& v2_name = instance.const_n().get_vertex(e_obj.target).name;
    for (size_t idx_tr1 = 0; idx_tr1 < tr_list.size(); idx_tr1++) {
      const auto  tr1      = tr_list.at(idx_tr1);
      const auto& tr1_name = instance.get_train_list().get_train(tr1).name;
      for (size_t idx_tr2 = idx_tr1 + 1; idx_tr2 < tr_list.size(); idx_tr2++) {
        const auto  tr2      = tr_list.at(idx_tr2);
        const auto& tr2_name = instance.get_train_list().get_train(tr2).name;
        vars["reverse_order"](tr1, tr2, idx) =
            model->addVar(0.0, 1.0, 0.0, GRB_BINARY,
                          "reverse_order_" + tr1_name + "_" + tr2_name + "_" +
                              v1_name + "-" + v2_name);
        vars["reverse_order"](tr2, tr1, idx) =
            model->addVar(0.0, 1.0, 0.0, GRB_BINARY,
                          "reverse_order_" + tr2_name + "_" + tr1_name + "_" +
                              v1_name + "-" + v2_name);
      }
    }
  }
}

void cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::set_objective() {
  GRBLinExpr obj_expr = 0;
  for (size_t tr = 0; tr < num_tr; tr++) {
    const auto  exit_node     = instance.get_schedule(tr).get_exit();
    const auto& min_exit_time = instance.get_schedule(tr).get_t_n_range().first;
    const auto& tr_weight     = instance.get_train_weight(tr);

    obj_expr +=
        tr_weight * (vars["t_rear_departure"](tr, exit_node) - min_exit_time);
  }
  model->setObjective(obj_expr, GRB_MINIMIZE);
}

void cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::
    create_constraints() {
  PLOGD << "Create general path constraints";
  create_general_path_constraints();
  PLOGD << "Create travel times constraints";
  create_travel_times_constraints();
  PLOGD << "Create basic order constraints";
  create_basic_order_constraints();
  PLOGD << "Create basic TTD constraints";
  create_basic_ttd_constraints();
  PLOGD << "Create train rear constraints";
  create_train_rear_constraints();
  PLOGD << "Create basic reverse edge constraints";
  create_reverse_edge_constraints();
  PLOGD << "Create stopping constraints";
  create_stopping_constraints();
  if (!solver_strategy.use_lazy_constraints) {
    PLOGD << "Create vertex headway constraints";
    create_vertex_headway_constraints();
    PLOGD << "Create headway constraints";
    create_headway_constraints();
  }
}

void cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::
    fill_tr_stop_data() {
  tr_stop_data.clear();
  tr_stop_data.reserve(num_tr);

  for (size_t tr = 0; tr < num_tr; tr++) {
    std::vector<
        std::vector<std::pair<size_t, std::vector<std::vector<size_t>>>>>
        tr_data;
    tr_data.reserve(instance.get_schedule(tr).get_stops().size());
    for (const auto& stop : instance.get_schedule(tr).get_stops()) {
      tr_data.emplace_back(instance.possible_stop_vertices(
          tr, stop.get_station_name(),
          instance.edges_used_by_train(tr, model_detail.fix_routes, false)));
    }
    tr_stop_data.emplace_back(tr_data);
  }
}

void cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::
    fill_velocity_extensions() {
  velocity_extensions.clear();
  if (model_detail.velocity_refinement_strategy ==
      VelocityRefinementStrategy::None) {
    fill_velocity_extensions_using_none_strategy();
  } else if (model_detail.velocity_refinement_strategy ==
             VelocityRefinementStrategy::MinOneStep) {
    fill_velocity_extensions_using_min_one_step_strategy();
  } else {
    throw exceptions::InvalidInputException(
        "Velocity refinement strategy not implemented.");
  }
}

void cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::
    fill_velocity_extensions_using_none_strategy() {
  velocity_extensions.reserve(num_tr);
  for (size_t tr = 0; tr < num_tr; tr++) {
    std::vector<std::vector<double>> tr_velocity_extensions;
    tr_velocity_extensions.reserve(num_vertices);
    const auto& tr_max_speed =
        instance.get_train_list().get_train(tr).max_speed;
    for (size_t v = 0; v < num_vertices; v++) {
      if (instance.get_schedule(tr).get_entry() == v) {
        tr_velocity_extensions.emplace_back(
            std::vector<double>{instance.get_schedule(tr).get_v_0()});
        continue;
      }

      std::vector<double> v_velocity_extensions = {0};
      const double        max_vertex_speed      = std::min(
          instance.const_n().maximal_vertex_speed(
              v,
              instance.edges_used_by_train(tr, model_detail.fix_routes, false)),
          tr_max_speed);
      double speed = 0;
      while (speed < max_vertex_speed) {
        speed += model_detail.max_velocity_delta;
        if (speed > max_vertex_speed) {
          speed = max_vertex_speed;
        }
        v_velocity_extensions.emplace_back(speed);
      }
      tr_velocity_extensions.emplace_back(v_velocity_extensions);
    }
    velocity_extensions.emplace_back(tr_velocity_extensions);
  }
}

void cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::
    fill_velocity_extensions_using_min_one_step_strategy() {
  velocity_extensions.reserve(num_tr);
  for (size_t tr = 0; tr < num_tr; tr++) {
    std::vector<std::vector<double>> tr_velocity_extensions;
    tr_velocity_extensions.reserve(num_vertices);
    const auto& tr_object = instance.get_train_list().get_train(tr);
    const auto  tr_speed_change =
        std::min(tr_object.acceleration, tr_object.deceleration);
    const auto& tr_max_speed = tr_object.max_speed;
    const auto& tr_length    = tr_object.length;
    for (size_t v = 0; v < num_vertices; v++) {
      if (instance.get_schedule(tr).get_entry() == v) {
        tr_velocity_extensions.emplace_back(
            std::vector<double>{instance.get_schedule(tr).get_v_0()});
        continue;
      }

      const double max_vertex_speed = std::min(
          instance.const_n().maximal_vertex_speed(
              v,
              instance.edges_used_by_train(tr, model_detail.fix_routes, false)),
          tr_max_speed);
      double min_n_length =
          instance.const_n().minimal_neighboring_edge_length(v);

      if (min_n_length > tr_length &&
          instance.get_schedule(tr).get_exit() == v) {
        min_n_length = tr_length;
      }

      std::vector<double> v_velocity_extensions = {0};
      double              speed                 = 0;
      while (speed < max_vertex_speed) {
        speed = std::min(
            {speed + model_detail.max_velocity_delta,
             std::sqrt(speed * speed + 2 * tr_speed_change * min_n_length),
             max_vertex_speed});
        v_velocity_extensions.emplace_back(speed);
      }
      tr_velocity_extensions.emplace_back(v_velocity_extensions);
    }
    velocity_extensions.emplace_back(tr_velocity_extensions);
  }
}

size_t cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::
    get_maximal_velocity_extension_size() const {
  size_t max_size = 0;
  for (const auto& tr_velocity_extensions : velocity_extensions) {
    for (const auto& v_velocity_extensions : tr_velocity_extensions) {
      max_size = std::max(max_size, v_velocity_extensions.size());
    }
  }
  return max_size;
}

void cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::
    initialize_variables(
        const SolutionSettingsMovingBlock& solution_settings_input,
        const SolverStrategyMovingBlock&   solver_strategy_input,
        const ModelDetail&                 model_detail_input) {
  if (solver_strategy_input.include_reverse_headways &&
      !solver_strategy_input.use_lazy_constraints) {
    throw exceptions::InvalidInputException(
        "Reverse headways can only be included with lazy constraints.");
  }
  if (solver_strategy_input.include_reverse_headways &&
      solver_strategy_input.lazy_constraint_selection_strategy !=
          LazyConstraintSelectionStrategy::AllChecked) {
    throw exceptions::InvalidInputException(
        "Reverse headway cannot be used if only violated lazy constraints are "
        "added.");
  }

  num_tr                  = instance.get_train_list().size();
  num_edges               = instance.const_n().number_of_edges();
  num_vertices            = instance.const_n().number_of_vertices();
  max_t                   = instance.max_t();
  this->solution_settings = solution_settings_input;
  this->solver_strategy   = solver_strategy_input;
  this->model_detail      = model_detail_input;
  this->ttd_sections      = instance.n().unbreakable_sections();
  this->num_ttd           = this->ttd_sections.size();
  this->fill_tr_stop_data();
  this->fill_velocity_extensions();
  this->fill_relevant_reverse_edges();
}

void cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::
    create_general_path_constraints() {
  for (size_t tr = 0; tr < num_tr; tr++) {
    const auto& tr_object = instance.get_train_list().get_train(tr);
    for (const auto& e :
         instance.edges_used_by_train(tr, model_detail.fix_routes, false)) {
      const auto&      edge       = instance.const_n().get_edge(e);
      const auto&      source_obj = instance.const_n().get_vertex(edge.source);
      const auto&      target_obj = instance.const_n().get_vertex(edge.target);
      const auto&      v1_values  = velocity_extensions.at(tr).at(edge.source);
      const auto&      v2_values  = velocity_extensions.at(tr).at(edge.target);
      const GRBLinExpr lhs        = vars["x"](tr, e);
      GRBLinExpr       rhs        = 0;
      const auto tmp_max_speed = std::min(tr_object.max_speed, edge.max_speed);
      for (size_t i = 0; i < v1_values.size(); i++) {
        if (v1_values.at(i) > tmp_max_speed) {
          continue;
        }
        for (size_t j = 0; j < v2_values.size(); j++) {
          if (v2_values.at(j) > tmp_max_speed) {
            continue;
          }
          if (cda_rail::possible_by_eom(v1_values.at(i), v2_values.at(j),
                                        tr_object.acceleration,
                                        tr_object.deceleration, edge.length)) {
            rhs += vars["y"](tr, e, i, j);
          }
        }
      }
      // Edge is used if one of the velocity extended arcs is used
      model->addConstr(lhs == rhs, "aggregate_edge_velocity_extension_" +
                                       tr_object.name + "_" + source_obj.name +
                                       "-" + target_obj.name);
    }
    const auto& schedule = instance.get_schedule(tr);
    const auto& entry    = schedule.get_entry();
    const auto& exit     = schedule.get_exit();
    const auto  edges_used_by_train =
        instance.edges_used_by_train(tr, model_detail.fix_routes, false);
    for (const auto& v :
         instance.vertices_used_by_train(tr, model_detail.fix_routes, false)) {
      if (v == entry) {
        GRBLinExpr lhs = 0;
        for (const auto& e : instance.const_n().out_edges(v)) {
          if (std::find(edges_used_by_train.begin(), edges_used_by_train.end(),
                        e) != edges_used_by_train.end()) {
            lhs += vars["x"](tr, e);
          }
        }
        // The entry vertex is only left but not entered
        model->addConstr(lhs == 1, "entry_vertex_" + tr_object.name + "_" +
                                       instance.const_n().get_vertex(v).name);
      } else if (v == exit) {
        GRBLinExpr lhs = 0;
        for (const auto& e : instance.const_n().in_edges(v)) {
          if (std::find(edges_used_by_train.begin(), edges_used_by_train.end(),
                        e) != edges_used_by_train.end()) {
            lhs += vars["x"](tr, e);
          }
        }
        // The exit vertex is only entered but not left
        model->addConstr(lhs == 1, "exit_vertex_" + tr_object.name + "_" +
                                       instance.const_n().get_vertex(v).name);
      } else {
        GRBLinExpr x_in_edges  = 0;
        GRBLinExpr x_out_edges = 0;
        for (const auto& e : instance.const_n().in_edges(v)) {
          if (std::find(edges_used_by_train.begin(), edges_used_by_train.end(),
                        e) != edges_used_by_train.end()) {
            x_in_edges += vars["x"](tr, e);
          }
        }
        for (const auto& e : instance.const_n().out_edges(v)) {
          if (std::find(edges_used_by_train.begin(), edges_used_by_train.end(),
                        e) != edges_used_by_train.end()) {
            x_out_edges += vars["x"](tr, e);
          }
        }
        // All other vertices are entered and left at most once
        model->addConstr(x_in_edges <= 1,
                         "in_edges_" + tr_object.name + "_" +
                             instance.const_n().get_vertex(v).name);
        model->addConstr(x_out_edges <= 1,
                         "out_edges_" + tr_object.name + "_" +
                             instance.const_n().get_vertex(v).name);
        const auto& v1_values = velocity_extensions.at(tr).at(v);
        for (size_t i = 0; i < v1_values.size(); i++) {
          GRBLinExpr lhs = 0;
          GRBLinExpr rhs = 0;
          for (const auto& e : instance.const_n().in_edges(v)) {
            if (std::find(edges_used_by_train.begin(),
                          edges_used_by_train.end(),
                          e) != edges_used_by_train.end()) {
              const auto& edge = instance.const_n().get_edge(e);
              const auto& v2_values =
                  velocity_extensions.at(tr).at(edge.source);
              const auto tmp_max_speed =
                  std::min(tr_object.max_speed, edge.max_speed);
              if (v1_values.at(i) > tmp_max_speed) {
                continue;
              }
              for (size_t j = 0; j < v2_values.size(); j++) {
                if (v2_values.at(j) > tmp_max_speed) {
                  continue;
                }
                if (cda_rail::possible_by_eom(v2_values.at(j), v1_values.at(i),
                                              tr_object.acceleration,
                                              tr_object.deceleration,
                                              edge.length)) {
                  lhs += vars["y"](tr, e, j, i);
                }
              }
            }
          }
          for (const auto& e : instance.const_n().out_edges(v)) {
            if (std::find(edges_used_by_train.begin(),
                          edges_used_by_train.end(),
                          e) != edges_used_by_train.end()) {
              const auto& edge = instance.const_n().get_edge(e);
              const auto  tmp_max_speed =
                  std::min(tr_object.max_speed, edge.max_speed);
              if (v1_values.at(i) > tmp_max_speed) {
                continue;
              }
              const auto& v2_values =
                  velocity_extensions.at(tr).at(edge.target);
              for (size_t j = 0; j < v2_values.size(); j++) {
                if (v2_values.at(j) > tmp_max_speed) {
                  continue;
                }
                if (cda_rail::possible_by_eom(v1_values.at(i), v2_values.at(j),
                                              tr_object.acceleration,
                                              tr_object.deceleration,
                                              edge.length)) {
                  rhs += vars["y"](tr, e, i, j);
                }
              }
            }
          }
          // And they fulfill a flow condition
          model->addConstr(lhs == rhs,
                           "vertex_velocity_extension_flow_condition_" +
                               tr_object.name + "_" +
                               instance.const_n().get_vertex(v).name + "_" +
                               std::to_string(v1_values.at(i)));
        }
      }
    }

    // Prevent illegal paths
    for (const auto& e :
         instance.edges_used_by_train(tr, model_detail.fix_routes, false)) {
      const auto& e_object  = instance.const_n().get_edge(e);
      const auto& v2        = e_object.target;
      const auto& out_edges = instance.const_n().out_edges(v2);
      const auto& v1_name = instance.const_n().get_vertex(e_object.source).name;
      const auto& v2_name = instance.const_n().get_vertex(v2).name;
      for (const auto& e2 : out_edges) {
        if (!instance.const_n().is_valid_successor(e, e2) &&
            std::find(edges_used_by_train.begin(), edges_used_by_train.end(),
                      e2) != edges_used_by_train.end()) {
          const auto& v3_name =
              instance.const_n()
                  .get_vertex(instance.const_n().get_edge(e2).target)
                  .name;
          model->addConstr(vars["x"](tr, e) + vars["x"](tr, e2) <= 1,
                           "illegal_path_" + tr_object.name + "_" + v1_name +
                               "-" + v2_name + "-" + v3_name);
        }
      }
    }
  }
}

void cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::
    create_travel_times_constraints() {
  for (size_t tr = 0; tr < num_tr; tr++) {
    const auto& tr_object = instance.get_train_list().get_train(tr);
    for (const auto& e :
         instance.edges_used_by_train(tr, model_detail.fix_routes, false)) {
      const auto& edge          = instance.const_n().get_edge(e);
      const auto& v1_values     = velocity_extensions.at(tr).at(edge.source);
      const auto& v2_values     = velocity_extensions.at(tr).at(edge.target);
      const auto  tmp_max_speed = std::min(tr_object.max_speed, edge.max_speed);
      for (size_t i = 0; i < v1_values.size(); i++) {
        if (v1_values.at(i) > tmp_max_speed) {
          continue;
        }
        for (size_t j = 0; j < v2_values.size(); j++) {
          if (v2_values.at(j) > tmp_max_speed) {
            continue;
          }
          if (cda_rail::possible_by_eom(v1_values.at(i), v2_values.at(j),
                                        tr_object.acceleration,
                                        tr_object.deceleration, edge.length)) {
            // t_front_arrival >= t_rear_departure + minimal travel time if arc
            // is used
            const auto& min_t_arc = cda_rail::min_travel_time(
                v1_values.at(i), v2_values.at(j), tmp_max_speed,
                tr_object.acceleration, tr_object.deceleration, edge.length);
            const auto& max_t_arc = cda_rail::max_travel_time(
                v1_values.at(i), v2_values.at(j), V_MIN, tr_object.acceleration,
                tr_object.deceleration, edge.length, edge.breakable);
            model->addConstr(
                vars["t_front_arrival"](tr, edge.target) +
                        (ub_timing_variable(tr) + min_t_arc) *
                            (1 - vars["y"](tr, e, i, j)) >=
                    vars["t_front_departure"](tr, edge.source) + min_t_arc,
                "edge_minimal_travel_time_" + tr_object.name + "_" +
                    instance.const_n().get_vertex(edge.source).name + "-" +
                    instance.const_n().get_vertex(edge.target).name + "_" +
                    std::to_string(v1_values.at(i)) + "-" +
                    std::to_string(v2_values.at(j)));

            if (max_t_arc >= std::numeric_limits<double>::infinity()) {
              continue;
            }

            // t_front_arrival <= t_rear_departure + maximal travel time if arc
            // is used
            model->addConstr(
                vars["t_front_arrival"](tr, edge.target) <=
                    vars["t_front_departure"](tr, edge.source) + max_t_arc +
                        (ub_timing_variable(tr) - max_t_arc) *
                            (1 - vars["y"](tr, e, i, j)),
                "edge_maximal_travel_time_" + tr_object.name + "_" +
                    instance.const_n().get_vertex(edge.source).name + "-" +
                    instance.const_n().get_vertex(edge.target).name + "_" +
                    std::to_string(v1_values.at(i)) + "-" +
                    std::to_string(v2_values.at(j)));
          }
        }
      }
    }

    const auto e_used_tr =
        instance.edges_used_by_train(tr, model_detail.fix_routes, false);
    for (const auto& v :
         instance.vertices_used_by_train(tr, model_detail.fix_routes, false)) {
      // t_front_departure >= t_front_arrival
      model->addConstr(vars["t_front_departure"](tr, v) >=
                           vars["t_front_arrival"](tr, v),
                       "tr_dep_after_arrival_" + tr_object.name + "_" +
                           instance.const_n().get_vertex(v).name);

      if (velocity_extensions.at(tr).at(v).at(0) != 0) {
        continue;
      }

      // t_front_departure <= t_front_arrival if train cannot be stopped at
      // vertex v
      GRBLinExpr speed_0_arcs = 0;
      for (const auto& e_in : instance.const_n().in_edges(v)) {
        if (std::find(e_used_tr.begin(), e_used_tr.end(), e_in) !=
            e_used_tr.end()) {
          const auto& e_in_object = instance.const_n().get_edge(e_in);
          const auto& v1_velocities =
              velocity_extensions.at(tr).at(e_in_object.source);
          assert(velocity_extensions.at(tr).at(v).at(0) == 0);
          const auto tmp_max_speed =
              std::min(tr_object.max_speed, e_in_object.max_speed);
          for (size_t i = 0; i < v1_velocities.size(); i++) {
            if (v1_velocities.at(i) > tmp_max_speed) {
              continue;
            }
            if (cda_rail::possible_by_eom(
                    v1_velocities.at(i), 0, tr_object.acceleration,
                    tr_object.deceleration, e_in_object.length)) {
              speed_0_arcs += vars["y"](tr, e_in, i, 0);
            }
          }
        }
      }
      for (const auto& e_out : instance.const_n().out_edges(v)) {
        if (std::find(e_used_tr.begin(), e_used_tr.end(), e_out) !=
            e_used_tr.end()) {
          const auto& e_out_object = instance.const_n().get_edge(e_out);
          const auto& v2_velocities =
              velocity_extensions.at(tr).at(e_out_object.target);
          assert(velocity_extensions.at(tr).at(v).at(0) == 0);
          const auto tmp_max_speed =
              std::min(tr_object.max_speed, e_out_object.max_speed);
          for (size_t i = 0; i < v2_velocities.size(); i++) {
            if (v2_velocities.at(i) > tmp_max_speed) {
              continue;
            }
            if (cda_rail::possible_by_eom(
                    0, v2_velocities.at(i), tr_object.acceleration,
                    tr_object.deceleration, e_out_object.length)) {
              speed_0_arcs += vars["y"](tr, e_out, 0, i);
            }
          }
        }
      }
      model->addConstr(vars["t_front_departure"](tr, v) <=
                           vars["t_front_arrival"](tr, v) +
                               ub_timing_variable(tr) * speed_0_arcs,
                       "tr_might_stop_at_vertex_" + tr_object.name + "_" +
                           instance.const_n().get_vertex(v).name);
    }
  }
}

double
cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::ub_timing_variable(
    size_t tr) const {
  return instance.get_schedule(tr).get_t_n_range().second;
}

void cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::
    create_basic_order_constraints() {
  for (size_t e = 0; e < num_edges; e++) {
    const auto& tr_on_edge = instance.trains_on_edge_mixed_routing(
        e, model_detail.fix_routes, false);
    const auto e_obj = instance.const_n().get_edge(e);
    const auto v1    = instance.const_n().get_vertex(e_obj.source);
    const auto v2    = instance.const_n().get_vertex(e_obj.target);
    for (const auto& tr1 : tr_on_edge) {
      for (const auto& tr2 : tr_on_edge) {
        if (tr1 == tr2) {
          continue;
        }

        model->addConstr(
            vars["order"](tr1, tr2, e) + vars["order"](tr2, tr1, e) <=
                0.5 * (vars["x"](tr1, e) + vars["x"](tr2, e)),
            "edge_order_1_" + instance.get_train_list().get_train(tr1).name +
                "_" + instance.get_train_list().get_train(tr2).name + "_" +
                v1.name + "-" + v2.name);

        model->addConstr(
            vars["order"](tr1, tr2, e) + vars["order"](tr2, tr1, e) >=
                vars["x"](tr1, e) + vars["x"](tr2, e) - 1,
            "edge_order_2_" + instance.get_train_list().get_train(tr1).name +
                "_" + instance.get_train_list().get_train(tr2).name + "_" +
                v1.name + "-" + v2.name);
      }
    }
  }
}

void cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::
    create_train_rear_constraints() {
  for (size_t tr = 0; tr < num_tr; tr++) {
    // Rear departure time is equal to front departure time at certain position
    const auto& tr_object = instance.get_train_list().get_train(tr);
    const auto& schedule  = instance.get_schedule(tr);
    const auto& exit      = schedule.get_exit();
    const auto& v_n       = schedule.get_v_n();
    const auto  edges_used_by_train =
        instance.edges_used_by_train(tr, model_detail.fix_routes, false);

    // NOLINTNEXTLINE(readability-identifier-naming)
    const auto M = ub_timing_variable(tr);

    for (const auto& v :
         instance.vertices_used_by_train(tr, model_detail.fix_routes, false)) {
      if (v == exit) {
        // In case the train has partially left the network use edge case
        // constraints
        const auto v_max_speed =
            instance.const_n().maximal_vertex_speed(v, edges_used_by_train);
        const auto v_exit_velocities = velocity_extensions.at(tr).at(v);
        // t_rear_departure(v) >= t_front_departure(v)  + min_t selected by
        // incoming edge speed
        const auto          in_edges          = instance.const_n().in_edges(v);
        std::vector<size_t> relevant_in_edges = {};
        for (const auto& e : in_edges) {
          if (std::find(edges_used_by_train.begin(), edges_used_by_train.end(),
                        e) != edges_used_by_train.end()) {
            relevant_in_edges.push_back(e);
          }
        }
        GRBLinExpr min_travel_time_expr = 0;
        GRBLinExpr max_travel_time_expr = 0;
        for (size_t i = 0; i < v_exit_velocities.size(); i++) {
          const auto& v_exit_velocity = v_exit_velocities.at(i);
          if (cda_rail::possible_by_eom(
                  v_exit_velocity, v_n, tr_object.acceleration,
                  tr_object.deceleration, tr_object.length)) {
            const auto min_t_to_full_exit = cda_rail::min_travel_time(
                v_exit_velocity, v_n, v_max_speed, tr_object.acceleration,
                tr_object.deceleration, tr_object.length);
            const auto max_t_to_full_exit = cda_rail::max_travel_time(
                v_exit_velocity, v_n, V_MIN, tr_object.acceleration,
                tr_object.deceleration, tr_object.length, false);
            for (const auto& e_in : relevant_in_edges) {
              const auto& e_in_object = instance.const_n().get_edge(e_in);
              const auto  tmp_max_speed =
                  std::min(tr_object.max_speed, e_in_object.max_speed);
              if (v_exit_velocity > tmp_max_speed) {
                continue;
              }
              const auto& v1_velocities =
                  velocity_extensions.at(tr).at(e_in_object.source);
              for (size_t j = 0; j < v1_velocities.size(); j++) {
                if (v1_velocities.at(j) > tmp_max_speed) {
                  continue;
                }
                if (cda_rail::possible_by_eom(
                        v1_velocities.at(j), v_exit_velocity,
                        tr_object.acceleration, tr_object.deceleration,
                        e_in_object.length)) {
                  min_travel_time_expr +=
                      vars["y"](tr, e_in, j, i) * min_t_to_full_exit;
                  max_travel_time_expr +=
                      vars["y"](tr, e_in, j, i) * max_t_to_full_exit;
                }
              }
            }
          }
        }
        model->addConstr(vars["t_rear_departure"](tr, v) >=
                             vars["t_front_departure"](tr, v) +
                                 min_travel_time_expr,
                         "rear_departure_vertex_c1_" + tr_object.name + "_" +
                             instance.const_n().get_vertex(v).name);
        model->addConstr(vars["t_rear_departure"](tr, v) <=
                             vars["t_front_departure"](tr, v) +
                                 max_travel_time_expr,
                         "rear_departure_vertex_c2_" + tr_object.name + "_" +
                             instance.const_n()
                                 .get_vertex(v)
                                 .name); // not needed because objective pushes
                                         // rear departure down
      } else {
        // Otherwise deduce limits from last path edge
        const auto possible_paths =
            instance.const_n().all_paths_of_length_starting_in_vertex(
                v, tr_object.length, exit, edges_used_by_train);
        for (size_t p_ind = 0; p_ind < possible_paths.size(); p_ind++) {
          const auto&  p                 = possible_paths.at(p_ind);
          const double p_len_last_vertex = std::accumulate(
              p.begin(), p.end() - 1, 0.0,
              [this](double sum, const auto& edge_index) {
                return sum + instance.const_n().get_edge(edge_index).length;
              });
          assert(p_len_last_vertex >= 0);
          assert(p_len_last_vertex <= tr_object.length);
          const auto& last_edge     = p.back();
          const auto& last_edge_obj = instance.const_n().get_edge(last_edge);

          GRBLinExpr lhs = vars["t_rear_departure"](tr, v) +
                           M * static_cast<double>(p.size());
          for (const auto& e_p : p) {
            lhs -= M * vars["x"](tr, e_p);
          }

          if (last_edge_obj.target == exit &&
              last_edge_obj.length + p_len_last_vertex < tr_object.length) {
            // The train has partially left the network through the last edge
            const auto outside_len =
                tr_object.length - p_len_last_vertex - last_edge_obj.length;
            assert(outside_len >= 0);
            assert(outside_len <= tr_object.length);
            const auto& v_exit_velocities = velocity_extensions.at(tr).at(exit);

            const auto tr_max_speed_tmp =
                std::min({tr_object.max_speed,
                          instance.const_n().maximal_vertex_speed(
                              exit, edges_used_by_train),
                          last_edge_obj.max_speed});

            GRBLinExpr min_travel_time_expr = 0;
            GRBLinExpr max_travel_time_expr = 0;
            for (size_t i = 0; i < v_exit_velocities.size(); i++) {
              const auto& v_exit_velocity = v_exit_velocities.at(i);
              if (v_exit_velocity > tr_max_speed_tmp) {
                continue;
              }
              if (cda_rail::possible_by_eom(
                      v_exit_velocity, v_n, tr_object.acceleration,
                      tr_object.deceleration, tr_object.length)) {
                const auto min_t_to_required_pos =
                    cda_rail::min_travel_time_from_start(
                        v_exit_velocity, v_n, tr_max_speed_tmp,
                        tr_object.acceleration, tr_object.deceleration,
                        tr_object.length, outside_len);
                const auto max_t_to_required_pos =
                    cda_rail::max_travel_time_from_start(
                        v_exit_velocity, v_n, V_MIN, tr_object.acceleration,
                        tr_object.deceleration, tr_object.length, outside_len,
                        false);
                if (v_exit_velocity > tr_max_speed_tmp) {
                  continue;
                }
                const auto& v1_velocities =
                    velocity_extensions.at(tr).at(last_edge_obj.source);
                for (size_t j = 0; j < v1_velocities.size(); j++) {
                  if (v1_velocities.at(j) > tr_max_speed_tmp) {
                    continue;
                  }
                  if (cda_rail::possible_by_eom(
                          v1_velocities.at(j), v_exit_velocity,
                          tr_object.acceleration, tr_object.deceleration,
                          last_edge_obj.length)) {
                    min_travel_time_expr +=
                        vars["y"](tr, last_edge, j, i) * min_t_to_required_pos;
                    max_travel_time_expr +=
                        vars["y"](tr, last_edge, j, i) * max_t_to_required_pos;
                  }
                }
              }
            }

            model->addConstr(lhs >= vars["t_front_departure"](tr, exit) +
                                        min_travel_time_expr,
                             "rear_departure_half_leaving_1_" + tr_object.name +
                                 "_" + instance.const_n().get_vertex(v).name +
                                 "_" + std::to_string(p_ind));
            model->addConstr(lhs <= vars["t_front_departure"](tr, exit) +
                                        max_travel_time_expr,
                             "rear_departure_half_leaving_2_" + tr_object.name +
                                 "_" + instance.const_n().get_vertex(v).name +
                                 "_" + std::to_string(p_ind));

          } else {
            // The relevant point is on an actual edge
            assert(last_edge_obj.length + p_len_last_vertex >=
                   tr_object.length);

            const auto rel_pt_on_edge = tr_object.length - p_len_last_vertex;
            const auto v_0_velocities =
                velocity_extensions.at(tr).at(last_edge_obj.source);
            const auto v_1_velocities =
                velocity_extensions.at(tr).at(last_edge_obj.target);

            if (rel_pt_on_edge + 1e-6 >= last_edge_obj.length) {
              // Directly use corresponding variable
              model->addConstr(
                  lhs >= vars["t_front_departure"](tr, last_edge_obj.target),
                  "rear_departure_2_" + tr_object.name + "_" +
                      instance.const_n().get_vertex(v).name + "_" +
                      std::to_string(p_ind));
            } else {
              // Only in this case there is no corresponding variable. Note that
              // objective pushes rear departure down.
              GRBLinExpr t_ref_1 =
                  vars["t_front_departure"](tr, last_edge_obj.source);
              GRBLinExpr t_ref_2 =
                  vars["t_front_arrival"](tr, last_edge_obj.target);
              const auto v_max_rel_e =
                  std::min(last_edge_obj.max_speed, tr_object.max_speed);
              for (size_t i = 0; i < v_0_velocities.size(); i++) {
                if (v_0_velocities.at(i) > v_max_rel_e) {
                  continue;
                }
                for (size_t j = 0; j < v_1_velocities.size(); j++) {
                  if (v_1_velocities.at(j) > v_max_rel_e) {
                    continue;
                  }
                  if (cda_rail::possible_by_eom(
                          v_0_velocities.at(i), v_1_velocities.at(j),
                          tr_object.acceleration, tr_object.deceleration,
                          last_edge_obj.length)) {
                    t_ref_1 += vars["y"](tr, last_edge, i, j) *
                               cda_rail::min_travel_time_from_start(
                                   v_0_velocities.at(i), v_1_velocities.at(j),
                                   v_max_rel_e, tr_object.acceleration,
                                   tr_object.deceleration, last_edge_obj.length,
                                   rel_pt_on_edge);
                    const auto max_travel_time =
                        cda_rail::max_travel_time_to_end(
                            v_0_velocities.at(i), v_1_velocities.at(j), V_MIN,
                            tr_object.acceleration, tr_object.deceleration,
                            last_edge_obj.length, rel_pt_on_edge,
                            last_edge_obj.breakable);
                    t_ref_2 -= vars["y"](tr, last_edge, i, j) *
                               (max_travel_time >=
                                        std::numeric_limits<double>::infinity()
                                    ? M
                                    : max_travel_time);
                  }
                }
              }

              model->addConstr(lhs >= t_ref_1,
                               "rear_departure_1_" + tr_object.name + "_" +
                                   instance.const_n().get_vertex(v).name + "_" +
                                   std::to_string(p_ind));
              model->addConstr(lhs >= t_ref_2,
                               "rear_departure_2_" + tr_object.name + "_" +
                                   instance.const_n().get_vertex(v).name + "_" +
                                   std::to_string(p_ind));
            }
          }
        }
      }
    }
  }
}

void cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::
    create_stopping_constraints() {
  for (size_t tr = 0; tr < num_tr; tr++) {
    const auto& tr_object = instance.get_train_list().get_train(tr);
    // NOLINTNEXTLINE(readability-identifier-naming)
    const auto M = ub_timing_variable(tr);

    // Stop at exactly one stop using lhs
    const auto& tr_schedule = instance.get_schedule(tr);
    const auto& tr_stops    = tr_schedule.get_stops();
    for (size_t stop = 0; stop < tr_stops.size(); stop++) {
      const auto& stop_data         = tr_stop_data.at(tr).at(stop);
      const auto& stop_object       = tr_stops.at(stop);
      const auto& stop_station_name = stop_object.get_station_name();
      GRBLinExpr  lhs               = 0;
      for (const auto& [v, paths] : stop_data) {
        lhs += vars["stop"](tr, stop, v);

        // If stopped then t_front_departure - t_front_arrival >= stop_time,
        // otherwise unconstrained Hence, >= stop_time * stop
        model->addConstr(
            vars["t_front_departure"](tr, v) - vars["t_front_arrival"](tr, v) >=
                stop_object.get_min_stopping_time() * vars["stop"](tr, stop, v),
            "min_stop_time_" + tr_object.name + "_" + stop_station_name +
                "_vertex_" + instance.const_n().get_vertex(v).name);

        // If stopped then t_front_arrival is within desired arrival interval
        const auto t_0_interval = stop_object.get_begin_range();
        // t >= t_0 * stop
        model->addConstr(vars["t_front_arrival"](tr, v) >=
                             t_0_interval.first * vars["stop"](tr, stop, v),
                         "min_arrival_time_" + tr_object.name + "_" +
                             stop_station_name + "_vertex_" +
                             instance.const_n().get_vertex(v).name);
        // t <= t_0 + M * (1 - stop)
        model->addConstr(
            vars["t_front_arrival"](tr, v) <=
                t_0_interval.second + M * (1 - vars["stop"](tr, stop, v)),
            "max_arrival_time_" + tr_object.name + "_" + stop_station_name +
                "_vertex_" + instance.const_n().get_vertex(v).name);

        // If stopped then t_front_departure is within desired departure
        // interval
        const auto t_n_interval = stop_object.get_end_range();
        // t >= t_n * stop
        model->addConstr(vars["t_front_departure"](tr, v) >=
                             t_n_interval.first * vars["stop"](tr, stop, v),
                         "min_departure_time_" + tr_object.name + "_" +
                             stop_station_name + "_vertex_" +
                             instance.const_n().get_vertex(v).name);
        // t <= t_n + M * (1 - stop)
        model->addConstr(
            vars["t_front_departure"](tr, v) <=
                t_n_interval.second + M * (1 - vars["stop"](tr, stop, v)),
            "max_departure_time_" + tr_object.name + "_" + stop_station_name +
                "_vertex_" + instance.const_n().get_vertex(v).name);

        // Train can only stop if one of the valid edge paths is used
        GRBLinExpr path_expr = 0;
        for (size_t p_index = 0; p_index < paths.size(); p_index++) {
          const auto& p = paths.at(p_index);
          // The following variable should be binary, however since only one
          // direction of inference is needed, continuous should suffice
          const auto tmp_var = model->addVar(
              0.0, 1.0, 0.0, GRB_CONTINUOUS,
              "stop_path_" + tr_object.name + "_" + stop_station_name +
                  "_vertex_" + instance.const_n().get_vertex(v).name +
                  "_path_" + std::to_string(p_index));
          path_expr += tmp_var;
          for (const auto& e : p) {
            model->addConstr(tmp_var <= vars["x"](tr, e),
                             "stop_path_" + tr_object.name + "_" +
                                 stop_station_name + "_vertex_" +
                                 instance.const_n().get_vertex(v).name +
                                 "_path_" + std::to_string(p_index) + "_edge_" +
                                 std::to_string(e));
          }
          model->addConstr(vars["stop"](tr, stop, v) >= tmp_var,
                           "use_path_only_if_stopped_" + tr_object.name + "_" +
                               stop_station_name + "_vertex_" +
                               instance.const_n().get_vertex(v).name +
                               "_path_" + std::to_string(p_index));
        }
        model->addConstr(vars["stop"](tr, stop, v) <= path_expr,
                         "stop_only_if_path_is_used_" + tr_object.name + "_" +
                             stop_station_name + "_vertex_" +
                             instance.const_n().get_vertex(v).name);
      }
      model->addConstr(lhs == 1,
                       "stop_at_one_vertex_" +
                           instance.get_train_list().get_train(tr).name + "_" +
                           stop_station_name);
    }

    // Initial
    const auto& t0_range = tr_schedule.get_t_0_range();
    model->addConstr(vars["t_front_arrival"](tr, tr_schedule.get_entry()) >=
                         t0_range.first,
                     "initial_arrival_time_lb_" + tr_object.name);
    model->addConstr(vars["t_front_arrival"](tr, tr_schedule.get_entry()) <=
                         t0_range.second,
                     "initial_arrival_time_ub_" + tr_object.name);

    // Final
    const auto& tn_range = tr_schedule.get_t_n_range();
    model->addConstr(vars["t_rear_departure"](tr, tr_schedule.get_exit()) >=
                         tn_range.first,
                     "final_departure_time_lb_" + tr_object.name);
    model->addConstr(vars["t_rear_departure"](tr, tr_schedule.get_exit()) <=
                         tn_range.second,
                     "final_departure_time_ub_" + tr_object.name);
  }
}

void cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::
    create_headway_constraints() {
  for (size_t tr = 0; tr < num_tr; tr++) {
    const auto& tr_object = instance.get_train_list().get_train(tr);
    const auto  tr_used_edges =
        instance.edges_used_by_train(tr, model_detail.fix_routes, false);
    const auto& tr_schedule_object = instance.get_schedule(tr);
    const auto& entry_node         = tr_schedule_object.get_entry();
    const auto  t_bound            = ub_timing_variable(tr);

    for (const auto v :
         instance.vertices_used_by_train(tr, model_detail.fix_routes, false)) {
      const auto v_velocities = velocity_extensions.at(tr).at(v);
      for (size_t v_source_index = 0; v_source_index < v_velocities.size();
           v_source_index++) {
        const auto& vel = v_velocities.at(v_source_index);
        if (vel > tr_object.max_speed) {
          continue;
        }
        const auto bd = vel * vel / (2 * tr_object.deceleration);
        // What if bd is outside network. Then relation to point where ma was
        // set to exit node or leave as it is?
        const auto brake_paths =
            instance.const_n().all_paths_of_length_starting_in_vertex(
                v, std::max(EPS, bd), {},
                tr_used_edges); // min EPS so that following edge is detected
                                // for speed 0
        for (size_t p_index = 0; p_index < brake_paths.size(); p_index++) {
          const auto& p     = brake_paths.at(p_index);
          const auto  p_len = std::accumulate(
              p.begin(), p.end(), 0.0,
              [this](double sum, const auto& edge_index) {
                return sum + instance.const_n().get_edge(edge_index).length;
              });

          // Variables to decide if path was used. First edge must leave from
          // desired velocity extension.
          const GRBLinExpr edge_path_expr = get_edge_path_expr(tr, p, vel);
          const auto       tmp_max_speed =
              std::min(tr_object.max_speed,
                       instance.const_n().get_edge(p.front()).max_speed);

          const auto tr_on_last_edge = instance.trains_on_edge_mixed_routing(
              p.back(), model_detail.fix_routes, false);

          const auto& last_edge_object = instance.const_n().get_edge(p.back());

          for (const auto& tr2 : tr_on_last_edge) {
            if (tr == tr2) {
              continue;
            }

            const auto& t_bound_tmp =
                std::max(t_bound, ub_timing_variable(tr2));

            const GRBLinExpr lhs =
                vars["t_front_departure"](tr, v) +
                t_bound_tmp * (static_cast<double>(p.size()) - edge_path_expr) +
                t_bound_tmp * (1 - vars["order"](tr, tr2, p.back()));
            std::vector<GRBLinExpr> rhs;
            if (p_len + EPS >= bd && p_len - EPS <= bd) {
              // Target vertex is exactly the desired moving authority
              // t_front_departure(tr, v) >= t_rear_departure(tr2, target) if
              // order(tr, tr2, e) = 1 and path p chosen.
              rhs.emplace_back(
                  vars["t_rear_departure"](tr2, last_edge_object.target));
            } else {
              assert(p_len > bd && p_len - last_edge_object.length <= bd);
              const auto  target_point = bd - p_len + last_edge_object.length;
              const auto& v_tr2_source_velocities =
                  velocity_extensions.at(tr2).at(last_edge_object.source);
              const auto& v_tr2_target_velocities =
                  velocity_extensions.at(tr2).at(last_edge_object.target);
              rhs.emplace_back(
                  vars["t_rear_departure"](tr2, last_edge_object.source));
              rhs.emplace_back(
                  vars["t_rear_departure"](tr2, last_edge_object.target));
              const auto& tr2_object = instance.get_train_list().get_train(tr2);
              const auto  max_speed =
                  std::min(tr2_object.max_speed, last_edge_object.max_speed);
              for (size_t v_tr2_source_index = 0;
                   v_tr2_source_index < v_tr2_source_velocities.size();
                   v_tr2_source_index++) {
                const auto& vel_tr2_source =
                    v_tr2_source_velocities.at(v_tr2_source_index);
                if (vel_tr2_source > max_speed) {
                  continue;
                }
                for (size_t v_tr2_target_index = 0;
                     v_tr2_target_index < v_tr2_target_velocities.size();
                     v_tr2_target_index++) {
                  const auto& vel_tr2_target =
                      v_tr2_target_velocities.at(v_tr2_target_index);
                  if (vel_tr2_target > max_speed) {
                    continue;
                  }
                  if (cda_rail::possible_by_eom(vel_tr2_source, vel_tr2_target,
                                                tr2_object.acceleration,
                                                tr2_object.deceleration,
                                                last_edge_object.length)) {
                    // first: += y * min_t
                    // second: -= y * max_t
                    rhs.at(0) +=
                        vars["y"](tr2, p.back(), v_tr2_source_index,
                                  v_tr2_target_index) *
                        cda_rail::min_travel_time_from_start(
                            vel_tr2_source, vel_tr2_target, max_speed,
                            tr2_object.acceleration, tr2_object.deceleration,
                            last_edge_object.length, target_point);
                    const auto max_travel_time =
                        cda_rail::max_travel_time_to_end(
                            vel_tr2_source, vel_tr2_target, V_MIN,
                            tr2_object.acceleration, tr2_object.deceleration,
                            last_edge_object.length, target_point,
                            last_edge_object.breakable);
                    rhs.at(1) -=
                        vars["y"](tr2, p.back(), v_tr2_source_index,
                                  v_tr2_target_index) *
                        (max_travel_time > t_bound_tmp ? t_bound_tmp
                                                       : max_travel_time);
                  }
                }
              }
            }
            for (size_t rhs_idx = 0; rhs_idx < rhs.size(); rhs_idx++) {
              model->addConstr(
                  lhs >= rhs.at(rhs_idx),
                  "headway_" + std::to_string(rhs_idx) + "-" +
                      std::to_string(rhs.size()) + "_" + tr_object.name + "_" +
                      instance.get_train_list().get_train(tr2).name + "_" +
                      instance.const_n().get_vertex(v).name + "_" +
                      std::to_string(vel) + "_" + std::to_string(p_index));
            }
          }

          // Headways on intersecting TTD sections
          const auto intersecting_ttd =
              cda_rail::Network::get_intersecting_ttd(p, ttd_sections);
          for (const auto& [ttd_index, e_index] : intersecting_ttd) {
            const auto& p_tmp =
                std::vector<size_t>(p.begin(), p.begin() + e_index);
            const auto p_tmp_len = std::accumulate(
                p_tmp.begin(), p_tmp.end(), 0.0,
                [this](double sum, const auto& edge_index) {
                  return sum + instance.const_n().get_edge(edge_index).length;
                });
            GRBLinExpr edge_tmp_path_expr = 0;
            for (const auto& e_tmp : p_tmp) {
              edge_tmp_path_expr += vars["x"](tr, e_tmp);
            }

            const auto obd = bd - p_tmp_len;

            assert(obd >= 0);

            const auto tr_on_ttd = instance.trains_in_section(
                ttd_sections.at(ttd_index), model_detail.fix_routes, false);
            for (const auto& tr2 : tr_on_ttd) {
              if (tr == tr2) {
                continue;
              }

              const auto t_bound_tmp =
                  std::max(t_bound, ub_timing_variable(tr2));

              GRBLinExpr lhs_from_rear =
                  vars["t_front_departure"](tr, v) +
                  t_bound_tmp *
                      (static_cast<double>(p_tmp.size()) - edge_tmp_path_expr);
              const GRBLinExpr rhs =
                  vars["t_ttd_departure"](tr2, ttd_index) +
                  t_bound_tmp * (vars["order_ttd"](tr, tr2, ttd_index) - 1);

              bool is_relevant = obd < GRB_EPS;

              if (obd >= GRB_EPS) {
                assert(vel >= GRB_EPS);
                // Get all possible edges before v
                if (v == entry_node) {
                  // Train is entering the network
                  assert(v_velocities.size() == 1);
                  // Assume previous constant line speed
                  lhs_from_rear -= vel <= GRB_EPS ? 0 : obd / vel;
                  is_relevant = true;
                } else {
                  std::vector<size_t> rel_in_edges;
                  for (const auto& e : instance.const_n().in_edges(v)) {
                    if (std::find(tr_used_edges.begin(), tr_used_edges.end(),
                                  e) != tr_used_edges.end()) {
                      rel_in_edges.push_back(e);
                    }
                  }
                  // Note: In some cases there might be no relevant in edges, in
                  // particular if it is an entry node of a different train that
                  // might not be reachable from the network itself.
                  for (const auto& e_before_v : rel_in_edges) {
                    const auto& e_before_v_obj =
                        instance.const_n().get_edge(e_before_v);
                    const auto& v_before_v = e_before_v_obj.source;
                    const auto  v_before_v_velocities =
                        velocity_extensions.at(tr).at(v_before_v);
                    const auto& e_before_v_tmp_max =
                        std::min(tmp_max_speed, e_before_v_obj.max_speed);
                    if (vel > e_before_v_tmp_max) {
                      continue;
                    }
                    for (size_t v_before_v_index = 0;
                         v_before_v_index < v_before_v_velocities.size();
                         v_before_v_index++) {
                      const auto& vel_before_v =
                          v_before_v_velocities.at(v_before_v_index);
                      if (vel_before_v > e_before_v_tmp_max ||
                          vel_before_v * vel_before_v /
                                  (2 * tr_object.deceleration) >=
                              e_before_v_obj.length + p_tmp_len) {
                        continue;
                      }
                      if (cda_rail::possible_by_eom(
                              vel_before_v, vel, tr_object.acceleration,
                              tr_object.deceleration, e_before_v_obj.length)) {
                        lhs_from_rear -=
                            vars["y"](tr, e_before_v, v_before_v_index,
                                      v_source_index) *
                            cda_rail::min_time_from_rear_to_ma_point(
                                vel_before_v, vel, V_MIN, e_before_v_tmp_max,
                                tr_object.acceleration, tr_object.deceleration,
                                e_before_v_obj.length, obd);
                        is_relevant = true;

                        const auto max_from_front =
                            cda_rail::max_time_from_front_to_ma_point(
                                vel_before_v, vel, V_MIN,
                                tr_object.acceleration, tr_object.deceleration,
                                e_before_v_obj.length, obd,
                                e_before_v_obj.breakable);
                        const GRBLinExpr lhs_from_front =
                            vars["t_front_departure"](tr, v_before_v) +
                            std::min(max_from_front, t_bound_tmp) +
                            t_bound_tmp *
                                (static_cast<double>(p_tmp.size()) + 1 -
                                 vars["y"](tr, e_before_v, v_before_v_index,
                                           v_source_index) -
                                 edge_tmp_path_expr);
                        model->addConstr(
                            lhs_from_front >= rhs,
                            "headway_ttd_from_front_" + tr_object.name + "_" +
                                instance.get_train_list().get_train(tr2).name +
                                "_" + instance.const_n().get_vertex(v).name +
                                "_" + std::to_string(vel) + "_" +
                                std::to_string(p_index) + "_" +
                                std::to_string(e_before_v) + "_" +
                                std::to_string(vel_before_v));
                      }
                    }
                  }
                }
              }
              if (is_relevant) {
                model->addConstr(
                    lhs_from_rear >= rhs,
                    "headway_ttd_" + tr_object.name + "_" +
                        instance.get_train_list().get_train(tr2).name + "_" +
                        instance.const_n().get_vertex(v).name + "_" +
                        std::to_string(vel) + "_" + std::to_string(p_index) +
                        "_" + std::to_string(ttd_index));
              }
            }
          }
        }
      }
    }
  }
}

void cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::
    create_basic_ttd_constraints() {
  for (size_t i = 0; i < ttd_sections.size(); i++) {
    const auto& ttd_section = ttd_sections.at(i);
    const auto  tr_on_ttd =
        instance.trains_in_section(ttd_section, model_detail.fix_routes, false);
    for (const auto& tr : tr_on_ttd) {
      const auto t_bound = ub_timing_variable(tr);

      // x_ttd aggregates x values
      const auto e_tr =
          instance.edges_used_by_train(tr, model_detail.fix_routes, false);
      // relevant edges are intersection of ttd_section and e_tr
      std::vector<size_t> relevant_edges;
      for (const auto& e : ttd_section) {
        if (std::find(e_tr.begin(), e_tr.end(), e) != e_tr.end()) {
          relevant_edges.push_back(e);
        }
      }
      GRBLinExpr rhs = 0;
      for (const auto& e : relevant_edges) {
        const auto e_object = instance.const_n().get_edge(e);
        const auto v1_name =
            instance.const_n().get_vertex(e_object.source).name;
        const auto v2_name =
            instance.const_n().get_vertex(e_object.target).name;
        model->addConstr(vars["x_ttd"](tr, i) >= vars["x"](tr, e),
                         "aggregate_edge_ttd_1_" +
                             instance.get_train_list().get_train(tr).name +
                             "_" + std::to_string(i) + "_" + v1_name + "-" +
                             v2_name);
        rhs += vars["x"](tr, e);

        // Moreover bound t_ttd_departure
        // >= t_rear_departure(v2) * x(e)
        // where 0 <= t_rear_departure <= t_bound continuous
        // x(e) is binary
        // Hence, equivalent to
        // t_ttd >= t - t_bound * (1 - x)
        // t_ttd >= 0 (already by definition)
        // Because we are only interested in bounding the time from below no
        // other constraints are needed.
        model->addConstr(vars["t_ttd_departure"](tr, i) >=
                             vars["t_rear_departure"](tr, e_object.target) -
                                 t_bound * (1 - vars["x"](tr, e)),
                         "ttd_departure_bound_" +
                             instance.get_train_list().get_train(tr).name +
                             "_" + std::to_string(i) + "_" + v1_name + "-" +
                             v2_name);
      }
      model->addConstr(vars["x_ttd"](tr, i) <= rhs,
                       "aggregate_edge_ttd_2_" +
                           instance.get_train_list().get_train(tr).name + "_" +
                           std::to_string(i));

      // Order constraints as usual
      for (const auto& tr2 : tr_on_ttd) {
        if (tr == tr2) {
          continue;
        }
        model->addConstr(
            vars["order_ttd"](tr, tr2, i) + vars["order_ttd"](tr2, tr, i) <=
                0.5 * (vars["x_ttd"](tr, i) + vars["x_ttd"](tr2, i)),
            "ttd_order_1_" + instance.get_train_list().get_train(tr).name +
                "_" + instance.get_train_list().get_train(tr2).name + "_" +
                std::to_string(i));
        model->addConstr(
            vars["order_ttd"](tr, tr2, i) + vars["order_ttd"](tr2, tr, i) >=
                vars["x_ttd"](tr, i) - vars["x_ttd"](tr2, i) - 1,
            "ttd_order_2_" + instance.get_train_list().get_train(tr).name +
                "_" + instance.get_train_list().get_train(tr2).name + "_" +
                std::to_string(i));
      }
    }
  }
}

void cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::
    create_reverse_edge_constraints() {
  for (size_t idx = 0; idx < relevant_reverse_edges.size(); idx++) {
    const auto& [e1, e2] = relevant_reverse_edges.at(idx);
    const auto tr_list =
        instance.trains_in_section({e1, e2}, model_detail.fix_routes, false);
    const auto tr_list_1 = instance.trains_on_edge_mixed_routing(
        e1, model_detail.fix_routes, false);
    const auto tr_list_2 = instance.trains_on_edge_mixed_routing(
        e2, model_detail.fix_routes, false);

    const auto  e_obj   = instance.const_n().get_edge(e1);
    const auto& v1_name = instance.const_n().get_vertex(e_obj.source).name;
    const auto& v2_name = instance.const_n().get_vertex(e_obj.target).name;

    for (const auto& tr1 : tr_list_1) {
      const auto& tr1_name = instance.get_train_list().get_train(tr1).name;
      for (const auto& tr2 : tr_list_2) {
        if (tr1 == tr2) {
          continue;
        }
        const auto& tr2_name = instance.get_train_list().get_train(tr2).name;
        model->addConstr(vars["reverse_order"](tr1, tr2, idx) +
                                 vars["reverse_order"](tr2, tr1, idx) >=
                             vars["x"](tr1, e1) + vars["x"](tr2, e2) - 1,
                         "reverse_order_lb_" + tr1_name + "_" + tr2_name + "_" +
                             v1_name + "-" + v2_name);
      }
    }

    for (size_t idx_tr1 = 0; idx_tr1 < tr_list.size(); idx_tr1++) {
      const auto  tr1      = tr_list.at(idx_tr1);
      const auto& tr1_name = instance.get_train_list().get_train(tr1).name;
      const auto  ub_val_1 = ub_timing_variable(tr1);
      for (size_t idx_tr2 = idx_tr1 + 1; idx_tr2 < tr_list.size(); idx_tr2++) {
        const auto  tr2      = tr_list.at(idx_tr2);
        const auto& tr2_name = instance.get_train_list().get_train(tr2).name;
        model->addConstr(vars["reverse_order"](tr1, tr2, idx) +
                                 vars["reverse_order"](tr2, tr1, idx) <=
                             1,
                         "reverse_order_ub_" + tr1_name + "_" + tr2_name + "_" +
                             v1_name + "-" + v2_name);

        const auto ub_val_2 = ub_timing_variable(tr2);
        const auto t_bound  = std::max(ub_val_1, ub_val_2);

        model->addConstr(
            vars["t_front_departure"](tr1, e_obj.source) +
                    t_bound * (1 - vars["reverse_order"](tr1, tr2, idx)) >=
                vars["t_rear_departure"](tr2, e_obj.source),
            "reverse_order_1_" + tr1_name + "_" + tr2_name + "_" + v1_name +
                "-" + v2_name);
        model->addConstr(
            vars["t_front_departure"](tr1, e_obj.target) +
                    t_bound * (1 - vars["reverse_order"](tr1, tr2, idx)) >=
                vars["t_rear_departure"](tr2, e_obj.target),
            "reverse_order_2_" + tr1_name + "_" + tr2_name + "_" + v1_name +
                "-" + v2_name);

        model->addConstr(
            vars["t_front_departure"](tr2, e_obj.source) +
                    t_bound * (1 - vars["reverse_order"](tr2, tr1, idx)) >=
                vars["t_rear_departure"](tr1, e_obj.source),
            "reverse_order_1b_" + tr2_name + "_" + tr1_name + "_" + v1_name +
                "-" + v2_name);
        model->addConstr(
            vars["t_front_departure"](tr2, e_obj.target) +
                    t_bound * (1 - vars["reverse_order"](tr2, tr1, idx)) >=
                vars["t_rear_departure"](tr1, e_obj.target),
            "reverse_order_2b_" + tr2_name + "_" + tr1_name + "_" + v1_name +
                "-" + v2_name);
      }
    }
  }
}

void cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::
    create_vertex_headway_constraints() {
  // If a line headway is specified (most importantly on exit nodes), then obey
  // This only takes into account if the same previous or next edge is used
  for (size_t e = 0; e < num_edges; e++) {
    const auto& tr_on_edge = instance.trains_on_edge_mixed_routing(
        e, model_detail.fix_routes, false);
    if (tr_on_edge.size() <= 1) {
      continue;
    }

    const auto& e_object        = instance.const_n().get_edge(e);
    const auto& source_v        = e_object.source;
    const auto& target_v        = e_object.target;
    const auto& source_v_object = instance.const_n().get_vertex(source_v);
    const auto& target_v_object = instance.const_n().get_vertex(target_v);

    for (size_t tr1_index = 1; tr1_index < tr_on_edge.size(); tr1_index++) {
      const auto& tr1         = tr_on_edge.at(tr1_index);
      const auto  tr1_t_bound = ub_timing_variable(tr1);
      for (size_t tr2_index = 0; tr2_index < tr1_index; tr2_index++) {
        const auto& tr2         = tr_on_edge.at(tr2_index);
        const auto  tr2_t_bound = ub_timing_variable(tr2);
        const auto  t_bound     = std::max(tr1_t_bound, tr2_t_bound);
        // Add headway constraints to both source and target vertices depending
        // on train order
        model->addConstr(
            vars["t_front_departure"](tr1, source_v) +
                    (t_bound + source_v_object.headway) *
                        (1 - vars["order"](tr1, tr2, e)) >=
                vars["t_rear_departure"](tr2, source_v) +
                    source_v_object.headway,
            "headway_vertex_source_1_" +
                instance.get_train_list().get_train(tr1).name + "_" +
                instance.get_train_list().get_train(tr2).name + "_" +
                source_v_object.name + "-" + target_v_object.name);
        model->addConstr(
            vars["t_front_departure"](tr2, source_v) +
                    (t_bound + source_v_object.headway) *
                        (1 - vars["order"](tr2, tr1, e)) >=
                vars["t_rear_departure"](tr1, source_v) +
                    source_v_object.headway,
            "headway_vertex_source_2_" +
                instance.get_train_list().get_train(tr1).name + "_" +
                instance.get_train_list().get_train(tr2).name + "_" +
                source_v_object.name + "-" + target_v_object.name);
        model->addConstr(
            vars["t_front_departure"](tr1, target_v) +
                    (t_bound + target_v_object.headway) *
                        (1 - vars["order"](tr1, tr2, e)) >=
                vars["t_rear_departure"](tr2, target_v) +
                    target_v_object.headway,
            "headway_vertex_target_1_" +
                instance.get_train_list().get_train(tr1).name + "_" +
                instance.get_train_list().get_train(tr2).name + "_" +
                source_v_object.name + "-" + target_v_object.name);
        model->addConstr(
            vars["t_front_departure"](tr2, target_v) +
                    (t_bound + target_v_object.headway) *
                        (1 - vars["order"](tr2, tr1, e)) >=
                vars["t_rear_departure"](tr1, target_v) +
                    target_v_object.headway,
            "headway_vertex_target_2_" +
                instance.get_train_list().get_train(tr1).name + "_" +
                instance.get_train_list().get_train(tr2).name + "_" +
                source_v_object.name + "-" + target_v_object.name);
      }
    }
  }
}

GRBLinExpr
cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::get_edge_path_expr(
    size_t tr, const std::vector<size_t>& p, double initial_velocity,
    bool also_higher_velocities) {
  // Get linear expression that sums up all corresponding binary variables.
  // For the first edge, only extended vertices starting with the desired
  // velocity are considered If, optionally, also_higher_velocities is true,
  // then also edges leaving with any higher velocity are considered.
  GRBLinExpr edge_path_expr = 0;

  const auto& tr_object     = instance.get_train_list().get_train(tr);
  const auto& e_1           = p.front();
  const auto& e_1_obj       = instance.const_n().get_edge(e_1);
  const auto  tmp_max_speed = std::min(tr_object.max_speed, e_1_obj.max_speed);
  const auto& v_source_velocities =
      velocity_extensions.at(tr).at(e_1_obj.source);
  const auto& v_target_velocities =
      velocity_extensions.at(tr).at(e_1_obj.target);
  for (size_t v_source_index = 0; v_source_index < v_source_velocities.size();
       v_source_index++) {
    const auto& vel_source = v_source_velocities.at(v_source_index);
    if (!also_higher_velocities &&
        std::abs(vel_source - initial_velocity) > EPS) {
      continue;
    }
    if (vel_source + EPS < initial_velocity || vel_source > tmp_max_speed) {
      continue;
    }
    for (size_t v_target_index = 0; v_target_index < v_target_velocities.size();
         v_target_index++) {
      const auto& vel_target = v_target_velocities.at(v_target_index);
      if (vel_target > tmp_max_speed) {
        continue;
      }
      if (cda_rail::possible_by_eom(vel_source, vel_target,
                                    tr_object.acceleration,
                                    tr_object.deceleration, e_1_obj.length)) {
        edge_path_expr += vars["y"](tr, e_1, v_source_index, v_target_index);
      }
    }
  }
  for (const auto& e_p : p) {
    if (e_p != e_1) {
      edge_path_expr += vars["x"](tr, e_p);
    }
  }

  return edge_path_expr;
}

void cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::
    fill_relevant_reverse_edges() {
  const auto relevant_breakable_edges =
      instance.const_n().relevant_breakable_edges();
  relevant_reverse_edges.clear();
  for (const auto& e : relevant_breakable_edges) {
    const auto reverse_edge = instance.const_n().get_reverse_edge_index(e);
    if (reverse_edge.has_value()) {
      relevant_reverse_edges.emplace_back(e, reverse_edge.value());
    }
  }
}

// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay,performance-inefficient-string-concatenation)
