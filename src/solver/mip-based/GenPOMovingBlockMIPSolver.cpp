#include "solver/mip-based/GenPOMovingBlockMIPSolver.hpp"

#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "EOMHelper.hpp"
#include "GeneralHelper.hpp"
#include "MultiArray.hpp"
#include "StringHelper.hpp"
#include "gurobi_c++.h"
#include "gurobi_c.h"
#include "plog/Log.h"
#include "plog/Logger.h"
#include "plog/Severity.h"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"
#include "solver/mip-based/GeneralMIPSolver.hpp"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <numeric>
#include <optional>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

using std::size_t;

// NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay,performance-inefficient-string-concatenation,bugprone-unchecked-optional-access)

cda_rail::instances::SolGeneralPerformanceOptimizationInstance
cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::solve(
    const ModelDetail&                 model_detail_input,
    const SolverStrategyMovingBlock&   solver_strategy_input,
    const SolutionSettingsMovingBlock& solution_settings_input, int time_limit,
    bool debug_input, bool overwrite_severity) {
  /**
   * Solves initiated performance optimization problem with moving block
   * signaling/routing. Only breakable edges can use moving block. On all
   * others, only one train is allowed (in practice Flankenschutz can be
   * included this way). Trains are only routed if no route is specified.
   *
   * @param time_limit: time limit for the solver in seconds. If -1, no time
   * limit is set.
   * @param debug_input: if true, the debug output is enabled.
   * @param overwrite_severity: if true, the severity of the log is overwritten
   *
   * @return: respective solution object
   */

  std::optional<LazyCallback> cb;
  if (solver_strategy_input.use_lazy_constraints) {
    cb = LazyCallback(this);
    this->solve_init_general_mip(debug_input, overwrite_severity,
                                 &(cb.value()));
  } else {
    this->solve_init_general_mip(debug_input, overwrite_severity);
  }

  if (!m_instance.get_const_network().is_consistent_for_transformation()) {
    PLOGE << "Instance is not consistent for transformation.";
    throw exceptions::ConsistencyException();
  }

  PLOGI << "Create model";

  const instances::GeneralPerformanceOptimizationInstance old_instance =
      m_instance;
  this->m_instance.discretize_stops();

  this->initialize_variables(solution_settings_input, solver_strategy_input,
                             model_detail_input);

  PLOGD << "Create variables";
  create_variables();
  PLOGD << "Set objective";
  set_objective();
  PLOGD << "Create constraints";
  create_constraints();

  m_model->update();

  PLOGD << "Fix numerical issues with small coefficients";
  // TODO: Can we prevent this from being necessary by rounding at the source.
  // On the other hand, this does not take long to fix.
  // NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  size_t       num_fixed   = 0;
  const double integer_tol = m_model->getEnv().get(GRB_DoubleParam_IntFeasTol);
  auto*        vars_tmp    = m_model->getVars();
  const int    num_vars    = m_model->get(GRB_IntAttr_NumVars);
  for (size_t i = 0; i < num_vars; i++) {
    auto col_v = m_model->getCol(vars_tmp[i]);
    for (size_t j = 0; j < col_v.size(); j++) {
      if (std::abs(col_v.getCoeff(static_cast<int>(j))) < integer_tol &&
          col_v.getCoeff(static_cast<int>(j)) != 0) {
        auto c = col_v.getConstr(static_cast<int>(j));
        m_model->chgCoeff(c, vars_tmp[i], 0);
        num_fixed++;
      }
    }
  }
  // NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  PLOGD << "Fixed " << num_fixed << " coefficients";

  PLOGI << "Model created. Optimize.";
  if (plog::get()->checkSeverity(plog::debug) || time_limit > 0) {
    m_model_created = std::chrono::high_resolution_clock::now();
    m_create_time   = std::chrono::duration_cast<std::chrono::milliseconds>(
                          m_model_created - m_start)
                          .count();

    auto time_left = time_limit - (m_create_time / 1000);
    if (time_left < 0 && time_limit > 0) {
      time_left = 1;
    }
    if (time_limit > 0) {
      m_model->set(GRB_DoubleParam_TimeLimit, static_cast<double>(time_left));
    }
    PLOGD << "Model created in "
          << (static_cast<double>(m_create_time) / 1000.0) << " s";
    if (time_limit > 0) {
      PLOGD << "Time left: " << time_left << " s";
    } else {
      PLOGD << "Time left: "
            << "No Limit";
    }
  }

  if (m_solver_strategy.use_lazy_constraints) {
    m_model->set(GRB_IntParam_LazyConstraints, 1);
  }

  PLOGD << "Set absolute MIP gap to " << m_solver_strategy.abs_mip_gap;
  m_model->set(GRB_DoubleParam_MIPGapAbs, m_solver_strategy.abs_mip_gap);

  m_model->optimize();

  IF_PLOG(plog::debug) {
    m_model_solved = std::chrono::high_resolution_clock::now();
    m_solve_time   = std::chrono::duration_cast<std::chrono::milliseconds>(
                         m_model_solved - m_model_created)
                         .count();
    PLOGD << "Model created in "
          << (static_cast<double>(m_create_time) / 1000.0) << " s";
    PLOGD << "Model solved in " << (static_cast<double>(m_solve_time) / 1000.0)
          << " s";
    PLOGD << "Total time "
          << (static_cast<double>(m_create_time + m_solve_time) / 1000.0)
          << " s";
  }

  instances::SolGeneralPerformanceOptimizationInstance solution(old_instance);
  extract_solution(solution);

  if (m_solution_settings.export_option == ExportOption::ExportLP ||
      m_solution_settings.export_option == ExportOption::ExportSolutionAndLP ||
      m_solution_settings.export_option ==
          ExportOption::ExportSolutionWithInstanceAndLP) {
    PLOGI << "Saving model and solution";
    const std::filesystem::path path = m_solution_settings.path;

    if (!is_directory_and_create(path)) {
      PLOGE << "Could not create directory " << path.string();
      throw exceptions::ExportException("Could not create directory " +
                                        path.string());
    }

    if (m_model->get(GRB_IntAttr_SolCount) > 0) {
      m_model->write((path / (m_solution_settings.name + ".json")).string());
    }

    PLOGD << "Add " << m_lazy_constraints.size() << " lazy constraints";
    for (size_t i = 0; i < m_lazy_constraints.size(); i++) {
      m_model->addConstr(m_lazy_constraints.at(i), "Lazy" + std::to_string(i));
    }
    m_model->update();

    m_model->write((path / (m_solution_settings.name + ".mps")).string());
  }

  if (m_solution_settings.export_option == ExportOption::ExportSolution ||
      m_solution_settings.export_option ==
          ExportOption::ExportSolutionWithInstance ||
      m_solution_settings.export_option == ExportOption::ExportSolutionAndLP ||
      m_solution_settings.export_option ==
          ExportOption::ExportSolutionWithInstanceAndLP) {
    const bool export_instance =
        (m_solution_settings.export_option ==
             ExportOption::ExportSolutionWithInstance ||
         m_solution_settings.export_option ==
             ExportOption::ExportSolutionWithInstanceAndLP);
    PLOGI << "Saving solution";
    std::filesystem::path path = m_solution_settings.path;
    path /= m_solution_settings.name;
    solution.export_solution(path.parent_path(), path.filename().string(),
                             export_instance, {});
  }

  cleanup();

  this->m_instance = old_instance;

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
  m_vars["t_front_arrival"]   = MultiArray<GRBVar>(m_num_tr, m_num_vertices);
  m_vars["t_front_departure"] = MultiArray<GRBVar>(m_num_tr, m_num_vertices);
  m_vars["t_rear_departure"]  = MultiArray<GRBVar>(m_num_tr, m_num_vertices);
  m_vars["waiting_at_vertex"] = MultiArray<GRBVar>(m_num_tr, m_num_vertices);
  m_vars["t_ttd_departure"]   = MultiArray<GRBVar>(m_num_tr, m_num_ttd);

  for (size_t tr = 0; tr < m_num_tr; tr++) {
    const auto& tr_name =
        m_instance.get_const_train_list().get_train(tr).get_name();
    for (const auto v : m_instance.vertices_used_by_train(
             tr, m_model_detail.fix_routes, false)) {
      const auto& v_name = m_instance.get_const_network().get_vertex(v).name;
      m_vars["t_front_arrival"](tr, v) = m_model->addVar(
          0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS,
          "t_front_arrival_" + sanitize(tr_name) + "_" + sanitize(v_name));
      m_vars["t_front_departure"](tr, v) = m_model->addVar(
          0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS,
          "t_front_departure_" + sanitize(tr_name) + "_" + sanitize(v_name));
      m_vars["t_rear_departure"](tr, v) = m_model->addVar(
          0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS,
          "t_rear_departure_" + sanitize(tr_name) + "_" + sanitize(v_name));
      m_vars["waiting_at_vertex"](tr, v) = m_model->addVar(
          0.0, 1.0, 0.0, GRB_BINARY,
          "waiting_at_vertex_" + sanitize(tr_name) + "_" + sanitize(v_name));
    }
    for (const auto& ttd : m_instance.sections_used_by_train(
             tr, m_ttd_sections, m_model_detail.fix_routes, false)) {
      m_vars["t_ttd_departure"](tr, ttd) = m_model->addVar(
          0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS,
          "t_ttd_departure_" + sanitize(tr_name) + "_" + std::to_string(ttd));
    }
  }
}

void cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::
    create_general_edge_variables() {
  m_vars["x"]         = MultiArray<GRBVar>(m_num_tr, m_num_edges);
  m_vars["order"]     = MultiArray<GRBVar>(m_num_tr, m_num_tr, m_num_edges);
  m_vars["x_ttd"]     = MultiArray<GRBVar>(m_num_tr, m_num_ttd);
  m_vars["order_ttd"] = MultiArray<GRBVar>(m_num_tr, m_num_tr, m_num_ttd);

  for (size_t tr = 0; tr < m_num_tr; tr++) {
    const auto& tr_name =
        m_instance.get_const_train_list().get_train(tr).get_name();
    for (const auto e :
         m_instance.edges_used_by_train(tr, m_model_detail.fix_routes, false)) {
      m_vars["x"](tr, e) = m_model->addVar(
          0.0, 1.0, 0.0, GRB_BINARY,
          "x_" + sanitize(tr_name) + "_" +
              sanitize(m_instance.get_const_network().get_edge_name(e)));
    }
    for (const auto& ttd : m_instance.sections_used_by_train(
             tr, m_ttd_sections, m_model_detail.fix_routes, false)) {
      m_vars["x_ttd"](tr, ttd) = m_model->addVar(0.0, 1.0, 0.0, GRB_BINARY,
                                                 "x_ttd_" + sanitize(tr_name) +
                                                     "_" + std::to_string(ttd));
    }
  }
  for (size_t e = 0; e < m_num_edges; e++) {
    const auto tr_on_e =
        m_instance.all_trains_on_edge(e, m_model_detail.fix_routes, false);
    const auto& e_name = m_instance.get_const_network().get_edge_name(e);
    for (const auto& tr1 : tr_on_e) {
      const auto& tr1_name =
          m_instance.get_const_train_list().get_train(tr1).get_name();
      for (const auto& tr2 : tr_on_e) {
        if (tr1 != tr2) {
          const auto& tr2_name =
              m_instance.get_const_train_list().get_train(tr2).get_name();
          m_vars["order"](tr1, tr2, e) =
              m_model->addVar(0.0, 1.0, 0.0, GRB_BINARY,
                              "order_" + sanitize(tr1_name) + "_" +
                                  sanitize(tr2_name) + "_" + sanitize(e_name));
        }
      }
    }
  }
  for (size_t ttd = 0; ttd < m_num_ttd; ttd++) {
    const auto tr_on_ttd = m_instance.trains_in_section(
        m_ttd_sections.at(ttd), m_model_detail.fix_routes, false);
    for (const auto& tr1 : tr_on_ttd) {
      const auto& tr1_name =
          m_instance.get_const_train_list().get_train(tr1).get_name();
      for (const auto& tr2 : tr_on_ttd) {
        if (tr1 != tr2) {
          const auto& tr2_name =
              m_instance.get_const_train_list().get_train(tr2).get_name();
          m_vars["order_ttd"](tr1, tr2, ttd) = m_model->addVar(
              0.0, 1.0, 0.0, GRB_BINARY,
              "order_ttd_" + sanitize(tr1_name) + "_" + sanitize(tr2_name) +
                  "_" + std::to_string(ttd));
        }
      }
    }
  }
}

void cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::
    create_stop_variables() {
  size_t max_num_stops = 0;
  for (size_t tr = 0; tr < m_num_tr; tr++) {
    max_num_stops = std::max(
        max_num_stops, m_instance.get_const_schedule(tr).get_stops().size());
  }
  m_vars["stop"] = MultiArray<GRBVar>(m_num_tr, max_num_stops, m_num_vertices);
  m_vars["service_delay"] = MultiArray<GRBVar>(m_num_tr, max_num_stops);

  for (size_t tr = 0; tr < m_num_tr; tr++) {
    const auto& tr_name =
        m_instance.get_const_train_list().get_train(tr).get_name();
    for (size_t stop = 0;
         stop < m_instance.get_const_schedule(tr).get_stops().size(); stop++) {
      const auto& stop_name = m_instance.get_const_schedule(tr)
                                  .get_stops()
                                  .at(stop)
                                  .get_station()
                                  .name;
      const auto& stop_data = m_tr_stop_data.at(tr).at(stop);
      for (const auto& [v, edges] : stop_data) {
        m_vars["stop"](tr, stop, v) = m_model->addVar(
            0.0, 1.0, 0.0, GRB_BINARY,
            "stop_" + sanitize(tr_name) + "_" + sanitize(stop_name) + "_" +
                sanitize(m_instance.get_const_network().get_vertex(v).name));
      }
      m_vars["service_delay"](tr, stop) = m_model->addVar(
          0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS,
          "service_delay_" + sanitize(tr_name) + "_" + sanitize(stop_name));
    }
  }
}

void cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::
    create_velocity_extended_variables() {
  const auto max_velocity_extension_size =
      get_maximal_velocity_extension_size();
  m_vars["y"] =
      MultiArray<GRBVar>(m_num_tr, m_num_edges, max_velocity_extension_size,
                         max_velocity_extension_size);

  for (size_t tr = 0; tr < m_num_tr; tr++) {
    const auto& train = m_instance.get_const_train_list().get_train(tr);
    for (const auto e :
         m_instance.edges_used_by_train(tr, m_model_detail.fix_routes, false)) {
      const auto& edge      = m_instance.get_const_network().get_edge(e);
      const auto& edge_name = m_instance.get_const_network().get_edge_name(
          {edge.source, edge.target});
      const auto& v_1 = m_velocity_extensions.at(tr).at(edge.source);
      const auto& v_2 = m_velocity_extensions.at(tr).at(edge.target);
      const auto  tmp_max_speed =
          std::min(train.get_max_speed(), edge.max_speed);
      for (size_t i = 0; i < v_1.size(); i++) {
        if (v_1.at(i) > tmp_max_speed) {
          continue;
        }
        for (size_t j = 0; j < v_2.size(); j++) {
          if (v_2.at(j) > tmp_max_speed) {
            continue;
          }
          if (cda_rail::possible_by_eom(
                  v_1.at(i), v_2.at(j), train.get_acceleration(),
                  train.get_deceleration(), edge.length)) {
            m_vars["y"](tr, e, i, j) = m_model->addVar(
                0.0, 1.0, 0.0, GRB_BINARY,
                "y_" + sanitize(train.get_name()) + "_" + sanitize(edge_name) +
                    "_" + std::to_string(v_1.at(i)) + "_" +
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
  m_vars["reverse_order"] =
      MultiArray<GRBVar>(m_num_tr, m_num_tr, m_relevant_reverse_edges.size());

  for (size_t idx = 0; idx < m_relevant_reverse_edges.size(); idx++) {
    const auto& [e1, e2] = m_relevant_reverse_edges.at(idx);
    const auto tr_list   = m_instance.trains_in_section(
        {e1, e2}, m_model_detail.fix_routes, false);
    const auto  e_obj = m_instance.get_const_network().get_edge(e1);
    const auto& v1_name =
        m_instance.get_const_network().get_vertex(e_obj.source).name;
    const auto& v2_name =
        m_instance.get_const_network().get_vertex(e_obj.target).name;
    for (auto const& tr1 : tr_list) {
      const auto& tr1_name =
          m_instance.get_const_train_list().get_train(tr1).get_name();
      for (auto const& tr2 : tr_list) {
        if (tr1 == tr2) {
          continue;
        }
        const auto& tr2_name =
            m_instance.get_const_train_list().get_train(tr2).get_name();
        m_vars["reverse_order"](tr1, tr2, idx) = m_model->addVar(
            0.0, 1.0, 0.0, GRB_BINARY,
            "reverse_order_" + sanitize(tr1_name) + "_" + sanitize(tr2_name) +
                "_" + sanitize(v1_name) + "-" + sanitize(v2_name));
      }
    }
  }
}

void cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::set_objective() {
  m_objective_expr = 0;
  for (size_t tr = 0; tr < m_num_tr; tr++) {
    const auto  exit_node = m_instance.get_const_schedule(tr).get_exit_vertex();
    const auto& min_exit_time =
        m_instance.get_const_schedule(tr).get_exit_time();
    const auto& tr_weight = m_instance.get_train_weight(tr);

    // TODO: Verify objective
    m_objective_expr +=
        tr_weight * (m_vars["t_rear_departure"](tr, exit_node) - min_exit_time);

    auto const& scheduled_stops = m_instance.get_const_schedule(tr).get_stops();
    for (size_t stop_idx = 0; stop_idx < scheduled_stops.size(); stop_idx++) {
      m_objective_expr += m_instance.get_station_delay_weight() * tr_weight /
                          static_cast<double>(scheduled_stops.size()) *
                          m_vars["service_delay"](tr, stop_idx);
    }
  }
  m_model->setObjective(m_objective_expr, GRB_MINIMIZE);
}

void cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::
    create_constraints() {
  PLOGD << "Create general path constraints";
  create_general_path_constraints();
  PLOGD << "Create travel times constraints";
  create_travel_times_constraints();
  PLOGD << "Create basic TTD constraints";
  create_basic_ttd_constraints();
  PLOGD << "Create train rear constraints";
  create_train_rear_constraints();
  PLOGD << "Create stopping constraints";
  create_stopping_constraints();
  if (!m_solver_strategy.use_lazy_constraints) {
    PLOGD << "Create basic order constraints";
    create_basic_order_constraints();
    PLOGD << "Create vertex headway constraints";
    create_vertex_headway_constraints();
    PLOGD << "Create reverse edge constraints";
    create_reverse_edge_constraints();
    if (this->m_model_detail.simplify_headway_constraints) {
      PLOGD << "Create simplified headway constraints";
      create_simplified_headway_constraints();
    } else {
      PLOGD << "Create headway constraints";
      create_headway_constraints();
    }
  }
}

void cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::
    fill_tr_stop_data() {
  m_tr_stop_data.clear();
  m_tr_stop_data.reserve(m_num_tr);

  for (size_t tr = 0; tr < m_num_tr; tr++) {
    std::vector<
        std::vector<std::pair<size_t, std::vector<cda_rail::index_vector>>>>
        tr_data;
    tr_data.reserve(m_instance.get_const_schedule(tr).get_stops().size());
    for (const auto& stop : m_instance.get_const_schedule(tr).get_stops()) {
      tr_data.emplace_back(m_instance.possible_stop_vertices(
          tr, stop.get_station().name,
          m_instance.edges_used_by_train(tr, m_model_detail.fix_routes,
                                         false)));
    }
    m_tr_stop_data.emplace_back(tr_data);
  }
}

void cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::
    fill_velocity_extensions() {
  m_velocity_extensions.clear();
  if (m_model_detail.velocity_refinement_strategy ==
      VelocityRefinementStrategy::None) {
    fill_velocity_extensions_using_none_strategy();
  } else if (m_model_detail.velocity_refinement_strategy ==
             VelocityRefinementStrategy::MinOneStep) {
    fill_velocity_extensions_using_min_one_step_strategy();
  } else {
    throw exceptions::InvalidInputException(
        "Velocity refinement strategy not implemented.");
  }
}

void cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::
    fill_velocity_extensions_using_none_strategy() {
  m_velocity_extensions.reserve(m_num_tr);
  for (size_t tr = 0; tr < m_num_tr; tr++) {
    std::vector<std::vector<double>> tr_velocity_extensions;
    tr_velocity_extensions.reserve(m_num_vertices);
    const auto& tr_max_speed =
        m_instance.get_const_train_list().get_train(tr).get_max_speed();
    for (size_t v = 0; v < m_num_vertices; v++) {
      if (m_instance.get_const_schedule(tr).get_entry_vertex() == v) {
        tr_velocity_extensions.emplace_back(std::vector<double>{
            m_instance.get_const_schedule(tr).get_initial_velocity()});
        continue;
      }

      std::vector<double> v_velocity_extensions = {0};
      const double        max_vertex_speed =
          std::min(m_instance.get_const_network().maximal_vertex_speed(
                       v, m_instance.edges_used_by_train(
                              tr, m_model_detail.fix_routes, false)),
                   tr_max_speed);
      double speed = 0;
      while (speed < max_vertex_speed) {
        speed += m_model_detail.max_velocity_delta;
        speed = std::min(speed, max_vertex_speed);
        v_velocity_extensions.emplace_back(speed);
      }
      tr_velocity_extensions.emplace_back(v_velocity_extensions);
    }
    m_velocity_extensions.emplace_back(tr_velocity_extensions);
  }
}

void cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::
    fill_velocity_extensions_using_min_one_step_strategy() {
  m_velocity_extensions.reserve(m_num_tr);
  for (size_t tr = 0; tr < m_num_tr; tr++) {
    std::vector<std::vector<double>> tr_velocity_extensions;
    tr_velocity_extensions.reserve(m_num_vertices);
    const auto& tr_object = m_instance.get_const_train_list().get_train(tr);
    const auto  tr_speed_change =
        std::min(tr_object.get_acceleration(), tr_object.get_deceleration());
    const auto& tr_max_speed = tr_object.get_max_speed();
    const auto& tr_length    = tr_object.get_length();
    for (size_t v = 0; v < m_num_vertices; v++) {
      if (m_instance.get_const_schedule(tr).get_entry_vertex() == v) {
        tr_velocity_extensions.emplace_back(std::vector<double>{
            m_instance.get_const_schedule(tr).get_initial_velocity()});
        continue;
      }

      const double max_vertex_speed =
          std::min(m_instance.get_const_network().maximal_vertex_speed(
                       v, m_instance.edges_used_by_train(
                              tr, m_model_detail.fix_routes, false)),
                   tr_max_speed);
      double min_n_length =
          m_instance.get_const_network().minimal_neighboring_edge_length(v);

      if (min_n_length > tr_length &&
          m_instance.get_const_schedule(tr).get_exit_vertex() == v) {
        min_n_length = tr_length;
      }

      std::vector<double> v_velocity_extensions = {0};
      double              speed                 = 0;
      while (speed < max_vertex_speed) {
        // Buffer, because due to numerics some values might be slightly too
        // big.
        const auto sqrt_tmp = std::max(
            std::sqrt((speed * speed) + (2 * tr_speed_change * min_n_length)) -
                V_MIN,
            speed + V_MIN);
        speed = std::min({speed + m_model_detail.max_velocity_delta, sqrt_tmp,
                          max_vertex_speed});
        v_velocity_extensions.emplace_back(speed);
      }
      tr_velocity_extensions.emplace_back(v_velocity_extensions);
    }
    m_velocity_extensions.emplace_back(tr_velocity_extensions);
  }
}

size_t cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::
    get_maximal_velocity_extension_size() const {
  size_t max_size = 0;
  for (const auto& tr_velocity_extensions : m_velocity_extensions) {
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

  m_num_tr       = m_instance.get_const_train_list().size();
  m_num_edges    = m_instance.get_const_network().number_of_edges();
  m_num_vertices = m_instance.get_const_network().number_of_vertices();
  // max_t                   = m_instance.max_t();
  this->m_solution_settings = solution_settings_input;
  this->m_solver_strategy   = solver_strategy_input;
  this->m_model_detail      = model_detail_input;
  this->m_ttd_sections = m_instance.get_const_network().unbreakable_sections();
  this->m_num_ttd      = this->m_ttd_sections.size();
  this->fill_tr_stop_data();
  this->fill_velocity_extensions();
  this->fill_relevant_reverse_edges();
}

void cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::
    create_general_path_constraints() {
  for (size_t tr = 0; tr < m_num_tr; tr++) {
    const auto& tr_object = m_instance.get_const_train_list().get_train(tr);
    for (const auto& e :
         m_instance.edges_used_by_train(tr, m_model_detail.fix_routes, false)) {
      const auto& edge = m_instance.get_const_network().get_edge(e);
      const auto& source_obj =
          m_instance.get_const_network().get_vertex(edge.source);
      const auto& target_obj =
          m_instance.get_const_network().get_vertex(edge.target);
      const auto&      v1_values = m_velocity_extensions.at(tr).at(edge.source);
      const auto&      v2_values = m_velocity_extensions.at(tr).at(edge.target);
      const GRBLinExpr lhs       = m_vars["x"](tr, e);
      GRBLinExpr       rhs       = 0;
      const auto       tmp_max_speed =
          std::min(tr_object.get_max_speed(), edge.max_speed);
      for (size_t i = 0; i < v1_values.size(); i++) {
        if (v1_values.at(i) > tmp_max_speed) {
          continue;
        }
        for (size_t j = 0; j < v2_values.size(); j++) {
          if (v2_values.at(j) > tmp_max_speed) {
            continue;
          }
          if (cda_rail::possible_by_eom(v1_values.at(i), v2_values.at(j),
                                        tr_object.get_acceleration(),
                                        tr_object.get_deceleration(),
                                        edge.length)) {
            rhs += m_vars["y"](tr, e, i, j);
          }
        }
      }
      // Edge is used if one of the velocity extended arcs is used
      m_model->addConstr(lhs == rhs, "aggregate_edge_velocity_extension_" +
                                         sanitize(tr_object.get_name()) + "_" +
                                         sanitize(source_obj.name) + "-" +
                                         sanitize(target_obj.name));
    }
    const auto& schedule = m_instance.get_const_schedule(tr);
    const auto& entry    = schedule.get_entry_vertex();
    const auto& exit     = schedule.get_exit_vertex();
    const auto  edges_used_by_train =
        m_instance.edges_used_by_train(tr, m_model_detail.fix_routes, false);
    for (const auto& v : m_instance.vertices_used_by_train(
             tr, m_model_detail.fix_routes, false)) {
      if (v == entry) {
        GRBLinExpr lhs = 0;
        for (const auto& e : m_instance.get_const_network().out_edges(v)) {
          if (std::ranges::contains(edges_used_by_train, e)) {
            lhs += m_vars["x"](tr, e);
          }
        }
        // The entry vertex is only left but not entered
        m_model->addConstr(
            lhs == 1,
            "entry_vertex_" + sanitize(tr_object.get_name()) + "_" +
                sanitize(m_instance.get_const_network().get_vertex(v).name));
      } else if (v == exit) {
        GRBLinExpr lhs = 0;
        for (const auto& e : m_instance.get_const_network().in_edges(v)) {
          if (std::ranges::contains(edges_used_by_train, e)) {
            lhs += m_vars["x"](tr, e);
          }
        }
        // The exit vertex is only entered but not left
        m_model->addConstr(
            lhs == 1,
            "exit_vertex_" + sanitize(tr_object.get_name()) + "_" +
                sanitize(m_instance.get_const_network().get_vertex(v).name));
      } else {
        GRBLinExpr x_in_edges  = 0;
        GRBLinExpr x_out_edges = 0;
        for (const auto& e : m_instance.get_const_network().in_edges(v)) {
          if (std::ranges::contains(edges_used_by_train, e)) {
            x_in_edges += m_vars["x"](tr, e);
          }
        }
        for (const auto& e : m_instance.get_const_network().out_edges(v)) {
          if (std::ranges::contains(edges_used_by_train, e)) {
            x_out_edges += m_vars["x"](tr, e);
          }
        }
        // All other vertices are entered and left at most once
        m_model->addConstr(
            x_in_edges <= 1,
            "in_edges_" + sanitize(tr_object.get_name()) + "_" +
                sanitize(m_instance.get_const_network().get_vertex(v).name));
        m_model->addConstr(
            x_out_edges <= 1,
            "out_edges_" + sanitize(tr_object.get_name()) + "_" +
                sanitize(m_instance.get_const_network().get_vertex(v).name));
        const auto& v1_values = m_velocity_extensions.at(tr).at(v);
        for (size_t i = 0; i < v1_values.size(); i++) {
          GRBLinExpr lhs = 0;
          GRBLinExpr rhs = 0;
          for (const auto& e : m_instance.get_const_network().in_edges(v)) {
            if (std::ranges::contains(edges_used_by_train, e)) {
              const auto& edge = m_instance.get_const_network().get_edge(e);
              const auto& v2_values =
                  m_velocity_extensions.at(tr).at(edge.source);
              const auto tmp_max_speed =
                  std::min(tr_object.get_max_speed(), edge.max_speed);
              if (v1_values.at(i) > tmp_max_speed) {
                continue;
              }
              for (size_t j = 0; j < v2_values.size(); j++) {
                if (v2_values.at(j) > tmp_max_speed) {
                  continue;
                }
                if (cda_rail::possible_by_eom(v2_values.at(j), v1_values.at(i),
                                              tr_object.get_acceleration(),
                                              tr_object.get_deceleration(),
                                              edge.length)) {
                  lhs += m_vars["y"](tr, e, j, i);
                }
              }
            }
          }
          for (const auto& e : m_instance.get_const_network().out_edges(v)) {
            if (std::ranges::contains(edges_used_by_train, e)) {
              const auto& edge = m_instance.get_const_network().get_edge(e);
              const auto  tmp_max_speed =
                  std::min(tr_object.get_max_speed(), edge.max_speed);
              if (v1_values.at(i) > tmp_max_speed) {
                continue;
              }
              const auto& v2_values =
                  m_velocity_extensions.at(tr).at(edge.target);
              for (size_t j = 0; j < v2_values.size(); j++) {
                if (v2_values.at(j) > tmp_max_speed) {
                  continue;
                }
                if (cda_rail::possible_by_eom(v1_values.at(i), v2_values.at(j),
                                              tr_object.get_acceleration(),
                                              tr_object.get_deceleration(),
                                              edge.length)) {
                  rhs += m_vars["y"](tr, e, i, j);
                }
              }
            }
          }
          // And they fulfill a flow condition
          m_model->addConstr(
              lhs == rhs,
              "vertex_velocity_extension_flow_condition_" +
                  tr_object.get_name() + "_" +
                  m_instance.get_const_network().get_vertex(v).name + "_" +
                  std::to_string(v1_values.at(i)));
        }
      }
    }

    // Prevent illegal paths
    for (const auto& e :
         m_instance.edges_used_by_train(tr, m_model_detail.fix_routes, false)) {
      const auto& e_object  = m_instance.get_const_network().get_edge(e);
      const auto& v2        = e_object.target;
      const auto& out_edges = m_instance.get_const_network().out_edges(v2);
      const auto& v1_name =
          m_instance.get_const_network().get_vertex(e_object.source).name;
      const auto& v2_name = m_instance.get_const_network().get_vertex(v2).name;
      for (const auto& e2 : out_edges) {
        if (!m_instance.get_const_network().is_valid_successor(e, e2) &&
            std::ranges::contains(edges_used_by_train, e2)) {
          const auto& v3_name =
              m_instance.get_const_network()
                  .get_vertex(
                      m_instance.get_const_network().get_edge(e2).target)
                  .name;
          m_model->addConstr(m_vars["x"](tr, e) + m_vars["x"](tr, e2) <= 1,
                             "illegal_path_" + sanitize(tr_object.get_name()) +
                                 "_" + sanitize(v1_name) + "-" +
                                 sanitize(v2_name) + "-" + sanitize(v3_name));
        }
      }
    }
  }
}

void cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::
    create_travel_times_constraints() {
  for (size_t tr = 0; tr < m_num_tr; tr++) {
    const auto& tr_object = m_instance.get_const_train_list().get_train(tr);
    for (const auto& e :
         m_instance.edges_used_by_train(tr, m_model_detail.fix_routes, false)) {
      const auto& edge      = m_instance.get_const_network().get_edge(e);
      const auto& v1_values = m_velocity_extensions.at(tr).at(edge.source);
      const auto& v2_values = m_velocity_extensions.at(tr).at(edge.target);
      const auto  tmp_max_speed =
          std::min(tr_object.get_max_speed(), edge.max_speed);
      for (size_t i = 0; i < v1_values.size(); i++) {
        if (v1_values.at(i) > tmp_max_speed) {
          continue;
        }
        for (size_t j = 0; j < v2_values.size(); j++) {
          if (v2_values.at(j) > tmp_max_speed) {
            continue;
          }
          if (cda_rail::possible_by_eom(v1_values.at(i), v2_values.at(j),
                                        tr_object.get_acceleration(),
                                        tr_object.get_deceleration(),
                                        edge.length)) {
            // t_front_arrival >= t_rear_departure + minimal travel time if arc
            // is used
            const auto& min_t_arc = cda_rail::min_travel_time(
                v1_values.at(i), v2_values.at(j), tmp_max_speed,
                tr_object.get_acceleration(), tr_object.get_deceleration(),
                edge.length);
            const auto& max_t_arc = cda_rail::max_travel_time(
                v1_values.at(i), v2_values.at(j), V_MIN,
                tr_object.get_acceleration(), tr_object.get_deceleration(),
                edge.length, edge.breakable);
            m_model->addGenConstrIndicator(
                m_vars["y"](tr, e, i, j), 1,
                m_vars["t_front_arrival"](tr, edge.target) >=
                    m_vars["t_front_departure"](tr, edge.source) + min_t_arc,
                "edge_minimal_travel_time_" + sanitize(tr_object.get_name()) +
                    "_" +
                    m_instance.get_const_network()
                        .get_vertex(edge.source)
                        .name +
                    "-" +
                    m_instance.get_const_network()
                        .get_vertex(edge.target)
                        .name +
                    "_" + std::to_string(v1_values.at(i)) + "-" +
                    std::to_string(v2_values.at(j)));

            if (max_t_arc >= GRB_INFINITY) {
              continue;
            }

            // t_front_arrival <= t_rear_departure + maximal travel time if arc
            // is used
            m_model->addGenConstrIndicator(
                m_vars["y"](tr, e, i, j), 1,
                m_vars["t_front_arrival"](tr, edge.target) <=
                    m_vars["t_front_departure"](tr, edge.source) + max_t_arc,
                "edge_maximal_travel_time_" + sanitize(tr_object.get_name()) +
                    "_" +
                    m_instance.get_const_network()
                        .get_vertex(edge.source)
                        .name +
                    "-" +
                    m_instance.get_const_network()
                        .get_vertex(edge.target)
                        .name +
                    "_" + std::to_string(v1_values.at(i)) + "-" +
                    std::to_string(v2_values.at(j)));
          }
        }
      }
    }

    const auto e_used_tr =
        m_instance.edges_used_by_train(tr, m_model_detail.fix_routes, false);
    for (const auto& v : m_instance.vertices_used_by_train(
             tr, m_model_detail.fix_routes, false)) {
      // t_front_departure >= t_front_arrival
      m_model->addConstr(
          m_vars["t_front_departure"](tr, v) >=
              m_vars["t_front_arrival"](tr, v),
          "tr_dep_after_arrival_" + sanitize(tr_object.get_name()) + "_" +
              sanitize(m_instance.get_const_network().get_vertex(v).name));

      if (m_velocity_extensions.at(tr).at(v).at(0) != 0) {
        continue;
      }

      // t_front_departure <= t_front_arrival if train cannot be stopped at
      // vertex v
      GRBLinExpr speed_0_arcs = 0;
      for (const auto& e_in : m_instance.get_const_network().in_edges(v)) {
        if (std::ranges::contains(e_used_tr, e_in)) {
          const auto& e_in_object =
              m_instance.get_const_network().get_edge(e_in);
          const auto& v1_velocities =
              m_velocity_extensions.at(tr).at(e_in_object.source);
          assert(m_velocity_extensions.at(tr).at(v).at(0) == 0);
          const auto tmp_max_speed =
              std::min(tr_object.get_max_speed(), e_in_object.max_speed);
          for (size_t i = 0; i < v1_velocities.size(); i++) {
            if (v1_velocities.at(i) > tmp_max_speed) {
              continue;
            }
            if (cda_rail::possible_by_eom(
                    v1_velocities.at(i), 0, tr_object.get_acceleration(),
                    tr_object.get_deceleration(), e_in_object.length)) {
              speed_0_arcs += m_vars["y"](tr, e_in, i, 0);
            }
          }
        }
      }
      for (const auto& e_out : m_instance.get_const_network().out_edges(v)) {
        if (std::ranges::contains(e_used_tr, e_out)) {
          const auto& e_out_object =
              m_instance.get_const_network().get_edge(e_out);
          const auto& v2_velocities =
              m_velocity_extensions.at(tr).at(e_out_object.target);
          assert(m_velocity_extensions.at(tr).at(v).at(0) == 0);
          const auto tmp_max_speed =
              std::min(tr_object.get_max_speed(), e_out_object.max_speed);
          for (size_t i = 0; i < v2_velocities.size(); i++) {
            if (v2_velocities.at(i) > tmp_max_speed) {
              continue;
            }
            if (cda_rail::possible_by_eom(
                    0, v2_velocities.at(i), tr_object.get_acceleration(),
                    tr_object.get_deceleration(), e_out_object.length)) {
              speed_0_arcs += m_vars["y"](tr, e_out, 0, i);
            }
          }
        }
      }
      m_model->addConstr(
          m_vars["waiting_at_vertex"](tr, v) <= speed_0_arcs,
          "tr_can_only_stop_by_using_stopping_edge_" +
              sanitize(tr_object.get_name()) + "_" +
              sanitize(m_instance.get_const_network().get_vertex(v).name));
      m_model->addGenConstrIndicator(
          m_vars["waiting_at_vertex"](tr, v), 0,
          m_vars["t_front_departure"](tr, v) <=
              m_vars["t_front_arrival"](tr, v),
          "tr_cannot_stop_unless_allowed_" + sanitize(tr_object.get_name()) +
              "_" +
              sanitize(m_instance.get_const_network().get_vertex(v).name));
    }
  }
}

void cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::
    create_basic_order_constraints() {
  for (size_t e = 0; e < m_num_edges; e++) {
    const auto& tr_on_edge =
        m_instance.all_trains_on_edge(e, m_model_detail.fix_routes, false);
    const auto e_obj = m_instance.get_const_network().get_edge(e);
    const auto v1    = m_instance.get_const_network().get_vertex(e_obj.source);
    const auto v2    = m_instance.get_const_network().get_vertex(e_obj.target);
    for (const auto& tr1 : tr_on_edge) {
      for (const auto& tr2 : tr_on_edge) {
        if (tr1 == tr2) {
          continue;
        }

        m_model->addConstr(
            m_vars["order"](tr1, tr2, e) + m_vars["order"](tr2, tr1, e) <=
                0.5 * (m_vars["x"](tr1, e) + m_vars["x"](tr2, e)),
            "edge_order_1_" +
                sanitize(m_instance.get_const_train_list()
                             .get_train(tr1)
                             .get_name()) +
                "_" +
                sanitize(m_instance.get_const_train_list()
                             .get_train(tr2)
                             .get_name()) +
                "_" + sanitize(v1.name) + "-" + sanitize(v2.name));

        m_model->addConstr(
            m_vars["order"](tr1, tr2, e) + m_vars["order"](tr2, tr1, e) >=
                m_vars["x"](tr1, e) + m_vars["x"](tr2, e) - 1,
            "edge_order_2_" +
                sanitize(m_instance.get_const_train_list()
                             .get_train(tr1)
                             .get_name()) +
                "_" +
                sanitize(m_instance.get_const_train_list()
                             .get_train(tr2)
                             .get_name()) +
                "_" + sanitize(v1.name) + "-" + sanitize(v2.name));
      }
    }
  }
}

void cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::
    create_train_rear_constraints() {
  for (size_t tr = 0; tr < m_num_tr; tr++) {
    // Rear departure time is equal to front departure time at certain position
    const auto& tr_object = m_instance.get_const_train_list().get_train(tr);
    const auto& schedule  = m_instance.get_const_schedule(tr);
    const auto& exit      = schedule.get_exit_vertex();
    const auto& v_n       = schedule.get_exit_velocity();
    const auto  edges_used_by_train =
        m_instance.edges_used_by_train(tr, m_model_detail.fix_routes, false);

    // NOLINTNEXTLINE(readability-identifier-naming)

    for (const auto& v : m_instance.vertices_used_by_train(
             tr, m_model_detail.fix_routes, false)) {
      if (v == exit) {
        // In case the train has partially left the network use edge case
        // constraints
        const auto v_max_speed =
            m_instance.get_const_network().maximal_vertex_speed(
                v, edges_used_by_train);
        const auto v_exit_velocities = m_velocity_extensions.at(tr).at(v);
        // t_rear_departure(v) >= t_front_departure(v)  + min_t selected by
        // incoming edge speed
        const auto in_edges = m_instance.get_const_network().in_edges(v);
        cda_rail::index_vector relevant_in_edges = {};
        for (const auto& e : in_edges) {
          if (std::ranges::contains(edges_used_by_train, e)) {
            relevant_in_edges.push_back(e);
          }
        }
        GRBLinExpr min_travel_time_expr = 0;
        GRBLinExpr max_travel_time_expr = 0;
        for (size_t i = 0; i < v_exit_velocities.size(); i++) {
          const auto& v_exit_velocity = v_exit_velocities.at(i);
          if (cda_rail::possible_by_eom(
                  v_exit_velocity, v_n, tr_object.get_acceleration(),
                  tr_object.get_deceleration(), tr_object.get_length())) {
            const auto min_t_to_full_exit = cda_rail::min_travel_time(
                v_exit_velocity, v_n, v_max_speed, tr_object.get_acceleration(),
                tr_object.get_deceleration(), tr_object.get_length());
            const auto max_t_to_full_exit = cda_rail::max_travel_time(
                v_exit_velocity, v_n, V_MIN, tr_object.get_acceleration(),
                tr_object.get_deceleration(), tr_object.get_length(), false);
            for (const auto& e_in : relevant_in_edges) {
              const auto& e_in_object =
                  m_instance.get_const_network().get_edge(e_in);
              const auto tmp_max_speed =
                  std::min(tr_object.get_max_speed(), e_in_object.max_speed);
              if (v_exit_velocity > tmp_max_speed) {
                continue;
              }
              const auto& v1_velocities =
                  m_velocity_extensions.at(tr).at(e_in_object.source);
              for (size_t j = 0; j < v1_velocities.size(); j++) {
                if (v1_velocities.at(j) > tmp_max_speed) {
                  continue;
                }
                if (cda_rail::possible_by_eom(
                        v1_velocities.at(j), v_exit_velocity,
                        tr_object.get_acceleration(),
                        tr_object.get_deceleration(), e_in_object.length)) {
                  min_travel_time_expr +=
                      m_vars["y"](tr, e_in, j, i) * min_t_to_full_exit;
                  max_travel_time_expr +=
                      m_vars["y"](tr, e_in, j, i) * max_t_to_full_exit;
                }
              }
            }
          } else {
            // Velocity is not possible at exit vertex
            for (const auto& e_in : relevant_in_edges) {
              const auto& e_in_object =
                  m_instance.get_const_network().get_edge(e_in);
              const auto& e_in_source_vertex =
                  m_instance.get_const_network().get_vertex(e_in_object.source);
              const auto tmp_max_speed =
                  std::min(tr_object.get_max_speed(), e_in_object.max_speed);
              if (v_exit_velocity > tmp_max_speed) {
                continue;
              }
              const auto& v1_velocities =
                  m_velocity_extensions.at(tr).at(e_in_object.source);
              for (size_t j = 0; j < v1_velocities.size(); j++) {
                if (v1_velocities.at(j) > tmp_max_speed) {
                  continue;
                }
                if (cda_rail::possible_by_eom(
                        v1_velocities.at(j), v_exit_velocity,
                        tr_object.get_acceleration(),
                        tr_object.get_deceleration(), e_in_object.length)) {
                  m_model->addConstr(
                      m_vars["y"](tr, e_in, j, i) == 0,
                      "y_exit_velocity_" + std::to_string(v_exit_velocity) +
                          "_not_possible_from_" +
                          std::to_string(v1_velocities.at(j)) + "_at_" +
                          sanitize(e_in_source_vertex.name) + "_tr_" +
                          sanitize(tr_object.get_name()));
                }
              }
            }
          }
        }
        m_model->addConstr(
            m_vars["t_rear_departure"](tr, v) >=
                m_vars["t_front_departure"](tr, v) + min_travel_time_expr,
            "rear_departure_vertex_c1_" + sanitize(tr_object.get_name()) + "_" +
                sanitize(m_instance.get_const_network().get_vertex(v).name));
        m_model->addConstr(
            m_vars["t_rear_departure"](tr, v) <=
                m_vars["t_front_departure"](tr, v) + max_travel_time_expr,
            "rear_departure_vertex_c2_" + sanitize(tr_object.get_name()) + "_" +
                m_instance.get_const_network()
                    .get_vertex(v)
                    .name); // not needed because objective
                            // pushes rear departure down
      } else {
        // Otherwise deduce limits from last path edge
        const auto possible_paths =
            m_instance.get_const_network()
                .all_paths_of_length_starting_in_vertex(
                    v, tr_object.get_length(), exit, edges_used_by_train);
        for (size_t p_ind = 0; p_ind < possible_paths.size(); p_ind++) {
          const auto&  p = possible_paths.at(p_ind);
          const double p_len_last_vertex =
              std::accumulate(p.begin(), p.end() - 1, 0.0,
                              [this](double sum, const auto& edge_index) {
                                return sum + m_instance.get_const_network()
                                                 .get_edge(edge_index)
                                                 .length;
                              });
          assert(p_len_last_vertex >= 0);
          assert(p_len_last_vertex <= tr_object.get_length());
          const auto& last_edge = p.back();
          const auto& last_edge_obj =
              m_instance.get_const_network().get_edge(last_edge);

          auto const helper_binary = m_model->addVar(
              0.0, 1.0, 0.0, GRB_BINARY,
              "train_rear_constraint_path_binary_helper_" +
                  sanitize(tr_object.get_name()) + "_" +
                  sanitize(m_instance.get_const_network().get_vertex(v).name) +
                  "_" + std::to_string(p_ind));
          GRBLinExpr path_indicator_sum = 1;
          for (const auto& e_p : p) {
            path_indicator_sum += m_vars["x"](tr, e_p) - 1;
          }
          m_model->addConstr(
              helper_binary >= path_indicator_sum,
              "train_rear_constraint_path_binary_helper_1_" +
                  sanitize(tr_object.get_name()) + "_" +
                  sanitize(m_instance.get_const_network().get_vertex(v).name) +
                  "_" + std::to_string(p_ind));

          if (last_edge_obj.target == exit &&
              last_edge_obj.length + p_len_last_vertex <
                  tr_object.get_length()) {
            // The train has partially left the network through the last edge
            const auto outside_len = tr_object.get_length() -
                                     p_len_last_vertex - last_edge_obj.length;
            assert(outside_len >= 0);
            assert(outside_len <= tr_object.get_length());
            const auto& v_exit_velocities =
                m_velocity_extensions.at(tr).at(exit);

            const auto tr_max_speed_tmp =
                std::min({tr_object.get_max_speed(),
                          m_instance.get_const_network().maximal_vertex_speed(
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
                      v_exit_velocity, v_n, tr_object.get_acceleration(),
                      tr_object.get_deceleration(), tr_object.get_length())) {
                const auto min_t_to_required_pos =
                    cda_rail::min_travel_time_from_start(
                        v_exit_velocity, v_n, tr_max_speed_tmp,
                        tr_object.get_acceleration(),
                        tr_object.get_deceleration(), tr_object.get_length(),
                        outside_len);
                const auto max_t_to_required_pos =
                    cda_rail::max_travel_time_from_start(
                        v_exit_velocity, v_n, V_MIN,
                        tr_object.get_acceleration(),
                        tr_object.get_deceleration(), tr_object.get_length(),
                        outside_len, false);
                if (v_exit_velocity > tr_max_speed_tmp) {
                  continue;
                }
                const auto& v1_velocities =
                    m_velocity_extensions.at(tr).at(last_edge_obj.source);
                for (size_t j = 0; j < v1_velocities.size(); j++) {
                  if (v1_velocities.at(j) > tr_max_speed_tmp) {
                    continue;
                  }
                  if (cda_rail::possible_by_eom(
                          v1_velocities.at(j), v_exit_velocity,
                          tr_object.get_acceleration(),
                          tr_object.get_deceleration(), last_edge_obj.length)) {
                    min_travel_time_expr += m_vars["y"](tr, last_edge, j, i) *
                                            min_t_to_required_pos;
                    max_travel_time_expr += m_vars["y"](tr, last_edge, j, i) *
                                            max_t_to_required_pos;
                  }
                }
              }
            }

            // GRBLinExpr lhs = m_vars["t_rear_departure"](tr, v) +
            m_model->addGenConstrIndicator(
                helper_binary, 1,
                m_vars["t_rear_departure"](tr, v) >=
                    m_vars["t_front_departure"](tr, exit) +
                        min_travel_time_expr,
                "rear_departure_half_leaving_1_" +
                    sanitize(tr_object.get_name()) + "_" +
                    sanitize(
                        m_instance.get_const_network().get_vertex(v).name) +
                    "_" + std::to_string(p_ind));
            // Removed one constraint since t_rear is pushed down anyway
            /**m_model->addConstr(lhs - bigM <= m_vars["t_front_departure"](tr,
               exit) + max_travel_time_expr, "rear_departure_half_leaving_2_" +
               sanitize(tr_object.get_name()) +
                                 "_" +
               sanitize(m_instance.get_const_network().get_vertex(v).name) +
                                 "_" + std::to_string(p_ind));**/

          } else {
            // The relevant point is on an actual edge
            assert(last_edge_obj.length + p_len_last_vertex >=
                   tr_object.get_length());

            const auto rel_pt_on_edge =
                tr_object.get_length() - p_len_last_vertex;
            const auto v_0_velocities =
                m_velocity_extensions.at(tr).at(last_edge_obj.source);
            const auto v_1_velocities =
                m_velocity_extensions.at(tr).at(last_edge_obj.target);

            if (rel_pt_on_edge + 1e-6 >= last_edge_obj.length) {
              // Directly use corresponding variable
              m_model->addGenConstrIndicator(
                  helper_binary, 1,
                  m_vars["t_rear_departure"](tr, v) >=
                      m_vars["t_front_departure"](tr, last_edge_obj.target),
                  "rear_departure_2_" + sanitize(tr_object.get_name()) + "_" +
                      sanitize(
                          m_instance.get_const_network().get_vertex(v).name) +
                      "_" + std::to_string(p_ind));
            } else {
              // Only in this case there is no corresponding variable. Note that
              // objective pushes rear departure down.
              GRBLinExpr t_ref_1 =
                  m_vars["t_front_departure"](tr, last_edge_obj.source);
              GRBLinExpr t_ref_2 =
                  m_vars["t_front_arrival"](tr, last_edge_obj.target);
              const auto v_max_rel_e =
                  std::min(last_edge_obj.max_speed, tr_object.get_max_speed());

              auto const t_ref_2_relevant = m_model->addVar(
                  0.0, 1.0, 0.0, GRB_BINARY,
                  "train_rear_constraint_path_t_ref_2_relevant_helper_" +
                      sanitize(tr_object.get_name()) + "_" +
                      sanitize(
                          m_instance.get_const_network().get_vertex(v).name) +
                      "_" + std::to_string(p_ind));
              GRBLinExpr t_ref_2_relevant_sum = helper_binary;

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
                          tr_object.get_acceleration(),
                          tr_object.get_deceleration(), last_edge_obj.length)) {
                    t_ref_1 += m_vars["y"](tr, last_edge, i, j) *
                               cda_rail::min_travel_time_from_start(
                                   v_0_velocities.at(i), v_1_velocities.at(j),
                                   v_max_rel_e, tr_object.get_acceleration(),
                                   tr_object.get_deceleration(),
                                   last_edge_obj.length, rel_pt_on_edge);
                    const auto max_travel_time =
                        cda_rail::max_travel_time_to_end(
                            v_0_velocities.at(i), v_1_velocities.at(j), V_MIN,
                            tr_object.get_acceleration(),
                            tr_object.get_deceleration(), last_edge_obj.length,
                            rel_pt_on_edge, last_edge_obj.breakable);
                    if (max_travel_time >= GRB_INFINITY) {
                      t_ref_2_relevant_sum -= m_vars["y"](tr, last_edge, i, j);
                    } else {
                      t_ref_2 -=
                          m_vars["y"](tr, last_edge, i, j) * max_travel_time;
                    }
                  }
                }
              }

              m_model->addConstr(
                  t_ref_2_relevant >= t_ref_2_relevant_sum,
                  "train_rear_constraint_path_t_ref_2_relevant_helper_1_" +
                      sanitize(tr_object.get_name()) + "_" +
                      sanitize(
                          m_instance.get_const_network().get_vertex(v).name) +
                      "_" + std::to_string(p_ind));

              m_model->addGenConstrIndicator(
                  helper_binary, 1,
                  m_vars["t_rear_departure"](tr, v) >= t_ref_1,
                  "rear_departure_1_" + sanitize(tr_object.get_name()) + "_" +
                      sanitize(
                          m_instance.get_const_network().get_vertex(v).name) +
                      "_" + std::to_string(p_ind));
              m_model->addGenConstrIndicator(
                  t_ref_2_relevant, 1,
                  m_vars["t_rear_departure"](tr, v) >= t_ref_2,
                  "rear_departure_2_" + sanitize(tr_object.get_name()) + "_" +
                      sanitize(
                          m_instance.get_const_network().get_vertex(v).name) +
                      "_" + std::to_string(p_ind));
            }
          }
        }
      }
    }
  }
}

void cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::
    create_stopping_constraints() {
  for (size_t tr = 0; tr < m_num_tr; tr++) {
    const auto& tr_object = m_instance.get_const_train_list().get_train(tr);
    // NOLINTNEXTLINE(readability-identifier-naming)

    // Stop at exactly one stop using lhs
    const auto& tr_schedule = m_instance.get_const_schedule(tr);
    const auto& tr_stops    = tr_schedule.get_stops();
    for (size_t stop = 0; stop < tr_stops.size(); stop++) {
      const auto& stop_data         = m_tr_stop_data.at(tr).at(stop);
      const auto& stop_object       = tr_stops.at(stop);
      const auto& stop_station_name = stop_object.get_station().name;
      GRBLinExpr  lhs               = 0;
      for (const auto& [v, paths] : stop_data) {
        lhs += m_vars["stop"](tr, stop, v);

        // If stopped then t_front_departure - t_front_arrival >= stop_time,
        // otherwise unconstrained Hence, >= stop_time * stop
        m_model->addConstr(
            m_vars["t_front_departure"](tr, v) -
                    m_vars["t_front_arrival"](tr, v) >=
                stop_object.get_service_duration() *
                    m_vars["stop"](tr, stop, v),
            "min_stop_time_" + sanitize(tr_object.get_name()) + "_" +
                sanitize(stop_station_name) + "_vertex_" +
                sanitize(m_instance.get_const_network().get_vertex(v).name));

        // If stopped cannot leave before earliest departure time (kicks in if
        // train arrives too early)
        m_model->addConstr(
            m_vars["t_front_departure"](tr, v) >=
                stop_object.get_earliest_departure() *
                    m_vars["stop"](tr, stop, v),
            "earliest_departure_time_" + sanitize(tr_object.get_name()) + "_" +
                sanitize(stop_station_name) + "_vertex_" +
                sanitize(m_instance.get_const_network().get_vertex(v).name));

        // TODO: Check semantics objective
        // Add station delay for objective use (only lb, since objective pushes
        // down)
        m_model->addGenConstrIndicator(
            m_vars["stop"](tr, stop, v), 1,
            m_vars["service_delay"](tr, stop) >=
                m_vars["t_front_arrival"](tr, v) -
                    stop_object.get_service_time(),
            "service_delay_" + sanitize(tr_object.get_name()) + "_" +
                sanitize(stop_station_name) + "_vertex_" +
                sanitize(m_instance.get_const_network().get_vertex(v).name));

        // Train can only stop if one of the valid edge paths is used
        GRBLinExpr path_expr = 0;
        for (size_t p_index = 0; p_index < paths.size(); p_index++) {
          const auto& p = paths.at(p_index);
          // The following variable should be binary, however since only one
          // direction of inference is needed, continuous should suffice
          const auto tmp_var = m_model->addVar(
              0.0, 1.0, 0.0, GRB_CONTINUOUS,
              "stop_path_" + sanitize(tr_object.get_name()) + "_" +
                  sanitize(stop_station_name) + "_vertex_" +
                  sanitize(m_instance.get_const_network().get_vertex(v).name) +
                  "_path_" + std::to_string(p_index));
          path_expr += tmp_var;
          for (const auto& e : p) {
            m_model->addConstr(
                tmp_var <= m_vars["x"](tr, e),
                "stop_path_" + sanitize(tr_object.get_name()) + "_" +
                    sanitize(stop_station_name) + "_vertex_" +
                    sanitize(
                        m_instance.get_const_network().get_vertex(v).name) +
                    "_path_" + std::to_string(p_index) + "_edge_" +
                    std::to_string(e));
          }
          m_model->addConstr(
              m_vars["stop"](tr, stop, v) >= tmp_var,
              "use_path_only_if_stopped_" + sanitize(tr_object.get_name()) +
                  "_" + sanitize(stop_station_name) + "_vertex_" +
                  sanitize(m_instance.get_const_network().get_vertex(v).name) +
                  "_path_" + std::to_string(p_index));
        }
        m_model->addConstr(
            m_vars["stop"](tr, stop, v) <= path_expr,
            "stop_only_if_path_is_used_" + sanitize(tr_object.get_name()) +
                "_" + sanitize(stop_station_name) + "_vertex_" +
                sanitize(m_instance.get_const_network().get_vertex(v).name));
      }
      m_model->addConstr(
          lhs == 1,
          "stop_at_one_vertex_" +
              sanitize(
                  m_instance.get_const_train_list().get_train(tr).get_name()) +
              "_" + sanitize(stop_station_name));
    }

    // Initial
    m_model->addConstr(
        m_vars["t_front_arrival"](tr, tr_schedule.get_entry_vertex()) >=
            tr_schedule.get_entry_time(),
        "initial_arrival_time_lb_" + sanitize(tr_object.get_name()));

    // TODO: Add ub if no late entry allowed
    /**m_model->addConstr(
        m_vars["t_front_arrival"](tr, tr_schedule.get_entry_vertex()) <=
            tr_schedule.get_entry_time(),
        "initial_arrival_time_ub_" + sanitize(tr_object.get_name()));**/

    // Final
    m_model->addConstr(
        m_vars["t_rear_departure"](tr, tr_schedule.get_exit_vertex()) >=
            tr_schedule.get_exit_time(),
        "final_departure_time_lb_" + sanitize(tr_object.get_name()));
  }
}

void cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::
    create_headway_constraints() {
  for (size_t tr = 0; tr < m_num_tr; tr++) {
    const auto& tr_object = m_instance.get_const_train_list().get_train(tr);
    const auto  tr_used_edges =
        m_instance.edges_used_by_train(tr, m_model_detail.fix_routes, false);
    const auto& tr_schedule_object = m_instance.get_const_schedule(tr);
    const auto& entry_node         = tr_schedule_object.get_entry_vertex();

    for (const auto v : m_instance.vertices_used_by_train(
             tr, m_model_detail.fix_routes, false)) {
      const auto v_velocities = m_velocity_extensions.at(tr).at(v);
      for (size_t v_source_index = 0; v_source_index < v_velocities.size();
           v_source_index++) {
        const auto& vel = v_velocities.at(v_source_index);
        if (vel > tr_object.get_max_speed()) {
          continue;
        }
        const auto bd = vel * vel / (2 * tr_object.get_deceleration());
        // What if bd is outside network. Then relation to point where ma was
        // set to exit node or leave as it is?
        const auto brake_paths =
            m_instance.get_const_network()
                .all_paths_of_length_starting_in_vertex(
                    v, std::max(EPS, bd), {},
                    tr_used_edges); // min EPS so that following edge is
                                    // detected for speed 0
        for (size_t p_index = 0; p_index < brake_paths.size(); p_index++) {
          const auto& p = brake_paths.at(p_index);
          const auto  p_len =
              std::accumulate(p.begin(), p.end(), 0.0,
                              [this](double sum, const auto& edge_index) {
                                return sum + m_instance.get_const_network()
                                                 .get_edge(edge_index)
                                                 .length;
                              });

          // Variables to decide if path was used. First edge must leave from
          // desired velocity extension.
          const GRBLinExpr edge_path_expr = get_edge_path_expr(tr, p, vel);
          const auto       tmp_max_speed  = std::min(
              tr_object.get_max_speed(),
              m_instance.get_const_network().get_edge(p.front()).max_speed);

          const auto tr_on_last_edge = m_instance.all_trains_on_edge(
              p.back(), m_model_detail.fix_routes, false);

          const auto& last_edge_object =
              m_instance.get_const_network().get_edge(p.back());

          for (const auto& tr2 : tr_on_last_edge) {
            if (tr == tr2) {
              continue;
            }

            std::vector<GRBVar> rhs_binary_helper;
            rhs_binary_helper.emplace_back(m_model->addVar(
                0.0, 1.0, 0.0, GRB_BINARY,
                "headway_binary_helper_" + sanitize(tr_object.get_name()) +
                    "_" +
                    sanitize(m_instance.get_const_train_list()
                                 .get_train(tr2)
                                 .get_name()) +
                    "_" +
                    sanitize(
                        m_instance.get_const_network().get_vertex(v).name) +
                    "_" + std::to_string(vel) + "_" + std::to_string(p_index)));
            m_model->addConstr(
                rhs_binary_helper.at(0) >= m_vars["order"](tr, tr2, p.back()) +
                                               edge_path_expr -
                                               static_cast<double>(p.size()),
                "headway_binary_helper_1_" + sanitize(tr_object.get_name()) +
                    "_" +
                    sanitize(m_instance.get_const_train_list()
                                 .get_train(tr2)
                                 .get_name()) +
                    "_" +
                    sanitize(
                        m_instance.get_const_network().get_vertex(v).name) +
                    "_" + std::to_string(vel) + "_" + std::to_string(p_index));

            std::vector<GRBLinExpr> rhs;
            if (p_len + EPS >= bd && p_len - EPS <= bd) {
              // Target vertex is exactly the desired moving authority
              // t_front_departure(tr, v) >= t_rear_departure(tr2, target) if
              // order(tr, tr2, e) = 1 and path p chosen.
              rhs.emplace_back(
                  m_vars["t_rear_departure"](tr2, last_edge_object.target));
            } else {
              assert(p_len > bd && p_len - last_edge_object.length <= bd);
              const auto  target_point = bd - p_len + last_edge_object.length;
              const auto& v_tr2_source_velocities =
                  m_velocity_extensions.at(tr2).at(last_edge_object.source);
              const auto& v_tr2_target_velocities =
                  m_velocity_extensions.at(tr2).at(last_edge_object.target);
              rhs.emplace_back(
                  m_vars["t_rear_departure"](tr2, last_edge_object.source));
              rhs.emplace_back(
                  m_vars["t_rear_departure"](tr2, last_edge_object.target));
              rhs_binary_helper.emplace_back(m_model->addVar(
                  0.0, 1.0, 0.0, GRB_BINARY,
                  "headway_binary_helper_rhs1_relevant_" +
                      sanitize(tr_object.get_name()) + "_" +
                      sanitize(m_instance.get_const_train_list()
                                   .get_train(tr2)
                                   .get_name()) +
                      "_" +
                      sanitize(
                          m_instance.get_const_network().get_vertex(v).name) +
                      "_" + std::to_string(vel) + "_" +
                      std::to_string(p_index)));
              GRBLinExpr  rhs1_relevant_helper_sum = rhs_binary_helper.at(0);
              const auto& tr2_object =
                  m_instance.get_const_train_list().get_train(tr2);
              const auto max_speed = std::min(tr2_object.get_max_speed(),
                                              last_edge_object.max_speed);
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
                                                tr2_object.get_acceleration(),
                                                tr2_object.get_deceleration(),
                                                last_edge_object.length)) {
                    // first: += y * min_t
                    // second: -= y * max_t
                    rhs.at(0) += m_vars["y"](tr2, p.back(), v_tr2_source_index,
                                             v_tr2_target_index) *
                                 cda_rail::min_travel_time_from_start(
                                     vel_tr2_source, vel_tr2_target, max_speed,
                                     tr2_object.get_acceleration(),
                                     tr2_object.get_deceleration(),
                                     last_edge_object.length, target_point);
                    const auto max_travel_time =
                        cda_rail::max_travel_time_to_end(
                            vel_tr2_source, vel_tr2_target, V_MIN,
                            tr2_object.get_acceleration(),
                            tr2_object.get_deceleration(),
                            last_edge_object.length, target_point,
                            last_edge_object.breakable);
                    if (max_travel_time >= GRB_INFINITY) {
                      rhs1_relevant_helper_sum -=
                          m_vars["y"](tr2, p.back(), v_tr2_source_index,
                                      v_tr2_target_index);
                    } else {
                      rhs.at(1) -=
                          m_vars["y"](tr2, p.back(), v_tr2_source_index,
                                      v_tr2_target_index) *
                          max_travel_time;
                    }
                  }
                }
              }
              m_model->addConstr(
                  rhs_binary_helper.at(1) >= rhs1_relevant_helper_sum,
                  "headway_binary_helper_rhs1_relevant_helper_1_" +
                      sanitize(tr_object.get_name()) + "_" +
                      sanitize(m_instance.get_const_train_list()
                                   .get_train(tr2)
                                   .get_name()) +
                      "_" +
                      sanitize(
                          m_instance.get_const_network().get_vertex(v).name) +
                      "_" + std::to_string(vel) + "_" +
                      std::to_string(p_index));
            }
            assert(rhs_binary_helper.size() == rhs.size());
            for (size_t rhs_idx = 0; rhs_idx < rhs.size(); rhs_idx++) {
              m_model->addGenConstrIndicator(
                  rhs_binary_helper.at(rhs_idx), 1,
                  m_vars["t_front_arrival"](tr, v) >= rhs.at(rhs_idx),
                  "headway_" + std::to_string(rhs_idx) + "-" +
                      std::to_string(rhs.size()) + "_" +
                      sanitize(tr_object.get_name()) + "_" +
                      sanitize(m_instance.get_const_train_list()
                                   .get_train(tr2)
                                   .get_name()) +
                      "_" +
                      sanitize(
                          m_instance.get_const_network().get_vertex(v).name) +
                      "_" + std::to_string(vel) + "_" +
                      std::to_string(p_index));
            }
          }

          // Headways on intersecting TTD sections
          const auto intersecting_ttd =
              cda_rail::Network::get_intersecting_ttd(p, m_ttd_sections);
          for (const auto& [ttd_index, e_index] : intersecting_ttd) {
            const auto& p_tmp = cda_rail::index_vector(
                p.begin(), p.begin() + static_cast<std::ptrdiff_t>(e_index));
            const auto p_tmp_len =
                std::accumulate(p_tmp.begin(), p_tmp.end(), 0.0,
                                [this](double sum, const auto& edge_index) {
                                  return sum + m_instance.get_const_network()
                                                   .get_edge(edge_index)
                                                   .length;
                                });
            GRBLinExpr edge_tmp_path_expr = 0;
            for (const auto& e_tmp : p_tmp) {
              edge_tmp_path_expr += m_vars["x"](tr, e_tmp);
            }

            auto const edge_tmp_binary = m_model->addVar(
                0.0, 1.0, 0.0, GRB_BINARY,
                "headway_ttd_" + std::to_string(ttd_index) + "_path_binary_" +
                    sanitize(tr_object.get_name()) + "_" +
                    sanitize(
                        m_instance.get_const_network().get_vertex(v).name) +
                    "_" + std::to_string(vel) + "_" + std::to_string(p_index));

            m_model->addConstr(
                edge_tmp_binary >=
                    edge_tmp_path_expr - static_cast<double>(p_tmp.size()) + 1,
                "headway_ttd_" + std::to_string(ttd_index) +
                    "_path_binary_helper_" + sanitize(tr_object.get_name()) +
                    "_" +
                    sanitize(
                        m_instance.get_const_network().get_vertex(v).name) +
                    "_" + std::to_string(vel) + "_" + std::to_string(p_index));

            const auto obd = bd - p_tmp_len;

            assert(obd >= 0);

            const auto tr_on_ttd = m_instance.trains_in_section(
                m_ttd_sections.at(ttd_index), m_model_detail.fix_routes, false);
            for (const auto& tr2 : tr_on_ttd) {
              if (tr == tr2) {
                continue;
              }

              const auto tr2_name_sanitized = sanitize(
                  m_instance.get_const_train_list().get_train(tr2).get_name());
              const auto v_name_sanitized =
                  sanitize(m_instance.get_const_network().get_vertex(v).name);

              auto const path_order_helper_binary = m_model->addVar(
                  0.0, 1.0, 0.0, GRB_BINARY,
                  "headway_ttd_" + std::to_string(ttd_index) +
                      "_path_order_binary_" + sanitize(tr_object.get_name()) +
                      "_" + tr2_name_sanitized + "_" + v_name_sanitized + "_" +
                      std::to_string(vel) + "_" + std::to_string(p_index));
              m_model->addConstr(
                  path_order_helper_binary >=
                      path_order_helper_binary +
                          m_vars["order_ttd"](tr, tr2, ttd_index) - 1,
                  "headway_ttd_" + std::to_string(ttd_index) +
                      "_path_order_helper_binary_" +
                      sanitize(tr_object.get_name()) + "_" +
                      tr2_name_sanitized + "_" + v_name_sanitized + "_" +
                      std::to_string(vel) + "_" + std::to_string(p_index));

              GRBLinExpr lhs_from_rear = m_vars["t_front_arrival"](tr, v);

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
                  cda_rail::index_vector rel_in_edges;
                  for (const auto& e :
                       m_instance.get_const_network().in_edges(v)) {
                    if (std::ranges::contains(tr_used_edges, e)) {
                      rel_in_edges.push_back(e);
                    }
                  }
                  // Note: In some cases there might be no relevant in edges, in
                  // particular if it is an entry node of a different train that
                  // might not be reachable from the network itself.
                  for (const auto& e_before_v : rel_in_edges) {
                    const auto& e_before_v_obj =
                        m_instance.get_const_network().get_edge(e_before_v);
                    const auto& v_before_v = e_before_v_obj.source;
                    const auto  v_before_v_velocities =
                        m_velocity_extensions.at(tr).at(v_before_v);
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
                                  (2 * tr_object.get_deceleration()) >=
                              e_before_v_obj.length + p_tmp_len) {
                        continue;
                      }
                      if (cda_rail::possible_by_eom(
                              vel_before_v, vel, tr_object.get_acceleration(),
                              tr_object.get_deceleration(),
                              e_before_v_obj.length)) {
                        lhs_from_rear -=
                            m_vars["y"](tr, e_before_v, v_before_v_index,
                                        v_source_index) *
                            cda_rail::min_time_from_rear_to_ma_point(
                                vel_before_v, vel, V_MIN, e_before_v_tmp_max,
                                tr_object.get_acceleration(),
                                tr_object.get_deceleration(),
                                e_before_v_obj.length, obd);
                        is_relevant = true;

                        const auto tr2_name_sanitized =
                            sanitize(m_instance.get_const_train_list()
                                         .get_train(tr2)
                                         .get_name());
                        const auto v_name_sanitized = sanitize(
                            m_instance.get_const_network().get_vertex(v).name);

                        auto const path_vel_binary = m_model->addVar(
                            0.0, 1.0, 0.0, GRB_BINARY,
                            "headway_ttd_" + std::to_string(ttd_index) +
                                "_path_vel_binary_" +
                                sanitize(tr_object.get_name()) + "_" +
                                tr2_name_sanitized + "_" + v_name_sanitized +
                                "_" + std::to_string(vel) + "_" +
                                std::to_string(p_index) + "_" +
                                std::to_string(e_before_v) + "_" +
                                std::to_string(vel_before_v));
                        m_model->addConstr(
                            path_vel_binary >= path_order_helper_binary +
                                                   m_vars["y"](tr, e_before_v,
                                                               v_before_v_index,
                                                               v_source_index) -
                                                   1,
                            "headway_ttd_" + std::to_string(ttd_index) +
                                "_path_vel_constraint_" +
                                sanitize(tr_object.get_name()) + "_" +
                                tr2_name_sanitized + "_" + v_name_sanitized +
                                "_" + std::to_string(vel) + "_" +
                                std::to_string(p_index) + "_" +
                                std::to_string(e_before_v) + "_" +
                                std::to_string(vel_before_v));

                        const auto max_from_front =
                            cda_rail::max_time_from_front_to_ma_point(
                                vel_before_v, vel, V_MIN,
                                tr_object.get_acceleration(),
                                tr_object.get_deceleration(),
                                e_before_v_obj.length, obd,
                                e_before_v_obj.breakable);
                        m_model->addGenConstrIndicator(
                            path_vel_binary, 1,
                            m_vars["t_front_departure"](tr, v_before_v) +
                                    max_from_front >=
                                m_vars["t_ttd_departure"](tr2, ttd_index),
                            "headway_ttd_" + std::to_string(ttd_index) +
                                "from_front_" + sanitize(tr_object.get_name()) +
                                "_" +
                                m_instance.get_const_train_list()
                                    .get_train(tr2)
                                    .get_name() +
                                "_" +
                                sanitize(m_instance.get_const_network()
                                             .get_vertex(v)
                                             .name) +
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
                m_model->addGenConstrIndicator(
                    path_order_helper_binary, 1,
                    lhs_from_rear >= m_vars["t_ttd_departure"](tr2, ttd_index),
                    "headway_ttd_" + sanitize(tr_object.get_name()) + "_" +
                        m_instance.get_const_train_list()
                            .get_train(tr2)
                            .get_name() +
                        "_" +
                        sanitize(
                            m_instance.get_const_network().get_vertex(v).name) +
                        "_" + std::to_string(vel) + "_" +
                        std::to_string(p_index) + "_" +
                        std::to_string(ttd_index));
              }
            }
          }
        }
      }
    }
  }
}

void cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::
    create_simplified_headway_constraints() {
  // This creates simplified headway constraints.
  // They are less accurate, but should make the model easier to solve
  // No problem if the solution is only used as a starting solution to fix some
  // parameters

  for (size_t tr = 0; tr < m_num_tr; tr++) {
    const auto& tr_object = m_instance.get_const_train_list().get_train(tr);

    for (const auto e : m_instance.edges_used_by_train(
             tr, this->m_model_detail.fix_routes, false)) {
      const auto& e_obj    = m_instance.get_const_network().get_edge(e);
      const auto& v_source = e_obj.source;
      const auto& v_target = e_obj.target;
      const auto& v_source_object =
          m_instance.get_const_network().get_vertex(v_source);
      const auto& v_target_object =
          m_instance.get_const_network().get_vertex(v_target);

      auto [hw_max, headway_tr_on_e, hw_max_ttd, headway_tr_on_ttd] =
          get_edge_headway_expressions(tr, e);

      // departure because ma might move forward, otherwise arrival and
      // departure are equal due to non-zero velocity
      // NOLINTNEXTLINE(misc-const-correctness)
      GRBVar tr_t_var = m_vars["t_front_departure"](tr, v_source);

      const auto tr_on_e =
          m_instance.all_trains_on_edge(e, m_model_detail.fix_routes, false);
      for (const auto& tr2 : tr_on_e) {
        if (tr == tr2) {
          continue;
        }
        const auto tr2_t_var = m_vars["t_rear_departure"](tr2, v_target);

        m_model->addGenConstrIndicator(
            m_vars["order"](tr, tr2, e), 1,
            tr_t_var - tr2_t_var >= headway_tr_on_e,
            "headway_simplified_" + sanitize(tr_object.get_name()) + "_" +
                sanitize(m_instance.get_const_train_list()
                             .get_train(tr2)
                             .get_name()) +
                "_" + sanitize(v_source_object.name) + "_" +
                sanitize(v_target_object.name));
      }

      // TTD constraint on entering edge
      const auto neighboring_edges =
          m_instance.get_const_network().neighboring_edges(v_source);
      const auto intersecting_ttd =
          cda_rail::Network::get_intersecting_ttd({e}, m_ttd_sections);
      for (const auto& [ttd_index, _] : intersecting_ttd) {
        const auto& ttd_section = m_ttd_sections.at(ttd_index);
        // If all of neighboring_edges are in ttd_section, then it is not an
        // entering edge Hence, if at least one neighboring edge is not in
        // ttd_section, then we have an entering edge
        const bool is_entering_edge = std::ranges::any_of(
            neighboring_edges, [&ttd_section](const auto& e_tmp) {
              return !std::ranges::contains(ttd_section, e_tmp);
            });
        if (is_entering_edge) {
          // We need a constraint for each other train in the TTD section
          const auto tr_on_ttd = m_instance.trains_in_section(
              ttd_section, m_model_detail.fix_routes, false);
          for (const auto& tr2 : tr_on_ttd) {
            if (tr == tr2) {
              continue;
            }
            const auto tr2_t_var = m_vars["t_ttd_departure"](tr2, ttd_index);
            m_model->addGenConstrIndicator(
                m_vars["order_ttd"](tr, tr2, ttd_index), 1,
                tr_t_var - tr2_t_var >= headway_tr_on_ttd,
                "headway_simplified_ttd_" + sanitize(tr_object.get_name()) +
                    "_" +
                    m_instance.get_const_train_list()
                        .get_train(tr2)
                        .get_name() +
                    "_" + sanitize(v_source_object.name) + "_" +
                    sanitize(v_target_object.name) + "_ttd" +
                    std::to_string(ttd_index));
          }
        }
      }
    }
  }
}

void cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::
    create_basic_ttd_constraints() {
  for (size_t i = 0; i < m_ttd_sections.size(); i++) {
    const auto& ttd_section = m_ttd_sections.at(i);
    const auto  tr_on_ttd   = m_instance.trains_in_section(
        ttd_section, m_model_detail.fix_routes, false);
    for (auto const& tr : tr_on_ttd) {
      const auto& tr_name =
          m_instance.get_const_train_list().get_train(tr).get_name();

      // x_ttd aggregates x values
      const auto e_tr =
          m_instance.edges_used_by_train(tr, m_model_detail.fix_routes, false);
      // relevant edges are intersection of ttd_section and e_tr
      cda_rail::index_vector relevant_edges;
      for (const auto& e : ttd_section) {
        if (std::ranges::contains(e_tr, e)) {
          relevant_edges.push_back(e);
        }
      }
      GRBLinExpr rhs = 0;
      for (const auto& e : relevant_edges) {
        const auto e_object = m_instance.get_const_network().get_edge(e);
        const auto v1_name =
            m_instance.get_const_network().get_vertex(e_object.source).name;
        const auto v2_name =
            m_instance.get_const_network().get_vertex(e_object.target).name;
        m_model->addConstr(m_vars["x_ttd"](tr, i) >= m_vars["x"](tr, e),
                           "aggregate_edge_ttd_1_" +
                               sanitize(m_instance.get_const_train_list()
                                            .get_train(tr)
                                            .get_name()) +
                               "_" + std::to_string(i) + "_" +
                               sanitize(v1_name) + "-" + sanitize(v2_name));
        rhs += m_vars["x"](tr, e);

        // Moreover bound t_ttd_departure
        // >= t_rear_departure(v2) * x(e)
        // where 0 <= t_rear_departure <= t_bound continuous
        // x(e) is binary
        // Hence, equivalent to
        // t_ttd >= t - t_bound * (1 - x)
        // t_ttd >= 0 (already by definition)
        // Because we are only interested in bounding the time from below no
        // other constraints are needed.
        m_model->addGenConstrIndicator(
            m_vars["x"](tr, e), 1,
            m_vars["t_ttd_departure"](tr, i) >=
                m_vars["t_rear_departure"](tr, e_object.target),
            "ttd_departure_bound_" +
                sanitize(m_instance.get_const_train_list()
                             .get_train(tr)
                             .get_name()) +
                "_" + std::to_string(i) + "_" + sanitize(v1_name) + "-" +
                sanitize(v2_name));
      }
      m_model->addConstr(
          m_vars["x_ttd"](tr, i) <= rhs,
          "aggregate_edge_ttd_2_" +
              sanitize(
                  m_instance.get_const_train_list().get_train(tr).get_name()) +
              "_" + std::to_string(i));

      for (auto const& tr2 : tr_on_ttd) {
        if (tr2 <= tr) {
          continue;
        }
        const auto& tr2_name =
            m_instance.get_const_train_list().get_train(tr2).get_name();

        // Order constraints as usual
        m_model->addConstr(
            m_vars["order_ttd"](tr, tr2, i) + m_vars["order_ttd"](tr2, tr, i) <=
                0.5 * (m_vars["x_ttd"](tr, i) + m_vars["x_ttd"](tr2, i)),
            "ttd_order_1_" + sanitize(tr_name) + "_" + sanitize(tr2_name) +
                "_" + std::to_string(i));
        m_model->addConstr(
            m_vars["order_ttd"](tr, tr2, i) + m_vars["order_ttd"](tr2, tr, i) >=
                m_vars["x_ttd"](tr, i) - m_vars["x_ttd"](tr2, i) - 1,
            "ttd_order_2_" + sanitize(tr_name) + "_" + sanitize(tr2_name) +
                "_" + std::to_string(i));

        // If tr1 follows tr2 then t_ttd_departure(tr1) >= t_ttd_departure(tr2)
        m_model->addGenConstrIndicator(m_vars["order_ttd"](tr, tr2, i), 1,
                                       m_vars["t_ttd_departure"](tr, i) >=
                                           m_vars["t_ttd_departure"](tr2, i),
                                       "ttd_order_3_" + sanitize(tr_name) +
                                           "_" + sanitize(tr2_name) + "_" +
                                           std::to_string(i));

        // If tr2 follows tr1 then t_ttd_departure(tr2) >= t_ttd_departure(tr1)
        m_model->addGenConstrIndicator(m_vars["order_ttd"](tr2, tr, i), 1,
                                       m_vars["t_ttd_departure"](tr2, i) >=
                                           m_vars["t_ttd_departure"](tr, i),
                                       "ttd_order_4_" + sanitize(tr2_name) +
                                           "_" + sanitize(tr_name) + "_" +
                                           std::to_string(i));
      }
    }
  }
}

void cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::
    create_reverse_edge_constraints() {
  for (size_t idx = 0; idx < m_relevant_reverse_edges.size(); idx++) {
    const auto& [e1, e2] = m_relevant_reverse_edges.at(idx);
    const auto tr_list_1 =
        m_instance.all_trains_on_edge(e1, m_model_detail.fix_routes, false);
    const auto tr_list_2 =
        m_instance.all_trains_on_edge(e2, m_model_detail.fix_routes, false);

    const auto  e_obj = m_instance.get_const_network().get_edge(e1);
    const auto& v1_name =
        m_instance.get_const_network().get_vertex(e_obj.source).name;
    const auto& v2_name =
        m_instance.get_const_network().get_vertex(e_obj.target).name;

    for (const auto& tr1 : tr_list_1) {
      const auto& tr1_name =
          m_instance.get_const_train_list().get_train(tr1).get_name();
      for (const auto& tr2 : tr_list_2) {
        if (tr1 == tr2) {
          continue;
        }
        const auto& tr2_name =
            m_instance.get_const_train_list().get_train(tr2).get_name();
        m_model->addConstr(m_vars["reverse_order"](tr1, tr2, idx) +
                                   m_vars["reverse_order"](tr2, tr1, idx) >=
                               m_vars["x"](tr1, e1) + m_vars["x"](tr2, e2) - 1,
                           "reverse_order_lb_" + sanitize(tr1_name) + "_" +
                               sanitize(tr2_name) + "_" + sanitize(v1_name) +
                               "-" + sanitize(v2_name));
        m_model->addConstr(m_vars["reverse_order"](tr1, tr2, idx) +
                                   m_vars["reverse_order"](tr2, tr1, idx) <=
                               1,
                           "reverse_order_ub_" + sanitize(tr1_name) + "_" +
                               sanitize(tr2_name) + "_" + sanitize(v1_name) +
                               "-" + sanitize(v2_name));

        // If tr1 follows tr2 then front of tr1 >= rear of tr2 at source vertex
        // (of e1)
        m_model->addGenConstrIndicator(
            m_vars["reverse_order"](tr1, tr2, idx), 1,
            m_vars["t_front_arrival"](tr1, e_obj.source) >=
                m_vars["t_rear_departure"](tr2, e_obj.source),
            "reverse_order_1_" + sanitize(tr1_name) + "_" + sanitize(tr2_name) +
                "_" + sanitize(v1_name) + "-" + sanitize(v2_name));

        // If tr2 follows tr1 then front of tr2 >= rear of tr1 at source vertex
        // of e2, hence, target vertex of e1
        m_model->addGenConstrIndicator(
            m_vars["reverse_order"](tr2, tr1, idx), 1,
            m_vars["t_front_arrival"](tr2, e_obj.target) >=
                m_vars["t_rear_departure"](tr1, e_obj.target),
            "reverse_order_2_" + sanitize(tr2_name) + "_" + sanitize(tr1_name) +
                "_" + sanitize(v1_name) + "-" + sanitize(v2_name));
      }
    }
  }
}

void cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::
    create_vertex_headway_constraints() {
  // If a line headway is specified (most importantly on exit nodes), then obey
  // This only takes into account if the same previous or next edge is used
  for (size_t e = 0; e < m_num_edges; e++) {
    const auto& tr_on_edge =
        m_instance.all_trains_on_edge(e, m_model_detail.fix_routes, false);
    if (tr_on_edge.size() <= 1) {
      continue;
    }

    const auto& e_object = m_instance.get_const_network().get_edge(e);
    const auto& source_v = e_object.source;
    const auto& target_v = e_object.target;
    const auto& source_v_object =
        m_instance.get_const_network().get_vertex(source_v);
    const auto& target_v_object =
        m_instance.get_const_network().get_vertex(target_v);

    for (auto const& tr1 : tr_on_edge) {
      const auto& tr1_object = m_instance.get_const_train_list().get_train(tr1);

      auto [hw_s1_max, hw_s1, hw_t1_max, hw_t1] =
          get_vertex_headway_expressions(tr1, e);

      for (auto const& tr2 : tr_on_edge) {
        if (tr2 <= tr1) {
          continue;
        }
        const auto& tr2_object =
            m_instance.get_const_train_list().get_train(tr2);

        auto [hw_s2_max, hw_s2, hw_t2_max, hw_t2] =
            get_vertex_headway_expressions(tr2, e);

        // Add headway constraints to both source and target vertices depending
        // on train order
        m_model->addGenConstrIndicator(
            m_vars["order"](tr1, tr2, e), 1,
            m_vars["t_front_arrival"](tr1, source_v) >=
                m_vars["t_rear_departure"](tr2, source_v) + hw_s1,
            "headway_vertex_source_1_" + sanitize(tr1_object.get_name()) + "_" +
                sanitize(tr2_object.get_name()) + "_" +
                sanitize(source_v_object.name) + "-" +
                sanitize(target_v_object.name));
        m_model->addGenConstrIndicator(
            m_vars["order"](tr2, tr1, e), 1,
            m_vars["t_front_arrival"](tr2, source_v) >=
                m_vars["t_rear_departure"](tr1, source_v) + hw_s2,
            "headway_vertex_source_2_" + sanitize(tr1_object.get_name()) + "_" +
                sanitize(tr2_object.get_name()) + "_" +
                sanitize(source_v_object.name) + "-" +
                sanitize(target_v_object.name));
        m_model->addGenConstrIndicator(
            m_vars["order"](tr1, tr2, e), 1,
            m_vars["t_front_arrival"](tr1, target_v) >=
                m_vars["t_rear_departure"](tr2, target_v) + hw_t1,
            "headway_vertex_target_1_" + sanitize(tr1_object.get_name()) + "_" +
                sanitize(tr2_object.get_name()) + "_" +
                sanitize(source_v_object.name) + "-" +
                sanitize(target_v_object.name));
        m_model->addGenConstrIndicator(
            m_vars["order"](tr2, tr1, e), 1,
            m_vars["t_front_arrival"](tr2, target_v) >=
                m_vars["t_rear_departure"](tr1, target_v) + hw_t2,
            "headway_vertex_target_2_" + sanitize(tr1_object.get_name()) + "_" +
                sanitize(tr2_object.get_name()) + "_" +
                sanitize(source_v_object.name) + "-" +
                sanitize(target_v_object.name));
      }
    }
  }
}

GRBLinExpr
cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::get_edge_path_expr(
    size_t tr, const cda_rail::index_vector& p, double initial_velocity,
    bool also_higher_velocities) {
  // Get linear expression that sums up all corresponding binary variables.
  // For the first edge, only extended vertices starting with the desired
  // velocity are considered If, optionally, also_higher_velocities is true,
  // then also edges leaving with any higher velocity are considered.
  GRBLinExpr edge_path_expr = 0;

  const auto& tr_object = m_instance.get_const_train_list().get_train(tr);
  const auto& e_1       = p.front();
  const auto& e_1_obj   = m_instance.get_const_network().get_edge(e_1);
  const auto  tmp_max_speed =
      std::min(tr_object.get_max_speed(), e_1_obj.max_speed);
  const auto& v_source_velocities =
      m_velocity_extensions.at(tr).at(e_1_obj.source);
  const auto& v_target_velocities =
      m_velocity_extensions.at(tr).at(e_1_obj.target);
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
      if (cda_rail::possible_by_eom(
              vel_source, vel_target, tr_object.get_acceleration(),
              tr_object.get_deceleration(), e_1_obj.length)) {
        edge_path_expr += m_vars["y"](tr, e_1, v_source_index, v_target_index);
      }
    }
  }
  for (const auto& e_p : p) {
    if (e_p != e_1) {
      edge_path_expr += m_vars["x"](tr, e_p);
    }
  }

  return edge_path_expr;
}

void cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::
    fill_relevant_reverse_edges() {
  const auto relevant_breakable_edges =
      m_instance.get_const_network().relevant_breakable_edges();
  m_relevant_reverse_edges.clear();
  for (const auto& e : relevant_breakable_edges) {
    const auto reverse_edge =
        m_instance.get_const_network().get_reverse_edge_index(e);
    if (reverse_edge.has_value()) {
      m_relevant_reverse_edges.emplace_back(e, reverse_edge.value());
    }
  }
}

double cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::headway(
    const cda_rail::Train& tr_obj, const cda_rail::Edge& e_obj, double v_1,
    double v_2, bool entry_vertex) {
  // If Value is within GRB_EPS set to 0
  round_small_numbers_to_zero_inplace(v_1, GRB_EPS);
  round_small_numbers_to_zero_inplace(v_2, GRB_EPS);

  if (const auto bd = v_1 * v_1 / (2 * tr_obj.get_deceleration());
      bd >= e_obj.length - GRB_EPS) {
    if (v_1 <= GRB_EPS) {
      return 0;
    }
    return entry_vertex
               ? (bd - e_obj.length) / v_1
               : min_time_to_push_ma_backward(
                     v_1, tr_obj.get_acceleration(), tr_obj.get_deceleration(),
                     std::max<double>(bd - e_obj.length, 0.0));
  }

  const auto obd = v_2 * v_2 / (2 * tr_obj.get_deceleration());

  return -max_time_from_front_to_ma_point(
      v_1, v_2, V_MIN, tr_obj.get_acceleration(), tr_obj.get_acceleration(),
      e_obj.length, obd, e_obj.breakable);
}

std::tuple<double, GRBLinExpr, double, GRBLinExpr> cda_rail::solver::mip_based::
    GenPOMovingBlockMIPSolver::get_vertex_headway_expressions(size_t tr,
                                                              size_t e) {
  const auto& e_object = m_instance.get_const_network().get_edge(e);
  const auto& source_v = e_object.source;
  const auto& target_v = e_object.target;
  const auto& source_v_object =
      m_instance.get_const_network().get_vertex(source_v);
  const auto& target_v_object =
      m_instance.get_const_network().get_vertex(target_v);
  const auto& tr_object = m_instance.get_const_train_list().get_train(tr);

  auto       hw_s1_max = source_v_object.headway;
  auto       hw_t1_max = target_v_object.headway;
  GRBLinExpr hw_s1     = source_v_object.headway;
  GRBLinExpr hw_t1     = target_v_object.headway;

  const auto& tr_source_velocities = m_velocity_extensions.at(tr).at(source_v);
  const auto& tr_target_velocities = m_velocity_extensions.at(tr).at(target_v);

  // Strengthen vertex headway by velocity minimal headway times if
  // applicable
  if (this->m_model_detail.strengthen_vertex_headway_constraints) {
    for (size_t s_vel_idx = 0; s_vel_idx < tr_source_velocities.size();
         s_vel_idx++) {
      const auto& source_vel = tr_source_velocities.at(s_vel_idx);
      if (source_vel > tr_object.get_max_speed() ||
          source_vel > e_object.max_speed) {
        continue;
      }
      const auto source_velocity_headway = min_time_to_push_ma_fully_backward(
          source_vel, tr_object.get_acceleration(),
          tr_object.get_deceleration());
      hw_s1_max = std::max(hw_s1_max, source_velocity_headway);
      for (size_t t_vel_idx = 0; t_vel_idx < tr_target_velocities.size();
           t_vel_idx++) {
        const auto& target_vel = tr_target_velocities.at(t_vel_idx);
        if (target_vel > tr_object.get_max_speed() ||
            target_vel > e_object.max_speed) {
          continue;
        }
        const auto target_velocity_headway = min_time_to_push_ma_fully_backward(
            target_vel, tr_object.get_acceleration(),
            tr_object.get_deceleration());
        hw_t1_max = std::max(hw_t1_max, target_velocity_headway);
        if (cda_rail::possible_by_eom(
                source_vel, target_vel, tr_object.get_acceleration(),
                tr_object.get_deceleration(), e_object.length)) {
          // Add more headway if velocity headway is larger than vertex
          // required headway
          if (source_velocity_headway > source_v_object.headway) {
            hw_s1 += m_vars["y"](tr, e, s_vel_idx, t_vel_idx) *
                     (source_velocity_headway - source_v_object.headway);
          }
          if (target_velocity_headway > target_v_object.headway) {
            hw_t1 += m_vars["y"](tr, e, s_vel_idx, t_vel_idx) *
                     (target_velocity_headway - target_v_object.headway);
          }
        }
      }
    }
  }

  return {hw_s1_max, hw_s1, hw_t1_max, hw_t1};
}

std::tuple<double, GRBLinExpr, double, GRBLinExpr> cda_rail::solver::mip_based::
    GenPOMovingBlockMIPSolver::get_edge_headway_expressions(size_t tr,
                                                            size_t e) {
  const auto& e_obj     = m_instance.get_const_network().get_edge(e);
  const auto& tr_object = m_instance.get_const_train_list().get_train(tr);
  const auto& v_source  = e_obj.source;
  const auto& v_target  = e_obj.target;
  const auto& v_source_velocities = m_velocity_extensions.at(tr).at(v_source);
  const auto& v_target_velocities = m_velocity_extensions.at(tr).at(v_target);

  const auto& tr_schedule_object = m_instance.get_const_schedule(tr);
  const auto& entry_node         = tr_schedule_object.get_entry_vertex();

  GRBLinExpr headway_tr_on_e   = 0;
  GRBLinExpr headway_tr_on_ttd = 0;
  double     hw_max            = 0;
  double     hw_max_ttd        = 0;

  for (size_t v_source_index = 0; v_source_index < v_source_velocities.size();
       v_source_index++) {
    const auto& vel_source = v_source_velocities.at(v_source_index);
    if (vel_source > tr_object.get_max_speed() ||
        vel_source > e_obj.max_speed) {
      continue;
    }
    for (size_t v_target_index = 0; v_target_index < v_target_velocities.size();
         v_target_index++) {
      const auto& vel_target = v_target_velocities.at(v_target_index);
      if (vel_target > tr_object.get_max_speed() ||
          vel_target > e_obj.max_speed) {
        continue;
      }
      if (cda_rail::possible_by_eom(
              vel_source, vel_target, tr_object.get_acceleration(),
              tr_object.get_deceleration(), e_obj.length)) {
        auto       hw_tmp = headway(tr_object, e_obj, vel_source, vel_target,
                                    v_source_index == entry_node);
        const auto hw_tmp_ttd = min_time_to_push_ma_fully_backward(
            vel_source, tr_object.get_acceleration(),
            tr_object.get_deceleration());

        hw_max     = std::max(hw_tmp, hw_max);
        hw_max_ttd = std::max(hw_tmp_ttd, hw_max_ttd);

        headway_tr_on_e +=
            m_vars["y"](tr, e, v_source_index, v_target_index) * hw_tmp;

        headway_tr_on_ttd +=
            m_vars["y"](tr, e, v_source_index, v_target_index) * hw_tmp_ttd;
      }
    }
  }

  return {hw_max, headway_tr_on_e, hw_max_ttd, headway_tr_on_ttd};
}

void cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::cleanup() {
  GeneralMIPSolver::cleanup();
  m_solution_settings = {};
  m_model_detail      = {};
  m_solver_strategy   = {};
  m_num_tr            = 0;
  m_num_edges         = 0;
  m_num_vertices      = 0;
  m_num_ttd           = 0;
  // max_t = 0;
  m_ttd_sections.clear();
  m_tr_stop_data.clear();
  m_velocity_extensions.clear();
  m_relevant_reverse_edges.clear();
}

// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay,performance-inefficient-string-concatenation,bugprone-unchecked-optional-access)
