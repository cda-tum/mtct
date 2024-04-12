#include "solver/mip-based/GenPOMovingBlockMIPSolver.hpp"

#include "EOMHelper.hpp"
#include "MultiArray.hpp"
#include "gurobi_c++.h"
#include "solver/mip-based/GeneralMIPSolver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <utility>
#include <vector>

// NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay)

cda_rail::instances::SolGeneralPerformanceOptimizationInstance
cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::solve(
    const ModelDetail&                 model_detail_input,
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

  this->solve_init_gen_po_mb(time_limit, debug_input);

  if (!instance.n().is_consistent_for_transformation()) {
    PLOGE << "Instance is not consistent for transformation.";
    throw exceptions::ConsistencyException();
  }

  PLOGI << "Create model";

  const instances::GeneralPerformanceOptimizationInstance old_instance =
      instance;
  this->instance.discretize_stops();

  this->initialize_variables(solution_settings_input, model_detail_input);

  PLOGD << "Create variables";
  create_variables();
  PLOGD << "Set objective";
  set_objective();
  PLOGD << "Create constraints";
  create_constraints();

  PLOGI << "Model created. Optimize";
  model->optimize();

  this->instance = old_instance;

  return {};
}

void cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::
    create_variables() {
  create_timing_variables();
  create_general_edge_variables();
  create_velocity_extended_variables();
  create_stop_variables();
}

void cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::
    create_timing_variables() {
  vars["t_front_arrival"]   = MultiArray<GRBVar>(num_tr, num_vertices);
  vars["t_front_departure"] = MultiArray<GRBVar>(num_tr, num_vertices);
  vars["t_rear_departure"]  = MultiArray<GRBVar>(num_tr, num_vertices);
  vars["t_ttd_departure"]   = MultiArray<GRBVar>(num_tr, num_ttd);

  for (size_t tr = 0; tr < num_tr; tr++) {
    const double ub_timing_dept = ub_timing_variable(tr);
    for (const auto v :
         instance.vertices_used_by_train(tr, model_detail.fix_routes, false)) {
      vars["t_front_arrival"](tr, v) =
          model->addVar(0.0, ub_timing_dept, 0.0, GRB_CONTINUOUS);
      vars["t_front_departure"](tr, v) =
          model->addVar(0.0, ub_timing_dept, 0.0, GRB_CONTINUOUS);
      vars["t_rear_departure"](tr, v) =
          model->addVar(0.0, ub_timing_dept, 0.0, GRB_CONTINUOUS);
    }
    for (const auto& ttd : instance.sections_used_by_train(
             tr, ttd_sections, model_detail.fix_routes, false)) {
      vars["t_ttd_departure"](tr, ttd) =
          model->addVar(0.0, ub_timing_dept, 0.0, GRB_CONTINUOUS);
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
    for (const auto e :
         instance.edges_used_by_train(tr, model_detail.fix_routes, false)) {
      vars["x"](tr, e) = model->addVar(0.0, 1.0, 0.0, GRB_BINARY);
    }
    for (const auto& ttd : instance.sections_used_by_train(
             tr, ttd_sections, model_detail.fix_routes, false)) {
      vars["x_ttd"](tr, ttd) = model->addVar(0.0, 1.0, 0.0, GRB_BINARY);
    }
  }
  for (size_t e = 0; e < num_edges; e++) {
    const auto tr_on_e = instance.trains_on_edge_mixed_routing(
        e, model_detail.fix_routes, false);
    for (const auto& tr1 : tr_on_e) {
      for (const auto& tr2 : tr_on_e) {
        if (tr1 != tr2) {
          vars["order"](tr1, tr2, e) = model->addVar(0.0, 1.0, 0.0, GRB_BINARY);
        }
      }
    }
  }
  for (size_t ttd = 0; ttd < num_ttd; ttd++) {
    const auto tr_on_ttd = instance.trains_in_section(
        ttd_sections.at(ttd), model_detail.fix_routes, false);
    for (const auto& tr1 : tr_on_ttd) {
      for (const auto& tr2 : tr_on_ttd) {
        if (tr1 != tr2) {
          vars["order_ttd"](tr1, tr2, ttd) =
              model->addVar(0.0, 1.0, 0.0, GRB_BINARY);
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
    for (size_t stop = 0; stop < instance.get_schedule(tr).get_stops().size();
         stop++) {
      const auto& stop_data = tr_stop_data.at(tr).at(stop);
      for (const auto& [v, edges] : stop_data) {
        vars["stop"](tr, stop, v) = model->addVar(0.0, 1.0, 0.0, GRB_BINARY);
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
      const auto& v_1  = velocity_extensions.at(tr).at(edge.source);
      const auto& v_2  = velocity_extensions.at(tr).at(edge.target);
      for (size_t i = 0; i < v_1.size(); i++) {
        for (size_t j = 0; j < v_2.size(); j++) {
          if (cda_rail::possible_by_eom(v_1.at(i), v_2.at(j),
                                        train.acceleration, train.deceleration,
                                        edge.length)) {
            vars["y"](tr, e, i, j) = model->addVar(0.0, 1.0, 0.0, GRB_BINARY);
          }
        }
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
  // TODO
  create_general_path_constraints();
  create_travel_times_constraints();
  create_basic_order_constraints();
  create_train_rear_constraints();
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
        const ModelDetail&                 model_detail_input) {
  num_tr                  = instance.get_train_list().size();
  num_edges               = instance.const_n().number_of_edges();
  num_vertices            = instance.const_n().number_of_vertices();
  max_t                   = instance.max_t();
  this->solution_settings = solution_settings_input;
  this->model_detail      = model_detail_input;
  this->ttd_sections      = instance.n().unbreakable_sections();
  this->num_ttd           = this->ttd_sections.size();
  this->fill_tr_stop_data();
  this->fill_velocity_extensions();
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
      for (size_t i = 0; i < v1_values.size(); i++) {
        for (size_t j = 0; j < v2_values.size(); j++) {
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
              for (size_t j = 0; j < v2_values.size(); j++) {
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
              const auto& v2_values =
                  velocity_extensions.at(tr).at(edge.target);
              for (size_t j = 0; j < v2_values.size(); j++) {
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
  }
}

void cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::
    create_travel_times_constraints() {
  for (size_t tr = 0; tr < num_tr; tr++) {
    const auto& tr_object = instance.get_train_list().get_train(tr);
    for (const auto& e :
         instance.edges_used_by_train(tr, model_detail.fix_routes, false)) {
      const auto& edge      = instance.const_n().get_edge(e);
      const auto& v1_values = velocity_extensions.at(tr).at(edge.source);
      const auto& v2_values = velocity_extensions.at(tr).at(edge.target);
      for (size_t i = 0; i < v1_values.size(); i++) {
        for (size_t j = 0; j < v2_values.size(); j++) {
          if (cda_rail::possible_by_eom(v1_values.at(i), v2_values.at(j),
                                        tr_object.acceleration,
                                        tr_object.deceleration, edge.length)) {
            // t_front_arrival >= t_rear_departure + minimal travel time if arc
            // is used
            const auto& min_t_arc = cda_rail::min_travel_time(
                v1_values.at(i), v2_values.at(i), edge.max_speed,
                tr_object.acceleration, tr_object.deceleration, edge.length);
            const auto& max_t_arc = cda_rail::max_travel_time(
                v1_values.at(i), v2_values.at(i), V_MIN, tr_object.acceleration,
                tr_object.deceleration, edge.length, edge.breakable);
            model->addConstr(
                vars["t_front_arrival"](tr, edge.target) +
                        (ub_timing_variable(tr) + min_t_arc) *
                            (1 - vars["y"](tr, e, i, j)) >=
                    vars["t_rear_departure"](tr, edge.source) + min_t_arc,
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
                    vars["t_rear_departure"](tr, edge.source) + max_t_arc +
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
          for (size_t i = 0; i < v1_velocities.size(); i++) {
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
          for (size_t i = 0; i < v2_velocities.size(); i++) {
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
                vars["x"](tr1, e) - vars["x"](tr2, e) - 1,
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
    const auto& tr_object = instance.get_train_list().get_train(tr);
    const auto& schedule  = instance.get_schedule(tr);
    const auto& exit      = schedule.get_exit();
    const auto& v_n       = schedule.get_v_n();
    const auto  edges_used_by_train =
        instance.edges_used_by_train(tr, model_detail.fix_routes, false);

    const auto M = ub_timing_variable(tr);

    for (const auto& v :
         instance.vertices_used_by_train(tr, model_detail.fix_routes, false)) {
      if (v == exit) {
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
              const auto& v1_velocities =
                  velocity_extensions.at(tr).at(e_in_object.source);
              for (size_t j = 0; j < v1_velocities.size(); j++) {
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
                         "rear_departure_vertex_" + tr_object.name + "_" +
                             instance.const_n().get_vertex(v).name);
        model->addConstr(vars["t_rear_departure"](tr, v) <=
                             vars["t_front_departure"](tr, v) +
                                 max_travel_time_expr,
                         "rear_departure_vertex_" + tr_object.name + "_" +
                             instance.const_n().get_vertex(v).name);
      } else {
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
          assert(last_edge_obj.length + p_len_last_vertex >= tr_object.length);

          GRBLinExpr lhs = vars["t_rear_departure"](tr, v) + M * p.size();
          for (const auto& e_p : p) {
            lhs -= M * vars["x"](tr, e_p);
          }

          const auto rel_pt_on_edge = tr_object.length - p_len_last_vertex;
          const auto v_0_velocities =
              velocity_extensions.at(tr).at(last_edge_obj.source);
          const auto v_1_velocities =
              velocity_extensions.at(tr).at(last_edge_obj.target);
          GRBLinExpr t_ref_1 =
              vars["t_front_departure"](tr, last_edge_obj.source);
          GRBLinExpr t_ref_2 =
              vars["t_front_arrival"](tr, last_edge_obj.target);

          const auto v_max_rel_e =
              std::min(last_edge_obj.max_speed, tr_object.max_speed);
          for (size_t i = 0; i < v_0_velocities.size(); i++) {
            for (size_t j = 0; j < v_1_velocities.size(); j++) {
              if (cda_rail::possible_by_eom(
                      v_0_velocities.at(i), v_1_velocities.at(j),
                      tr_object.acceleration, tr_object.deceleration,
                      last_edge_obj.length)) {
                t_ref_1 +=
                    vars["y"](tr, last_edge, i, j) *
                    cda_rail::min_time_from_front_to_ma_point(
                        v_0_velocities.at(i), v_1_velocities.at(j), v_max_rel_e,
                        tr_object.acceleration, tr_object.deceleration,
                        last_edge_obj.length, rel_pt_on_edge);
                const auto max_travel_time = cda_rail::max_travel_time_to_end(
                    v_0_velocities.at(i), v_1_velocities.at(j), V_MIN,
                    tr_object.acceleration, tr_object.deceleration,
                    last_edge_obj.length, rel_pt_on_edge,
                    last_edge_obj.breakable);
                t_ref_2 -=
                    vars["y"](tr, last_edge, i, j) *
                    (max_travel_time >= std::numeric_limits<double>::infinity()
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

// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay)
