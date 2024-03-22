#include "solver/mip-based/GenPOMovingBlockMIPSolver.hpp"

#include "EOMHelper.hpp"
#include "MultiArray.hpp"
#include "gurobi_c++.h"

#include <algorithm>
#include <cmath>
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

  instances::GeneralPerformanceOptimizationInstance old_instance = instance;
  this->instance.discretize_stops();

  num_tr                  = instance.get_train_list().size();
  num_edges               = instance.const_n().number_of_edges();
  num_vertices            = instance.const_n().number_of_vertices();
  max_t                   = instance.max_t();
  this->solution_settings = solution_settings_input;
  this->model_detail      = model_detail_input;
  this->ttd_sections      = instance.n().unbreakable_sections();
  this->num_ttd           = this->ttd_sections.size();
  this->fill_tr_stop_data();

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
    const double tr_approx_leaving_time =
        instance.get_approximate_leaving_time(tr);
    for (const auto v :
         instance.vertices_used_by_train(tr, model_detail.fix_routes, false)) {
      vars["t_front_arrival"](tr, v) =
          model->addVar(0.0, max_t, 0.0, GRB_CONTINUOUS);
      vars["t_front_departure"](tr, v) = model->addVar(
          0.0, max_t + tr_approx_leaving_time, 0.0, GRB_CONTINUOUS);
      vars["t_rear_departure"](tr, v) = model->addVar(
          0.0, max_t + tr_approx_leaving_time, 0.0, GRB_CONTINUOUS);
    }
    for (const auto& ttd : instance.sections_used_by_train(
             tr, ttd_sections, model_detail.fix_routes, false)) {
      vars["t_ttd_departure"](tr, ttd) = model->addVar(
          0.0, max_t + tr_approx_leaving_time, 0.0, GRB_CONTINUOUS);
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
}

void cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::
    fill_tr_stop_data() {
  tr_stop_data.reserve(num_tr);

  for (size_t tr = 0; tr < num_tr; tr++) {
    std::vector<
        std::vector<std::pair<size_t, std::vector<std::vector<size_t>>>>>
        tr_data;
    tr_data.reserve(instance.get_schedule(tr).get_stops().size());
    for (const auto& stop : instance.get_schedule(tr).get_stops()) {
      tr_data.emplace_back(
          instance.possible_stop_vertices(tr, stop.get_station_name()));
    }
    tr_stop_data.emplace_back(tr_data);
  }
}

void cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::
    fill_velocity_extensions() {
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
      const double        max_vertex_speed =
          std::min(instance.const_n().maximal_vertex_speed(v), tr_max_speed);
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

      const double max_vertex_speed =
          std::min(instance.const_n().maximal_vertex_speed(v), tr_max_speed);
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

// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay)
