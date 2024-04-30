#include "Definitions.hpp"
#include "EOMHelper.hpp"
#include "MultiArray.hpp"
#include "gurobi_c++.h"
#include "solver/mip-based/GenPOMovingBlockMIPSolver.hpp"
#include "solver/mip-based/GeneralMIPSolver.hpp"

#include <cstddef>
#include <unordered_map>
#include <utility>
#include <vector>

void cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::extract_solution(
    cda_rail::instances::SolGeneralPerformanceOptimizationInstance& sol) const {
  PLOGI << "Extracting solution object...";

  // Is there a solution?
  if (const auto grb_status = model->get(GRB_IntAttr_Status);
      grb_status == GRB_OPTIMAL) {
    PLOGD << "Solution status: Optimal";
    sol.set_status(SolutionStatus::Optimal);
  } else if (grb_status == GRB_INFEASIBLE) {
    PLOGD << "Solution status: Infeasible";
    sol.set_status(SolutionStatus::Infeasible);
  } else if (model->get(GRB_IntAttr_SolCount) >= 1) {
    PLOGD << "Solution status: Feasible (optimality unknown)";
    sol.set_status(SolutionStatus::Feasible);
  } else if (grb_status == GRB_TIME_LIMIT &&
             model->get(GRB_IntAttr_SolCount) == 0) {
    PLOGD << "Solution status: Timeout (Feasibility unknown)";
    sol.set_status(SolutionStatus::Timeout);
  } else {
    PLOGE << "Solution status code " << grb_status << " unknown";
    throw exceptions::ConsistencyException(
        "Gurobi status code " + std::to_string(grb_status) + " unknown.");
  }

  if (const auto sol_count = model->get(GRB_IntAttr_SolCount);
      sol_count < 0.5) {
    return;
  }

  const auto mip_obj_val =
      static_cast<int>(std::round(model->get(GRB_DoubleAttr_ObjVal)));
  sol.set_obj(mip_obj_val);
  PLOGD << "MIP objective: " << mip_obj_val;

  // Extract routes
  PLOGD << "Setting routes...";
  std::vector<std::vector<std::pair<size_t, double>>> route_markers;
  route_markers.reserve(num_tr);
  sol.reset_routes();
  for (int tr = 0; tr < num_tr; tr++) {
    bool        tr_routed = false;
    const auto& tr_object = instance.get_train_list().get_train(tr);
    sol.add_empty_route(tr_object.name);
    const auto entry             = instance.get_schedule(tr).get_entry();
    auto       edges_to_consider = instance.const_n().out_edges(entry);

    double                                 current_pos = 0;
    std::vector<std::pair<size_t, double>> route_marker_tr;
    route_marker_tr.emplace_back(entry, current_pos);
    while (!edges_to_consider.empty()) {
      const auto& edge_id = edges_to_consider.back();
      edges_to_consider.pop_back();
      if (!vars.at("x").at(tr, edge_id).sameAs(GRBVar()) &&
          vars.at("x").at(tr, edge_id).get(GRB_DoubleAttr_X) > 0.5) {
        const auto& edge_object = instance.const_n().get_edge(edge_id);
        current_pos += edge_object.length;
        route_marker_tr.emplace_back(edge_object.target, current_pos);
        const auto& [old_edge_id, old_edge_pos] =
            instance.const_n().get_old_edge(edge_id);
        if (old_edge_pos == 0) {
          sol.push_back_edge_to_route(tr_object.name, old_edge_id);
          tr_routed = true;
        }
        edges_to_consider = instance.const_n().out_edges(
            instance.const_n().get_edge(edge_id).target);
      }
    }
    route_markers.push_back(route_marker_tr);
    sol.set_train_routed_value(tr_object.name, tr_routed);
  }

  // Save routing times
  PLOGD << "Setting timings and velocities...";
  for (int tr = 0; tr < num_tr; tr++) {
    const auto& tr_object = instance.get_train_list().get_train(tr);
    for (const auto& [vertex_id, pos] : route_markers[tr]) {
      const auto time_1 =
          vars.at("t_front_arrival").at(tr, vertex_id).get(GRB_DoubleAttr_X);
      const auto time_2 =
          vars.at("t_front_departure").at(tr, vertex_id).get(GRB_DoubleAttr_X);
      const auto vertex_speed = extract_speed(tr, vertex_id);
      sol.add_train_pos(tr_object.name, time_1, pos);
      sol.add_train_speed(tr_object.name, time_1, vertex_speed);
      if (time_2 > time_1 + GRB_EPS) {
        sol.add_train_pos(tr_object.name, time_2, pos);
        sol.add_train_speed(tr_object.name, time_2, vertex_speed);
      }
    }
  }

  PLOGI << "DONE! Solution extracted.";
}

double cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::extract_speed(
    size_t tr, size_t vertex_id) const {
  assert(model->get(GRB_IntAttr_SolCount) >= 1);
  const auto& tr_object      = instance.get_train_list().get_train(tr);
  const auto  delta_consider = instance.const_n().neighboring_edges(vertex_id);
  const auto  edges_used_by_td =
      instance.edges_used_by_train(tr, model_detail.fix_routes, false);
  std::vector<size_t> edges_to_consider;
  for (const auto& edge_id : delta_consider) {
    if (std::find(edges_used_by_td.begin(), edges_used_by_td.end(), edge_id) !=
        edges_used_by_td.end()) {
      edges_to_consider.push_back(edge_id);
    }
  }

  for (const auto& edge_id : edges_to_consider) {
    const auto edge_obj = instance.const_n().get_edge(edge_id);
    assert(edge_obj.source == vertex_id || edge_obj.target == vertex_id);
    const auto v1_extensions = velocity_extensions.at(tr).at(edge_obj.source);
    const auto v2_extensions = velocity_extensions.at(tr).at(edge_obj.target);
    for (size_t v1_idx = 0; v1_idx < v1_extensions.size(); v1_idx++) {
      const auto& v1 = v1_extensions.at(v1_idx);
      for (size_t v2_idx = 0; v2_idx < v2_extensions.size(); v2_idx++) {
        const auto& v2 = v2_extensions.at(v2_idx);
        if (possible_by_eom(v1, v2, tr_object.acceleration,
                            tr_object.deceleration, edge_obj.length)) {
          GRBVar rel_var = vars.at("y").at(tr, edge_id, v1_idx, v2_idx);
          if (!rel_var.sameAs(GRBVar()) &&
              rel_var.get(GRB_DoubleAttr_X) > 0.5) {
            return edge_obj.source == vertex_id ? v1 : v2;
          }
        }
      }
    }
  }
  throw exceptions::ConsistencyException("No speed found for train " +
                                         tr_object.name + " at vertex " +
                                         std::to_string(vertex_id));
}
