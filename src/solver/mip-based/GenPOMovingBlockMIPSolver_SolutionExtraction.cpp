#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "EOMHelper.hpp"
#include "gurobi_c++.h"
#include "gurobi_c.h"
#include "plog/Log.h"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"
#include "solver/mip-based/GenPOMovingBlockMIPSolver.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

using std::size_t;

// NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay,performance-inefficient-string-concatenation,bugprone-unchecked-optional-access)

void cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::extract_solution(
    cda_rail::instances::SolGeneralPerformanceOptimizationInstance& sol) const {
  PLOGI << "Extracting solution object...";

  // Is there a solution?
  if (const auto grb_status = m_model->get(GRB_IntAttr_Status);
      grb_status == GRB_OPTIMAL) {
    PLOGD << "Solution status: Optimal";
    sol.set_status(SolutionStatus::Optimal);
  } else if (grb_status == GRB_INFEASIBLE) {
    PLOGD << "Solution status: Infeasible";
    sol.set_status(SolutionStatus::Infeasible);
  } else if (m_model->get(GRB_IntAttr_SolCount) >= 1) {
    PLOGD << "Solution status: Feasible (optimality unknown)";
    sol.set_status(SolutionStatus::Feasible);
  } else if (grb_status == GRB_TIME_LIMIT &&
             m_model->get(GRB_IntAttr_SolCount) == 0) {
    PLOGD << "Solution status: Timeout (Feasibility unknown)";
    sol.set_status(SolutionStatus::Timeout);
  } else {
    PLOGE << "Solution status code " << grb_status << " unknown";
    throw exceptions::ConsistencyException(
        "Gurobi status code " + std::to_string(grb_status) + " unknown.");
  }

  if (const auto sol_count = m_model->get(GRB_IntAttr_SolCount);
      sol_count < 0.5) {
    sol.set_solution_not_found();
    return;
  }

  const auto mip_obj_val =
      static_cast<int>(std::round(m_model->get(GRB_DoubleAttr_ObjVal)));
  sol.set_solution_found();
  sol.set_obj(mip_obj_val);
  PLOGD << "MIP objective: " << mip_obj_val;

  // Extract routes
  PLOGD << "Setting routes...";
  std::vector<std::vector<std::pair<size_t, double>>> route_markers;
  route_markers.reserve(m_num_tr);
  sol.reset_routes();
  for (size_t tr = 0; tr < m_num_tr; tr++) {
    bool        tr_routed = false;
    const auto& tr_object = m_instance.get_const_train_list().get_train(tr);
    sol.add_empty_route(tr_object.get_name());
    const auto entry = m_instance.get_const_schedule(tr).get_entry_vertex();
    auto edges_to_consider = m_instance.get_const_network().out_edges(entry);

    double                                 current_pos = 0;
    std::vector<std::pair<size_t, double>> route_marker_tr;
    route_marker_tr.emplace_back(entry, current_pos);
    while (!edges_to_consider.empty()) {
      const auto& edge_id_ptr = edges_to_consider.begin();
      auto const  edge_id     = *edge_id_ptr;
      edges_to_consider.erase(edge_id_ptr);
      GRBVar x_var = m_vars.at("x").at(tr, edge_id);
      if (!x_var.sameAs(GRBVar()) && x_var.get(GRB_DoubleAttr_X) > 0.5) {
        const auto& edge_object =
            m_instance.get_const_network().get_edge(edge_id);
        current_pos += edge_object.length;
        route_marker_tr.emplace_back(edge_object.target, current_pos);
        const auto& [old_edge_id, old_edge_pos] =
            m_instance.get_const_network().get_old_edge(edge_id);
        if (old_edge_pos == 0) {
          sol.push_back_edge_to_route(tr_object.get_name(), old_edge_id);
          tr_routed = true;
        }
        edges_to_consider = m_instance.get_const_network().out_edges(
            m_instance.get_const_network().get_edge(edge_id).target);
      }
    }
    route_markers.push_back(route_marker_tr);
  }

  // Save routing times
  PLOGD << "Setting timings and velocities...";
  for (size_t tr = 0; tr < m_num_tr; tr++) {
    const auto& tr_object   = m_instance.get_const_train_list().get_train(tr);
    const auto& tr_schedule = m_instance.get_const_schedule(tr);
    for (const auto& [vertex_id, pos] : route_markers[tr]) {
      const auto time_1 =
          m_vars.at("t_front_arrival").at(tr, vertex_id).get(GRB_DoubleAttr_X);
      const auto time_2       = m_vars.at("t_front_departure")
                                    .at(tr, vertex_id)
                                    .get(GRB_DoubleAttr_X);
      const auto vertex_speed = extract_speed(tr, vertex_id);
      sol.add_train_pos(tr_object.get_name(), time_1, pos);
      sol.add_train_speed(tr_object.get_name(), time_1, vertex_speed);
      if (time_2 > time_1 + GRB_EPS) {
        sol.add_train_pos(tr_object.get_name(), time_2, pos);
        sol.add_train_speed(tr_object.get_name(), time_2, vertex_speed);
      }

      if (vertex_id == tr_schedule.get_exit_vertex()) {
        const auto last_time  = m_vars.at("t_rear_departure")
                                    .at(tr, vertex_id)
                                    .get(GRB_DoubleAttr_X);
        const auto last_speed = tr_schedule.get_exit_velocity();
        sol.add_train_pos(tr_object.get_name(), last_time,
                          pos + tr_object.get_length());
        sol.add_train_speed(tr_object.get_name(), last_time, last_speed);
      }
    }
  }

  PLOGI << "DONE! Solution extracted.";
}

double cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::extract_speed(
    size_t tr, size_t vertex_id) const {
  assert(m_model->get(GRB_IntAttr_SolCount) >= 1);
  const auto& tr_object = m_instance.get_const_train_list().get_train(tr);
  const auto  delta_consider =
      m_instance.get_const_network().neighboring_edges(vertex_id);
  const auto edges_used_by_td =
      m_instance.edges_used_by_train(tr, m_model_detail.fix_routes, false);
  cda_rail::index_vector edges_to_consider;
  for (const auto& edge_id : delta_consider) {
    if (std::ranges::contains(edges_used_by_td, edge_id)) {
      edges_to_consider.push_back(edge_id);
    }
  }

  for (const auto& edge_id : edges_to_consider) {
    const auto edge_obj = m_instance.get_const_network().get_edge(edge_id);
    assert(edge_obj.source == vertex_id || edge_obj.target == vertex_id);
    const auto v1_extensions = m_velocity_extensions.at(tr).at(edge_obj.source);
    const auto v2_extensions = m_velocity_extensions.at(tr).at(edge_obj.target);
    for (size_t v1_idx = 0; v1_idx < v1_extensions.size(); v1_idx++) {
      const auto& v1 = v1_extensions.at(v1_idx);
      for (size_t v2_idx = 0; v2_idx < v2_extensions.size(); v2_idx++) {
        const auto& v2 = v2_extensions.at(v2_idx);
        if (possible_by_eom(v1, v2, tr_object.get_acceleration(),
                            tr_object.get_deceleration(), edge_obj.length)) {
          GRBVar rel_var = m_vars.at("y").at(tr, edge_id, v1_idx, v2_idx);
          if (!rel_var.sameAs(GRBVar()) &&
              rel_var.get(GRB_DoubleAttr_X) > 0.5) {
            return edge_obj.source == vertex_id ? v1 : v2;
          }
        }
      }
    }
  }
  throw exceptions::ConsistencyException("No speed found for train " +
                                         tr_object.get_name() + " at vertex " +
                                         std::to_string(vertex_id));
}

// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay,performance-inefficient-string-concatenation,bugprone-unchecked-optional-access)
