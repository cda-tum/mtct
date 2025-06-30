#include "solver/astar-based/GenPOMovingBlockAStarSolver.hpp"

#include "EOMHelper.hpp"
#include "plog/Log.h"
#include "plog/Logger.h"
#include "plog/Severity.h"
#include "simulator/GreedyHeuristic.hpp"
#include "simulator/GreedySimulator.hpp"

#include <algorithm>
#include <cstddef>
#include <unordered_set>
#include <vector>

std::unordered_set<cda_rail::solver::astar_based::GreedySimulatorState>
cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::
    next_states_single_edge(
        const cda_rail::simulator::GreedySimulator& simulator) {
  /**
   * This function determines all possible next states. This state could be
   * obtained by:
   * - a new train entering the network
   * - any single train advancing to a next edge
   * - any single train stopping at the current edge (if it is a valid next stop
   * edge)
   */

  std::unordered_set<GreedySimulatorState> next_states;

  for (size_t tr = 0;
       tr < simulator.get_instance()->get_timetable().get_train_list().size();
       ++tr) {
    const auto& train_edges = simulator.get_train_edges_of_tr(tr);
    if (train_edges.empty()) {
      // Train can enter the network
      const auto& tr_schedule =
          simulator.get_instance()->get_timetable().get_schedule(tr);
      const auto& tr_obj =
          simulator.get_instance()->get_train_list().get_train(tr);
      const auto entry_paths =
          simulator.get_instance()
              ->const_n()
              .all_paths_of_length_starting_in_vertex(
                  tr_schedule.get_entry(),
                  cda_rail::braking_distance(tr_schedule.get_v_0(),
                                             tr_obj.deceleration),
                  tr_schedule.get_exit());
      for (const auto& path : entry_paths) {
        GreedySimulatorState new_state{
            .train_edges    = simulator.get_train_edges(),
            .ttd_orders     = simulator.get_ttd_orders(),
            .vertex_orders  = simulator.get_vertex_orders(),
            .stop_positions = simulator.get_stop_positions()};
        new_state.train_edges.at(tr) = path;
        new_state.vertex_orders.at(tr_schedule.get_entry()).emplace_back(tr);
        next_state_ttd_helper(tr, new_state, simulator, path);
        next_state_exit_vertex_helper(tr, new_state, simulator);
        next_states.insert(new_state);
      }
    } else {
      if (simulator.is_current_pos_valid_stop_position(tr)) {
        // Train can stop at the current edge
        GreedySimulatorState new_state{
            .train_edges    = simulator.get_train_edges(),
            .ttd_orders     = simulator.get_ttd_orders(),
            .vertex_orders  = simulator.get_vertex_orders(),
            .stop_positions = simulator.get_stop_positions()};
        new_state.stop_positions.at(tr).emplace_back(
            simulator.train_edge_length(tr));
        next_state_exit_vertex_helper(tr, new_state, simulator);
        next_states.insert(new_state);
      }
      const auto next_edges =
          simulator.get_instance()->const_n().get_successors(
              simulator.get_train_edges_of_tr(tr).back());
      for (const auto& next_edge : next_edges) {
        GreedySimulatorState new_state{
            .train_edges    = simulator.get_train_edges(),
            .ttd_orders     = simulator.get_ttd_orders(),
            .vertex_orders  = simulator.get_vertex_orders(),
            .stop_positions = simulator.get_stop_positions()};
        new_state.train_edges.at(tr).emplace_back(next_edge);
        next_state_ttd_helper(tr, new_state, simulator, {next_edge});
        next_state_exit_vertex_helper(tr, new_state, simulator);
        next_states.insert(new_state);
      }
    }
  }

  return next_states;
}

void cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::
    next_state_ttd_helper(
        size_t tr, cda_rail::solver::astar_based::GreedySimulatorState& state,
        const cda_rail::simulator::GreedySimulator& simulator,
        const std::vector<size_t>&                  new_edges) {
  const auto& ttd_sections = simulator.get_ttd_sections();

  for (size_t ttd_id = 0; ttd_id < ttd_sections.size(); ++ttd_id) {
    if (std::find(state.ttd_orders.at(ttd_id).begin(),
                  state.ttd_orders.at(ttd_id).end(),
                  tr) != state.ttd_orders.at(ttd_id).end()) {
      // Train is already in the TTD section, no need to check further
      continue;
    }
    const auto& ttd_section = ttd_sections.at(ttd_id);
    for (const auto& edge : new_edges) {
      if (std::find(ttd_section.begin(), ttd_section.end(), edge) !=
          ttd_section.end()) {
        // Edge is part of the TTD section
        state.ttd_orders.at(ttd_id).emplace_back(tr);
        break; // No need to check further edge
      }
    }
  }
}

void cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::
    next_state_exit_vertex_helper(
        size_t tr, cda_rail::solver::astar_based::GreedySimulatorState& state,
        const cda_rail::simulator::GreedySimulator& simulator) {
  const auto& last_edge_id = state.train_edges.at(tr).back();
  const auto& last_edge =
      simulator.get_instance()->const_n().get_edge(last_edge_id);
  const auto& tr_schedule =
      simulator.get_instance()->get_timetable().get_schedule(tr);
  if ((tr_schedule.get_exit() == last_edge.target) &&
      (state.stop_positions.at(tr).size() == tr_schedule.get_stops().size())) {
    if (std::find(state.vertex_orders.at(last_edge.target).begin(),
                  state.vertex_orders.at(last_edge.target).end(),
                  tr) == state.vertex_orders.at(last_edge.target).end()) {
      // Train has reached the exit vertex, add it to the vertex orders
      state.vertex_orders.at(last_edge.target).emplace_back(tr);
    }
  }
}

cda_rail::instances::SolGeneralPerformanceOptimizationInstance<
    cda_rail::instances::GeneralPerformanceOptimizationInstance>
cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::solve(
    const cda_rail::solver::astar_based::ModelDetail& model_detail_input,
    const cda_rail::solver::astar_based::SolverStrategyMBAStar&
                                                     solver_strategy_input,
    const cda_rail::solver::GeneralSolutionSettings& solution_settings_input,
    int time_limit, bool debug_input, bool overwrite_severity) {
  this->solve_init_general(time_limit, debug_input, overwrite_severity);

  const auto ttd_section = instance.const_n().unbreakable_sections();
  simulator::GreedySimulator simulator(instance, ttd_section);

  std::unordered_set<GreedySimulatorState> explored_states;
  MinPriorityQueue                         pq;

  PLOGI << "Starting A* search";

  const auto [init_feas, init_exit_times, init_braking, init_headways] =
      simulator.simulate(model_detail_input.dt,
                         model_detail_input.late_entry_possible,
                         model_detail_input.late_exit_possible,
                         model_detail_input.late_stop_possible,
                         model_detail_input.limit_speed_by_leaving_edges);
  const auto init_obj = simulator::objective_val(simulator, init_exit_times);
  const auto init_heuristic = simulator::full_greedy_heuristic(
      solver_strategy_input.braking_time_heuristic_type,
      solver_strategy_input.remaining_time_heuristic_type, simulator,
      init_exit_times, init_braking, model_detail_input.late_stop_possible,
      model_detail_input.late_exit_possible,
      solver_strategy_input.consider_earliest_exit);

  PLOGD << "Initial state: final = "
        << (simulator.is_final_state() ? "yes" : "no")
        << ", objective = " << init_obj
        << ", heuristic = " << init_heuristic.second
        << ", total = " << init_obj + init_heuristic.second
        << ", heuristic feasibility = "
        << (init_heuristic.first ? "feasible" : "infeasible");

  if (init_feas && init_heuristic.first) {
    const GreedySimulatorState init_state{
        .train_edges    = simulator.get_train_edges(),
        .ttd_orders     = simulator.get_ttd_orders(),
        .vertex_orders  = simulator.get_vertex_orders(),
        .stop_positions = simulator.get_stop_positions()};
    pq.push({{init_obj + init_heuristic.second, simulator.is_final_state()},
             init_state});
    explored_states.insert(init_state);
  }

  size_t iteration = 0;

  // A* iteration
  while (!pq.empty()) {
    iteration++;
    PLOGD << "----------------------------";
    PLOGD << "Iteration " << iteration << ", queue size: " << pq.size();

    const auto [current_obj, current_state] = pq.top();
    pq.pop();
    PLOGD << "Exploring state with objective = " << current_obj.first
          << ", final = " << (current_obj.second ? "yes" : "no");
    if (current_obj.second) {
      PLOGI << "Found final state with objective = " << current_obj.first
            << " after " << iteration << " iterations.";
      return {};
    }

    simulator.set_train_edges(current_state.train_edges);
    simulator.set_ttd_orders(current_state.ttd_orders);
    simulator.set_vertex_orders(current_state.vertex_orders);
    simulator.set_stop_positions(current_state.stop_positions);

    const auto next_states_set =
        next_states(simulator, solver_strategy_input.next_state_strategy);
    PLOGD << "Found " << next_states_set.size() << " next states.";
    size_t i = 0;
    for (const auto& s : next_states_set) {
      i++;
      PLOGD << "Processing next state " << i << "/" << next_states_set.size();
      simulator.set_train_edges(s.train_edges);
      simulator.set_ttd_orders(s.ttd_orders);
      simulator.set_vertex_orders(s.vertex_orders);
      simulator.set_stop_positions(s.stop_positions);
      if (explored_states.contains(s)) {
        PLOGD << "State already explored, skipping.";
        continue;
      }

      const auto [feas, exit_times, braking_times, headways] =
          simulator.simulate(model_detail_input.dt,
                             model_detail_input.late_entry_possible,
                             model_detail_input.late_exit_possible,
                             model_detail_input.late_stop_possible,
                             model_detail_input.limit_speed_by_leaving_edges);
      if (!feas) {
        PLOGD << "State is infeasible, skipping.";
        continue;
      }
      const auto obj = simulator::objective_val(simulator, exit_times);
      const auto [heuristic_feas, heuristic_val] =
          simulator::full_greedy_heuristic(
              solver_strategy_input.braking_time_heuristic_type,
              solver_strategy_input.remaining_time_heuristic_type, simulator,
              exit_times, braking_times, model_detail_input.late_stop_possible,
              model_detail_input.late_exit_possible,
              solver_strategy_input.consider_earliest_exit);
      const auto new_obj = obj + heuristic_val;
      const auto final   = simulator.is_final_state();
      PLOGD << "Objective = " << obj << ", heuristic = " << heuristic_val
            << ", total = " << new_obj << ", feasibility = "
            << (heuristic_feas ? "feasible" : "infeasible")
            << ", final = " << (final ? "yes" : "no");
      if (heuristic_feas) {
        pq.push({{new_obj, final}, s});
        explored_states.insert(s);
        PLOGD << "State added to priority queue.";
      }
    }
  }

  PLOGI << "No final state found after " << iteration << " iterations.";

  return {};
}
