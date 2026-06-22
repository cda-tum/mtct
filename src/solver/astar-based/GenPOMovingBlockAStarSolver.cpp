#include "solver/astar-based/GenPOMovingBlockAStarSolver.hpp"

#include "Definitions.hpp"
#include "EOMHelper.hpp"
#include "plog/Log.h"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"
#include "simulator/GreedyHeuristic.hpp"
#include "simulator/GreedySimulator.hpp"
#include "solver/GeneralSolver.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <iterator>
#include <optional>
#include <ranges>
#include <unordered_set>
#include <vector>

// --------------------
// SOLVE
// --------------------

// NOLINTBEGIN (cppcoreguidelines-pro-type-reinterpret-cast)
cda_rail::instances::SolGeneralPerformanceOptimizationInstance
cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::solve(
    const cda_rail::solver::astar_based::ModelDetail& model_detail_input,
    const cda_rail::solver::astar_based::SolverStrategyMBAStar&
                                                     solver_strategy_input,
    const cda_rail::solver::GeneralSolutionSettings& solution_settings_input,
    int time_limit, bool debug_input, bool overwrite_severity) {
  this->solve_init_general(time_limit, debug_input, overwrite_severity);

  cda_rail::exceptions::throw_if_less_than(solver_strategy_input.a_star_weight,
                                           1.0, "A* weight");

  const auto ttd_sections =
      get_instance().get_const_network().unbreakable_sections();
  simulator::GreedySimulator simulator(get_instance(), ttd_sections);

  std::unordered_set<simulator::SimulatorState> explored_states;
  MinPriorityQueue                              pq;

  cda_rail::instances::SolGeneralPerformanceOptimizationInstance sol_object(
      get_instance());

  sol_object.reset_routes();

  m_model_created =
      std::chrono::high_resolution_clock::now(); // Start model creation timer

  PLOGI << "Starting A* search";

  // const auto [init_feas, init_exit_times, init_braking, init_headways] =
  const auto init_simulator_result = simulator.simulate(
      model_detail_input.dt, model_detail_input.late_entry_possible,
      model_detail_input.limit_speed_by_leaving_edges, false,
      solver_strategy_input.time_aware_state_transitions);
  const auto init_obj =
      simulator::objective_val(simulator, init_simulator_result.exit_times,
                               init_simulator_result.stop_times);
  const auto [init_heuristic_feas, init_heuristic_val] =
      simulator::full_greedy_heuristic(
          solver_strategy_input.remaining_time_heuristic_type, simulator,
          init_simulator_result, solver_strategy_input.consider_earliest_exit);

  PLOGD << "Initial state: final = "
        << (simulator.is_final_state() ? "yes" : "no")
        << ", objective = " << init_obj
        << ", heuristic = " << init_heuristic_val
        << ", total = " << init_obj + init_heuristic_val
        << ", heuristic feasibility = "
        << (init_heuristic_feas ? "feasible" : "infeasible");

  if (init_simulator_result.success && init_heuristic_feas) {
    const simulator::SimulatorState init_state{
        .train_edges    = simulator.get_train_edges(),
        .ttd_orders     = simulator.get_ttd_orders(),
        .vertex_orders  = simulator.get_vertex_orders(),
        .stop_positions = simulator.get_stop_positions()};
    pq.push(
        {init_obj + solver_strategy_input.a_star_weight * init_heuristic_val,
         simulator.is_final_state(), init_state, init_simulator_result});
    explored_states.insert(init_state);
  }

  size_t                    iteration = 0;
  double                    best_obj  = cda_rail::INF;
  simulator::SimulatorState best_state;

  // A* iteration
  while (!pq.empty()) {
    // If timeout is reached break the loop
    if (time_limit > 0) {
      const auto now = std::chrono::high_resolution_clock::now();
      const auto elapsed =
          std::chrono::duration_cast<std::chrono::seconds>(now - m_start)
              .count();
      if (elapsed >= time_limit) {
        PLOGD << "Timeout reached after " << elapsed
              << " seconds, stopping search.";
        if (!sol_object.has_solution()) {
          sol_object.set_status(cda_rail::SolutionStatus::Timeout);
        }
        break;
      }
    }

    iteration++;

    const auto current_state_objective_pair = pq.top();
    pq.pop();

    if (iteration % DEBUG_LOGGING_RATE == 0) {
      PLOGD << "----------------------------";
      PLOGD << "Iteration " << iteration << ", queue size: " << pq.size();
      PLOGD << "Best objective so far: " << best_obj;
      PLOGD << "Current lower bound: "
            << current_state_objective_pair.objective;
    } else {
      PLOGV << "----------------------------";
      PLOGV << "Iteration " << iteration << ", queue size: " << pq.size();
      PLOGV << "Best objective so far: " << best_obj;
      PLOGV << "Current lower bound: "
            << current_state_objective_pair.objective;
    }

    if (current_state_objective_pair.is_final_state) {
      PLOGD << "Optimal solution found, obj = "
            << current_state_objective_pair.objective << ", after " << iteration
            << " iterations, "
            << std::chrono::duration_cast<std::chrono::seconds>(
                   std::chrono::high_resolution_clock::now() - m_start)
                   .count()
            << " seconds.";
      best_obj   = current_state_objective_pair.objective;
      best_state = current_state_objective_pair.state;
      sol_object.set_obj(best_obj);
      sol_object.set_solution_found();
      sol_object.set_status(cda_rail::SolutionStatus::Optimal);
      break;
    }

    const auto next_states_set =
        next_states(current_state_objective_pair.state,
                    current_state_objective_pair.results, model_detail_input,
                    solver_strategy_input, &get_instance(), ttd_sections);
    PLOGV << "Found " << next_states_set.size() << " next states.";
    size_t i = 0;
    for (const auto& s : next_states_set) {
      i++;
      PLOGV << "Processing next state " << i << "/" << next_states_set.size();
      if (explored_states.contains(s)) {
        PLOGV << "State already explored, skipping.";
        continue;
      }
      simulator.set_simulator_state(s);

      const auto sim_res = simulator.simulate(
          model_detail_input.dt, model_detail_input.late_entry_possible,
          model_detail_input.limit_speed_by_leaving_edges, false,
          solver_strategy_input.time_aware_state_transitions);
      if (!sim_res.success) {
        PLOGV << "State is infeasible, skipping.";
        continue;
      }
      const auto obj = simulator::objective_val(simulator, sim_res.exit_times,
                                                sim_res.stop_times);
      const auto [heuristic_feas, heuristic_val] =
          simulator::full_greedy_heuristic(
              solver_strategy_input.remaining_time_heuristic_type, simulator,
              sim_res, solver_strategy_input.consider_earliest_exit);
      const auto new_obj =
          obj + solver_strategy_input.a_star_weight * heuristic_val;
      const auto final = simulator.is_final_state();
      PLOGV << "Objective = " << obj << ", heuristic = " << heuristic_val
            << ", total = " << new_obj << ", feasibility = "
            << (heuristic_feas ? "feasible" : "infeasible")
            << ", final = " << (final ? "yes" : "no");
      if (final && new_obj < best_obj) {
        PLOGD << "Explored new best final state with objective = " << new_obj
              << " after " << iteration << " iterations, "
              << std::chrono::duration_cast<std::chrono::seconds>(
                     std::chrono::high_resolution_clock::now() - m_start)
                     .count()
              << " seconds.";
        best_obj   = new_obj;
        best_state = s;
        sol_object.set_obj(best_obj);
        sol_object.set_solution_found();
        sol_object.set_status(cda_rail::SolutionStatus::Feasible);
      }
      if (heuristic_feas) {
        pq.push({new_obj, final, s, sim_res});
        explored_states.insert(s);
        PLOGV << "State added to priority queue.";
      }
    }
  }

  m_model_solved =
      std::chrono::high_resolution_clock::now(); // Finished model solving

  PLOGD << "Terminated after " << iteration << " iterations, "
        << (std::chrono::duration_cast<std::chrono::milliseconds>(
                m_model_solved - m_start)
                .count() /
            1000.0)
        << " seconds.";

  PLOGI << "Extracting solution object...";

  if (sol_object.has_solution()) {
    // Add solution data to the solution object

    simulator.set_simulator_state(best_state);

    // Determine trajectories
    const auto final_simulation_result = simulator.simulate(
        model_detail_input.dt, model_detail_input.late_entry_possible,
        model_detail_input.limit_speed_by_leaving_edges, true,
        false); // no partial routes should exist
    if (!final_simulation_result.success) {
      throw cda_rail::exceptions::ConsistencyException(
          "Final trajectory extraction failed for a previously feasible "
          "state.");
    }

    for (size_t tr = 0; tr < get_instance().get_const_train_list().size();
         ++tr) {
      const auto& tr_name = sol_object.get_instance()
                                ->get_const_train_list()
                                .get_train(tr)
                                .get_name();
      const auto& tr_trajectory =
          final_simulation_result.train_trajectories.at(tr);
      const auto& tr_edges = best_state.train_edges.at(tr);

      if (tr_edges.empty()) {
        throw std::runtime_error("Empty train edge list for train " + tr_name);
      }
      sol_object.add_empty_route(tr_name);
      for (const auto& e : tr_edges) {
        sol_object.push_back_edge_to_route(tr_name, e);
      }
      for (const auto& [time, posvel] : tr_trajectory) {
        sol_object.add_train_pos(tr_name, time, posvel.pos);
        sol_object.add_train_speed(tr_name, time, posvel.vel);
      }
    }
  }

  if (pq.empty() && !sol_object.has_solution()) {
    sol_object.set_status(cda_rail::SolutionStatus::Infeasible);
  }

  PLOGI << "DONE! Solution extracted.";

  switch (sol_object.get_status()) {
  case cda_rail::SolutionStatus::Optimal:
    PLOGI << "Found optimal solution with objective " << sol_object.get_obj();
    break;
  case cda_rail::SolutionStatus::Feasible:
    PLOGI << "Found feasible solution with objective " << sol_object.get_obj();
    break;
  case cda_rail::SolutionStatus::Infeasible:
    PLOGI << "Problem is infeasible";
    break;
  case cda_rail::SolutionStatus::Timeout:
    PLOGI << "Search terminated due to timeout.";
    break;
  default:
    PLOGW << "Unknown solution status encountered.";
    break;
  }

  if (solution_settings_input.export_option ==
          GeneralExportOption::ExportSolution ||
      solution_settings_input.export_option ==
          GeneralExportOption::ExportSolutionWithInstance) {
    const bool export_instance =
        (solution_settings_input.export_option ==
         GeneralExportOption::ExportSolutionWithInstance);
    PLOGI << "Saving solution";
    sol_object.export_solution(solution_settings_input.working_directory,
                               solution_settings_input.solution_subdirectory,
                               export_instance,
                               solution_settings_input.parameter_identifier);
  }

  return sol_object;
}
// NOLINTEND (cppcoreguidelines-pro-type-reinterpret-cast)

// -----------------------
// PRIVATE HELPER
// ----------------------

cda_rail::index_set cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::
    get_all_trains_for_state_transition(
        simulator::SimulatorState const& simulator_state,
        instances::GeneralPerformanceOptimizationInstance const* instance) {
  // All trains whose last edge is not their exit edge
  index_set trains;
  std::ranges::copy_if(
      std::views::iota(size_t{0}, instance->get_const_train_list().size()),
      std::inserter(trains, trains.end()),
      [&simulator_state, &instance](size_t const tr) {
        return simulator_state.train_edges.at(tr).empty() ||
               (instance->get_const_network()
                    .get_edge(simulator_state.train_edges.at(tr).back())
                    .target !=
                instance->get_const_schedule(tr).get_exit_vertex());
      });
  return trains;
}

cda_rail::index_set cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::
    get_next_time_aware_train(
        simulator::SimulatorState const&   simulator_state,
        simulator::SimulatorResults const& simulator_result,
        instances::GeneralPerformanceOptimizationInstance const* instance) {
  /**
   * In the time-aware case, ordering is done by simulated exit time / scheduled
   * entry. In case of tie, trains with higher weight win. In case of tie,
   * existing trains win over new trains. In case of tie, either train wins. No
   * train is returned if all trains have left the network. At most one train is
   * returned.
   */

  struct Candidate {
    size_t train;
    double time;
    double weight;
    bool   is_on_network;
  };

  auto const candidate_for = [&simulator_state, &simulator_result, &instance](
                                 size_t const tr) -> std::optional<Candidate> {
    auto const& train_edges = simulator_state.train_edges.at(tr);
    auto const  weight      = instance->get_train_weight(tr);

    if (train_edges.empty()) {
      return Candidate{.train = tr,
                       .time =
                           instance->get_const_schedule(tr).get_entry_time(),
                       .weight        = weight,
                       .is_on_network = false};
    }

    auto const exit_vertex = instance->get_const_schedule(tr).get_exit_vertex();
    auto const current_vertex =
        instance->get_const_network().get_edge(train_edges.back()).target;
    if (current_vertex == exit_vertex) {
      return std::nullopt;
    }

    return Candidate{.train         = tr,
                     .time          = simulator_result.exit_times.at(tr),
                     .weight        = weight,
                     .is_on_network = true};
  };

  auto const is_better = [](Candidate const& candidate,
                            Candidate const& incumbent) {
    if (candidate.time != incumbent.time) {
      return candidate.time < incumbent.time;
    }
    if (candidate.weight != incumbent.weight) {
      return candidate.weight > incumbent.weight;
    }
    return candidate.is_on_network && !incumbent.is_on_network;
  };

  std::vector<Candidate> candidates;
  for (auto const tr :
       std::views::iota(size_t{0}, instance->get_const_train_list().size())) {
    if (auto const candidate = candidate_for(tr); candidate.has_value()) {
      candidates.emplace_back(candidate.value());
    }
  }

  if (candidates.empty()) {
    return index_set{};
  }

  auto const best = std::ranges::min_element(
      candidates, [&is_better](Candidate const& lhs, Candidate const& rhs) {
        return is_better(lhs, rhs);
      });
  return {best->train};
}

cda_rail::index_set cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::
    get_relevant_trains_for_state_transition(
        simulator::SimulatorState const&   simulator_state,
        simulator::SimulatorResults const& simulator_result,
        instances::GeneralPerformanceOptimizationInstance const* instance,
        SolverStrategyMBAStar const& solver_strategy) {
  /**
   * Returns "all" trains if not time-aware. Returns only the next train if
   * time-aware. Trains with fully specified route are not returned. In the
   * time-aware case, ordering is done by braking time / scheduled entry. In
   * case of tie, trains with higher weight win. In case of tie, existing
   * trains win over new trains. In case of tie, either train wins.
   */

  return solver_strategy.time_aware_state_transitions
             ? get_next_time_aware_train(simulator_state, simulator_result,
                                         instance)
             : get_all_trains_for_state_transition(simulator_state, instance);
}

std::vector<cda_rail::index_vector>
cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::get_entry_paths(
    size_t                                                   tr,
    instances::GeneralPerformanceOptimizationInstance const* instance) {
  const auto& tr_schedule = instance->get_const_schedule(tr);
  const auto& tr_obj      = instance->get_const_train_list().get_train(tr);
  return instance->get_const_network().all_paths_of_length_starting_in_vertex(
      tr_schedule.get_entry_vertex(),
      cda_rail::braking_distance(tr_schedule.get_initial_velocity(),
                                 tr_obj.get_deceleration()),
      tr_schedule.get_exit_vertex(), {}, true);
}

std::vector<cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::
                PathExtensionData>
cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::get_path_extensions(
    size_t tr, simulator::SimulatorState const& simulator_state,
    NextStateStrategy next_state_strategy,
    instances::GeneralPerformanceOptimizationInstance const* instance,
    std::vector<cda_rail::index_set> const&                  ttd_sections) {
  std::vector<PathExtensionData> path_extensions{};

  auto const& tr_edges    = simulator_state.train_edges.at(tr);
  bool        tr_entering = tr_edges.empty();

  if (tr_entering) {
    const auto entry_paths = get_entry_paths(tr, instance);
    for (auto const& entry_path : entry_paths) {
      path_extensions.emplace_back(entry_path, entry_path.size() - 1);
    }
  } else {
    auto const& successor_edges =
        instance->get_const_network().get_successors(tr_edges.back());
    for (auto const& successor_edge : successor_edges) {
      path_extensions.emplace_back(cda_rail::index_vector({successor_edge}), 0);
    }
  }

  if (next_state_strategy == NextStateStrategy::SingleEdge) {
    return path_extensions;
  }

  assert(next_state_strategy == NextStateStrategy::NextTTD ||
         next_state_strategy == NextStateStrategy::NextRelevantTTD);

  // For the next TTD strategy, we need to extend the path until the next TTD
  // (multiple options per path possible)
  std::vector<PathExtensionData> extended_path_extensions;
  for (auto const& path_extension : path_extensions) {
    auto const next_ttd_paths =
        instance->get_const_network().all_paths_ending_at_ttd(
            path_extension.path.back(), ttd_sections,
            instance->get_const_schedule(tr).get_exit_vertex(),
            next_state_strategy == NextStateStrategy::NextRelevantTTD, true);
    for (auto const& next_ttd_path : next_ttd_paths) {
      PathExtensionData new_path_extension{
          .path = path_extension.path,
          .stop_possible_from_idx_onward =
              path_extension.stop_possible_from_idx_onward};
      new_path_extension.path.insert(new_path_extension.path.end(),
                                     next_ttd_path.begin(),
                                     next_ttd_path.end());
      extended_path_extensions.emplace_back(new_path_extension);
    }
  }
  return extended_path_extensions;
}

cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::IndexBound
cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::
    infer_order_insertion_bounds(size_t const                  tr,
                                 cda_rail::index_vector const& prev_order,
                                 cda_rail::index_vector const& next_order,
                                 cda_rail::index_set const&    tr_sharing_path,
                                 bool const                    insertAtEnd) {
  if (insertAtEnd) {
    return {.lb = next_order.size(), .ub = next_order.size()};
  }

  // Get position of tr in prev_order
  size_t const tr_idx_in_prev_order =
      std::ranges::find(prev_order, tr) - prev_order.begin();
  assert(tr_idx_in_prev_order < prev_order.size());

  IndexBound retval{.lb = 0, .ub = next_order.size()};
  for (auto const& tr_other : tr_sharing_path) {
    size_t const tr_other_idx_in_prev_order =
        std::ranges::find(prev_order, tr_other) - prev_order.begin();
    size_t const tr_other_idx_in_next_order =
        std::ranges::find(next_order, tr_other) - next_order.begin();

    assert(tr_other_idx_in_prev_order < prev_order.size());
    assert(tr_other_idx_in_next_order < next_order.size());
    assert(tr_idx_in_prev_order != tr_other_idx_in_prev_order);
    if (tr_idx_in_prev_order < tr_other_idx_in_prev_order) {
      // tr travels before tr_other
      retval.ub = std::min(retval.ub, tr_other_idx_in_next_order);
    } else {
      // tr travels after tr_other
      retval.lb = std::max(retval.lb, tr_other_idx_in_next_order + 1);
    }
  }

  return retval;
}

cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::IndexBound
cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::
    infer_order_entry_order_bounds(
        size_t tr, cda_rail::index_vector const& entry_order,
        bool late_entry_possible, bool insert_at_end,
        instances::GeneralPerformanceOptimizationInstance const* instance) {
  if (insert_at_end) {
    return {.lb = entry_order.size(), .ub = entry_order.size()};
  }

  IndexBound retval{.lb = 0, .ub = entry_order.size()};
  if (late_entry_possible) {
    return retval;
  }

  auto const& tr_schedule = instance->get_const_schedule(tr);
  for (size_t tr_other_idx = 0; tr_other_idx < entry_order.size();
       ++tr_other_idx) {
    auto const& tr_other_schedule =
        instance->get_const_schedule(entry_order.at(tr_other_idx));
    if (tr_schedule.get_entry_vertex() ==
            tr_other_schedule.get_entry_vertex() &&
        tr_other_schedule.get_entry_time() < tr_schedule.get_entry_time()) {
      retval.lb = std::max(retval.lb, tr_other_idx + 1);
    }
    if (tr_other_schedule.get_entry_time() > tr_schedule.get_entry_time()) {
      retval.ub = std::min(retval.ub, tr_other_idx);
    }
    if (tr_schedule.get_entry_vertex() == tr_other_schedule.get_exit_vertex() &&
        tr_other_schedule.get_exit_time() > tr_schedule.get_entry_time()) {
      retval.ub = std::min(retval.ub, tr_other_idx);
    }
  }
  return retval;
}

std::vector<cda_rail::simulator::SimulatorState> cda_rail::solver::astar_based::
    GenPOMovingBlockAStarSolver::extend_state_by_path_extension(
        size_t tr, simulator::SimulatorState state,
        PathExtensionData const& path_extension_data,
        instances::GeneralPerformanceOptimizationInstance const* instance) {
  if (state.stop_positions.at(tr).size() >=
      instance->get_const_schedule(tr).get_stops().size()) {
    // add entire path_extension_data.path to state.train_edges.at(tr) since no
    // stop left
    state.train_edges.at(tr).insert(state.train_edges.at(tr).end(),
                                    path_extension_data.path.begin(),
                                    path_extension_data.path.end());
    return {state};
  }

  // at least one stop left
  std::vector<cda_rail::simulator::SimulatorState> retval{};
  for (size_t i = 0; i < path_extension_data.path.size(); ++i) {
    state.train_edges.at(tr).emplace_back(path_extension_data.path.at(i));
    if (i >= path_extension_data.stop_possible_from_idx_onward) {
      if (instance->is_route_end_valid_stop_pos(
              tr, state.train_edges.at(tr),
              state.stop_positions.at(tr).size())) {
        retval.emplace_back(state);
        retval.back().stop_positions.at(tr).emplace_back(
            instance->get_const_network().length_of_path(
                state.train_edges.at(tr)));
      }
    }
  }

  if (instance->get_const_network()
          .get_edge(state.train_edges.at(tr).back())
          .target != instance->get_const_schedule(tr).get_exit_vertex()) {
    // only allowed to add edges without stopping if exit vertex is not reached,
    // because some stops are still open
    retval.emplace_back(state);
  }
  return retval;
}

std::vector<cda_rail::simulator::SimulatorState> cda_rail::solver::astar_based::
    GenPOMovingBlockAStarSolver::extend_train_orders_of_state(
        size_t tr, simulator::SimulatorState state,
        const ModelDetail&                      model_detail_input,
        const SolverStrategyMBAStar&            solver_strategy_input,
        std::vector<cda_rail::index_set> const& ttd_sections,
        instances::GeneralPerformanceOptimizationInstance const* instance) {
  auto const& tr_edges = state.train_edges.at(tr);
  if (tr_edges.empty()) {
    return {state};
  }

  auto const& tr_entry_vertex =
      instance->get_const_schedule(tr).get_entry_vertex();
  auto const tr_entry_order = state.vertex_orders.at(tr_entry_vertex);
  if (std::ranges::contains(tr_entry_order, tr)) {
    return extend_train_orders_of_state_recursive_helper(
        tr, state, solver_strategy_input, ttd_sections, instance,
        tr_entry_order, 0, std::nullopt);
  }

  auto const entry_bounds = infer_order_entry_order_bounds(
      tr, tr_entry_order, model_detail_input.late_entry_possible,
      !solver_strategy_input.time_aware_state_transitions, instance);
  std::vector<cda_rail::simulator::SimulatorState> retval{};
  for (size_t tr_entry_order_idx = entry_bounds.lb;
       tr_entry_order_idx <= entry_bounds.ub; ++tr_entry_order_idx) {
    state.vertex_orders.at(tr_entry_vertex)
        .insert(std::next(state.vertex_orders.at(tr_entry_vertex).begin(),
                          static_cast<std::ptrdiff_t>(tr_entry_order_idx)),
                tr);

    auto const state_extension = extend_train_orders_of_state_recursive_helper(
        tr, state, solver_strategy_input, ttd_sections, instance,
        state.vertex_orders.at(tr_entry_vertex), 0, std::nullopt);
    retval.insert(retval.end(), state_extension.begin(), state_extension.end());

    state.vertex_orders.at(tr_entry_vertex) = tr_entry_order;
  }
  return retval;
}

std::vector<cda_rail::simulator::SimulatorState> cda_rail::solver::astar_based::
    GenPOMovingBlockAStarSolver::extend_train_orders_of_state_recursive_helper(
        size_t tr, simulator::SimulatorState state,
        const SolverStrategyMBAStar&            solver_strategy_input,
        std::vector<cda_rail::index_set> const& ttd_sections,
        instances::GeneralPerformanceOptimizationInstance const* instance,
        cda_rail::index_vector const& prev_order, size_t first_edge_index,
        std::optional<size_t> const& safe_ttd) {
  auto const& tr_edges = state.train_edges.at(tr);
  assert(!tr_edges.empty());

  std::optional<size_t> first_new_ttd_route_edge_idx{};
  std::optional<size_t> last_new_ttd_route_edge_idx{};
  std::optional<size_t> next_ttd_id;
  for (size_t route_edge_idx = first_edge_index;
       route_edge_idx < tr_edges.size(); ++route_edge_idx) {
    auto const& edge_id = tr_edges.at(route_edge_idx);
    auto const& edge_ttd =
        cda_rail::Network::get_ttd_of_edge(edge_id, ttd_sections);
    if (edge_ttd.has_value() &&
        (!safe_ttd.has_value() || edge_ttd.value() != safe_ttd.value())) {
      if (!first_new_ttd_route_edge_idx.has_value()) {
        first_new_ttd_route_edge_idx = route_edge_idx;
        last_new_ttd_route_edge_idx  = route_edge_idx;
        next_ttd_id                  = edge_ttd;
      }
      if (next_ttd_id.has_value() && edge_ttd.value() == next_ttd_id.value()) {
        last_new_ttd_route_edge_idx = route_edge_idx;
      } else {
        break;
      }
    } else if (next_ttd_id.has_value()) {
      break;
    }
  }

  auto const tr_exit_vertex =
      instance->get_const_schedule(tr).get_exit_vertex();
  bool const tr_leaves_network_at_end_of_route =
      instance->get_const_network().get_edge(tr_edges.back()).target ==
      tr_exit_vertex;
  if (!first_new_ttd_route_edge_idx.has_value() &&
      !tr_leaves_network_at_end_of_route) {
    // No new order required -> DONE
    return {state};
  }

  if (next_ttd_id.has_value() &&
      std::ranges::contains(state.ttd_orders.at(next_ttd_id.value()), tr)) {
    // Train already present on TTD, no addition needed
    return extend_train_orders_of_state_recursive_helper(
        tr, state, solver_strategy_input, ttd_sections, instance,
        state.ttd_orders.at(next_ttd_id.value()),
        last_new_ttd_route_edge_idx.value(), next_ttd_id);
  }

  auto& next_order_editable = next_ttd_id.has_value()
                                  ? state.ttd_orders.at(next_ttd_id.value())
                                  : state.vertex_orders.at(tr_exit_vertex);
  auto const next_order     = next_order_editable;
  assert(!std::ranges::contains(next_order, tr));

  auto const& subpath = cda_rail::index_vector(
      std::next(tr_edges.begin(),
                static_cast<std::ptrdiff_t>(first_edge_index)),
      std::next(tr_edges.begin(), static_cast<std::ptrdiff_t>(
                                      first_new_ttd_route_edge_idx.value_or(
                                          tr_edges.size() - 1)) +
                                      1));
  auto tr_sharing_subpath = simulator::GeneralSimulator::trains_on_path(
      subpath, state.train_edges, instance->get_const_network(), true);
  tr_sharing_subpath.erase(tr);

  std::vector<simulator::SimulatorState> retval{};
  auto const tr_order_insertion_interval = infer_order_insertion_bounds(
      tr, prev_order, next_order, tr_sharing_subpath,
      !solver_strategy_input.time_aware_state_transitions);
  for (size_t insertion_idx = tr_order_insertion_interval.lb;
       insertion_idx <= tr_order_insertion_interval.ub; ++insertion_idx) {
    next_order_editable.insert(
        std::next(next_order_editable.begin(),
                  static_cast<std::ptrdiff_t>(insertion_idx)),
        tr);
    std::vector<simulator::SimulatorState> const state_extensions =
        next_ttd_id.has_value()
            ? extend_train_orders_of_state_recursive_helper(
                  tr, state, solver_strategy_input, ttd_sections, instance,
                  next_order_editable, last_new_ttd_route_edge_idx.value(),
                  next_ttd_id)
            : std::vector<simulator::SimulatorState>({state});
    retval.insert(retval.end(), state_extensions.begin(),
                  state_extensions.end());
    next_order_editable = next_order;
  }

  return retval;
}

std::vector<cda_rail::simulator::SimulatorState>
cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::next_states(
    const simulator::SimulatorState&   simulator_state,
    const simulator::SimulatorResults& simulator_results,
    const ModelDetail&                 model_detail_input,
    const SolverStrategyMBAStar&       solver_strategy_input,
    instances::GeneralPerformanceOptimizationInstance const* instance,
    std::vector<cda_rail::index_set> const&                  ttd_sections) {
  // Relevant trains
  auto const tr_to_advance = get_relevant_trains_for_state_transition(
      simulator_state, simulator_results, instance, solver_strategy_input);

  // for every train, get next states and combine
  std::vector<simulator::SimulatorState> next_states{};
  for (auto const& tr : tr_to_advance) {
    auto const tr_path_extensions = get_path_extensions(
        tr, simulator_state, solver_strategy_input.next_state_strategy,
        instance, ttd_sections);
    for (auto const& tr_path_extension : tr_path_extensions) {
      auto const tr_path_extended_states = extend_state_by_path_extension(
          tr, simulator_state, tr_path_extension, instance);
      for (auto const& tr_path_extended_state : tr_path_extended_states) {
        auto const tr_extended_paths = extend_train_orders_of_state(
            tr, tr_path_extended_state, model_detail_input,
            solver_strategy_input, ttd_sections, instance);
        next_states.insert(next_states.end(), tr_extended_paths.begin(),
                           tr_extended_paths.end());
      }
    }
  }
  return next_states;
}
