#include "solver/astar-based/GenPOMovingBlockAStarSolver.hpp"

#include "EOMHelper.hpp"
#include "simulator/GreedySimulator.hpp"

#include <cstddef>
#include <unordered_set>

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
                  cda_rail::braking_distance(tr_obj.max_speed,
                                             tr_obj.acceleration),
                  tr_schedule.get_exit());
      for (const auto& path : entry_paths) {
        GreedySimulatorState new_state{
            .train_edges    = simulator.get_train_edges(),
            .ttd_orders     = simulator.get_ttd_orders(),
            .vertex_orders  = simulator.get_vertex_orders(),
            .stop_positions = simulator.get_stop_positions()};
        new_state.train_edges.at(tr) = path;
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
        next_states.insert(new_state);
      }
    }
  }

  return next_states;
}
