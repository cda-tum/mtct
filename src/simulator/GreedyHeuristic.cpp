#include "simulator/GreedyHeuristic.hpp"

#include "simulator/GreedySimulator.hpp"

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <utility>
#include <vector>

double cda_rail::simulator::simple_braking_time_heuristic(
    size_t tr, const cda_rail::simulator::GreedySimulator& simulator,
    int tr_exit_time, const std::pair<int, double>& braking_time) {
  /**
   * This heuristic how much time was lost due to braking, i.e., the minimal
   * traveling time from the braking point minus the actual time spent. The
   * result will usually be negative.
   *
   * @param tr The train for which the heuristic is calculated.
   * @param simulator The simulator instance containing the train and its edges.
   * @param tr_exit_time The time at which the train exits the network.
   * @param braking_time A pair containing the time at which the train started
   * braking and the distance it has traveled while braking.
   *
   * @return The heuristic value, which is the difference between the time spent
   * braking and the minimal traveling time from the braking point.
   */

  assert(braking_time.first >= 0);
  assert(braking_time.second >= 0);
  assert(tr_exit_time >= 0);
  const auto& tr_edges = simulator.get_train_edges_of_tr(tr);
  assert(!tr_edges.empty());

  const auto& train = simulator.get_instance()->get_train_list().get_train(tr);
  double      ret_val = static_cast<double>(braking_time.first) -
                   static_cast<double>(tr_exit_time);
  double len = 0.0;
  for (auto it = tr_edges.rbegin();
       (len <= braking_time.second) && (it != tr_edges.rend()); ++it) {
    const auto& edge        = simulator.get_instance()->const_n().get_edge(*it);
    const auto& speed_limit = std::min(edge.max_speed, train.max_speed);
    const auto  rel_len     = std::min(edge.length, braking_time.second - len);
    ret_val += rel_len / speed_limit;
    len += edge.length;
  }
  return ret_val;
}
