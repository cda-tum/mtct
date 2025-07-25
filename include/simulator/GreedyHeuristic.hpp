#pragma once

#include "CustomExceptions.hpp"
#include "GreedySimulator.hpp"

#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace cda_rail::simulator {
enum class BrakingTimeHeuristicType : std::uint8_t { Simple = 0 };
enum class RemainingTimeHeuristicType : std::uint8_t { Zero = 0, Simple = 1 };

// Braking time heuristics for A*
[[nodiscard]] double
simple_braking_time_heuristic(size_t tr, const GreedySimulator& simulator,
                              int                           tr_exit_time,
                              const std::pair<int, double>& braking_time);

[[nodiscard]] inline double
braking_time_heuristic(BrakingTimeHeuristicType type, size_t tr,
                       const GreedySimulator& simulator, int tr_exit_time,
                       const std::pair<int, double>& braking_time) {
  switch (type) {
  case BrakingTimeHeuristicType::Simple:
    return simple_braking_time_heuristic(tr, simulator, tr_exit_time,
                                         braking_time);
  }
  // This should never be reached
  throw cda_rail::exceptions::ConsistencyException(
      "This code should not have been reachable...");
};

// Remaining time heuristics for A*
[[nodiscard]] std::pair<bool, double> simple_remaining_time_heuristic(
    size_t tr, const GreedySimulator& simulator, int tr_exit_time,
    double braking_time_heuristic, bool late_stop_possible,
    bool late_exit_possible, bool consider_earliest_exit);

[[nodiscard]] inline std::pair<bool, double>
remaining_time_heuristic(RemainingTimeHeuristicType type, size_t tr,
                         const GreedySimulator& simulator, int tr_exit_time,
                         double braking_time_heuristic, bool late_stop_possible,
                         bool late_exit_possible, bool consider_earliest_exit) {
  switch (type) {
  case RemainingTimeHeuristicType::Zero:
    return {true, 0.0};
  case RemainingTimeHeuristicType::Simple:
    return simple_remaining_time_heuristic(
        tr, simulator, tr_exit_time, braking_time_heuristic, late_stop_possible,
        late_exit_possible, consider_earliest_exit);
  }
  // This should never be reached
  throw cda_rail::exceptions::ConsistencyException(
      "This code should not have been reachable...");
};

[[nodiscard]] inline std::pair<bool, double>
greedy_heuristic(BrakingTimeHeuristicType   braking_time_heuristic_type,
                 RemainingTimeHeuristicType remaining_time_heuristic_type,
                 size_t tr, const GreedySimulator& simulator, int tr_exit_time,
                 const std::pair<int, double>& braking_time,
                 bool late_stop_possible, bool late_exit_possible,
                 bool consider_earliest_exit) {
  const double bt_val = braking_time_heuristic(
      braking_time_heuristic_type, tr, simulator, tr_exit_time, braking_time);
  const auto [feas, obj] = remaining_time_heuristic(
      remaining_time_heuristic_type, tr, simulator, tr_exit_time, bt_val,
      late_stop_possible, late_exit_possible, consider_earliest_exit);
  return {feas, bt_val + obj};
}

[[nodiscard]] inline std::pair<bool, double>
full_greedy_heuristic(BrakingTimeHeuristicType   braking_time_heuristic_type,
                      RemainingTimeHeuristicType remaining_time_heuristic_type,
                      const GreedySimulator&     simulator,
                      const std::vector<int>&    tr_exit_times,
                      const std::vector<std::pair<int, double>>& braking_times,
                      bool late_stop_possible, bool late_exit_possible,
                      bool consider_earliest_exit) {
  bool   feas = true;
  double obj  = 0.0;
  for (size_t tr = 0;
       tr < simulator.get_instance()->get_timetable().get_train_list().size();
       ++tr) {
    const auto [feas_tr, obj_tr] = greedy_heuristic(
        braking_time_heuristic_type, remaining_time_heuristic_type, tr,
        simulator, tr_exit_times.at(tr), braking_times.at(tr),
        late_stop_possible, late_exit_possible, consider_earliest_exit);
    feas = feas && feas_tr;
    obj += simulator.get_instance()->get_train_weights().at(tr) * obj_tr;
  }
  return {feas, obj};
};

[[nodiscard]] inline double
objective_val(const GreedySimulator&  simulator,
              const std::vector<int>& tr_exit_times) {
  double obj = 0.0;
  for (size_t tr = 0;
       tr < simulator.get_instance()->get_timetable().get_train_list().size();
       ++tr) {
    obj += simulator.get_instance()->get_train_weights().at(tr) *
           static_cast<double>(tr_exit_times.at(tr));
  }
  return obj;
};

} // namespace cda_rail::simulator
