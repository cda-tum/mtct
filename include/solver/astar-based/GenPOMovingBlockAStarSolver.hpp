#pragma once

#include "CustomExceptions.hpp"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"
#include "simulator/GreedyHeuristic.hpp"
#include "simulator/GreedySimulator.hpp"
#include "solver/GeneralSolver.hpp"

// NOLINTNEXTLINE(misc-include-cleaner)
#include "gtest/gtest_prod.h"
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <numeric>
#include <queue>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

// If TEST_FRIENDS has value true, the corresponding test is friended to test
// complex private functions
// This is not good practice, however after consideration, it was decided that
// - it is not reasonable to make the functions public
// - they have a complexity that should be tested
// - by only testing the overall solution, there is too much code tested at once
#ifndef TEST_FRIENDS
#define TEST_FRIENDS false
#endif
#if TEST_FRIENDS
class GenPOMovingBlockAStarSolver;
class GenPOMovingBlockAStarSolver_NextStates_Test;
class GenPOMovingBlockAStarSolver_NextStatesTTD_Test;
#endif

namespace cda_rail::solver::astar_based {
#define DEBUG_LOGGING_RATE 1000

enum class NextStateStrategy : std::uint8_t {
  SingleEdge = 0,
  NextTTD    = 1,
};

struct ModelDetail {
  int  dt                           = 6; // DB simulation default is 6 seconds
  bool late_entry_possible          = false;
  bool late_exit_possible           = false;
  bool late_stop_possible           = false;
  bool limit_speed_by_leaving_edges = true;
};

struct SolverStrategyMBAStar {
  simulator::BrakingTimeHeuristicType braking_time_heuristic_type =
      simulator::BrakingTimeHeuristicType::Simple;
  simulator::RemainingTimeHeuristicType remaining_time_heuristic_type =
      simulator::RemainingTimeHeuristicType::Simple;
  NextStateStrategy next_state_strategy    = NextStateStrategy::SingleEdge;
  bool              consider_earliest_exit = true;
};

struct GreedySimulatorState {
  std::vector<cda_rail::index_vector> train_edges;
  std::vector<cda_rail::index_vector> ttd_orders;
  std::vector<cda_rail::index_vector> vertex_orders;
  std::vector<std::vector<double>>    stop_positions;

  bool operator==(const GreedySimulatorState& other) const {
    return train_edges == other.train_edges && ttd_orders == other.ttd_orders &&
           vertex_orders == other.vertex_orders &&
           stop_positions == other.stop_positions;
  }

  bool operator>(const GreedySimulatorState& other) const {
    const double this_obj = std::accumulate(
        train_edges.begin(), train_edges.end(), 0.0,
        [](double sum, const auto& tr_edge) { return sum + tr_edge.size(); });
    const double other_obj = std::accumulate(
        other.train_edges.begin(), other.train_edges.end(), 0.0,
        [](double sum, const auto& tr_edge) { return sum + tr_edge.size(); });
    return this_obj > other_obj;
  }
};
} // namespace cda_rail::solver::astar_based

namespace std {
template <> struct hash<cda_rail::solver::astar_based::GreedySimulatorState> {
  size_t operator()(
      const cda_rail::solver::astar_based::GreedySimulatorState& state) const {
    // Based on boost::hash_combine implementation

    size_t seed         = 0;
    auto   hash_combine = [&seed](size_t h) {
      seed ^= h + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    };

    hash_combine(std::hash<size_t>{}(state.train_edges.size()));
    for (const auto& vec : state.train_edges) {
      hash_combine(std::hash<size_t>{}(vec.size()));
      for (const size_t v : vec) {
        hash_combine(std::hash<size_t>{}(v));
      }
    }
    hash_combine(std::hash<size_t>{}(state.ttd_orders.size()));
    for (const auto& vec : state.ttd_orders) {
      hash_combine(std::hash<size_t>{}(vec.size()));
      for (const size_t v : vec) {
        hash_combine(std::hash<size_t>{}(v));
      }
    }
    hash_combine(std::hash<size_t>{}(state.vertex_orders.size()));
    for (const auto& vec : state.vertex_orders) {
      hash_combine(std::hash<size_t>{}(vec.size()));
      for (const size_t v : vec) {
        hash_combine(std::hash<size_t>{}(v));
      }
    }
    hash_combine(std::hash<size_t>{}(state.stop_positions.size()));
    for (const auto& vec : state.stop_positions) {
      hash_combine(std::hash<size_t>{}(vec.size()));
      for (const double v : vec) {
        hash_combine(std::hash<double>{}(v));
      }
    }

    return seed;
  }
};
} // namespace std

namespace cda_rail::solver::astar_based {
class GenPOMovingBlockAStarSolver
    : public GeneralSolver<
          instances::GeneralPerformanceOptimizationInstance,
          instances::SolGeneralPerformanceOptimizationInstance<
              instances::GeneralPerformanceOptimizationInstance>> {
private:
#if TEST_FRIENDS
  FRIEND_TEST(::GenPOMovingBlockAStarSolver, NextStates);
  FRIEND_TEST(::GenPOMovingBlockAStarSolver, NextStatesTTD);
#endif

  using StateObjectivePair =
      std::pair<std::pair<double, bool>, GreedySimulatorState>;

  struct CompareByObjective {
    bool operator()(const StateObjectivePair& a,
                    const StateObjectivePair& b) const {
      return (a.first.first > b.first.first) ||
             (a.first.first == b.first.first && !a.first.second &&
              b.first.second) ||
             (a.first.first == b.first.first && !a.first.second &&
              !b.first.second && b.second > a.second);
    }
  };

  using MinPriorityQueue =
      std::priority_queue<StateObjectivePair, std::vector<StateObjectivePair>,
                          CompareByObjective>;

  [[nodiscard]] static std::unordered_set<GreedySimulatorState>
  next_states_single_edge(const simulator::GreedySimulator& simulator);
  [[nodiscard]] static std::unordered_set<GreedySimulatorState>
  next_states_next_ttd(const simulator::GreedySimulator& simulator);

  static void next_state_ttd_helper(size_t tr, GreedySimulatorState& state,
                                    const simulator::GreedySimulator& simulator,
                                    const cda_rail::index_vector& new_edges);
  static void
  next_state_exit_vertex_helper(size_t tr, GreedySimulatorState& state,
                                const simulator::GreedySimulator& simulator);
  [[nodiscard]] static std::unordered_set<GreedySimulatorState>
  next_states(const simulator::GreedySimulator& simulator,
              const NextStateStrategy&          next_state_strategy_input) {
    switch (next_state_strategy_input) {
    case NextStateStrategy::SingleEdge:
      return next_states_single_edge(simulator);
    case NextStateStrategy::NextTTD:
      return next_states_next_ttd(simulator);
    default:
      throw cda_rail::exceptions::ConsistencyException(
          "Unknown next state strategy.");
    }
  };

public:
  GenPOMovingBlockAStarSolver() = default;

  explicit GenPOMovingBlockAStarSolver(
      const instances::GeneralPerformanceOptimizationInstance& instance)
      : GeneralSolver<instances::GeneralPerformanceOptimizationInstance,
                      instances::SolGeneralPerformanceOptimizationInstance<
                          instances::GeneralPerformanceOptimizationInstance>>(
            instance) {};

  explicit GenPOMovingBlockAStarSolver(const std::filesystem::path& p)
      : GeneralSolver<instances::GeneralPerformanceOptimizationInstance,
                      instances::SolGeneralPerformanceOptimizationInstance<
                          instances::GeneralPerformanceOptimizationInstance>>(
            p) {};

  explicit GenPOMovingBlockAStarSolver(const std::string& path)
      : GeneralSolver<instances::GeneralPerformanceOptimizationInstance,
                      instances::SolGeneralPerformanceOptimizationInstance<
                          instances::GeneralPerformanceOptimizationInstance>>(
            path) {};

  explicit GenPOMovingBlockAStarSolver(const char* path)
      : GeneralSolver<instances::GeneralPerformanceOptimizationInstance,
                      instances::SolGeneralPerformanceOptimizationInstance<
                          instances::GeneralPerformanceOptimizationInstance>>(
            path) {};

  ~GenPOMovingBlockAStarSolver() = default;

  using GeneralSolver::solve;
  [[nodiscard]] instances::SolGeneralPerformanceOptimizationInstance<
      instances::GeneralPerformanceOptimizationInstance>
  solve(int time_limit, bool debug_input, bool overwrite_severity) override {
    return solve({}, {}, {}, time_limit, debug_input, overwrite_severity);
  };

  [[nodiscard]] instances::SolGeneralPerformanceOptimizationInstance<
      instances::GeneralPerformanceOptimizationInstance>
  solve(const ModelDetail&             model_detail_input,
        const SolverStrategyMBAStar&   solver_strategy_input,
        const GeneralSolutionSettings& solution_settings_input,
        int time_limit = -1, bool debug_input = false,
        bool overwrite_severity = true);
};
} // namespace cda_rail::solver::astar_based
