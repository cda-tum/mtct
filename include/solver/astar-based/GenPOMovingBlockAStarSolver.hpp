#pragma once

#include "CustomExceptions.hpp"
#include "Definitions.hpp"
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
#include <queue>
#include <string_view>
#include <unordered_set>
#include <vector>

// If TEST_FRIENDS has value true, the corresponding test is friended to test
// complex private functions.
// This is not good practice. However, after consideration, it was decided that
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
  double dt                  = 6.0; // DB simulation default is 6 seconds
  bool   late_entry_possible = false;
  bool   limit_speed_by_leaving_edges = true;
};

struct SolverStrategyMBAStar {
  simulator::BrakingTimeHeuristicType braking_time_heuristic_type =
      simulator::BrakingTimeHeuristicType::Simple;
  simulator::RemainingTimeHeuristicType remaining_time_heuristic_type =
      simulator::RemainingTimeHeuristicType::Simple;
  NextStateStrategy next_state_strategy    = NextStateStrategy::SingleEdge;
  bool              consider_earliest_exit = true;
  bool              time_aware_state_transitions = false;
  double            a_star_weight                = 1.0;
};

struct GreedySimulatorState {
  std::vector<cda_rail::index_vector> train_edges;
  std::vector<cda_rail::index_vector> ttd_orders;
  std::vector<cda_rail::index_vector> vertex_orders;
  std::vector<std::vector<double>>    stop_positions;

  bool operator==(const GreedySimulatorState& other) const;

  bool operator>(const GreedySimulatorState& other) const;
};
} // namespace cda_rail::solver::astar_based

template <>
struct std::hash<cda_rail::solver::astar_based::GreedySimulatorState> {
  size_t operator()(const cda_rail::solver::astar_based::GreedySimulatorState&
                        state) const noexcept;
}; // namespace std

namespace cda_rail::solver::astar_based {
class GenPOMovingBlockAStarSolver
    : public GeneralSolver<
          instances::GeneralPerformanceOptimizationInstance,
          instances::SolGeneralPerformanceOptimizationInstance> {
private:
#if TEST_FRIENDS
  FRIEND_TEST(::GenPOMovingBlockAStarSolver, NextStates);
  FRIEND_TEST(::GenPOMovingBlockAStarSolver, NextStatesTTD);
#endif

public:
  // ------------------------
  // CONSTRUCTOR
  /**
 * @brief Constructs a default solver instance.
 */

  GenPOMovingBlockAStarSolver() = default;
  /**
       * @brief Constructs the solver from a performance optimization instance.
       *
       * @param instance The performance optimization problem instance to solve.
       */
      explicit GenPOMovingBlockAStarSolver(
      const instances::GeneralPerformanceOptimizationInstance& instance)
      : GeneralSolver(instance) {};
  /**
       * @brief Constructs a solver instance from file-based configuration.
       *
       * @param instanceName The name of the instance.
       * @param instanceSubdirectory The subdirectory containing instance files.
       * @param working_directory The root working directory path.
       */
      GenPOMovingBlockAStarSolver(std::string_view const       instanceName,
                              std::string_view const       instanceSubdirectory,
                              std::filesystem::path const& working_directory)
      : GeneralSolver(instanceName, instanceSubdirectory, working_directory) {}

  ~GenPOMovingBlockAStarSolver() override = default;

  // Rule of 5 (due to virtual deconstructor)
  GenPOMovingBlockAStarSolver(const GenPOMovingBlockAStarSolver&) = default;
  GenPOMovingBlockAStarSolver(GenPOMovingBlockAStarSolver&&)      = default;
  GenPOMovingBlockAStarSolver&
  operator=(const GenPOMovingBlockAStarSolver&) = default;
  GenPOMovingBlockAStarSolver&
  operator=(GenPOMovingBlockAStarSolver&&) = default;

  // ----------------------------
  // SOLVE
  // ----------------------------

  using GeneralSolver::solve;
  /**
   * @brief Solves the problem with default model and strategy configurations.
   *
   * @param time_limit Time limit in seconds (-1 for no limit).
   * @param debug_input If true, enables debug mode.
   * @param overwrite_severity If true, overwrites solution severity.
   * @return Solution to the general performance optimization problem.
   */
  [[nodiscard]] instances::SolGeneralPerformanceOptimizationInstance
  solve(int time_limit, bool debug_input, bool overwrite_severity) override {
    return solve({}, {}, {}, time_limit, debug_input, overwrite_severity);
  };

  [[nodiscard]] instances::SolGeneralPerformanceOptimizationInstance
  solve(const ModelDetail&             model_detail_input,
        const SolverStrategyMBAStar&   solver_strategy_input,
        const GeneralSolutionSettings& solution_settings_input,
        int time_limit = -1, bool debug_input = false,
        bool overwrite_severity = true);

private:
  // ---------------------------
  // STATE TRANSITION HELPER
  // ---------------------------

  // Helper
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

  /**
   * @brief Generates the next reachable states using the specified transition strategy.
   *
   * @param simulator The greedy simulator providing the current state.
   * @param next_state_strategy_input The strategy to use for state transitions.
   * @return An unordered set of next possible states.
   * @throws cda_rail::exceptions::ConsistencyException If the transition strategy is unknown.
   */
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
  }

  // ----------------------
  // DATA STRUCTURE
  // ----------------------

  struct StateObjectivePair {
    double               objective{};
    bool                 is_final_state{};
    GreedySimulatorState state{};
  };

  struct CompareByObjective {
    /**
     * @brief Defines priority ordering for the A* search queue.
     *
     * Lower objectives have higher priority; final states are prioritized over
     * non-final states when objectives are equal; remaining ties use state comparison.
     *
     * @param a First state-objective pair.
     * @param b Second state-objective pair.
     * @return `true` if `a` should be dequeued after `b`, `false` otherwise.
     */
    bool operator()(const StateObjectivePair& a,
                    const StateObjectivePair& b) const {
      return (a.objective > b.objective) ||
             (a.objective == b.objective && !a.is_final_state &&
              b.is_final_state) ||
             (a.objective == b.objective && !a.is_final_state &&
              !b.is_final_state && b.state > a.state);
    }
  };

  using MinPriorityQueue =
      std::priority_queue<StateObjectivePair, std::vector<StateObjectivePair>,
                          CompareByObjective>;
};
} // namespace cda_rail::solver::astar_based
