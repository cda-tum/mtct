#pragma once

#include "Definitions.hpp"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"
#include "simulator/GeneralSimulator.hpp"
#include "simulator/GreedyHeuristic.hpp"
#include "simulator/GreedySimulator.hpp"
#include "solver/GeneralSolver.hpp"

// NOLINTNEXTLINE(misc-include-cleaner)
#include "gtest/gtest_prod.h"
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <optional>
#include <queue>
#include <string_view>
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
class GenPOMovingBlockAStarSolver_NextTrains_Test;
class GenPOMovingBlockAStarSolver_PathExtensions_Test;
class GenPOMovingBlockAStarSolver_InferInsertionBounds_Test;
class GenPOMovingBlockAStarSolver_ExtendStateWithPathExtension_Test;
class GenPOMovingBlockAStarSolver_ExtendTrainOrderEntry_Test;
class GenPOMovingBlockAStarSolver_ExtendTrainOrderExit_Test;
class GenPOMovingBlockAStarSolver_ExtendTrainOrderNothingToChange_Test;
class GenPOMovingBlockAStarSolver_NextStates_Test;
class GenPOMovingBlockAStarSolver_NextStatesTTD_Test;
#endif

namespace cda_rail::solver::astar_based {
#define DEBUG_LOGGING_RATE 1000

enum class NextStateStrategy : std::uint8_t {
  SingleEdge      = 0,
  NextTTD         = 1,
  NextRelevantTTD = 2,
};

struct ModelDetail {
  double dt                  = 6.0; // DB simulation default is 6 seconds
  bool   late_entry_possible = false;
  bool   limit_speed_by_leaving_edges = true;
};

struct SolverStrategyMBAStar {
  simulator::RemainingTimeHeuristicType remaining_time_heuristic_type =
      simulator::RemainingTimeHeuristicType::Simple;
  NextStateStrategy next_state_strategy    = NextStateStrategy::SingleEdge;
  bool              consider_earliest_exit = true;
  bool              time_aware_state_transitions = false;
  double            a_star_weight                = 1.0;
};
} // namespace cda_rail::solver::astar_based

namespace cda_rail::solver::astar_based {
class GenPOMovingBlockAStarSolver
    : public GeneralSolver<
          instances::GeneralPerformanceOptimizationInstance,
          instances::SolGeneralPerformanceOptimizationInstance> {
private:
#if TEST_FRIENDS
  FRIEND_TEST(::GenPOMovingBlockAStarSolver, NextTrains);
  FRIEND_TEST(::GenPOMovingBlockAStarSolver, PathExtensions);
  FRIEND_TEST(::GenPOMovingBlockAStarSolver, InferInsertionBounds);
  FRIEND_TEST(::GenPOMovingBlockAStarSolver, ExtendStateWithPathExtension);
  FRIEND_TEST(::GenPOMovingBlockAStarSolver, ExtendTrainOrderEntry);
  FRIEND_TEST(::GenPOMovingBlockAStarSolver, ExtendTrainOrderExit);
  FRIEND_TEST(::GenPOMovingBlockAStarSolver, ExtendTrainOrderNothingToChange);
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

  // Relevant Trains
  [[nodiscard]] static cda_rail::index_set get_all_trains_for_state_transition(
      simulator::SimulatorState const&                         simulator_state,
      instances::GeneralPerformanceOptimizationInstance const* instance);
  [[nodiscard]] static cda_rail::index_set get_next_time_aware_train(
      simulator::SimulatorState const&                         simulator_state,
      simulator::SimulatorResults const&                       simulator_result,
      instances::GeneralPerformanceOptimizationInstance const* instance);
  [[nodiscard]] static cda_rail::index_set
  get_relevant_trains_for_state_transition(
      simulator::SimulatorState const&                         simulator_state,
      simulator::SimulatorResults const&                       simulator_result,
      instances::GeneralPerformanceOptimizationInstance const* instance,
      SolverStrategyMBAStar const&                             solver_strategy);

  // Path Extensions
  [[nodiscard]] static std::vector<cda_rail::index_vector> get_entry_paths(
      size_t                                                   tr,
      instances::GeneralPerformanceOptimizationInstance const* instance);
  struct PathExtensionData {
    cda_rail::index_vector path{};
    size_t                 stop_possible_from_idx_onward{0};

    bool operator==(const PathExtensionData& other) const {
      return path == other.path && stop_possible_from_idx_onward ==
                                       other.stop_possible_from_idx_onward;
    }
  };
  [[nodiscard]] static std::vector<PathExtensionData> get_path_extensions(
      size_t tr, simulator::SimulatorState const& simulator_state,
      NextStateStrategy next_state_strategy,
      instances::GeneralPerformanceOptimizationInstance const* instance,
      std::vector<cda_rail::index_set> const&                  ttd_sections);

  // Train Order
  struct IndexBound {
    size_t lb{};
    size_t ub{};

    bool operator==(const IndexBound& other) const {
      return lb == other.lb && ub == other.ub;
    }
  };
  [[nodiscard]] static IndexBound infer_order_insertion_bounds(
      size_t tr, cda_rail::index_vector const& prev_order,
      cda_rail::index_vector const& next_order,
      cda_rail::index_set const& tr_sharing_path, bool insert_at_end);
  [[nodiscard]] static IndexBound infer_order_entry_order_bounds(
      size_t tr, cda_rail::index_vector const& entry_order,
      bool late_entry_possible, bool insert_at_end,
      instances::GeneralPerformanceOptimizationInstance const* instance);

  // State Extension
  [[nodiscard]] static std::vector<simulator::SimulatorState>
  extend_state_by_path_extension(
      size_t tr, simulator::SimulatorState state,
      PathExtensionData const& path_extension_data,
      instances::GeneralPerformanceOptimizationInstance const* instance);
  [[nodiscard]] static std::vector<simulator::SimulatorState>
  extend_train_orders_of_state(
      size_t tr, simulator::SimulatorState state,
      const ModelDetail&                      model_detail_input,
      const SolverStrategyMBAStar&            solver_strategy_input,
      std::vector<cda_rail::index_set> const& ttd_sections,
      instances::GeneralPerformanceOptimizationInstance const* instance);
  [[nodiscard]] static std::vector<simulator::SimulatorState>
  extend_train_orders_of_state_recursive_helper(
      size_t tr, simulator::SimulatorState state,
      const SolverStrategyMBAStar&            solver_strategy_input,
      std::vector<cda_rail::index_set> const& ttd_sections,
      instances::GeneralPerformanceOptimizationInstance const* instance,
      cda_rail::index_vector const& prev_order, size_t first_edge_index,
      std::optional<size_t> const& safe_ttd);

  /**
   * @brief Generates the next reachable states using the specified transition
   * strategy.
   *
   * @param simulator_state The current simulator state
   * @param simulator_results The current simulator results
   * @param model_detail_input The model details, in particular, if late entry
   * is possible
   * @param solver_strategy_input The strategy to use, in particular, for state
   * transitions.
   * @param instance The problem instance.
   * @param ttd_sections The TTD sections of the network.
   * @return A vector of next possible states.
   * @throws cda_rail::exceptions::ConsistencyException If the transition
   * strategy is unknown.
   */
  [[nodiscard]] static std::vector<simulator::SimulatorState>
  next_states(const simulator::SimulatorState&   simulator_state,
              const simulator::SimulatorResults& simulator_results,
              const ModelDetail&                 model_detail_input,
              const SolverStrategyMBAStar&       solver_strategy_input,
              instances::GeneralPerformanceOptimizationInstance const* instance,
              std::vector<cda_rail::index_set> const& ttd_sections);

  // ----------------------
  // DATA STRUCTURE
  // ----------------------

  struct StateObjectivePair {
    double                      objective{};
    bool                        is_final_state{};
    simulator::SimulatorState   state{};
    simulator::SimulatorResults results{};
  };

  struct CompareByObjective {
    /**
     * @brief Defines priority ordering for the A* search queue.
     *
     * Lower objectives have higher priority; final states are prioritized over
     * non-final states when objectives are equal; remaining ties use state
     * comparison.
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
