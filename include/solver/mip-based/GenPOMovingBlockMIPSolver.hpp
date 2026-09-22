#pragma once

#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "datastructure/RailwayNetwork.hpp"
#include "datastructure/Train.hpp"
#include "gurobi_c++.h"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"
#include "solver/GeneralSolver.hpp"
#include "solver/mip-based/GeneralMIPSolver.hpp"

// NOLINTNEXTLINE(misc-include-cleaner)
#include "gtest/gtest_prod.h"
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

// If TEST_FRIENDS has value true, the corresponding test is friended to test
// complex private functions
// This is not good practice, however after consideration, it was decided that
// - it is not reasonable to make the functions public, because they are only
// needed to build the model
// - they have a complexity that should be tested
// - by only testing the overall solution, there is too much code tested at once
#ifndef TEST_FRIENDS
#define TEST_FRIENDS false
#endif
#if TEST_FRIENDS
class GenPOMovingBlockMIPSolver;
class GenPOMovingBlockMIPSolver_PrivateFillFunctions_Test;
#endif

namespace cda_rail::solver::mip_based {

using std::size_t;

/** @brief The name of @p strategy, as it is reported and parsed. */
constexpr std::string
velocity_refinement_strategy_to_string(VelocityRefinementStrategy strategy) {
  switch (strategy) {
  case VelocityRefinementStrategy::None:
    return "None";
  case VelocityRefinementStrategy::MinOneStep:
    return "MinOneStep";
  default:
    throw cda_rail::exceptions::ConsistencyException(
        "Unknown velocity refinement strategy");
  }
}

// The maximal delay is the time horizon of the model and with it the big-M of
// all timing constraints. A big-M of size M turns the tolerance eps with which
// a solver still accepts a binary variable as integral into a slack of M * eps
// seconds within the corresponding constraint. A horizon that is numerically
// infinite hence voids the travel times altogether, which is why the default
// is a large but finite one. A delay of more than a day is not to be expected
// in any realistic instance.
constexpr double DEFAULT_MAX_DELAY = 24 * 60 * 60; // one day

// Smallest and default integrality tolerance supported by Gurobi.
constexpr double MIN_INT_FEAS_TOL     = 1e-9;
constexpr double DEFAULT_INT_FEAS_TOL = 1e-5;

/**
 * @brief How accurately the moving block MIP models the operation.
 *
 * Every setting here changes the set of feasible solutions, in contrast to
 * SolverStrategyMovingBlock, which only changes how the very same model is
 * solved.
 */
struct ModelDetailMovingBlock {
  bool                       fix_routes         = false;
  double                     max_velocity_delta = 5.55; // 20 km/h
  VelocityRefinementStrategy velocity_refinement_strategy =
      VelocityRefinementStrategy::MinOneStep;
  bool simplify_headway_constraints          = false;
  bool strengthen_vertex_headway_constraints = false;
  bool allow_late_entry                      = false;
  // Bound every timing variable by the earliest time at which the
  // corresponding event can happen, i.e. by the minimal running time the train
  // needs to get there. Without these bounds the only lower bound on the
  // objective is the scheduled timetable, and since everything that forces a
  // train to travel at all is a big-M constraint, the LP relaxation is then
  // free of any running time. Disabling this is only useful to measure the
  // effect.
  bool   use_minimum_time_bounds = true;
  double max_exit_delay          = DEFAULT_MAX_DELAY;
  double max_station_delay       = DEFAULT_MAX_DELAY;
};

/**
 * @brief Which of the headway constraints the lazy callback adds.
 *
 * `OnlyViolated` adds every constraint that the incumbent violates,
 * `OnlyFirstFound` stops after the first of them, and `AllChecked` adds every
 * constraint it looked at, whether it was violated or not.
 */
enum class LazyConstraintSelectionStrategy : std::uint8_t {
  OnlyViolated   = 0,
  OnlyFirstFound = 1,
  AllChecked     = 2,
};

/** @brief The name of @p strategy, as it is reported and parsed. */
constexpr std::string lazy_constraint_selection_strategy_to_string(
    LazyConstraintSelectionStrategy strategy) {
  switch (strategy) {
  case LazyConstraintSelectionStrategy::OnlyViolated:
    return "OnlyViolated";
  case LazyConstraintSelectionStrategy::OnlyFirstFound:
    return "OnlyFirstFound";
  case LazyConstraintSelectionStrategy::AllChecked:
    return "AllChecked";
  default:
    throw cda_rail::exceptions::ConsistencyException(
        "Unknown lazy constraint selection strategy");
  }
}

/**
 * @brief Which train pairs the lazy callback checks against each other.
 *
 * `OnlyAdjacent` only checks a train against its direct neighbors in the order
 * of the edge or TTD section, while `All` checks it against every other train
 * ordered there.
 */
enum class LazyTrainSelectionStrategy : std::uint8_t {
  OnlyAdjacent = 0,
  All          = 1,
};

/** @brief The name of @p strategy, as it is reported and parsed. */
constexpr std::string
lazy_train_selection_strategy_to_string(LazyTrainSelectionStrategy strategy) {
  switch (strategy) {
  case LazyTrainSelectionStrategy::OnlyAdjacent:
    return "OnlyAdjacent";
  case LazyTrainSelectionStrategy::All:
    return "All";
  default:
    throw cda_rail::exceptions::ConsistencyException(
        "Unknown lazy train selection strategy");
  }
}

/**
 * @brief How the moving block MIP is handed to and solved by Gurobi.
 *
 * The headway constraints are the ones there are too many of to add upfront,
 * which is why they are separated lazily by default. The remaining lazy
 * settings are ignored if they are not.
 */
struct SolverStrategyMovingBlock {
  bool use_indicator_constraints = false;
  bool use_lazy_constraints =
      true; // If false, the following settings are ignored
  bool include_reverse_headways               = false;
  bool include_higher_velocities_in_edge_expr = false;
  LazyConstraintSelectionStrategy lazy_constraint_selection_strategy =
      LazyConstraintSelectionStrategy::OnlyViolated;
  LazyTrainSelectionStrategy lazy_train_selection_strategy =
      LazyTrainSelectionStrategy::OnlyAdjacent;
  double abs_mip_gap = 10;
};

/**
 * @brief Routes the trains of a moving block instance by a MILP.
 *
 * The model decides, for every train, which edges it uses, at which velocity
 * it passes every vertex, and at what time, where the velocities are taken
 * from a discrete set per vertex, see VelocityRefinementStrategy. The
 * separation of the trains is expressed by their order on the edges and TTD
 * sections, whose headway constraints are separated lazily.
 */
class GenPOMovingBlockMIPSolver
    : public GeneralMIPSolver<
          instances::GeneralPerformanceOptimizationInstance,
          instances::SolGeneralPerformanceOptimizationInstance> {
private:
#if TEST_FRIENDS
  FRIEND_TEST(::GenPOMovingBlockMIPSolver, PrivateFillFunctions);
#endif

  SolutionSettingsMovingBlock m_solution_settings = {};
  ModelDetailMovingBlock      m_model_detail      = {};
  SolverStrategyMovingBlock   m_solver_strategy   = {};
  size_t                      m_num_tr            = 0;
  size_t                      m_num_edges         = 0;
  size_t                      m_num_vertices      = 0;
  size_t                      m_num_ttd           = 0;
  // int m_max_t = 0;
  std::vector<cda_rail::index_set> m_ttd_sections;
  // tr_stop_data:
  // For every train, for every station, list of possible stop vertices together
  // with respective edges
  std::vector<std::vector<
      std::vector<std::pair<size_t, std::vector<cda_rail::index_vector>>>>>
                                                m_tr_stop_data;
  std::vector<std::vector<std::vector<double>>> m_velocity_extensions;
  std::vector<std::pair<size_t, size_t>>        m_relevant_reverse_edges;
  // Earliest time at which the front of a train can arrive at a vertex, by
  // train and vertex. All zero if
  // ModelDetailMovingBlock::use_minimum_time_bounds is not set.
  std::vector<std::vector<double>> m_minimum_arrival_times;
  // Time the rear of a train needs to reach a vertex after its front, by
  // train. All zero if ModelDetailMovingBlock::use_minimum_time_bounds is not
  // set.
  std::vector<double> m_minimum_clearing_times;
  // Earliest time at which the rear of a train can leave its exit vertex, by
  // train. At least the scheduled exit time.
  std::vector<double> m_minimum_exit_times;
  // Earliest service delay of a scheduled stop, by train and stop. All zero if
  // ModelDetailMovingBlock::use_minimum_time_bounds is not set.
  std::vector<std::vector<double>> m_minimum_service_delays;

  /**
   * @brief Stores the given settings and derives everything the model needs.
   *
   * @throws cda_rail::exceptions::InvalidInputException If the settings
   *         contradict each other.
   */
  void initialize_variables(
      const SolutionSettingsMovingBlock& solution_settings_input,
      const SolverStrategyMovingBlock&   solver_strategy_input,
      const ModelDetailMovingBlock&      model_detail_input);

  /** @brief The scheduled exit time of a train plus the allowed delay. */
  double latest_exit_time(size_t tr) const;
  /** @brief The earliest time the front of a train can reach a vertex. */
  [[nodiscard]] double minimum_arrival_time(size_t tr, size_t v) const {
    return m_minimum_arrival_times.at(tr).at(v);
  };
  // Between the front and the rear of a train passing a vertex, the train
  // covers its own length, which takes at least its length divided by its
  // maximal speed.
  [[nodiscard]] double minimum_rear_departure_time(size_t tr, size_t v) const {
    return std::min(minimum_arrival_time(tr, v) +
                        m_minimum_clearing_times.at(tr),
                    latest_exit_time(tr));
  };

  /** @brief Computes the bounds of the timing variables. */
  void fill_minimum_time_bounds();
  /** @brief Computes where every train can serve every one of its stops. */
  void fill_tr_stop_data();
  /** @brief Collects the edge pairs that can be traversed in both ways. */
  void fill_relevant_reverse_edges();
  /** @brief Computes the discrete velocities of every train and vertex. */
  void fill_velocity_extensions();
  /** @brief Does so using an equidistant grid. */
  void fill_velocity_extensions_using_none_strategy();
  /** @brief Does so refining the grid where a step would be too large. */
  void fill_velocity_extensions_using_min_one_step_strategy();

  /** @brief The largest number of velocities of any train and vertex. */
  size_t get_maximal_velocity_extension_size() const;

  /**
   * @brief The headways a train has to obey at both ends of an edge.
   *
   * @param tr Index of the train
   * @param e Index of the edge
   * @return The headway at the source and at the target of the edge, each as
   *         its maximal value and as the expression of the chosen velocities
   */
  [[nodiscard]] std::tuple<double, GRBLinExpr, double, GRBLinExpr>
  get_vertex_headway_expressions(size_t tr, size_t e);
  /**
   * @brief The same for the headways of the edge and of its TTD section.
   *
   * These are the ones induced by the moving block, i.e., by the distance the
   * train needs to come to a stop, in contrast to the ones specified at the
   * vertices.
   */
  [[nodiscard]] std::tuple<double, GRBLinExpr, double, GRBLinExpr>
  get_edge_headway_expressions(size_t tr, size_t e);

  /** @brief Creates all variables of the model. */
  void create_variables();
  /** @brief Creates the variables holding the times of the events. */
  void create_timing_variables();
  /** @brief Creates the variables deciding which edges a train uses. */
  void create_general_edge_variables();
  /** @brief Creates the variables deciding where a train serves its stops. */
  void create_stop_variables();
  /** @brief Creates the variables deciding the velocities at the vertices. */
  void create_velocity_extended_variables();
  /**
   * @brief Creates the variables preventing collisions of trains traveling in
   *        opposite directions.
   */
  void create_reverse_edge_variables();

  /** @brief Sets the objective function, see the instance for its terms. */
  void set_objective();

  /** @brief Creates all constraints of the model. */
  void create_constraints();
  /** @brief The edges and velocities of a train form a path. */
  void create_general_path_constraints();
  /** @brief A train needs at least the minimal travel time on every edge. */
  void create_travel_times_constraints();
  /** @brief Two trains using the same edge are ordered on it. */
  void create_basic_order_constraints();
  /** @brief Relates the edges of a TTD section to the section itself. */
  void create_basic_ttd_constraints();
  /** @brief Relates the times of the rear of a train to those of its front. */
  void create_train_rear_constraints();
  /** @brief Two trains may not use an edge in opposite directions. */
  void create_reverse_edge_constraints();
  /** @brief Every stop is served at one of its possible positions. */
  void create_stopping_constraints();
  /** @brief Obeys the headways specified at the vertices, e.g. of a line. */
  void create_vertex_headway_constraints();
  /** @brief Separates the trains by the moving block headway. */
  void create_headway_constraints();
  /**
   * @brief Separates them by simplified headway constraints instead.
   *
   * These are weaker, hence the trains are separated less accurately, but the
   * model is easier to solve. Sufficient if the solution is only used as a
   * starting point.
   */
  void create_simplified_headway_constraints();

  // Helper for headway normal and lazy constraints
  [[nodiscard]] GRBLinExpr
  get_edge_path_expr(size_t tr, const cda_rail::index_vector& p,
                     double initial_velocity,
                     bool   also_higher_velocities = false);

  /** @brief Fills the solution object from the variable values. */
  void extract_solution(
      instances::SolGeneralPerformanceOptimizationInstance& sol) const;
  /** @brief The velocity a train passes a vertex at. */
  [[nodiscard]] double extract_speed(size_t tr, size_t vertex_id) const;
  /** @brief The time a train starts serving one of its stops. */
  [[nodiscard]] double extract_stop_time(size_t tr, size_t stop_idx) const;
  /**
   * @brief The headway time a train induces while it traverses an edge.
   *
   * @param tr_obj The train
   * @param e_obj The edge
   * @param v_0 Velocity at the source of the edge
   * @param v_1 Velocity at its target
   * @param entry_vertex Whether the source is the entry vertex of the train,
   *        at which it is not yet fully in the network
   * @return The time by which the corresponding events of a following train
   *         have to be apart, which can be negative
   */
  static double headway(const Train& tr_obj, const Edge& e_obj, double v_0,
                        double v_1, bool entry_vertex = false);

  class LazyCallback : public MessageCallback {
  private:
    GenPOMovingBlockMIPSolver* solver;

    /**
     * @brief The routes of the current solution.
     *
     * @return For every train, its edges together with the distance of their
     *         source vertex from the start of the route.
     */
    std::vector<std::vector<std::pair<size_t, double>>> get_routes();
    std::vector<std::unordered_map<size_t, double>>     get_train_velocities(
        const std::vector<std::vector<std::pair<size_t, double>>>& routes);
    std::vector<std::pair<std::vector<std::pair<size_t, bool>>,
                          std::vector<std::pair<size_t, bool>>>>
                                        get_train_orders_on_edges(
                                            const std::vector<std::vector<std::pair<size_t, double>>>& routes);
    std::vector<cda_rail::index_vector> get_train_orders_on_ttd();

    bool create_lazy_edge_and_ttd_headway_constraints(
        const std::vector<std::vector<std::pair<size_t, double>>>& routes,
        const std::vector<std::unordered_map<size_t, double>>& train_velocities,
        const std::vector<std::pair<std::vector<std::pair<size_t, bool>>,
                                    std::vector<std::pair<size_t, bool>>>>&
                                                   train_orders_on_edges,
        const std::vector<cda_rail::index_vector>& train_orders_on_ttd);
    bool create_lazy_simplified_edge_constraints(
        const std::vector<std::vector<std::pair<size_t, double>>>& routes,
        const std::vector<std::unordered_map<size_t, double>>& train_velocities,
        const std::vector<std::pair<std::vector<std::pair<size_t, bool>>,
                                    std::vector<std::pair<size_t, bool>>>>&
                                                   train_orders_on_edges,
        const std::vector<cda_rail::index_vector>& train_orders_on_ttd);
    bool create_lazy_vertex_headway_constraints(
        const std::vector<std::vector<std::pair<size_t, double>>>& routes,
        const std::vector<std::unordered_map<size_t, double>>& train_velocities,
        const std::vector<std::pair<std::vector<std::pair<size_t, bool>>,
                                    std::vector<std::pair<size_t, bool>>>>&
            train_orders_on_edges);
    bool create_lazy_reverse_edge_constraints(
        const std::vector<std::pair<std::vector<std::pair<size_t, bool>>,
                                    std::vector<std::pair<size_t, bool>>>>&
            train_orders_on_edges);

  public:
    explicit LazyCallback(GenPOMovingBlockMIPSolver* solver) : solver(solver) {}

  protected:
    void callback() override;
  };

protected:
  void cleanup() override;

public:
  GenPOMovingBlockMIPSolver() = default;

  explicit GenPOMovingBlockMIPSolver(
      const instances::GeneralPerformanceOptimizationInstance& instance)
      : GeneralMIPSolver<instances::GeneralPerformanceOptimizationInstance,
                         instances::SolGeneralPerformanceOptimizationInstance>(
            instance) {};

  template <typename... Args>
  explicit GenPOMovingBlockMIPSolver(Args&&... args)
    requires(!IsSingleInstanceArgument<Args...>::value)
      : GeneralMIPSolver(std::forward<Args>(args)...) {}

  ~GenPOMovingBlockMIPSolver() override = default;

  using GeneralSolver::solve;
  [[nodiscard]] instances::SolGeneralPerformanceOptimizationInstance
  solve(int time_limit, bool debug_input, bool overwrite_severity) override {
    return solve({}, {}, {}, time_limit, debug_input, overwrite_severity);
  };

  /**
   * @brief Solves the instance under moving block signaling.
   *
   * Only breakable edges use moving block. On all others, only one train is
   * allowed at a time, which is how Flankenschutz can be modelled in practice.
   * Trains are only routed if no route is specified.
   *
   * @param model_detail_input How accurately the operation is modelled.
   * @param solver_strategy_input How the model is solved.
   * @param solution_settings_input Where and what is exported.
   * @param time_limit Time limit in seconds. No limit if negative.
   * @param debug_input If true, the debug output is enabled.
   * @param overwrite_severity If true, the severity of the log is overwritten
   *        even if this decreases the logging level.
   * @return Solution object containing status, objective value, and solution.
   */
  [[nodiscard]] instances::SolGeneralPerformanceOptimizationInstance
  solve(const ModelDetailMovingBlock&      model_detail_input,
        const SolverStrategyMovingBlock&   solver_strategy_input,
        const SolutionSettingsMovingBlock& solution_settings_input,
        int time_limit = -1, bool debug_input = false,
        bool overwrite_severity = true);
};

} // namespace cda_rail::solver::mip_based
