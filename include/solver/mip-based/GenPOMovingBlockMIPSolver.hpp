#pragma once

#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "datastructure/RailwayNetwork.hpp"
#include "datastructure/Train.hpp"
#include "gurobi_c++.h"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"
#include "simulator/GeneralSimulator.hpp"
#include "solver/GeneralSolver.hpp"
#include "solver/mip-based/GeneralMIPSolver.hpp"

// NOLINTNEXTLINE(misc-include-cleaner)
#include "gtest/gtest_prod.h"
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
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
class GenPOMovingBlockMIPSolver_WarmStartIsFeasible_Test;
class GenPOMovingBlockMIPSolver_WarmStartRoutesTrainWithoutRoute_Test;
class GenPOMovingBlockMIPSolver_NoWarmStartIfFixedRouteMissesStop_Test;
class GenPOMovingBlockMIPSolver_NoWarmStartWithoutModelVelocities_Test;
class GenPOMovingBlockMIPSolver_SmallCoefficientsAreRemoved_Test;
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
  // needs to get there, including the stops it has to serve on the way.
  // Without these bounds the only lower bound on the
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
 * The strategies are ordered by how many constraints they add:
 * `OnlyFirstFound` adds only the first constraint that the incumbent violates,
 * `OnlyViolated` adds every constraint that the incumbent violates, and
 * `AllChecked` adds every constraint it looked at, whether it was violated or
 * not. With every strategy, a constraint that was already added is only added
 * again if the incumbent violates it.
 */
enum class LazyConstraintSelectionStrategy : std::uint8_t {
  OnlyFirstFound = 0,
  OnlyViolated   = 1,
  AllChecked     = 2,
};

/** @brief The name of @p strategy, as it is reported and parsed. */
constexpr std::string lazy_constraint_selection_strategy_to_string(
    LazyConstraintSelectionStrategy strategy) {
  switch (strategy) {
  case LazyConstraintSelectionStrategy::OnlyFirstFound:
    return "OnlyFirstFound";
  case LazyConstraintSelectionStrategy::OnlyViolated:
    return "OnlyViolated";
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
 * @brief For which velocities the lazy callback adds a headway constraint
 *        between two trains on an edge.
 *
 * Once the callback decides, see LazyConstraintSelectionStrategy, that a train
 * passing a vertex has to be separated from a train ahead, the constraint
 * depends on the velocity at which the train passes the vertex, since its
 * moving authority ends the further ahead the faster it is.
 * `OnlyCurrent` adds the constraint of the velocity in the incumbent.
 * `ExtendToHigher` adds no further constraint, but extends this one to every
 * higher velocity, i.e., it also binds if the train passes the vertex faster.
 * This is valid, since a higher velocity only extends the moving authority,
 * but weaker than the constraint of the higher velocity itself.
 * `All` adds the constraint of every velocity at the vertex, each of which
 * only holds for its own velocity.
 *
 * Only the full headway constraints on edges are affected. The simplified
 * ones, see ModelDetailMovingBlock::simplify_headway_constraints, already
 * express the headway of every velocity in a single constraint, and the
 * headway constraints of TTD sections always use the velocities of the
 * incumbent.
 */
enum class LazyVelocitySelectionStrategy : std::uint8_t {
  OnlyCurrent    = 0,
  ExtendToHigher = 1,
  All            = 2,
};

/** @brief The name of @p strategy, as it is reported and parsed. */
constexpr std::string lazy_velocity_selection_strategy_to_string(
    LazyVelocitySelectionStrategy strategy) {
  switch (strategy) {
  case LazyVelocitySelectionStrategy::OnlyCurrent:
    return "OnlyCurrent";
  case LazyVelocitySelectionStrategy::ExtendToHigher:
    return "ExtendToHigher";
  case LazyVelocitySelectionStrategy::All:
    return "All";
  default:
    throw cda_rail::exceptions::ConsistencyException(
        "Unknown lazy velocity selection strategy");
  }
}

/**
 * @brief How the moving block MIP is handed to and solved by Gurobi.
 *
 * The headway constraints are the ones there are too many of to add upfront,
 * which is why they are separated lazily by default. The remaining lazy
 * settings are ignored if they are not.
 *
 * If `use_warm_start` is set, the solver starts from the completion of the
 * initial A* state, see astar_based::complete_state, if there is one. The MIP
 * start fixes every integer variable as in the simulation of that completion,
 * i.e., its routes, stops and orders, and the velocities closest to the
 * simulated ones. Gurobi computes the times.
 */
struct SolverStrategyMovingBlock {
  bool use_indicator_constraints = false;
  bool use_lazy_constraints =
      true; // If false, the following settings are ignored
  bool                          include_reverse_headways = false;
  LazyVelocitySelectionStrategy lazy_velocity_selection_strategy =
      LazyVelocitySelectionStrategy::OnlyCurrent;
  LazyConstraintSelectionStrategy lazy_constraint_selection_strategy =
      LazyConstraintSelectionStrategy::OnlyViolated;
  LazyTrainSelectionStrategy lazy_train_selection_strategy =
      LazyTrainSelectionStrategy::OnlyAdjacent;
  double abs_mip_gap    = 10;
  bool   use_warm_start = false;
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
  FRIEND_TEST(::GenPOMovingBlockMIPSolver, WarmStartIsFeasible);
  FRIEND_TEST(::GenPOMovingBlockMIPSolver, WarmStartRoutesTrainWithoutRoute);
  FRIEND_TEST(::GenPOMovingBlockMIPSolver, NoWarmStartIfFixedRouteMissesStop);
  FRIEND_TEST(::GenPOMovingBlockMIPSolver, NoWarmStartWithoutModelVelocities);
  FRIEND_TEST(::GenPOMovingBlockMIPSolver, SmallCoefficientsAreRemoved);
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
  /**
   * @brief Statistics of a solve for the solver data export.
   *
   * Every value is -1 if it is unavailable, e.g., since the model was not
   * created, the optimization never started, or Gurobi could not report it.
   */
  struct SolveStatistics {
    // Number of lazy constraints handed to Gurobi during the solve. A
    // constraint added before is handed over again only if the offered
    // solution violates it, and then counts again. Zero without lazy
    // constraints.
    int num_lazy_constraints_added = -1;
    // Number of linear constraints (rows) of the created model, i.e., without
    // the lazy constraints and without general constraints such as indicator
    // constraints.
    int model_num_rows = -1;
    // Number of variables (columns) of the created model.
    int model_num_columns = -1;
    // Number of branch-and-bound nodes Gurobi explored.
    double node_count = -1;
  };
  /**
   * @brief Collects the settings and statistics of a solve for the solver data
   * export.
   *
   * The statistics are exported as num_lazy_constraints_added_int,
   * model_num_rows_int, model_num_columns_int, and node_count_double.
   *
   * @param model_detail The model detail the solve uses
   * @param solver_strategy The solver strategy the solve uses
   * @param solution_settings The solution settings the solve uses
   * @param time_limit The time limit in seconds passed to the solver
   * @param statistics The statistics of the solve
   * @return The settings and statistics as further data of the solver data
   * export
   */
  [[nodiscard]] static FurtherData
  get_further_data(const ModelDetailMovingBlock&      model_detail,
                   const SolverStrategyMovingBlock&   solver_strategy,
                   const SolutionSettingsMovingBlock& solution_settings,
                   int time_limit, const SolveStatistics& statistics);

  /** @brief The scheduled exit time of a train plus the allowed delay. */
  double latest_exit_time(size_t tr) const;
  /** @brief The earliest time the front of a train can reach a vertex. */
  [[nodiscard]] double minimum_arrival_time(size_t tr, size_t v) const {
    return m_minimum_arrival_times.at(tr).at(v);
  };
  /**
   * @brief The earliest time the front of a train can arrive at one of the
   *        possible vertices of a stop, at most its latest exit time.
   */
  [[nodiscard]] double minimum_stop_arrival_time(size_t tr, size_t stop) const;
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

  /**
   * @brief Creates the variables, objective and constraints of the model.
   *
   * The model has to be initialized and the stops discretized, as solve()
   * does.
   */
  void create_model(const SolutionSettingsMovingBlock& solution_settings_input,
                    const SolverStrategyMovingBlock&   solver_strategy_input,
                    const ModelDetailMovingBlock&      model_detail_input);
  /**
   * @brief Sets every constraint coefficient of the model whose absolute value
   *        is below the integrality tolerance to 0, since such coefficients
   *        cause numerical issues.
   *
   * The model has to be updated before.
   */
  void remove_small_coefficients();
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

  /**
   * @brief The initial A* state, completed by astar_based::complete_state.
   *
   * As in the A* search, the completion uses the greedy simulator, since only
   * it decides orders first come, first served. With fixed routes, the
   * initial state already has the route of every train that has one, and it
   * serves each of its stops at the first valid stop vertex. A train without
   * a route can take any route in the model, hence the completion routes it.
   *
   * @return The final state, or `std::nullopt` if the completion fails, e.g.,
   *         since a fixed route misses a stop.
   */
  [[nodiscard]] std::optional<simulator::SimulatorState>
  completed_initial_state() const;
  /**
   * @brief When and how fast a train passes a vertex of its route in a
   *        simulation.
   *
   * @param departure When its front leaves the vertex
   * @param velocity Its velocity then
   * @param stands_still Whether it stands still at the vertex, in particular
   *        to serve a stop there. Never at its exit vertex, see
   *        set_warm_start().
   */
  struct VertexPassing {
    double departure;
    double velocity;
    bool   stands_still;
  };
  /**
   * @brief For every vertex of a route, the index of the velocity in
   *        m_velocity_extensions at which the train passes it.
   *
   * Among the velocities whose consecutive pairs all have a variable y, the
   * ones with the least sum of squared differences to the simulated ones are
   * chosen. A train that stands still at a vertex passes it at velocity 0, and
   * it passes its exit vertex at a velocity from which it can reach its exit
   * velocity.
   *
   * @param tr Index of the train
   * @param route The edges of its route
   * @param passings How the train passes the vertices of the route, starting
   *        with its entry vertex
   * @return The indices, or `std::nullopt` if no velocities form such a path.
   */
  [[nodiscard]] std::optional<std::vector<size_t>>
  velocity_indices(size_t tr, const cda_rail::index_vector& route,
                   const std::vector<VertexPassing>& passings) const;
  /**
   * @brief Sets the MIP start of SolverStrategyMovingBlock::use_warm_start.
   *
   * The completed initial state, see completed_initial_state(), is simulated
   * by the greedy simulator. Every integer variable is set accordingly: the
   * routes and stops are those of the state, the velocities those of
   * velocity_indices(), and two trains are ordered on an edge, a TTD section
   * or a bidirectional track by when their fronts enter it in the simulation.
   * A train waits wherever it passes at velocity 0. Every other integer
   * variable of m_vars is 0. The helper binaries of indicator constraints,
   * which are not part of m_vars, are left to Gurobi. The model does not let a
   * train stand still at its exit vertex, from where it could not reach its
   * exit velocity. If it does so in the simulation, it passes its exit vertex
   * at the closest velocity the model allows instead.
   *
   * Nothing is set if the completion fails, or if the model has no velocities
   * for how a train passes its route in the simulation, see
   * velocity_indices(). Gurobi rejects the start if the times cannot be
   * completed, e.g., since the velocities of the model are discrete.
   */
  void set_warm_start();

  class LazyCallback : public MessageCallback {
  private:
    /** @brief A lazy constraint lhs >= rhs. */
    struct LazyConstraint {
      GRBLinExpr lhs;
      GRBLinExpr rhs;
    };

    /** @brief The rule by which lazy constraints are created. */
    enum class LazyConstraintKind : std::uint8_t {
      VertexHeadway  = 0,
      EdgeHeadway    = 1,
      TtdHeadway     = 2,
      ReverseEdge    = 3,
      SimplifiedEdge = 4,
      SimplifiedTtd  = 5,
    };

    GenPOMovingBlockMIPSolver* solver;
    // The lazy constraints added so far, by a key that identifies how they
    // were created, see add_lazy_constraints().
    std::map<std::vector<size_t>, std::vector<LazyConstraint>>
        m_added_lazy_constraints;
    // Gurobi's feasibility tolerance, with which a constraint that was added
    // before counts as violated.
    double m_feasibility_tolerance = 0;
    // Number of addLazy() calls so far, see
    // SolveStatistics::num_lazy_constraints_added.
    int m_num_lazy_constraints_added = 0;

    /**
     * @brief Adds lazy constraints unless they were added before.
     *
     * Constraints that were added before are only added again if the current
     * solution violates them by more than the feasibility tolerance. Gurobi
     * already enforces them, so such a violation is within Gurobi's own
     * tolerances, and adding them again does not reject the solution.
     *
     * @param kind The rule by which the constraints are created
     * @param key Every index that determines the constraints for this rule,
     *        i.e., equal keys always lead to equal constraints
     * @param create Creates the constraints as a std::vector<LazyConstraint>,
     *        only called for a new key
     * @return Whether one of the newly created constraints is violated by the
     *         current solution, i.e., whether the solution is rejected. Like
     *         Gurobi, this measures the violation relative to the magnitude of
     *         the constraint at the current solution. Always false for
     *         constraints that were added before.
     */
    template <typename CreateConstraints>
    [[nodiscard]] bool add_lazy_constraints(LazyConstraintKind       kind,
                                            std::vector<size_t>      key,
                                            const CreateConstraints& create);
    /** @brief The value of an expression in the current solution. */
    [[nodiscard]] double get_solution_value(const GRBLinExpr& expr);
    /** @brief The index of a velocity among those of a train at a vertex. */
    [[nodiscard]] size_t velocity_index(size_t tr, size_t v, double vel) const;

    /** @brief Where the moving authority of a train ends on its route. */
    struct MovingAuthorityEnd {
      // The edges from the vertex the train passes up to the one the moving
      // authority ends on
      cda_rail::index_vector path;
      // The distance of the end from the source of the last of these edges
      double pos_on_edge;
    };
    /**
     * @brief Where the moving authority of a train ends if it passes a vertex
     *        of its route with a given braking distance.
     *
     * @param route The route of the train, see get_routes()
     * @param r_v_idx Index of the vertex within the route, not its last one
     * @param braking_distance The braking distance of the train at the vertex
     * @return The end, which is the end of the route (the exit) if the
     *         braking distance reaches beyond it
     */
    [[nodiscard]] MovingAuthorityEnd
    moving_authority_end(const std::vector<std::pair<size_t, double>>& route,
                         size_t r_v_idx, double braking_distance) const;
    /**
     * @brief Adds the headway constraints by which a train passing a vertex
     *        at a given velocity follows another train on the edge its moving
     *        authority ends on.
     *
     * The constraints only bind if the train passes the vertex at this
     * velocity (or a higher one, see
     * LazyVelocitySelectionStrategy::ExtendToHigher), uses the path to the
     * end of its moving authority, and follows the other train on the edge.
     *
     * @param tr Index of the following train
     * @param other_tr Index of the train ahead
     * @param v_idx Index of the vertex
     * @param vel The velocity at which tr passes v_idx
     * @param ma_end Where the moving authority of tr ends at this velocity,
     *        see moving_authority_end()
     * @return Whether the current solution violates one of the added
     *         constraints, see add_lazy_constraints()
     */
    [[nodiscard]] bool
    add_lazy_edge_headway_constraints(size_t tr, size_t other_tr, size_t v_idx,
                                      double                    vel,
                                      const MovingAuthorityEnd& ma_end);

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

    /** @brief The number of lazy constraints handed to Gurobi so far. */
    [[nodiscard]] int get_num_lazy_constraints_added() const {
      return m_num_lazy_constraints_added;
    }

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
