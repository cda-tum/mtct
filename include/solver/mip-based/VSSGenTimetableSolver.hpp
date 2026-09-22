#pragma once
#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "GeneralMIPSolver.hpp"
#include "VSSModel.hpp"
#include "datastructure/Timetable.hpp"
#include "gurobi_c++.h"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"
#include "solver/GeneralSolver.hpp"
#include "unordered_map"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace cda_rail::solver::mip_based {
using std::size_t;

/**
 * @brief How the iterative approach increases the number of VSS per edge.
 *
 * `Fixed` works with absolute numbers of VSS, whereas `Relative` works with
 * fractions of the number that is theoretically possible on an edge.
 */
enum class UpdateStrategyVSSGen : std::uint8_t { Fixed = 0, Relative = 1 };

/** @brief The name of @p strategy, as it is reported and parsed. */
constexpr std::string update_strategy_to_string(UpdateStrategyVSSGen strategy) {
  switch (strategy) {
  case UpdateStrategyVSSGen::Fixed:
    return "Fixed";
  case UpdateStrategyVSSGen::Relative:
    return "Relative";
  default:
    throw cda_rail::exceptions::ConsistencyException("Unknown update strategy");
  }
}

/** @brief The name of @p strategy, as it is reported and parsed. */
constexpr std::string
optimality_strategy_to_string(OptimalityStrategy strategy) {
  switch (strategy) {
  case OptimalityStrategy::Optimal:
    return "Optimal";
  case OptimalityStrategy::TradeOff:
    return "TradeOff";
  case OptimalityStrategy::Feasible:
    return "Feasible";
  default:
    throw cda_rail::exceptions::ConsistencyException(
        "Unknown optimality strategy");
  }
}

/**
 * @brief How the VSS generation model is solved.
 *
 * Instead of solving the full model at once, the number of VSS allowed per
 * edge can be increased iteratively until optimality is proven. The remaining
 * settings only apply to that approach.
 */
struct SolverStrategyVSSGen {
  bool                         iterative_approach = false;
  cda_rail::OptimalityStrategy optimality_strategy =
      cda_rail::OptimalityStrategy::Optimal;
  UpdateStrategyVSSGen update_strategy = UpdateStrategyVSSGen::Fixed;
  double               initial_value   = 1;
  double               update_value    = 2;
  bool                 include_cuts    = true;
};

/** @brief How accurately the VSS generation model describes the operation. */
struct ModelDetailVSSGen {
  double delta_t        = 15;
  bool   fix_routes     = true;
  bool   train_dynamics = true;
  bool   braking_curves = true;
};

/**
 * @brief The same, plus how much of a moving block solution is used.
 *
 * Everything that is fixed restricts the model to what the moving block
 * solution did, which speeds up the solving process but can cut off better
 * solutions. A hint is only a starting point and hence keeps the model as it
 * is. The routes are always the ones of that solution, since everything else
 * refers to them.
 */
struct ModelDetailMBInformation {
  double delta_t                    = 15;
  bool   train_dynamics             = true;
  bool   braking_curves             = true;
  bool   fix_stop_positions         = true;
  bool   fix_exact_positions        = true;
  bool   fix_exact_velocities       = true;
  bool   hint_approximate_positions = true;
  bool   fix_order_on_edges         = true;
};

/** @brief How the VSS borders and the braking distances are modelled. */
struct ModelSettingsVSSGen {
  // NOLINTNEXTLINE(readability-redundant-member-init)
  vss::Model model_type{};
  bool       use_pwl{false};
  bool       use_schedule_cuts{true};
};

/**
 * @brief Adds as few VSS as possible so that the timetable can be operated.
 *
 * The model is time-discretized: for every train and every time step it is
 * decided where the train is, and the VSS borders have to separate every two
 * trains that are on the same TTD section at the same time. In contrast to the
 * moving block problem, the timetable is operated exactly as specified, so
 * that the number of VSS borders is the only thing being minimized.
 */
class VSSGenTimetableSolver
    : public GeneralMIPSolver<
          instances::GeneralPerformanceOptimizationInstance,
          instances::SolVSSGeneralPerformanceOptimizationInstance> {
  friend class VSSGenTimetableSolverWithMovingBlockInformation;

private:
  // Instance variables
  double                                 dt{-1};
  size_t                                 num_t{0};
  size_t                                 num_tr{0};
  size_t                                 num_edges{0};
  size_t                                 num_vertices{0};
  size_t                                 num_breakable_sections{0};
  std::vector<cda_rail::index_vector>    unbreakable_sections;
  std::vector<cda_rail::index_vector>    no_border_vss_sections;
  std::vector<std::pair<size_t, size_t>> train_interval;
  std::vector<std::pair<std::optional<size_t>, std::optional<size_t>>>
                         breakable_edges_pairs;
  cda_rail::index_vector no_border_vss_vertices;
  cda_rail::index_vector relevant_edges;
  cda_rail::index_vector breakable_edges;
  bool                   fix_routes{false};
  vss::Model             vss_model{
      vss::ModelType::Continuous}; // This line is correct and intended.
                                   // vss_model is of type vss::Model
  bool                   include_train_dynamics{false};
  bool                   include_braking_curves{false};
  bool                   use_pwl{false};
  bool                   use_schedule_cuts{false};
  bool                   iterative_vss{false};
  OptimalityStrategy     optimality_strategy{OptimalityStrategy::Optimal};
  UpdateStrategyVSSGen   iterative_update_strategy{UpdateStrategyVSSGen::Fixed};
  double                 iterative_initial_value{1.0};
  double                 iterative_update_value{2.0};
  bool                   iterative_include_cuts{true};
  bool                   iterative_include_cuts_tmp{true};
  bool                   postprocess{false};
  cda_rail::index_vector max_vss_per_edge_in_iteration{};
  std::unordered_map<size_t, size_t> breakable_edge_indices{};
  std::vector<std::pair<cda_rail::index_vector, cda_rail::index_vector>>
      fwd_bwd_sections{};

  // Variable functions
  /** @brief Creates all variables of the requested model variant. */
  void create_variables();
  /** @brief Creates the variables that every variant needs. */
  void create_general_variables();
  /** @brief Creates the variables needed if the routes are fixed. */
  void create_fixed_routes_variables();
  /** @brief Creates the variables needed if the routes are not fixed. */
  void create_free_routes_variables();
  /** @brief Creates the variables deciding the VSS of a discretized network. */
  void create_discretized_variables();
  /** @brief Creates the variables needed if the network is not discretized. */
  void create_non_discretized_variables();
  /** @brief Creates the variables corresponding to the braking distances. */
  void create_brakelen_variables();
  /** @brief Creates the variables needed if trains only stop at VSS. */
  void create_only_stop_at_vss_variables();
  /** @brief Creates those of them that refer to non-discretized VSS. */
  void create_non_discretized_only_stop_at_vss_variables();

  // Constraint functions
  /** @brief Creates all constraints of the requested model variant. */
  void create_constraints();
  /** @brief Creates the constraints that every variant needs. */
  void create_general_constraints();
  /** @brief Creates the constraints that only appear if routes are fixed. */
  void create_fixed_routes_constraints();
  /** @brief Creates the constraints that only appear if they are not. */
  void create_free_routes_constraints();
  /**
   * @brief Creates the VSS constraints of a discretized network, i.e., two
   *        trains on a NoBorderVSS section have to be separated by a chosen
   *        vertex.
   */
  void create_discretized_constraints();
  /** @brief Creates the constraints if the network is not discretized. */
  void create_non_discretized_constraints();
  /** @brief Creates the constraints limiting acceleration and deceleration. */
  void create_acceleration_constraints();
  /** @brief Creates the constraints related to the braking distances. */
  void create_brakelen_constraints();

  // Helper functions for constraints
  /** @brief Creates the general boundary conditions, i.e., on the speed. */
  void create_general_boundary_constraints();

  /**
   * @brief Creates the constraints of a train being in a station, in which
   *        case all of its other position variables and its speed are zero.
   */
  void create_general_schedule_constraints();
  /** @brief Only one train may be on an unbreakable section at a time. */
  void create_unbreakable_sections_constraints();
  /** @brief No train may exceed the maximal speed of an edge. */
  void create_general_speed_constraints();
  /**
   * @brief A breakable section may only be occupied in one direction at a
   *        time.
   *
   * This prevents trains from blocking each other, since reversing trains are
   * not modelled.
   */
  void create_reverse_occupation_constraints();

  /** @brief The trains move along their fixed routes. */
  void create_fixed_routes_position_constraints();
  /** @brief Creates the boundary conditions of the fixed routes. */
  void create_boundary_fixed_routes_constraints();
  /** @brief Creates the edge occupation of trains with fixed routes. */
  void create_fixed_routes_occupation_constraints();
  /** @brief Constrains lambda and mu for fixed routes in stations. */
  void create_fixed_route_schedule_constraints();
  /** @brief Cuts off solutions that are not possible in any way. */
  void create_fixed_routes_impossibility_cuts();
  /** @brief Creates the constraints on common entry and exit points. */
  void create_fixed_routes_no_overlap_entry_exit_constraints();

  /**
   * @brief Creates the constraints that only appear if the network is not
   *        discretized, but are general enough to appear in every variant.
   */
  void create_non_discretized_general_constraints();
  /** @brief Creates the position constraints of the non-discretized VSS. */
  void create_non_discretized_position_constraints();
  /** @brief Creates those VSS constraints if the routes are not fixed. */
  void create_non_discretized_free_route_constraints();
  /** @brief Creates those VSS constraints if the routes are fixed. */
  void create_non_discretized_fixed_route_constraints();
  /** @brief Places the VSS borders by the separation functions. */
  void create_non_discretized_fraction_constraints();
  /** @brief Does so in the alternative formulation, see ModelType. */
  void create_non_discretized_alt_fraction_constraints();
  /** @brief Trains may only stop at a VSS border. */
  void create_non_discretized_general_only_stop_at_vss_constraints();
  /** @brief Adds what that needs if the routes are not fixed. */
  void create_non_discretized_free_routes_only_stop_at_vss_constraints();
  /** @brief Adds what that needs if the routes are fixed. */
  void create_non_discretized_fixed_routes_only_stop_at_vss_constraints();

  /** @brief Creates the constraints positioning the trains. */
  void create_free_routes_position_constraints();
  /** @brief Creates the constraints ensuring the correct overlap. */
  void create_free_routes_overlap_constraints();
  /** @brief Creates the boundary conditions of the free routes. */
  void create_boundary_free_routes_constraints();
  /** @brief Connects the position and the occupation variables. */
  void create_free_routes_occupation_constraints();
  /** @brief Cuts off positions that are impossible by the schedule. */
  void create_free_routes_impossibility_cuts();
  /** @brief Creates the constraints on common entry and exit points. */
  void create_free_routes_no_overlap_entry_exit_constraints();

  // Objective
  /** @brief Sets the objective function, i.e., the number of VSS borders. */
  void set_objective();

  // Helper functions
  /** @brief Sets the remaining time limit of the Gurobi model. */
  void set_timeout(int time_limit);
  /** @brief Runs the optimization, iteratively if that was asked for. */
  [[nodiscard]] std::optional<
      instances::SolVSSGeneralPerformanceOptimizationInstance>
  optimize(const std::optional<
               instances::GeneralPerformanceOptimizationInstance>& old_instance,
           int                                                     time_limit);
  /**
   * @brief Exports the MIP model itself if solution_settings ask for it.
   *
   * The model is written into the standard solution directory of the given
   * solution, i.e., to the very same place the solution itself is exported to,
   * using solution_settings.model_name as file name. This happens
   * independently of the export option, i.e., the model can be exported
   * without the solution and vice versa.
   *
   * @param sol_object The solution defining the export directory
   * @param solution_settings Settings describing where and what to export
   */
  void export_lp_model_if_applicable(
      const instances::SolVSSGeneralPerformanceOptimizationInstance& sol_object,
      const SolutionSettingsVSSGen& solution_settings);
  /**
   * @brief Collects the solver settings written to solver_data.json.
   *
   * @param solution_settings The export settings used
   * @param time_limit The time limit in seconds passed to the solver
   * @return The settings as further data of the solver data export
   */
  [[nodiscard]] FurtherData
  get_further_data(const SolutionSettingsVSSGen& solution_settings,
                   int                           time_limit) const;
  /**
   * @brief The unbreakable sections traversed by a train.
   *
   * @param train_index Index of the train
   * @return Indices of the unbreakable sections
   */
  [[nodiscard]] cda_rail::index_vector
  unbreakable_section_indices(size_t train_index) const;
  /** @brief Clusters the forward and backward edges of every section. */
  void calculate_fwd_bwd_sections();
  /** @brief Does so for the sections of a discretized network. */
  void calculate_fwd_bwd_sections_discretized();
  /** @brief Does so for the sections of a network that is not. */
  void calculate_fwd_bwd_sections_non_discretized();
  /** @brief The maximal braking distance of a train. */
  [[nodiscard]] double get_max_brakelen(const size_t& tr) const;

  /**
   * @brief The trains sharing an entry or exit vertex, sorted by the time at
   *        which they enter or leave.
   */
  [[nodiscard]] std::pair<std::vector<cda_rail::index_vector>,
                          std::vector<cda_rail::index_vector>>
  common_entry_exit_vertices() const;

  /**
   * @brief The stops surrounding a point in time, see
   *        get_temporary_impossibility_struct.
   */
  struct TemporaryImpossibilityStruct {
    bool                   to_use;
    size_t                 t_before;
    size_t                 t_after;
    double                 v_before;
    double                 v_after;
    cda_rail::index_vector edges_before;
    cda_rail::index_vector edges_after;
  };
  /**
   * @brief The last stop before and the first stop after a point in time.
   *
   * The times and velocities are those of the entry and the exit of the train
   * if there is no such stop. Not to be used while the train is being
   * serviced, which `to_use` reports.
   *
   * @param tr Index of the train
   * @param t Time index
   * @return The surrounding stops of the train at that time
   */
  [[nodiscard]] TemporaryImpossibilityStruct
  get_temporary_impossibility_struct(const size_t& tr, const size_t& t) const;

  /**
   * @brief Time step indices during which a train is serviced at a stop.
   *
   * The stop itself is given in continuous time, the model only knows time step
   * indices. The first index is the last time step at or before the start of
   * the service, the second index the first time step at or after the earliest
   * departure.
   *
   * @param stop The scheduled stop in question.
   * @return Pair of time step indices (t0, t1) belonging to the stop.
   */
  [[nodiscard]] std::pair<size_t, size_t>
  stop_time_indices(const ScheduledStop& stop) const;

  [[nodiscard]] double
  max_distance_travelled(const size_t& tr, const size_t& time_steps,
                         const double& v0, const double& a_max,
                         const bool& braking_distance) const;

  [[nodiscard]] instances::SolVSSGeneralPerformanceOptimizationInstance
  extract_solution(
      bool postprocess, bool full_model,
      const std::optional<instances::GeneralPerformanceOptimizationInstance>&
          old_instance) const;

  /**
   * @brief Adds the VSS borders chosen by the discretized model to a solution.
   *
   * In the discretized model a VSS border is a vertex of the discretized
   * network. The solution object, however, refers to the original network, in
   * which such a vertex is a position on the edge it was created from. The
   * border is added to that edge and, if it exists, to its reverse edge.
   *
   * @param sol_obj Solution object the VSS borders are added to
   * @return Number of VSS borders chosen by the model
   */
  int extract_discretized_vss_positions(
      instances::SolVSSGeneralPerformanceOptimizationInstance& sol_obj) const;

  /**
   * @brief The route a train takes according to the occupation variables.
   *
   * Only meaningful if the routes are not fixed. For the discretized VSS model
   * the returned edges belong to the discretized network, which has to be
   * mapped back to the original network before it is written into a solution
   * object.
   *
   * @param tr Index of the train
   * @return Edges of the train's route in the order they are traversed
   */
  [[nodiscard]] cda_rail::index_vector extract_model_route(size_t tr) const;

  /**
   * @brief Increases the number of VSS allowed on an edge, see
   *        UpdateStrategyVSSGen.
   *
   * @param relevant_edge_index Index of the edge within the relevant edges
   * @param obj_ub Objective value of the best solution found so far
   * @param cut_expr Expression of the cut excluding the explored search space
   * @return `false` if the edge already allows every possible VSS
   */
  bool update_vss(size_t relevant_edge_index, double obj_ub,
                  GRBLinExpr& cut_expr);
  /** @brief Sets that number, relaxing the bounds of its variables. */
  void update_max_vss_on_edge(size_t relevant_edge_index, size_t new_max_vss,
                              GRBLinExpr& cut_expr);
  /**
   * @brief Stores the given settings and derives everything the model needs.
   *
   * @return The original instance if it had to be discretized, since the
   *         solution refers to it, and nothing otherwise.
   */
  [[nodiscard]] std::optional<instances::GeneralPerformanceOptimizationInstance>
  initialize_variables(const ModelDetailVSSGen&      model_detail,
                       const ModelSettingsVSSGen&    model_settings,
                       const SolverStrategyVSSGen&   solver_strategy,
                       const SolutionSettingsVSSGen& solution_settings,
                       int time_limit, bool debug_input,
                       bool overwrite_severity);

protected:
  void cleanup() override;

  /** @brief Initializes the logging and the Gurobi model. */
  void solve_init_vss_gen_timetable(bool debug_input, bool overwrite_severity) {
    this->solve_init_general_mip(debug_input, overwrite_severity);
  };

public:
  ~VSSGenTimetableSolver() override = default;
  // Constructors
  explicit VSSGenTimetableSolver(
      const instances::GeneralPerformanceOptimizationInstance& instance)
      : GeneralMIPSolver(instance) {};
  template <typename... Args>
  explicit VSSGenTimetableSolver(Args&&... args)
    requires(!IsSingleInstanceArgument<Args...>::value)
      : GeneralMIPSolver(std::forward<Args>(args)...) {}

  // Methods
  /**
   * @brief Solves the instance, adding as few VSS borders as possible.
   *
   * @param model_detail How accurately the operation is modelled.
   * @param model_settings How the VSS borders and braking curves are modelled.
   * @param solver_strategy How the model is solved.
   * @param solution_settings Where and what is exported.
   * @param time_limit Time limit in seconds. No limit if negative.
   * @param debug_input If true, (more detailed) debug output is printed.
   * @param overwrite_severity If true, the severity of the log is overwritten
   *        even if this decreases the logging level.
   * @return Solution object containing status, objective value, and solution.
   */
  [[nodiscard]] instances::SolVSSGeneralPerformanceOptimizationInstance
  solve(const ModelDetailVSSGen&      model_detail,
        const ModelSettingsVSSGen&    model_settings    = {},
        const SolverStrategyVSSGen&   solver_strategy   = {},
        const SolutionSettingsVSSGen& solution_settings = {},
        int time_limit = -1, bool debug_input = false,
        bool overwrite_severity = true);

  using GeneralSolver::solve;
  [[nodiscard]] instances::SolVSSGeneralPerformanceOptimizationInstance
  solve(int time_limit, bool debug_input, bool overwrite_severity) override {
    return solve({}, {}, {}, {}, time_limit, debug_input, overwrite_severity);
  }
};

/**
 * @brief The same solver, guided by a moving block solution of the instance.
 *
 * A moving block solution is a lower bound on what any VSS layout can achieve,
 * so it is a good starting point. How much of it is fixed and how much is only
 * hinted to the solver is given by ModelDetailMBInformation.
 */
class VSSGenTimetableSolverWithMovingBlockInformation
    : public VSSGenTimetableSolver {
private:
  instances::SolGeneralPerformanceOptimizationInstance m_moving_block_solution;
  bool m_fix_orders_on_edges{true};
  bool m_fix_stop_positions{true};
  bool m_fix_exact_positions{true};
  bool m_fix_exact_velocities{true};
  bool m_hint_approximate_positions{true};

  // Additional functions
  /** @brief Adds everything the moving block solution contributes. */
  void include_additional_information();
  /** @brief Fixes the order in which the trains traverse the edges. */
  void fix_oder_on_edges();
  /** @brief Fixes the positions at which the trains serve their stops. */
  void fix_stop_positions_constraints();
  /** @brief Fixes or bounds the positions and velocities at the vertices. */
  void fix_exact_positions_and_velocities_constraints();
  /** @brief Hints the positions of the trains at every point in time. */
  void hint_approximate_positions_constraints();

protected:
  void cleanup() override;

public:
  ~VSSGenTimetableSolverWithMovingBlockInformation() override = default;
  // Constructors
  explicit VSSGenTimetableSolverWithMovingBlockInformation(
      const instances::SolGeneralPerformanceOptimizationInstance&
          moving_block_solution)
      : VSSGenTimetableSolver(*moving_block_solution.get_instance()),
        m_moving_block_solution(moving_block_solution) {
    // All information extracted from the moving block solution (train orders,
    // positions, velocities, ...) refers to the routes of that solution, which
    // may differ from the routes specified in the instance. Since this solver
    // always uses fixed routes, the instance routes are replaced by the ones
    // of the moving block solution.
    m_instance.set_routes(m_moving_block_solution.get_const_solution_routes());
  };

  // Methods
  /**
   * @brief Solves the instance, guided by the moving block solution.
   *
   * Works as the solve function of the parent class, except that the routes
   * and, depending on @p model_detail_mb_information, further parts of the
   * moving block solution are fixed or hinted to the solver.
   *
   * @return Solution object containing status, objective value, and solution.
   */
  [[nodiscard]] instances::SolVSSGeneralPerformanceOptimizationInstance
  solve(const ModelDetailMBInformation& model_detail_mb_information,
        const ModelSettingsVSSGen&      model_settings    = {},
        const SolverStrategyVSSGen&     solver_strategy   = {},
        const SolutionSettingsVSSGen&   solution_settings = {},
        int time_limit = -1, bool debug_input = false,
        bool overwrite_severity = true);

  using GeneralSolver::solve;
  [[nodiscard]] instances::SolVSSGeneralPerformanceOptimizationInstance
  solve(int time_limit, bool debug_input, bool overwrite_severity) override {
    return solve({}, {}, {}, {}, time_limit, debug_input, overwrite_severity);
  }
};

} // namespace cda_rail::solver::mip_based
