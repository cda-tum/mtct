#pragma once

#include "Definitions.hpp"
#include "datastructure/GeneralTimetable.hpp"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"
#include "solver/GeneralSolver.hpp"
#include "solver/mip-based/GeneralMIPSolver.hpp"

#include "gtest/gtest_prod.h"
#include <cstddef>
#include <cstdint>
#include <filesystem>
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

struct ModelDetail {
  bool                       fix_routes         = false;
  double                     max_velocity_delta = 5.55; // 20 km/h
  VelocityRefinementStrategy velocity_refinement_strategy =
      VelocityRefinementStrategy::MinOneStep;
  bool simplify_headway_constraints          = false;
  bool strengthen_vertex_headway_constraints = false;
};

enum class LazyConstraintSelectionStrategy : std::uint8_t {
  OnlyViolated   = 0,
  OnlyFirstFound = 1,
  AllChecked     = 2
};

enum class LazyTrainSelectionStrategy : std::uint8_t {
  OnlyAdjacent = 0,
  All          = 1
};

struct SolverStrategyMovingBlock {
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

class GenPOMovingBlockMIPSolver
    : public GeneralMIPSolver<
          instances::GeneralPerformanceOptimizationInstance,
          instances::SolGeneralPerformanceOptimizationInstance<
              instances::GeneralPerformanceOptimizationInstance>> {
private:
#if TEST_FRIENDS
  FRIEND_TEST(::GenPOMovingBlockMIPSolver, PrivateFillFunctions);
#endif

  SolutionSettingsMovingBlock      solution_settings = {};
  ModelDetail                      model_detail      = {};
  SolverStrategyMovingBlock        solver_strategy   = {};
  size_t                           num_tr            = 0;
  size_t                           num_edges         = 0;
  size_t                           num_vertices      = 0;
  size_t                           num_ttd           = 0;
  int                              max_t             = 0;
  std::vector<std::vector<size_t>> ttd_sections;
  // tr_stop_data:
  // For every train, for every station, list of possible stop vertices together
  // with respective edges
  std::vector<std::vector<
      std::vector<std::pair<size_t, std::vector<std::vector<size_t>>>>>>
                                                tr_stop_data;
  std::vector<std::vector<std::vector<double>>> velocity_extensions;
  std::vector<std::pair<size_t, size_t>>        relevant_reverse_edges;

  void initialize_variables(
      const SolutionSettingsMovingBlock& solution_settings_input,
      const SolverStrategyMovingBlock&   solver_strategy_input,
      const ModelDetail&                 model_detail_input);

  double ub_timing_variable(size_t tr) const;

  void fill_tr_stop_data();
  void fill_relevant_reverse_edges();
  void fill_velocity_extensions();
  void fill_velocity_extensions_using_none_strategy();
  void fill_velocity_extensions_using_min_one_step_strategy();

  size_t get_maximal_velocity_extension_size() const;

  [[nodiscard]] std::tuple<double, GRBLinExpr, double, GRBLinExpr>
  get_vertex_headway_expressions(size_t tr, size_t e);
  [[nodiscard]] std::tuple<double, GRBLinExpr, double, GRBLinExpr>
  get_edge_headway_expressions(size_t tr, size_t e);

  void create_variables();
  void create_timing_variables();
  void create_general_edge_variables();
  void create_stop_variables();
  void create_velocity_extended_variables();
  void create_reverse_edge_variables();

  void set_objective();

  void create_constraints();
  void create_general_path_constraints();
  void create_travel_times_constraints();
  void create_basic_order_constraints();
  void create_basic_ttd_constraints();
  void create_train_rear_constraints();
  void create_reverse_edge_constraints();
  void create_stopping_constraints();
  void create_vertex_headway_constraints();
  void create_headway_constraints();
  void create_simplified_headway_constraints();

  // Helper for headway normal and lazy constraints
  [[nodiscard]] GRBLinExpr
  get_edge_path_expr(size_t tr, const std::vector<size_t>& p,
                     double initial_velocity,
                     bool   also_higher_velocities = false);

  void extract_solution(
      instances::SolGeneralPerformanceOptimizationInstance<
          instances::GeneralPerformanceOptimizationInstance>& sol) const;
  double        extract_speed(size_t tr, size_t vertex_id) const;
  static double headway(const Train& tr_obj, const Edge& e_obj, double v_0,
                        double v_1, bool entry_vertex = false);

  class LazyCallback : public MessageCallback {
  private:
    GenPOMovingBlockMIPSolver* solver;

    std::vector<std::vector<std::pair<size_t, double>>> get_routes();
    std::vector<std::unordered_map<size_t, double>>     get_train_velocities(
            const std::vector<std::vector<std::pair<size_t, double>>>& routes);
    std::vector<std::pair<std::vector<std::pair<size_t, bool>>,
                          std::vector<std::pair<size_t, bool>>>>
    get_train_orders_on_edges(
        const std::vector<std::vector<std::pair<size_t, double>>>& routes);
    std::vector<std::vector<size_t>> get_train_orders_on_ttd();

    bool create_lazy_edge_and_ttd_headway_constraints(
        const std::vector<std::vector<std::pair<size_t, double>>>& routes,
        const std::vector<std::unordered_map<size_t, double>>& train_velocities,
        const std::vector<std::pair<std::vector<std::pair<size_t, bool>>,
                                    std::vector<std::pair<size_t, bool>>>>&
                                                train_orders_on_edges,
        const std::vector<std::vector<size_t>>& train_orders_on_ttd);
    bool create_lazy_simplified_edge_constraints(
        const std::vector<std::vector<std::pair<size_t, double>>>& routes,
        const std::vector<std::unordered_map<size_t, double>>& train_velocities,
        const std::vector<std::pair<std::vector<std::pair<size_t, bool>>,
                                    std::vector<std::pair<size_t, bool>>>>&
                                                train_orders_on_edges,
        const std::vector<std::vector<size_t>>& train_orders_on_ttd);
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
  virtual void cleanup() override;

public:
  GenPOMovingBlockMIPSolver() = default;

  explicit GenPOMovingBlockMIPSolver(
      const instances::GeneralPerformanceOptimizationInstance& instance)
      : GeneralMIPSolver<
            instances::GeneralPerformanceOptimizationInstance,
            instances::SolGeneralPerformanceOptimizationInstance<
                instances::GeneralPerformanceOptimizationInstance>>(instance) {
        };

  explicit GenPOMovingBlockMIPSolver(const std::filesystem::path& p)
      : GeneralMIPSolver<
            instances::GeneralPerformanceOptimizationInstance,
            instances::SolGeneralPerformanceOptimizationInstance<
                instances::GeneralPerformanceOptimizationInstance>>(p) {};

  explicit GenPOMovingBlockMIPSolver(const std::string& path)
      : GeneralMIPSolver<
            instances::GeneralPerformanceOptimizationInstance,
            instances::SolGeneralPerformanceOptimizationInstance<
                instances::GeneralPerformanceOptimizationInstance>>(path) {};

  explicit GenPOMovingBlockMIPSolver(const char* path)
      : GeneralMIPSolver<
            instances::GeneralPerformanceOptimizationInstance,
            instances::SolGeneralPerformanceOptimizationInstance<
                instances::GeneralPerformanceOptimizationInstance>>(path) {};

  ~GenPOMovingBlockMIPSolver() = default;

  using GeneralSolver::solve;
  [[nodiscard]] instances::SolGeneralPerformanceOptimizationInstance<
      instances::GeneralPerformanceOptimizationInstance>
  solve(int time_limit, bool debug_input) override {
    return solve({}, {}, {}, time_limit, debug_input);
  };

  [[nodiscard]] instances::SolGeneralPerformanceOptimizationInstance<
      instances::GeneralPerformanceOptimizationInstance>
  solve(const ModelDetail&                 model_detail_input,
        const SolverStrategyMovingBlock&   solver_strategy_input,
        const SolutionSettingsMovingBlock& solution_settings_input,
        int time_limit = -1, bool debug_input = false);
};

} // namespace cda_rail::solver::mip_based
