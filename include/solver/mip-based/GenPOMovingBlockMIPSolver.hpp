#pragma once

#include "Definitions.hpp"
#include "datastructure/GeneralTimetable.hpp"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"
#include "solver/GeneralSolver.hpp"
#include "solver/mip-based/GeneralMIPSolver.hpp"

#include "gtest/gtest_prod.h"
#include <cstddef>
#include <filesystem>
#include <string>
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
};

class GenPOMovingBlockMIPSolver
    : public GeneralMIPSolver<
          instances::GeneralPerformanceOptimizationInstance,
          instances::SolGeneralPerformanceOptimizationInstance> {
private:
#if TEST_FRIENDS
  FRIEND_TEST(::GenPOMovingBlockMIPSolver, PrivateFillFunctions);
#endif

  SolutionSettingsMovingBlock      solution_settings = {};
  ModelDetail                      model_detail      = {};
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

  void initialize_variables(
      const SolutionSettingsMovingBlock& solution_settings_input,
      const ModelDetail&                 model_detail_input);

  double ub_timing_variable(size_t tr) const;

  void fill_tr_stop_data();
  void fill_velocity_extensions();
  void fill_velocity_extensions_using_none_strategy();
  void fill_velocity_extensions_using_min_one_step_strategy();

  size_t get_maximal_velocity_extension_size() const;

  void create_variables();
  void create_timing_variables();
  void create_general_edge_variables();
  void create_stop_variables();
  void create_velocity_extended_variables();

  void set_objective();

  void create_constraints();
  void create_general_path_constraints();
  void create_travel_times_constraints();
  void create_basic_order_constraints();
  void create_basic_ttd_constraints();
  void create_train_rear_constraints();
  void create_stopping_constraints();
  void create_headway_constraints();

protected:
  void solve_init_gen_po_mb(int time_limit, bool debug_input) {
    this->solve_init_general_mip(time_limit, debug_input);
  };

public:
  GenPOMovingBlockMIPSolver() = default;

  explicit GenPOMovingBlockMIPSolver(
      const instances::GeneralPerformanceOptimizationInstance& instance)
      : GeneralMIPSolver<instances::GeneralPerformanceOptimizationInstance,
                         instances::SolGeneralPerformanceOptimizationInstance>(
            instance){};

  explicit GenPOMovingBlockMIPSolver(const std::filesystem::path& p)
      : GeneralMIPSolver<instances::GeneralPerformanceOptimizationInstance,
                         instances::SolGeneralPerformanceOptimizationInstance>(
            p){};

  explicit GenPOMovingBlockMIPSolver(const std::string& path)
      : GeneralMIPSolver<instances::GeneralPerformanceOptimizationInstance,
                         instances::SolGeneralPerformanceOptimizationInstance>(
            path){};

  explicit GenPOMovingBlockMIPSolver(const char* path)
      : GeneralMIPSolver<instances::GeneralPerformanceOptimizationInstance,
                         instances::SolGeneralPerformanceOptimizationInstance>(
            path){};

  ~GenPOMovingBlockMIPSolver() = default;

  using GeneralSolver::solve;
  [[nodiscard]] instances::SolGeneralPerformanceOptimizationInstance
  solve(int time_limit, bool debug_input) override {
    return solve({}, {}, time_limit, debug_input);
  };

  [[nodiscard]] instances::SolGeneralPerformanceOptimizationInstance
  solve(const ModelDetail&                 model_detail_input,
        const SolutionSettingsMovingBlock& solution_settings_input,
        int time_limit, bool debug_input);
};

} // namespace cda_rail::solver::mip_based
