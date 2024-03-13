#include "datastructure/GeneralTimetable.hpp"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"
#include "solver/GeneralSolver.hpp"
#include "solver/mip-based/GeneralMIPSolver.hpp"

#include <filesystem>
#include <string>

namespace cda_rail::solver::mip_based {
class GenPOMovingBlockMIPSolver
    : public GeneralMIPSolver<
          instances::GeneralPerformanceOptimizationInstance,
          instances::SolGeneralPerformanceOptimizationInstance> {
private:
  SolutionSettingsMovingBlock solution_settings = {};
  size_t                      num_tr            = 0;
  size_t                      num_edges         = 0;
  size_t                      num_vertices      = 0;
  int                         max_t             = 0;

  void create_variables();
  void create_timing_variables();

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
    return solve({}, time_limit, debug_input);
  };

  [[nodiscard]] instances::SolGeneralPerformanceOptimizationInstance
  solve(const SolutionSettingsMovingBlock& solution_settings_input,
        int time_limit, bool debug_input);
};
} // namespace cda_rail::solver::mip_based
