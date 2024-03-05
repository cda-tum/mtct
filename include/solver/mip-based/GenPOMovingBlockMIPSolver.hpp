#include "datastructure/GeneralTimetable.hpp"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"
#include "solver/mip-based/GeneralMIPSolver.hpp"

#include <filesystem>
#include <string>

namespace cda_rail::solver::mip_based {
class GenPOMovingBlockMIPSolver
    : public GeneralMIPSolver<
          instances::GeneralPerformanceOptimizationInstance,
          instances::SolGeneralPerformanceOptimizationInstance> {
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
  solve(int time_limit, bool debug_input) override;
};
} // namespace cda_rail::solver::mip_based
