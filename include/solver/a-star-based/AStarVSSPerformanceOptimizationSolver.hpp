#pragma once
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"
#include "solver/GeneralSolver.hpp"

#include <filesystem>
#include <string>
#include <utility>

namespace cda_rail::solver::astar_based {
class AStarVSSPerformanceOptimizationSolver
    : public GeneralSolver<
          instances::GeneralPerformanceOptimizationInstance,
          instances::SolVSSGeneralPerformanceOptimizationInstance<
              instances::GeneralPerformanceOptimizationInstance>> {
private:
  // TODO: Implement

public:
  // Constructors. TODO: Implement
  explicit AStarVSSPerformanceOptimizationSolver(
      const instances::GeneralPerformanceOptimizationInstance& instance);
  explicit AStarVSSPerformanceOptimizationSolver(
      const std::filesystem::path& p);
  explicit AStarVSSPerformanceOptimizationSolver(const std::string& path);
  explicit AStarVSSPerformanceOptimizationSolver(const char* path);

  // TODO: Implement missing functions

  using GeneralSolver::solve;
  [[nodiscard]] instances::SolGeneralPerformanceOptimizationInstance<
      instances::GeneralPerformanceOptimizationInstance>
  solve(int time_limit, bool debug_input) override;
};
} // namespace cda_rail::solver::astar_based
