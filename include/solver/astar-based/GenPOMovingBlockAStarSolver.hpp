#pragma once

#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"
#include "solver/GeneralSolver.hpp"

namespace cda_rail::solver::astar_based {
class GenPOMovingBlockAStarSolver
    : public GeneralSolver<
          instances::GeneralPerformanceOptimizationInstance,
          instances::SolGeneralPerformanceOptimizationInstance<
              instances::GeneralPerformanceOptimizationInstance>> {};
} // namespace cda_rail::solver::astar_based
