#define TEST_FRIENDS true

#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"
#include "solver/mip-based/GenPOMovingBlockMIPSolver.hpp"

#include "gtest/gtest.h"
#include <filesystem>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

// NOLINTBEGIN (clang-analyzer-deadcode.DeadStores)

TEST(GenPOMovingBlockMIPSolver, PrivateFillFunctions) {
  // TODO

  cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver solver;
  solver.max_t = 1;
  EXPECT_EQ(solver.max_t, 1);
}

// NOLINTEND (clang-analyzer-deadcode.DeadStores)
