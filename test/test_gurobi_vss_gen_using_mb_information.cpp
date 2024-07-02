#include "VSSModel.hpp"
#include "solver/mip-based/VSSGenTimetableSolver.hpp"

#include "gtest/gtest.h"
#include <filesystem>
#include <iostream>
#include <string>

TEST(VSSGenMBInfoSolver, Default1) {
  cda_rail::solver::mip_based::VSSGenTimetableSolverWithMovingBlockInformation
      solver("./example-networks-mb-solutions/SimpleStation/");

  const auto sol = solver.solve();

  EXPECT_TRUE(sol.has_solution());
  EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(sol.get_obj(), 1);
  EXPECT_EQ(sol.get_mip_obj(), 1);
}
