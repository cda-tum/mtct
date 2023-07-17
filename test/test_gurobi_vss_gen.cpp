/**
#include "solver/mip-based/VSSGenTimetableSolver.hpp"

#include "gtest/gtest.h"
#include <string>

TEST(Solver, GurobiVSSGenFixedRoute) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  auto obj_val =
      solver.solve(15, true, false, true, true, false, true, -1, true);
  EXPECT_EQ(obj_val, 1);
}
**/
