#include "solver/mip-based/VSSGenTimetableSolver.hpp"
#include <string>

#include "gtest/gtest.h"

TEST(Solver, GurobiVSSGenFixedRoute) {
    cda_rail::solver::mip_based::VSSGenTimetableSolver solver("./example-networks/SimpleStation/");

   auto obj_val = solver.solve();
   EXPECT_EQ(obj_val, 1);
}