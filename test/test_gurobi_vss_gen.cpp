#include "solver/mip-based/VSSGenTimetableSolver.hpp"
#include <string>

#include "gtest/gtest.h"

TEST(Solver, GurobiVSSGenFixedRoute) {
    cda_rail::solver::mip_based::VSSGenTimetableSolver solver("./example-networks/Fig11/");

    solver.solve(15, true, false, true, false);
}