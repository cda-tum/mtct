#include "solver/mip-based/VSSGenTimetableSolver.hpp"

#include "gtest/gtest.h"
#include <string>

TEST(Solver, GurobiVSSGenDeltaTDefault) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  std::cout << "--------------------- TEST 1 ---------------------------"
            << std::endl;
  auto obj_val_6 = solver.solve(6);
  std::cout << "--------------------- TEST 2 ---------------------------"
            << std::endl;
  auto obj_val_15 = solver.solve(15);
  std::cout << "--------------------- TEST 3 ---------------------------"
            << std::endl;
  auto obj_val_11 = solver.solve(11);
  std::cout << "--------------------- TEST 4 ---------------------------"
            << std::endl;
  auto obj_val_18 = solver.solve(18);
  std::cout << "--------------------- TEST 5 ---------------------------"
            << std::endl;
  auto obj_val_30 = solver.solve(30);

  EXPECT_EQ(obj_val_6, 1);
  EXPECT_EQ(obj_val_15, 1);
  EXPECT_EQ(obj_val_11, 1);
  EXPECT_EQ(obj_val_18, 1);
  EXPECT_EQ(obj_val_30, 1);
}

TEST(Solver, GurobiVSSGenFixedRoute) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  auto obj_val = solver.solve();
  EXPECT_EQ(obj_val, 1);
}
