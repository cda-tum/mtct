#include "Definitions.hpp"
#include "solver/mip-based/VSSGenTimetableSolver.hpp"

#include "gtest/gtest.h"
#include <filesystem>
#include <iostream>
#include <string>

TEST(Solver, GurobiVSSGenDeltaT) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  std::cout << "--------------------- TEST 1 ---------------------------"
            << std::endl;
  auto obj_val_1 = solver.solve(30, true, cda_rail::VSSModel::CONTINUOUS);
  std::cout << "--------------------- TEST 2 ---------------------------"
            << std::endl;
  auto obj_val_2 = solver.solve(30, false, cda_rail::VSSModel::CONTINUOUS);
  std::cout << "--------------------- TEST 3 ---------------------------"
            << std::endl;
  auto obj_val_3 =
      solver.solve(30, true, cda_rail::VSSModel::DISCRETE, false, false);

  EXPECT_EQ(obj_val_1, 1);
  EXPECT_EQ(obj_val_2, 1);
  EXPECT_EQ(obj_val_3, 1);
}

TEST(Solver, GurobiVSSGenDefault) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  // Test various options
  std::cout << "--------------------- DEFAULT ---------------------------"
            << std::endl;
  auto obj_val_default = solver.solve();
  EXPECT_EQ(obj_val_default, 1);
}

TEST(Solver, GurobiVSSGenModelDetailFixed) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  std::cout << "--------------------- TEST 1 ---------------------------"
            << std::endl;
  auto obj_val_1 = solver.solve(15, true, cda_rail::VSSModel::CONTINUOUS, true,
                                true, false, true, 60, true, true, "test_1");
  EXPECT_TRUE(std::filesystem::exists("test_1.mps"));
  EXPECT_TRUE(std::filesystem::exists("test_1.sol"));
  std::filesystem::remove("test_1.mps");
  std::filesystem::remove("test_1.sol");
  EXPECT_FALSE(std::filesystem::exists("test_1.mps"));
  EXPECT_FALSE(std::filesystem::exists("test_1.sol"));

  std::cout << "--------------------- TEST 2 ---------------------------"
            << std::endl;
  auto obj_val_2 = solver.solve(15, true, cda_rail::VSSModel::CONTINUOUS, true,
                                true, false, true, 60, true, false);

  std::cout << "--------------------- TEST 3 ---------------------------"
            << std::endl;
  auto obj_val_3 = solver.solve(15, true, cda_rail::VSSModel::CONTINUOUS, true,
                                true, false, false, 60, true, false);

  std::cout << "--------------------- TEST 4 ---------------------------"
            << std::endl;
  auto obj_val_4 = solver.solve(15, true, cda_rail::VSSModel::CONTINUOUS, true,
                                true, true, true, 60, true, false);

  std::cout << "--------------------- TEST 5 ---------------------------"
            << std::endl;
  auto obj_val_5 = solver.solve(15, true, cda_rail::VSSModel::CONTINUOUS, true,
                                false, false, true, 60, true, false);

  std::cout << "--------------------- TEST 6 ---------------------------"
            << std::endl;
  auto obj_val_6 = solver.solve(15, true, cda_rail::VSSModel::CONTINUOUS, false,
                                false, false, true, 60, true, false);

  // Check if all objective values are 1
  EXPECT_EQ(obj_val_1, 1);
  EXPECT_EQ(obj_val_2, 1);
  EXPECT_EQ(obj_val_3, 1);
  EXPECT_EQ(obj_val_4, 1);
  EXPECT_EQ(obj_val_5, 1);
  EXPECT_EQ(obj_val_6, 1);
}

TEST(Solver, GurobiVSSGenModelDetailFree) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  std::cout << "--------------------- TEST 1 ---------------------------"
            << std::endl;
  auto obj_val_1 = solver.solve(15, false, cda_rail::VSSModel::CONTINUOUS, true,
                                true, false, true, 180, true, true, "test_1");
  EXPECT_TRUE(std::filesystem::exists("test_1.mps"));
  EXPECT_TRUE(std::filesystem::exists("test_1.sol"));
  std::filesystem::remove("test_1.mps");
  std::filesystem::remove("test_1.sol");
  EXPECT_FALSE(std::filesystem::exists("test_1.mps"));
  EXPECT_FALSE(std::filesystem::exists("test_1.sol"));

  std::cout << "--------------------- TEST 2 ---------------------------"
            << std::endl;
  auto obj_val_2 = solver.solve(15, false, cda_rail::VSSModel::CONTINUOUS, true,
                                true, false, true, 180, true, false);

  std::cout << "--------------------- TEST 3 ---------------------------"
            << std::endl;
  auto obj_val_3 = solver.solve(15, false, cda_rail::VSSModel::CONTINUOUS, true,
                                true, false, false, 180, true, false);

  std::cout << "--------------------- TEST 4 ---------------------------"
            << std::endl;
  auto obj_val_4 = solver.solve(15, false, cda_rail::VSSModel::CONTINUOUS, true,
                                true, true, true, 180, true, false);

  std::cout << "--------------------- TEST 5 ---------------------------"
            << std::endl;
  auto obj_val_5 = solver.solve(15, false, cda_rail::VSSModel::CONTINUOUS, true,
                                false, false, true, 180, true, false);

  std::cout << "--------------------- TEST 6 ---------------------------"
            << std::endl;
  auto obj_val_6 = solver.solve(15, false, cda_rail::VSSModel::CONTINUOUS,
                                false, false, false, true, 180, true, false);

  // Check if all objective values are 1
  EXPECT_EQ(obj_val_1, 1);
  EXPECT_EQ(obj_val_2, 1);
  EXPECT_EQ(obj_val_3, 1);
  EXPECT_EQ(obj_val_4, 1);
  EXPECT_EQ(obj_val_5, 1);
  EXPECT_EQ(obj_val_6, 1);
}

TEST(Solver, GurobiVSSGenVSSDiscrete) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  auto obj_val = solver.solve(15, true, cda_rail::VSSModel::DISCRETE, false,
                              false, false, true, 240, true, false);

  // Check if all objective values are 1
  EXPECT_EQ(obj_val, 1);
}
