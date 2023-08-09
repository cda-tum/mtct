#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "solver/mip-based/VSSGenTimetableSolver.hpp"

#include "gtest/gtest.h"
#include <filesystem>
#include <iostream>
#include <string>

TEST(Solver, ConsistencyExceptions) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  // Expect the following to throw exceptions of type
  // cda_rail::exceptions::ConsistencyException
  EXPECT_THROW(solver.solve(0, true, cda_rail::VSSModel::DISCRETE, {}),
               cda_rail::exceptions::ConsistencyException);
  EXPECT_THROW(solver.solve(0, true, cda_rail::VSSModel::DISCRETE,
                            {cda_rail::SeparationType::UNIFORM,
                             cda_rail::SeparationType::CHEBYCHEV}),
               cda_rail::exceptions::ConsistencyException);
  EXPECT_THROW(solver.solve(0, true, cda_rail::VSSModel::CONTINUOUS,
                            {cda_rail::SeparationType::UNIFORM}),
               cda_rail::exceptions::ConsistencyException);
  EXPECT_THROW(solver.solve(0, true, cda_rail::VSSModel::CONTINUOUS,
                            {cda_rail::SeparationType::UNIFORM,
                             cda_rail::SeparationType::CHEBYCHEV}),
               cda_rail::exceptions::ConsistencyException);
  EXPECT_THROW(solver.solve(0, true, cda_rail::VSSModel::LIMITED, {}),
               cda_rail::exceptions::ConsistencyException);
}

TEST(Solver, GurobiVSSGenDeltaT) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  std::cout << "--------------------- TEST 1 ---------------------------"
            << std::endl;
  const auto obj_val_1 = solver.solve(30, true, cda_rail::VSSModel::CONTINUOUS);
  std::cout << "--------------------- TEST 2 ---------------------------"
            << std::endl;
  const auto obj_val_2 =
      solver.solve(30, false, cda_rail::VSSModel::CONTINUOUS);
  std::cout << "--------------------- TEST 3 ---------------------------"
            << std::endl;
  const auto obj_val_3 =
      solver.solve(30, true, cda_rail::VSSModel::DISCRETE,
                   {cda_rail::SeparationType::UNIFORM}, false, false);

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
  const auto obj_val_default = solver.solve();
  EXPECT_EQ(obj_val_default, 1);
}

TEST(Solver, GurobiVSSGenModelDetailFixed) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  std::cout << "--------------------- TEST 1 ---------------------------"
            << std::endl;
  const auto obj_val_1 =
      solver.solve(15, true, cda_rail::VSSModel::CONTINUOUS, {}, true, true,
                   false, true, 60, true, true, "test_1");
  EXPECT_TRUE(std::filesystem::exists("test_1.mps"));
  EXPECT_TRUE(std::filesystem::exists("test_1.sol"));
  std::filesystem::remove("test_1.mps");
  std::filesystem::remove("test_1.sol");
  EXPECT_FALSE(std::filesystem::exists("test_1.mps"));
  EXPECT_FALSE(std::filesystem::exists("test_1.sol"));

  std::cout << "--------------------- TEST 2 ---------------------------"
            << std::endl;
  const auto obj_val_2 =
      solver.solve(15, true, cda_rail::VSSModel::CONTINUOUS, {}, true, true,
                   false, true, 60, true, false);

  std::cout << "--------------------- TEST 3 ---------------------------"
            << std::endl;
  const auto obj_val_3 =
      solver.solve(15, true, cda_rail::VSSModel::CONTINUOUS, {}, true, false,
                   false, false, 60, true, false);

  std::cout << "--------------------- TEST 4 ---------------------------"
            << std::endl;
  const auto obj_val_4 =
      solver.solve(15, true, cda_rail::VSSModel::CONTINUOUS, {}, true, true,
                   true, true, 60, true, false);

  std::cout << "--------------------- TEST 5 ---------------------------"
            << std::endl;
  const auto obj_val_5 =
      solver.solve(15, true, cda_rail::VSSModel::CONTINUOUS, {}, true, false,
                   false, true, 60, true, false);

  std::cout << "--------------------- TEST 6 ---------------------------"
            << std::endl;
  const auto obj_val_6 =
      solver.solve(15, true, cda_rail::VSSModel::CONTINUOUS, {}, false, false,
                   false, true, 60, true, false);

  // Check if all objective values are 1
  EXPECT_EQ(obj_val_1, 1);
  EXPECT_EQ(obj_val_2, 1);
  EXPECT_EQ(obj_val_3, 1);
  EXPECT_EQ(obj_val_4, 1);
  EXPECT_EQ(obj_val_5, 1);
  EXPECT_EQ(obj_val_6, 1);
}

TEST(Solver, GurobiVSSGenModelDetailFree1) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  std::cout << "--------------------- TEST 1 ---------------------------"
            << std::endl;
  const auto obj_val_1 =
      solver.solve(15, false, cda_rail::VSSModel::CONTINUOUS, {}, true, true,
                   false, true, 280, true, true, "test_1");
  EXPECT_TRUE(std::filesystem::exists("test_1.mps"));
  EXPECT_TRUE(std::filesystem::exists("test_1.sol"));
  std::filesystem::remove("test_1.mps");
  std::filesystem::remove("test_1.sol");
  EXPECT_FALSE(std::filesystem::exists("test_1.mps"));
  EXPECT_FALSE(std::filesystem::exists("test_1.sol"));

  EXPECT_EQ(obj_val_1, 1);
}

TEST(Solver, GurobiVSSGenModelDetailFree2) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  std::cout << "--------------------- TEST 2 ---------------------------"
            << std::endl;
  const auto obj_val_2 =
      solver.solve(15, false, cda_rail::VSSModel::CONTINUOUS, {}, true, true,
                   false, true, 280, true, false);

  EXPECT_EQ(obj_val_2, 1);
}

TEST(Solver, GurobiVSSGenModelDetailFree3) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  std::cout << "--------------------- TEST 3 ---------------------------"
            << std::endl;
  const auto obj_val_3 =
      solver.solve(15, false, cda_rail::VSSModel::CONTINUOUS, {}, true, true,
                   true, true, 280, true, false);

  EXPECT_EQ(obj_val_3, 1);
}

TEST(Solver, GurobiVSSGenModelDetailFree4) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  std::cout << "--------------------- TEST 4 ---------------------------"
            << std::endl;
  const auto obj_val_4 =
      solver.solve(15, false, cda_rail::VSSModel::CONTINUOUS, {}, true, false,
                   false, true, 280, true, false);

  EXPECT_EQ(obj_val_4, 1);
}

TEST(Solver, GurobiVSSGenModelDetailFree5) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  std::cout << "--------------------- TEST 5 ---------------------------"
            << std::endl;
  const auto obj_val_5 =
      solver.solve(15, false, cda_rail::VSSModel::CONTINUOUS, {}, false, false,
                   false, true, 280, true, false);

  EXPECT_EQ(obj_val_5, 1);
}

TEST(Solver, GurobiVSSGenVSSDiscrete) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  const auto obj_val = solver.solve(15, true, cda_rail::VSSModel::DISCRETE,
                                    {cda_rail::SeparationType::UNIFORM}, false,
                                    false, false, true, 375, true, false);

  // Check if all objective values are 1
  EXPECT_EQ(obj_val, 1);
}

TEST(Solver, OvertakeFixedContinuous) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/Overtake/");

  const auto obj_val_base =
      solver.solve(15, true, cda_rail::VSSModel::CONTINUOUS, {}, false, false,
                   false, true, 120, false, false);
  const auto obj_val_dynamics =
      solver.solve(15, true, cda_rail::VSSModel::CONTINUOUS, {}, true, false,
                   false, true, 120, false, false);
  const auto obj_val_braking =
      solver.solve(15, true, cda_rail::VSSModel::CONTINUOUS, {}, true, true,
                   false, true, 120, false, false);

  EXPECT_EQ(obj_val_base, 8);
  EXPECT_EQ(obj_val_dynamics, 8);
  EXPECT_EQ(obj_val_braking, 14);
}

TEST(Solver, OvertakeFreeContinuous) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/Overtake/");

  const auto obj_val_base =
      solver.solve(15, false, cda_rail::VSSModel::CONTINUOUS, {}, false, false,
                   false, true, 120, false, false);
  const auto obj_val_dynamics =
      solver.solve(15, false, cda_rail::VSSModel::CONTINUOUS, {}, true, false,
                   false, true, 120, false, false);
  const auto obj_val_braking =
      solver.solve(15, false, cda_rail::VSSModel::CONTINUOUS, {}, true, true,
                   false, true, 120, false, false);

  EXPECT_EQ(obj_val_base, 8);
  EXPECT_EQ(obj_val_dynamics, 8);
  EXPECT_EQ(obj_val_braking, 14);
}

TEST(Solver, Stammstrecke4FixedContinuous) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/Stammstrecke4Trains/");

  const auto obj_val_base =
      solver.solve(15, true, cda_rail::VSSModel::CONTINUOUS, {}, false, false,
                   false, true, 120, false, false);
  const auto obj_val_dynamics =
      solver.solve(15, true, cda_rail::VSSModel::CONTINUOUS, {}, true, false,
                   false, true, 120, false, false);
  const auto obj_val_braking =
      solver.solve(15, true, cda_rail::VSSModel::CONTINUOUS, {}, true, true,
                   false, true, 120, false, false);

  EXPECT_EQ(obj_val_base, 0);
  EXPECT_EQ(obj_val_dynamics, 6);
  EXPECT_EQ(obj_val_braking, 6);
}

TEST(Solver, Stammstrecke8FixedContinuous) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/Stammstrecke8Trains/");

  const auto obj_val_base =
      solver.solve(15, true, cda_rail::VSSModel::CONTINUOUS, {}, false, false,
                   false, true, 120, false, false);
  const auto obj_val_dynamics =
      solver.solve(15, true, cda_rail::VSSModel::CONTINUOUS, {}, true, false,
                   false, true, 120, false, false);
  const auto obj_val_braking =
      solver.solve(15, true, cda_rail::VSSModel::CONTINUOUS, {}, true, true,
                   false, true, 120, false, false);

  EXPECT_EQ(obj_val_base, 0);
  EXPECT_EQ(obj_val_dynamics, 14);
  EXPECT_EQ(obj_val_braking, 14);
}

TEST(Solver, Stammstrecke16FixedContinuous) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/Stammstrecke16Trains/");

  const auto obj_val_base =
      solver.solve(15, true, cda_rail::VSSModel::CONTINUOUS, {}, false, false,
                   false, true, 120, false, false);
  const auto obj_val_dynamics =
      solver.solve(15, true, cda_rail::VSSModel::CONTINUOUS, {}, true, false,
                   false, true, 120, false, false);
  const auto obj_val_braking =
      solver.solve(15, true, cda_rail::VSSModel::CONTINUOUS, {}, true, true,
                   false, true, 120, false, false);

  EXPECT_EQ(obj_val_base, 0);
  EXPECT_EQ(obj_val_dynamics, 15);
  EXPECT_EQ(obj_val_braking, 15);
}

TEST(Solver, SimpleStationLimitedUniform) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  const auto obj_val = solver.solve(15, true, cda_rail::VSSModel::LIMITED,
                                    {cda_rail::SeparationType::UNIFORM}, true,
                                    true, false, true, 60, true, false);

  // Check if all objective values are 1
  EXPECT_EQ(obj_val, 1);
}
