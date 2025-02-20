#include "Definitions.hpp"
#include "solver/mip-based/VSSGenTimetableSolver.hpp"

#include "gtest/gtest.h"
#include <filesystem>

TEST(VSSGenMBInfoSolver, Default1) {
  cda_rail::solver::mip_based::VSSGenTimetableSolverWithMovingBlockInformation
      solver("./example-networks-mb-solutions/SimpleStation/");

  const auto sol = solver.solve();

  EXPECT_TRUE(sol.has_solution());
  EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(sol.get_obj(), 1);
  EXPECT_EQ(sol.get_mip_obj(), 1);
}

TEST(VSSGenMBInfoSolver, Default2) {
  cda_rail::solver::mip_based::VSSGenTimetableSolverWithMovingBlockInformation
      solver("./example-networks-mb-solutions/HighSpeedTrack2Trains/");

  const auto sol = solver.solve();

  EXPECT_TRUE(sol.has_solution());
  EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(sol.get_obj(), 18);
  EXPECT_EQ(sol.get_mip_obj(), 18);
}

TEST(VSSGenMBInfoSolver, Default3) {
  cda_rail::solver::mip_based::VSSGenTimetableSolverWithMovingBlockInformation
      solver("./example-networks-mb-solutions/HighSpeedTrack5Trains/");

  const auto sol = solver.solve({15, true, false});

  EXPECT_TRUE(sol.has_solution());
  EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(sol.get_obj(), 10);
  EXPECT_EQ(sol.get_mip_obj(), 10);
}

TEST(VSSGenMBInfoSolver, Default4) {
  cda_rail::solver::mip_based::VSSGenTimetableSolverWithMovingBlockInformation
      solver("./example-networks-mb-solutions/Overtake/");

  const auto sol = solver.solve();

  EXPECT_TRUE(sol.has_solution());
  EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(sol.get_obj(), 14);
  EXPECT_EQ(sol.get_mip_obj(), 14);
}

TEST(VSSGenMBInfoSolver, Default5) {
  cda_rail::solver::mip_based::VSSGenTimetableSolverWithMovingBlockInformation
      solver("./example-networks-mb-solutions/SimpleNetwork/");

  const auto sol = solver.solve();

  EXPECT_TRUE(sol.has_solution());
  EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(sol.get_obj(), 13);
  EXPECT_EQ(sol.get_mip_obj(), 13);
}

TEST(VSSGenMBInfoSolver, Default5TimeoutExport) {
  cda_rail::solver::mip_based::VSSGenTimetableSolverWithMovingBlockInformation
      solver("./example-networks-mb-solutions/SimpleNetwork/");

  const auto sol = solver.solve(
      {5}, {}, {}, {false, cda_rail::ExportOption::ExportSolution}, 10);

  EXPECT_FALSE(sol.has_solution());
  EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Timeout);
  EXPECT_EQ(sol.get_obj(), -1);
  EXPECT_EQ(sol.get_mip_obj(), -1);

  // Expect folder model to exist and to contain folders solution and instance
  EXPECT_TRUE(std::filesystem::exists("model"));
  EXPECT_TRUE(std::filesystem::exists("model/solution"));
  EXPECT_TRUE(std::filesystem::exists("model/instance"));
  // Within solution there should be data.json
  EXPECT_TRUE(std::filesystem::exists("model/solution/data.json"));

  // Remove model folder
  std::filesystem::remove_all("model");
}

TEST(VSSGenMBInfoSolver, Default6) {
  cda_rail::solver::mip_based::VSSGenTimetableSolverWithMovingBlockInformation
      solver("./example-networks-mb-solutions/SingleTrack/");

  const auto sol = solver.solve();

  EXPECT_TRUE(sol.has_solution());
  EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(sol.get_obj(), 9);
  EXPECT_EQ(sol.get_mip_obj(), 9);
}

TEST(VSSGenMBInfoSolver, Default7) {
  cda_rail::solver::mip_based::VSSGenTimetableSolverWithMovingBlockInformation
      solver("./example-networks-mb-solutions/SingleTrackWithStation/");

  const auto sol = solver.solve();

  EXPECT_TRUE(sol.has_solution());
  EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(sol.get_obj(), 5);
  EXPECT_EQ(sol.get_mip_obj(), 5);
}

TEST(VSSGenMBInfoSolver, Default8) {
  cda_rail::solver::mip_based::VSSGenTimetableSolverWithMovingBlockInformation
      solver("./example-networks-mb-solutions/Stammstrecke4Trains/");

  const auto sol = solver.solve({5});

  EXPECT_TRUE(sol.has_solution());
  EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(sol.get_obj(), 6);
  EXPECT_EQ(sol.get_mip_obj(), 6);
}

TEST(VSSGenMBInfoSolver, Default9) {
  cda_rail::solver::mip_based::VSSGenTimetableSolverWithMovingBlockInformation
      solver("./example-networks-mb-solutions/Stammstrecke8Trains/");

  const auto sol = solver.solve({5});

  EXPECT_TRUE(sol.has_solution());
  EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(sol.get_obj(), 15);
  EXPECT_EQ(sol.get_mip_obj(), 15);
}
