#include "Definitions.hpp"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"
#include "solver/mip-based/VSSGenTimetableSolver.hpp"

#include "gtest/gtest.h"
#include <filesystem>
#include <string_view>
#include <system_error>

namespace {
constexpr std::string_view INSTANCE_SUBDIRECTORY = "atmos2023";
constexpr std::string_view SOLUTION_SUBDIRECTORY = "moving-block-solutions";

cda_rail::instances::SolGeneralPerformanceOptimizationInstance
load_moving_block_solution(std::string_view const instanceName) {
  const auto instance =
      cda_rail::instances::GeneralPerformanceOptimizationInstance(
          instanceName, INSTANCE_SUBDIRECTORY, "data");
  auto sol_obj =
      cda_rail::instances::SolGeneralPerformanceOptimizationInstance(instance);
  sol_obj.load_solution("data", SOLUTION_SUBDIRECTORY);
  return sol_obj;
}
} // namespace

TEST(VSSGenMBInfoSolver, Default1) {
  cda_rail::solver::mip_based::VSSGenTimetableSolverWithMovingBlockInformation
      solver(load_moving_block_solution("SimpleStation"));

  const auto sol = solver.solve();

  EXPECT_TRUE(sol.has_solution());
  EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(sol.get_obj(), 1);
}

TEST(VSSGenMBInfoSolver, Default2) {
  cda_rail::solver::mip_based::VSSGenTimetableSolverWithMovingBlockInformation
      solver(load_moving_block_solution("HighSpeedTrack2Trains"));

  const auto sol = solver.solve();

  EXPECT_TRUE(sol.has_solution());
  EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(sol.get_obj(), 18);
}

TEST(VSSGenMBInfoSolver, Default3) {
  cda_rail::solver::mip_based::VSSGenTimetableSolverWithMovingBlockInformation
      solver(load_moving_block_solution("HighSpeedTrack5Trains"));

  const auto sol = solver.solve({15, true, false});

  EXPECT_TRUE(sol.has_solution());
  EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(sol.get_obj(), 10);
}

TEST(VSSGenMBInfoSolver, Default4) {
  cda_rail::solver::mip_based::VSSGenTimetableSolverWithMovingBlockInformation
      solver(load_moving_block_solution("Overtake"));

  const auto sol = solver.solve();

  EXPECT_TRUE(sol.has_solution());
  EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(sol.get_obj(), 7);
}

TEST(VSSGenMBInfoSolver, Default5) {
  cda_rail::solver::mip_based::VSSGenTimetableSolverWithMovingBlockInformation
      solver(load_moving_block_solution("SimpleNetwork"));

  const auto sol = solver.solve();

  EXPECT_TRUE(sol.has_solution());
  EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(sol.get_obj(), 7);
}

TEST(VSSGenMBInfoSolver, Default5TimeoutExport) {
  // Both the instance and the moving block solution are read relative to the
  // current working directory, hence the solver has to be created before
  // switching to the temporary directory.
  cda_rail::solver::mip_based::VSSGenTimetableSolverWithMovingBlockInformation
      solver(load_moving_block_solution("SimpleNetwork"));

  // All exports happen relative to the current working directory. Hence, the
  // test is executed within a temporary directory that is removed afterwards,
  // even if an expectation fails in between.
  struct ScopedTempWorkingDirectory {
    std::filesystem::path original_directory;
    std::filesystem::path temporary_directory;
    ~ScopedTempWorkingDirectory() {
      std::error_code ignored;
      std::filesystem::current_path(original_directory, ignored);
      std::filesystem::remove_all(temporary_directory, ignored);
    }
  };

  const std::filesystem::path temp_dir =
      std::filesystem::temp_directory_path() /
      "cda_rail_test_vss_gen_mb_info_simple_network_export";
  std::filesystem::remove_all(temp_dir);
  ASSERT_TRUE(std::filesystem::create_directories(temp_dir));
  const ScopedTempWorkingDirectory temp_working_directory{
      std::filesystem::current_path(), temp_dir};
  std::filesystem::current_path(temp_dir);

  const auto sol = solver.solve(
      {5}, {}, {}, {false, cda_rail::ExportOption::ExportSolution}, 10);

  EXPECT_FALSE(sol.has_solution());
  EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Timeout);
  EXPECT_EQ(sol.get_obj(), -1);

  // Without an explicit path and name the solution is exported to
  // ./solutions/model/<instance_subdirectory>/<instance_name>
  const std::filesystem::path solution_dir =
      std::filesystem::path("solutions") / "model" / INSTANCE_SUBDIRECTORY /
      "SimpleNetwork";
  EXPECT_TRUE(std::filesystem::is_directory(solution_dir));
  std::error_code ec;
  for (const auto& file_name :
       {"solution_data.json", "routes.json", "train_pos.json",
        "train_speed.json", "train_exit_times.json", "train_stop_times.json",
        "vss_pos.json"}) {
    const auto file_path = solution_dir / file_name;
    EXPECT_TRUE(std::filesystem::exists(file_path))
        << "Missing file " << file_path;
    EXPECT_GT(std::filesystem::file_size(file_path, ec), 0)
        << "Empty file " << file_path;
  }

  // Neither the instance nor the model itself are expected to be exported
  EXPECT_FALSE(std::filesystem::exists("instances"));
  EXPECT_FALSE(std::filesystem::exists("networks"));
  EXPECT_FALSE(std::filesystem::exists("model.mps"));
  EXPECT_FALSE(std::filesystem::exists("model.sol"));
}

TEST(VSSGenMBInfoSolver, Default6) {
  cda_rail::solver::mip_based::VSSGenTimetableSolverWithMovingBlockInformation
      solver(load_moving_block_solution("SingleTrack"));

  const auto sol = solver.solve();

  EXPECT_TRUE(sol.has_solution());
  EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(sol.get_obj(), 10);
}

TEST(VSSGenMBInfoSolver, Default7) {
  cda_rail::solver::mip_based::VSSGenTimetableSolverWithMovingBlockInformation
      solver(load_moving_block_solution("SingleTrackWithStation"));

  const auto sol = solver.solve();

  EXPECT_TRUE(sol.has_solution());
  EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(sol.get_obj(), 5);
}

TEST(VSSGenMBInfoSolver, Default8) {
  cda_rail::solver::mip_based::VSSGenTimetableSolverWithMovingBlockInformation
      solver(load_moving_block_solution("Stammstrecke4Trains"));

  const auto sol = solver.solve({5});

  EXPECT_TRUE(sol.has_solution());
  EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(sol.get_obj(), 7);
}

TEST(VSSGenMBInfoSolver, Default9) {
  cda_rail::solver::mip_based::VSSGenTimetableSolverWithMovingBlockInformation
      solver(load_moving_block_solution("Stammstrecke8Trains"));

  const auto sol = solver.solve({5});

  EXPECT_TRUE(sol.has_solution());
  EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(sol.get_obj(), 14);
}
