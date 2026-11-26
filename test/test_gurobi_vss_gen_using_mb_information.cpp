#include "Definitions.hpp"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"
#include "solver/GeneralSolver.hpp"
#include "solver/astar-based/GenPOMovingBlockAStarSolver.hpp"
#include "solver/mip-based/GeneralMIPSolver.hpp"
#include "solver/mip-based/VSSGenTimetableSolver.hpp"

#include "gtest/gtest.h"
#include <filesystem>
#include <string_view>
#include <system_error>

namespace {
constexpr std::string_view INSTANCE_SUBDIRECTORY = "atmos2023";
constexpr std::string_view SOLUTION_SUBDIRECTORY = "moving-block-solutions";
constexpr std::string_view SOLUTION_SUBDIRECTORY_NO_TOLERANCE =
    "moving-block-solutions-no-tolerance";

cda_rail::instances::SolGeneralPerformanceOptimizationInstance
load_moving_block_solution(std::string_view const instanceName,
                           std::string_view const solutionSubdirectory) {
  const auto instance =
      cda_rail::instances::GeneralPerformanceOptimizationInstance(
          instanceName, INSTANCE_SUBDIRECTORY, "data");
  auto sol_obj =
      cda_rail::instances::SolGeneralPerformanceOptimizationInstance(instance);
  sol_obj.load_solution("data", solutionSubdirectory);
  return sol_obj;
}
} // namespace

TEST(VSSGenMBInfoSolver, Default1) {
  cda_rail::solver::mip_based::VSSGenTimetableSolverWithMovingBlockInformation
      solver(
          load_moving_block_solution("SimpleStation", SOLUTION_SUBDIRECTORY));

  const auto sol = solver.solve();

  EXPECT_TRUE(sol.has_solution());
  EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(sol.get_obj(), 1);
}

TEST(VSSGenMBInfoSolver, Default1ConvexifiedBrakingDistance) {
  cda_rail::solver::mip_based::VSSGenTimetableSolverWithMovingBlockInformation
      solver(
          load_moving_block_solution("SimpleStation", SOLUTION_SUBDIRECTORY));

  const auto sol = solver.solve({}, {.convexify_braking_distance = true});

  EXPECT_TRUE(sol.has_solution());
  EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(sol.get_obj(), 1);
}

TEST(VSSGenMBInfoSolver, Default1NoTolerance) {
  cda_rail::solver::mip_based::VSSGenTimetableSolverWithMovingBlockInformation
      solver(load_moving_block_solution("SimpleStation",
                                        SOLUTION_SUBDIRECTORY_NO_TOLERANCE));

  const auto sol = solver.solve();

  EXPECT_TRUE(sol.has_solution());
  EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(sol.get_obj(), 1);
}

TEST(VSSGenMBInfoSolver, Default1AStarSolution) {
  // A train that brakes into the time step containing its exit time must not
  // leave before it, otherwise the VSS model asks for a train that has already
  // left
  const cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      "SimpleStation", INSTANCE_SUBDIRECTORY, "data");
  cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver astar_solver(
      instance);
  const auto mb_sol = astar_solver.solve({.dt = 6}, {}, {}, -1, false);
  ASSERT_EQ(mb_sol.get_status(), cda_rail::SolutionStatus::Optimal);

  cda_rail::solver::mip_based::VSSGenTimetableSolverWithMovingBlockInformation
      solver(mb_sol);

  const auto sol = solver.solve();

  EXPECT_TRUE(sol.has_solution());
  EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(sol.get_obj(), 1);
}

TEST(VSSGenMBInfoSolver, Default2) {
  cda_rail::solver::mip_based::VSSGenTimetableSolverWithMovingBlockInformation
      solver(load_moving_block_solution("HighSpeedTrack2Trains",
                                        SOLUTION_SUBDIRECTORY));

  const auto sol = solver.solve();

  EXPECT_TRUE(sol.has_solution());
  EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(sol.get_obj(), 18);
}

TEST(VSSGenMBInfoSolver, Default2NoTolerance) {
  cda_rail::solver::mip_based::VSSGenTimetableSolverWithMovingBlockInformation
      solver(load_moving_block_solution("HighSpeedTrack2Trains",
                                        SOLUTION_SUBDIRECTORY_NO_TOLERANCE));

  const auto sol = solver.solve();

  EXPECT_TRUE(sol.has_solution());
  EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(sol.get_obj(), 18);
}

TEST(VSSGenMBInfoSolver, Default3) {
  cda_rail::solver::mip_based::VSSGenTimetableSolverWithMovingBlockInformation
      solver(load_moving_block_solution("HighSpeedTrack5Trains",
                                        SOLUTION_SUBDIRECTORY));

  const auto sol = solver.solve(
      {.delta_t = 15, .train_dynamics = true, .braking_curves = false});

  EXPECT_TRUE(sol.has_solution());
  EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(sol.get_obj(), 10);
}

TEST(VSSGenMBInfoSolver, Default3NoTolerance) {
  cda_rail::solver::mip_based::VSSGenTimetableSolverWithMovingBlockInformation
      solver(load_moving_block_solution("HighSpeedTrack5Trains",
                                        SOLUTION_SUBDIRECTORY_NO_TOLERANCE));

  const auto sol = solver.solve(
      {.delta_t = 15, .train_dynamics = true, .braking_curves = false});

  EXPECT_TRUE(sol.has_solution());
  EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(sol.get_obj(), 10);
}

TEST(VSSGenMBInfoSolver, Default4) {
  cda_rail::solver::mip_based::VSSGenTimetableSolverWithMovingBlockInformation
      solver(load_moving_block_solution("Overtake", SOLUTION_SUBDIRECTORY));

  const auto sol = solver.solve();

  EXPECT_TRUE(sol.has_solution());
  EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(sol.get_obj(), 7);
}

TEST(VSSGenMBInfoSolver, Default4NoTolerance) {
  cda_rail::solver::mip_based::VSSGenTimetableSolverWithMovingBlockInformation
      solver(load_moving_block_solution("Overtake",
                                        SOLUTION_SUBDIRECTORY_NO_TOLERANCE));

  const auto sol = solver.solve();

  EXPECT_TRUE(sol.has_solution());
  EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(sol.get_obj(), 7);
}

TEST(VSSGenMBInfoSolver, Default5) {
  cda_rail::solver::mip_based::VSSGenTimetableSolverWithMovingBlockInformation
      solver(load_moving_block_solution("SingleTrackCorridor",
                                        SOLUTION_SUBDIRECTORY));

  const auto sol = solver.solve();

  EXPECT_TRUE(sol.has_solution());
  EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(sol.get_obj(), 7);
}

TEST(VSSGenMBInfoSolver, Default5NoTolerance) {
  cda_rail::solver::mip_based::VSSGenTimetableSolverWithMovingBlockInformation
      solver(load_moving_block_solution("SingleTrackCorridor",
                                        SOLUTION_SUBDIRECTORY_NO_TOLERANCE));

  const auto sol = solver.solve();

  EXPECT_TRUE(sol.has_solution());
  EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(sol.get_obj(), 19);
}

TEST(VSSGenMBInfoSolver, Default5TimeoutExport) {
  // Both the instance and the moving block solution are read relative to the
  // current working directory, hence the solver has to be created before
  // switching to the temporary directory.
  cda_rail::solver::mip_based::VSSGenTimetableSolverWithMovingBlockInformation
      solver(load_moving_block_solution("SingleTrackCorridor",
                                        SOLUTION_SUBDIRECTORY));

  // All exports happen relative to the current working directory. Hence, the
  // test is executed within a temporary directory that is removed afterwards,
  // even if an expectation fails in between.
  struct ScopedTempWorkingDirectory {
    std::filesystem::path original_directory;
    std::filesystem::path temporary_directory;
    ~ScopedTempWorkingDirectory() {
      std::error_code ignored;
      std::filesystem::current_path(original_directory, ignored);
      try {
        std::filesystem::remove_all(temporary_directory, ignored);
      } catch (...) {
        // Not being able to clean up must not end the test run
        ignored = std::make_error_code(std::errc::not_enough_memory);
      }
    }
  };

  const std::filesystem::path temp_dir =
      std::filesystem::temp_directory_path() /
      "cda_rail_test_vss_gen_mb_info_single_track_corridor_export";
  std::filesystem::remove_all(temp_dir);
  ASSERT_TRUE(std::filesystem::create_directories(temp_dir));
  const ScopedTempWorkingDirectory temp_working_directory{
      .original_directory  = std::filesystem::current_path(),
      .temporary_directory = temp_dir,
  };
  std::filesystem::current_path(temp_dir);

  cda_rail::solver::mip_based::SolutionSettingsVSSGen solution_settings;
  solution_settings.export_option =
      cda_rail::solver::GeneralExportOption::ExportSolution;
  solution_settings.parameter_identifier = "tmpid";

  const auto sol = solver.solve({.delta_t = 5}, {}, {}, solution_settings, 10);

  EXPECT_FALSE(sol.has_solution());
  EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Timeout);
  EXPECT_EQ(sol.get_obj(), -1);

  // Without an explicit working directory and solution subdirectory the
  // solution is exported to ./solutions/unnamed-experiment/
  // <instance_subdirectory>/<instance_name>-<parameter_identifier>
  const std::filesystem::path solution_dir =
      std::filesystem::path("solutions") / "unnamed-experiment" /
      INSTANCE_SUBDIRECTORY / "SingleTrackCorridor-tmpid";
  EXPECT_TRUE(std::filesystem::is_directory(solution_dir));
  std::error_code ec;
  for (const auto& file_name : {
           "solution_data.json",
           "routes.json",
           "train_pos.json",
           "train_speed.json",
           "train_exit_times.json",
           "train_stop_times.json",
           "vss_pos.json",
           "solver_data.json",
       }) {
    const auto file_path = solution_dir / file_name;
    EXPECT_TRUE(std::filesystem::exists(file_path))
        << "Missing file " << file_path;
    const auto file_bytes = std::filesystem::file_size(file_path, ec);
    EXPECT_FALSE(ec) << "Unreadable file " << file_path;
    EXPECT_GT(file_bytes, 0) << "Empty file " << file_path;
  }

  // Neither the instance nor the model itself are expected to be exported
  EXPECT_FALSE(std::filesystem::exists("instances"));
  EXPECT_FALSE(std::filesystem::exists("networks"));
  EXPECT_FALSE(std::filesystem::exists(solution_dir / "model.mps"));
  EXPECT_FALSE(std::filesystem::exists(solution_dir / "model.sol"));
}

TEST(VSSGenMBInfoSolver, Default6) {
  cda_rail::solver::mip_based::VSSGenTimetableSolverWithMovingBlockInformation
      solver(load_moving_block_solution("SingleTrack", SOLUTION_SUBDIRECTORY));

  const auto sol = solver.solve();

  EXPECT_TRUE(sol.has_solution());
  EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(sol.get_obj(), 10);
}

TEST(VSSGenMBInfoSolver, Default6NoTolerance) {
  cda_rail::solver::mip_based::VSSGenTimetableSolverWithMovingBlockInformation
      solver(load_moving_block_solution("SingleTrack",
                                        SOLUTION_SUBDIRECTORY_NO_TOLERANCE));

  const auto sol = solver.solve();

  EXPECT_TRUE(sol.has_solution());
  EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(sol.get_obj(), 10);
}

TEST(VSSGenMBInfoSolver, Default7) {
  cda_rail::solver::mip_based::VSSGenTimetableSolverWithMovingBlockInformation
      solver(load_moving_block_solution("SingleTrackWithStation",
                                        SOLUTION_SUBDIRECTORY));

  const auto sol = solver.solve();

  EXPECT_TRUE(sol.has_solution());
  EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(sol.get_obj(), 5);
}

TEST(VSSGenMBInfoSolver, Default7NoTolerance) {
  cda_rail::solver::mip_based::VSSGenTimetableSolverWithMovingBlockInformation
      solver(load_moving_block_solution("SingleTrackWithStation",
                                        SOLUTION_SUBDIRECTORY_NO_TOLERANCE));

  const auto sol = solver.solve();

  EXPECT_TRUE(sol.has_solution());
  EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(sol.get_obj(), 5);
}

TEST(VSSGenMBInfoSolver, Default8) {
  cda_rail::solver::mip_based::VSSGenTimetableSolverWithMovingBlockInformation
      solver(load_moving_block_solution("Stammstrecke4Trains",
                                        SOLUTION_SUBDIRECTORY));

  const auto sol = solver.solve({.delta_t = 5});

  EXPECT_TRUE(sol.has_solution());
  EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(sol.get_obj(), 7);
}

TEST(VSSGenMBInfoSolver, Default8NoTolerance) {
  cda_rail::solver::mip_based::VSSGenTimetableSolverWithMovingBlockInformation
      solver(load_moving_block_solution("Stammstrecke4Trains",
                                        SOLUTION_SUBDIRECTORY_NO_TOLERANCE));

  const auto sol = solver.solve({.delta_t = 5});

  EXPECT_TRUE(sol.has_solution());
  EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(sol.get_obj(), 6);
}

TEST(VSSGenMBInfoSolver, Default9) {
  cda_rail::solver::mip_based::VSSGenTimetableSolverWithMovingBlockInformation
      solver(load_moving_block_solution("Stammstrecke8Trains",
                                        SOLUTION_SUBDIRECTORY));

  const auto sol = solver.solve({.delta_t = 5});

  EXPECT_TRUE(sol.has_solution());
  EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(sol.get_obj(), 14);
}

TEST(VSSGenMBInfoSolver, Default9NoTolerance) {
  cda_rail::solver::mip_based::VSSGenTimetableSolverWithMovingBlockInformation
      solver(load_moving_block_solution("Stammstrecke8Trains",
                                        SOLUTION_SUBDIRECTORY_NO_TOLERANCE));

  const auto sol = solver.solve({.delta_t = 5});

  EXPECT_TRUE(sol.has_solution());
  EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(sol.get_obj(), 14);
}
