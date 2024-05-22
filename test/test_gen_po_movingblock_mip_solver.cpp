#include <cstdlib>
#define TEST_FRIENDS true

#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"
#include "probleminstances/VSSGenerationTimetable.hpp"
#include "solver/mip-based/GenPOMovingBlockMIPSolver.hpp"

#include "gtest/gtest.h"
#include <filesystem>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#define EXPECT_APPROX_EQ(a, b)                                                 \
  EXPECT_TRUE(std::abs((a) - (b)) < 1e-2) << (a) << " !=(approx.) " << (b);

// NOLINTBEGIN (clang-analyzer-deadcode.DeadStores)

TEST(GenPOMovingBlockMIPSolver, All1b) {
  const std::vector<std::string> paths{"Overtake"};

  for (const auto& p : paths) {
    const std::string instance_path = "./example-networks/" + p + "/";
    const auto        instance_before_parse =
        cda_rail::instances::VSSGenerationTimetable(instance_path);
    const auto instance =
        cda_rail::instances::GeneralPerformanceOptimizationInstance::
            cast_from_vss_generation(instance_before_parse);
    cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver solver(instance);
    const auto                                             sol = solver.solve(
        {},
        {true, true, false,
                                                     cda_rail::solver::mip_based::LazyConstraintSelectionStrategy::
                                                         AllChecked,
                                                     cda_rail::solver::mip_based::LazyTrainSelectionStrategy::All},
        {}, 420);

    EXPECT_TRUE(true);
  }
}

// NOLINTEND (clang-analyzer-deadcode.DeadStores)
