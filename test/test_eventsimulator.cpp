#include <cstdlib>
#define TEST_FRIENDS true

#include "CustomExceptions.hpp"
#include "EOMHelper.hpp"
#include "datastructure/Timetable.hpp"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"
#include "simulator/EventSimulator.hpp"

#include "gtest/gtest.h"
#include <cmath>
#include <plog/Appenders/ColorConsoleAppender.h>
#include <plog/Formatters/TxtFormatter.h>
#include <plog/Init.h>
#include <plog/Log.h>
#include <plog/Severity.h>

using namespace cda_rail;

#define EXPECT_APPROX_EQ_6(a, b) EXPECT_APPROX_EQ(a, b, 1e-6)

#define EXPECT_APPROX_EQ(a, b, c)                                              \
  EXPECT_TRUE(std::abs((a) - (b)) < (c)) << (a) << " !=(approx.) " << (b)

// NOLINTBEGIN
// (clang-analyzer-deadcode.DeadStores,misc-const-correctness,clang-diagnostic-unused-result)

TEST(EventSimulator, CalculateOptimalTrainMovements) {
  instances::GeneralPerformanceOptimizationInstance instance{};

  simulator::EventSimulator simulator{instance, {}};

  // TODO: limit_speed_by_leaving_edges true or false?
  std::vector<cda_rail::simulator::EventSimulator::EdgeSegment> free_track{
      {1000, 50}, {2000, 100}, {200, 90}, {1000, 50}};
  auto initial_speed = 10.0;
  auto train         = Train("Deutsche Bahn", 300, 100, 10, 10);

  auto movements = simulator.calculate_optimal_train_movements(
      free_track, initial_speed, train);

  EXPECT_EQ(movements[0].u, 10.0);
  // TODO
}

TEST(EventSimulator, MoveTrain) {
  const std::unordered_set<size_t>         trains_in_network{0};
  simulator::EventSimulator::TrainPosition train_position{-300.0, 0.0};
  double                                   train_velocity = {0};
  std::vector<simulator::EventSimulator::TrainMovement> train_movements{
      {10, 50, 10, 120, 4}, {50, 50, 0, 500, 10}, {50, 0, -5, 250, 10}};
  auto t = 0.0;

  simulator::EventSimulator::move_train(train_position, train_velocity,
                                        train_movements, 6);

  EXPECT_EQ(train_position.front, 220);
  EXPECT_EQ(train_position.rear, -80);
  EXPECT_EQ(train_velocity, 50);
  EXPECT_EQ(train_movements.size(), 2);
  EXPECT_EQ(train_movements[0].u, 50);
  EXPECT_EQ(train_movements[0].v, 50);
  EXPECT_EQ(train_movements[0].a, 0);
  EXPECT_EQ(train_movements[0].s, 400);
  EXPECT_EQ(train_movements[0].t, 8);

  simulator::EventSimulator::move_train(train_position, train_velocity,
                                        train_movements, 8);

  EXPECT_EQ(train_position.front, 620);
  EXPECT_EQ(train_position.rear, 320);
  EXPECT_EQ(train_velocity, 50);
  EXPECT_EQ(train_movements.size(), 1);

  simulator::EventSimulator::move_train(train_position, train_velocity,
                                        train_movements, 2);

  EXPECT_EQ(train_position.front, 710);
  EXPECT_EQ(train_position.rear, 410);
  EXPECT_EQ(train_velocity, 40);
  EXPECT_EQ(train_movements.size(), 1);
  EXPECT_EQ(train_movements[0].u, 40);
  EXPECT_EQ(train_movements[0].v, 0);
  EXPECT_EQ(train_movements[0].a, -5);
  EXPECT_EQ(train_movements[0].s, 160);
  EXPECT_EQ(train_movements[0].t, 8);
}

// NOLINTEND
// (clang-analyzer-deadcode.DeadStores,misc-const-correctness,clang-diagnostic-unused-result)
