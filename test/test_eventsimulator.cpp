#include <cstdlib>
#define TEST_FRIENDS true

#include "Definitions.hpp"
#include "EOMHelper.hpp"
#include "datastructure/RailwayNetwork.hpp"
#include "datastructure/Timetable.hpp"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"
#include "simulator/EventSimulator.hpp"

#include "gtest/gtest.h"
#include <algorithm>
#include <cmath>
#include <vector>

using namespace cda_rail;

#define EXPECT_APPROX_EQ_6(a, b) EXPECT_APPROX_EQ(a, b, 1e-6)

#define EXPECT_APPROX_EQ(a, b, c)                                              \
  EXPECT_TRUE(std::abs((a) - (b)) < (c)) << (a) << " !=(approx.) " << (b)

// NOLINTBEGIN
// (clang-analyzer-deadcode.DeadStores,misc-const-correctness,clang-diagnostic-unused-result)

// The simulations themselves are tested for both simulators in
// test_greedysimulator.cpp.

TEST(EventSimulator, Movement) {
  // Accelerating from 0 to 10 within 5 s, driving at 10 for 10 s, and braking
  // to a stop within 5 s
  const simulator::EventSimulator::Movement movement{
      .t      = 100,
      .pos    = 50,
      .v      = 0,
      .phases = {{.duration = 5, .v_start = 0, .v_end = 10},
                 {.duration = 10, .v_start = 10, .v_end = 10},
                 {.duration = 5, .v_start = 10, .v_end = 0}}};

  EXPECT_APPROX_EQ_6(movement.end_time(), 120);
  EXPECT_APPROX_EQ_6(movement.end_pos(), 200);
  EXPECT_APPROX_EQ_6(movement.end_speed(), 0);
  EXPECT_APPROX_EQ_6(movement.final_braking_start(), 115);

  EXPECT_APPROX_EQ_6(movement.at(100).pos, 50);
  EXPECT_APPROX_EQ_6(movement.at(102.5).pos, 56.25);
  EXPECT_APPROX_EQ_6(movement.at(102.5).vel, 5);
  EXPECT_APPROX_EQ_6(movement.at(110).pos, 125);
  EXPECT_APPROX_EQ_6(movement.at(117.5).pos, 193.75);
  EXPECT_APPROX_EQ_6(movement.at(117.5).vel, 5);
  EXPECT_APPROX_EQ_6(movement.at(200).pos, 200);
  EXPECT_APPROX_EQ_6(movement.at(200).vel, 0);

  EXPECT_APPROX_EQ_6(movement.time_at(0), 100);
  EXPECT_APPROX_EQ_6(movement.time_at(56.25), 102.5);
  EXPECT_APPROX_EQ_6(movement.time_at(125), 110);
  EXPECT_APPROX_EQ_6(movement.time_at(193.75), 117.5);
  EXPECT_EQ(movement.time_at(201), INF);

  // With a deceleration of 2, the braking point is 25 m ahead at speed 10
  EXPECT_APPROX_EQ_6(movement.braking_point_time(50, 2), 100);
  for (const auto time : {101.0, 104.0, 108.0}) {
    const auto [pos, vel] = movement.at(time);
    EXPECT_APPROX_EQ_6(movement.braking_point_time(pos + (vel * vel / 4), 2),
                       time);
  }
  // Braking fully, the braking point stays at the end of the movement
  EXPECT_APPROX_EQ_6(movement.braking_point_time(200, 2), 115);
  EXPECT_EQ(movement.braking_point_time(201, 2), INF);

  const auto first_part = movement.until(150);
  EXPECT_APPROX_EQ_6(first_part.end_pos(), 150);
  EXPECT_APPROX_EQ_6(first_part.end_time(), 112.5);
  EXPECT_APPROX_EQ_6(first_part.end_speed(), 10);
  EXPECT_APPROX_EQ_6(movement.until(300).end_time(), 120);
  EXPECT_TRUE(movement.until(50).phases.empty());

  const simulator::EventSimulator::Movement standing{
      .t = 10, .pos = 5, .v = 0, .phases = {}};
  EXPECT_EQ(standing.end_time(), 10);
  EXPECT_EQ(standing.final_braking_start(), 10);
  EXPECT_EQ(standing.at(20).pos, 5);
  EXPECT_EQ(standing.time_at(6), INF);
}

namespace {
// Checks that the phases get from pos to target with consecutive speeds that
// stay within the speed limits and only change by accelerating or braking
// fully. The types are deduced since they are private.
template <typename Phases, typename Limits>
void check_phases(Phases const& phases, double pos, double v_0, double target,
                  double v_target, Limits const& limits, double a, double d) {
  const auto limit_at = [&limits](double x) {
    return std::ranges::find_if(
               limits,
               [x](auto const& limit) { return x <= limit.end + GRB_EPS; })
        ->v_max;
  };
  ASSERT_FALSE(phases.empty());
  EXPECT_LE(phases.front().v_start, v_0 + GRB_EPS);
  double x = pos;
  for (size_t i = 0; i < phases.size(); ++i) {
    const auto& phase = phases.at(i);
    EXPECT_GT(phase.duration, 0);
    if (i > 0) {
      EXPECT_APPROX_EQ_6(phase.v_start, phases.at(i - 1).v_end);
    }
    const auto acceleration = phase.acceleration();
    EXPECT_TRUE(std::abs(acceleration - a) < GRB_EPS ||
                std::abs(acceleration) < GRB_EPS ||
                std::abs(acceleration + d) < GRB_EPS)
        << "Acceleration " << acceleration;
    EXPECT_LE(phase.v_start, limit_at(x + GRB_EPS) + GRB_EPS);
    x += phase.distance();
    EXPECT_LE(phase.v_end, limit_at(x - GRB_EPS) + GRB_EPS);
  }
  EXPECT_APPROX_EQ_6(x, target);
  EXPECT_LE(phases.back().v_end, v_target + GRB_EPS);
}

template <typename Phases> double duration(Phases const& phases) {
  double sum = 0;
  for (const auto& phase : phases) {
    sum += phase.duration;
  }
  return sum;
}
} // namespace

TEST(EventSimulator, FastestPhases) {
  using Limits = std::vector<simulator::EventSimulator::SpeedLimit>;

  // A single speed limit
  const Limits single{{.end = 1100, .v_max = 50}};
  const auto   to_stop =
      simulator::EventSimulator::fastest_phases(0, 15, 1100, 0, single, 4, 2);
  check_phases(to_stop, 0, 15, 1100, 0, single, 4, 2);
  EXPECT_APPROX_EQ_6(duration(to_stop),
                     cda_rail::min_travel_time(15, 0, 50, 4, 2, 1100));
  // Accelerating, driving at the speed limit, and braking
  EXPECT_EQ(to_stop.size(), 3);

  const auto without_target_speed = simulator::EventSimulator::fastest_phases(
      100, 15, 1100, INF, single, 4, 2);
  check_phases(without_target_speed, 100, 15, 1100, INF, single, 4, 2);
  EXPECT_APPROX_EQ_6(
      duration(without_target_speed),
      cda_rail::min_travel_time_flexible_exit_speed(15, 50, 4, 1000));

  // Too short to reach the speed limit
  const auto peak =
      simulator::EventSimulator::fastest_phases(0, 0, 100, 0, single, 4, 2);
  check_phases(peak, 0, 0, 100, 0, single, 4, 2);
  EXPECT_EQ(peak.size(), 2);
  EXPECT_APPROX_EQ_6(duration(peak),
                     cda_rail::min_travel_time(0, 0, 50, 4, 2, 100));

  // A higher speed limit after a lower one is only used after the lower one
  const Limits increasing{{.end = 1000, .v_max = 30},
                          {.end = 2000, .v_max = 60}};
  const auto   after_lower = simulator::EventSimulator::fastest_phases(
      0, 30, 2000, 0, increasing, 1, 1);
  check_phases(after_lower, 0, 30, 2000, 0, increasing, 1, 1);
  EXPECT_APPROX_EQ_6(duration(after_lower),
                     (1000.0 / 30) +
                         cda_rail::min_travel_time(30, 0, 60, 1, 1, 1000));

  // Several lower speed limits ahead, of which not the first one is the most
  // restrictive, see Figure 3.3 of the thesis
  const Limits several{{.end = 1000, .v_max = 30},
                       {.end = 3000, .v_max = 60},
                       {.end = 3200, .v_max = 50},
                       {.end = 4200, .v_max = 30},
                       {.end = 5200, .v_max = 20}};
  for (const auto v_target : {0.0, 10.0, INF}) {
    const auto phases = simulator::EventSimulator::fastest_phases(
        0, 0, 5200, v_target, several, 1, 0.5);
    check_phases(phases, 0, 0, 5200, v_target, several, 1, 0.5);
  }
  // Braking for the limit of 30 at 3200 already covers the one of 50 at 3000
  const Limits ahead{several.begin() + 1, several.end() - 1};
  const auto   phases = simulator::EventSimulator::fastest_phases(
      2000, 60, 4200, INF, ahead, 1, 2);
  check_phases(phases, 2000, 60, 4200, INF, ahead, 1, 2);
  ASSERT_EQ(phases.size(), 3);
  EXPECT_APPROX_EQ_6(phases.at(1).v_end, 30);
  EXPECT_APPROX_EQ_6(2000 + phases.at(0).distance(), 3200 - 675);

  // A train that is too fast is slowed down instantaneously
  const auto too_fast = simulator::EventSimulator::fastest_phases(
      0, 40, 1000, INF, increasing, 1, 1);
  EXPECT_APPROX_EQ_6(too_fast.front().v_start, 30);

  EXPECT_TRUE(
      simulator::EventSimulator::fastest_phases(100, 10, 100, 0, single, 4, 2)
          .empty());
  EXPECT_TRUE(
      simulator::EventSimulator::fastest_phases(100, 10, 50, 0, single, 4, 2)
          .empty());
}

TEST(EventSimulator, SpeedLimits) {
  Network    network;
  const auto v0    = network.add_vertex("v0", VertexType::TTD);
  const auto v1    = network.add_vertex("v1", VertexType::TTD);
  const auto v2    = network.add_vertex("v2", VertexType::TTD);
  const auto v3    = network.add_vertex("v3", VertexType::TTD);
  const auto v0_v1 = network.add_edge(v0, v1, 100, 10, true);
  const auto v1_v2 = network.add_edge(v1, v2, 200, 20, true);
  const auto v2_v3 = network.add_edge(v2, v3, 100, 5, true);
  network.add_successor(v0_v1, v1_v2);
  network.add_successor(v1_v2, v2_v3);

  Timetable  timetable;
  const auto tr1 = timetable.add_train("Train1", 50, 15, 1, 1, true, 0, 0, v0,
                                       100, 5, v3, network);
  instances::GeneralPerformanceOptimizationInstance instance(network, timetable,
                                                             {});
  simulator::EventSimulator                         simulator(instance, {});
  simulator.set_train_edges_of_tr(tr1, {v0_v1, v1_v2, v2_v3});

  // The speed limit of an edge applies until the train's rear has left it
  const auto leaving = simulator.speed_limits(tr1, 0, 450, true);
  ASSERT_EQ(leaving.size(), 3);
  EXPECT_EQ(leaving.at(0).end, 150);
  EXPECT_EQ(leaving.at(0).v_max, 10);
  EXPECT_EQ(leaving.at(1).end, 300);
  EXPECT_EQ(leaving.at(1).v_max, 15); // the train's maximal speed
  EXPECT_EQ(leaving.at(2).end, 450);  // beyond the route's last edge
  EXPECT_EQ(leaving.at(2).v_max, 5);

  // Otherwise only while the train's front is on it
  const auto front = simulator.speed_limits(tr1, 50, 350, false);
  ASSERT_EQ(front.size(), 3);
  EXPECT_EQ(front.at(0).end, 100);
  EXPECT_EQ(front.at(0).v_max, 10);
  EXPECT_EQ(front.at(1).end, 300);
  EXPECT_EQ(front.at(1).v_max, 15);
  EXPECT_EQ(front.at(2).end, 350);
  EXPECT_EQ(front.at(2).v_max, 5);

  EXPECT_TRUE(simulator.speed_limits(tr1, 200, 200, true).empty());
}

TEST(EventSimulator, ExitMovement) {
  // Unrestricted, the train's rear passes the exit vertex at 5000 at time 105,
  // see GreedySimulator test ExitNotBeforeScheduledExitTime. Braking fully at
  // the latest, it gets there not before 117.5 at the minimal speed, and
  // stopping at the exit vertex, not before about 131.9.
  Network    network;
  const auto v0    = network.add_vertex("v0", VertexType::TTD, 0);
  const auto v1    = network.add_vertex("v1", VertexType::TTD, 0);
  const auto v0_v1 = network.add_edge(v0, v1, 5000, 50, true);

  for (const double exit_time :
       {100.0, 105.0, 110.0, 117.0, 120.0, 131.0, 135.0, 200.0}) {
    Timetable  timetable;
    const auto tr1 = timetable.add_train("Train1", 250, 50, 4, 2, true, 0, 50,
                                         v0, exit_time, 50, v1, network);
    instances::GeneralPerformanceOptimizationInstance instance(network,
                                                               timetable, {});
    simulator::EventSimulator                         simulator(instance, {});
    simulator.set_train_edges_of_tr(tr1, {v0_v1});

    const auto movement =
        simulator.exit_movement(tr1, 0, {.pos = 0, .vel = 50}, true);
    EXPECT_APPROX_EQ_6(movement.end_time(), std::max(exit_time, 105.0))
        << "exit time " << exit_time;
    EXPECT_APPROX_EQ_6(movement.end_pos(), 5250) << "exit time " << exit_time;

    // The train only waits at its exit vertex if it would be too early even
    // when stopping there, and it is never slower than V_MIN outside
    bool   waits = false;
    double x     = 0;
    for (const auto& phase : movement.phases) {
      if (phase.v_start < V_MIN && phase.v_end < V_MIN) {
        waits = true;
        EXPECT_APPROX_EQ_6(x, 5000) << "exit time " << exit_time;
      }
      x += phase.distance();
      if (x > 5000 + GRB_EPS) {
        EXPECT_GE(phase.v_end, V_MIN - GRB_EPS) << "exit time " << exit_time;
      }
    }
    EXPECT_EQ(waits, exit_time > 132) << "exit time " << exit_time;
  }
}

// NOLINTEND
// (clang-analyzer-deadcode.DeadStores,misc-const-correctness,clang-diagnostic-unused-result)
