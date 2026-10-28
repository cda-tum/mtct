#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "datastructure/RailwayNetwork.hpp"
#include "nlohmann/json.hpp"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"
#include "simulator/GreedySimulator.hpp"
#include "solver/astar-based/GenPOMovingBlockAStarSolver.hpp"

#include "gtest/gtest.h"
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>
#include <vector>

using namespace cda_rail;

// NOLINTBEGIN
// (clang-analyzer-deadcode.DeadStores,misc-const-correctness,clang-diagnostic-unused-result)

namespace {
// One edge of 1000 m from v0 to v1. Every train is 100 m long, may drive at
// 10 m/s and accelerates and decelerates with 1 m/s^2, so that its braking
// distance at 10 m/s is 50 m. It enters at time 0 with v_0 and is scheduled to
// leave at time 100, which is exactly when its front reaches v1 if it drives
// at 10 m/s all the way.
instances::GeneralPerformanceOptimizationInstance
one_edge_instance(size_t num_trains, double v_0 = 10, bool tim = true) {
  Network    network;
  const auto v0   = network.add_vertex("v0", VertexType::TTD, 0);
  const auto v1   = network.add_vertex("v1", VertexType::TTD, 0);
  const auto edge = network.add_edge(v0, v1, 1000, 10, true);

  instances::GeneralPerformanceOptimizationInstance instance(network);
  for (size_t tr = 1; tr <= num_trains; ++tr) {
    const auto tr_name = "Train" + std::to_string(tr);
    instance.add_train(tr_name, 100, 10, 1, 1, tim, 0, v_0, v0, 100, v_0, v1,
                       1);
    instance.add_empty_route(tr_name);
    instance.push_back_edge_to_route(tr_name, edge);
  }
  return instance;
}

// The same train on two edges of 500 m, v0 -> v1 -> v2, the first of which is
// a station. The train has to stop there with its front at v1, i.e., at 500 m,
// with its service starting no earlier than 40 and lasting 30 s. Driving at
// 10 m/s, it starts braking at 450 m and arrives at 55.
instances::GeneralPerformanceOptimizationInstance station_instance() {
  Network    network;
  const auto v0    = network.add_vertex("v0", VertexType::TTD, 0);
  const auto v1    = network.add_vertex("v1", VertexType::TTD, 0);
  const auto v2    = network.add_vertex("v2", VertexType::TTD, 0);
  const auto v0_v1 = network.add_edge(v0, v1, 500, 10, true);
  const auto v1_v2 = network.add_edge(v1, v2, 500, 10, true);
  network.add_successor(v0_v1, v1_v2);

  instances::GeneralPerformanceOptimizationInstance instance(network);
  instance.add_train("Train1", 100, 10, 1, 1, true, 0, 10, v0, 100, 10, v2, 1);
  instance.add_empty_station("Station");
  instance.add_track_to_station("Station", v0_v1);
  instance.insert_stop("Train1", "Station", 40, 30);
  instance.add_empty_route("Train1");
  instance.push_back_edge_to_route("Train1", v0_v1);
  instance.push_back_edge_to_route("Train1", v1_v2);
  return instance;
}

struct Sample {
  double t;
  double pos;
  double vel;
};

void add_samples(instances::SolGeneralPerformanceOptimizationInstance& sol,
                 const std::string&                                    tr_name,
                 const std::vector<Sample>& samples) {
  for (const auto& [t, pos, vel] : samples) {
    sol.add_train_pos(tr_name, t, pos);
    sol.add_train_speed(tr_name, t, vel);
  }
}

// Two trains following each other along the one-edge instance at 10 m/s as
// closely as moving block allows: the second one enters once the rear of the
// first one is 50 m, i.e., its braking distance, away from the entry, which is
// the case at time 15.
template <typename SolType>
SolType following_trains_solution(
    const instances::GeneralPerformanceOptimizationInstance& instance) {
  SolType sol(instance, SolutionStatus::Feasible, 0, true);
  add_samples(sol, "Train1", {{0, 0, 10}, {100, 1000, 10}});
  add_samples(sol, "Train2", {{15, 0, 10}, {115, 1000, 10}});
  sol.set_exit_times({100, 115});
  return sol;
}

// The one-edge instance with trains scheduled as slow_exit_solution operates
// them: the first one enters at time 0 and leaves at 154, the second one, if
// any, enters at 130 and leaves at 240.
instances::GeneralPerformanceOptimizationInstance
slow_exit_instance(size_t num_trains) {
  Network    network;
  const auto v0   = network.add_vertex("v0", VertexType::TTD, 0);
  const auto v1   = network.add_vertex("v1", VertexType::TTD, 0);
  const auto edge = network.add_edge(v0, v1, 1000, 10, true);

  instances::GeneralPerformanceOptimizationInstance instance(network);
  instance.add_train("Train1", 100, 10, 1, 1, true, 0, 10, v0, 154, 10, v1, 1);
  if (num_trains > 1) {
    instance.add_train("Train2", 100, 10, 1, 1, true, 130, 10, v0, 240, 10, v1,
                       1);
  }
  for (const auto& train : instance.get_const_train_list()) {
    instance.add_empty_route(train.get_name());
    instance.push_back_edge_to_route(train.get_name(), edge);
  }
  return instance;
}

// The first train drives at 10 m/s until 950 m, brakes to 2 m/s by 998 m at
// time 103, and then crawls out of the network, so that its front reaches the
// exit vertex at 104 and its rear passes it at 154. Its rear hence passes
// 950 m at 129. The second train, if any, enters at 130 and drives at 10 m/s
// until its rear passes the exit vertex at 240.
template <typename SolType>
SolType slow_exit_solution(
    const instances::GeneralPerformanceOptimizationInstance& instance) {
  SolType sol(instance, SolutionStatus::Feasible, 0, true);
  add_samples(sol, "Train1",
              {{0, 0, 10}, {95, 950, 10}, {103, 998, 2}, {154, 1100, 2}});
  if (instance.get_const_train_list().size() > 1) {
    add_samples(sol, "Train2", {{130, 0, 10}, {240, 1100, 10}});
    sol.set_exit_times({154, 240});
  } else {
    sol.set_exit_times({154});
  }
  return sol;
}
} // namespace

TEST(SolutionEvaluation, PositionBoundHoldsTrainBack) {
  // Until time 50, the train has to be able to stop before 300 m. It hence
  // stops there and waits until it may continue, after which it needs another
  // 85 s until its rear has passed the exit vertex, i.e., for the remaining
  // 700 m and its length of 100 m, 5 s of which are lost by accelerating.
  const auto                 instance = one_edge_instance(1);
  simulator::GreedySimulator simulator(instance, {}, {{0}}, {}, {{0}, {0}},
                                       {{}});
  simulator.set_position_bound(
      [](size_t /*tr*/,
         double t) -> std::optional<simulator::GreedySimulator::PositionBound> {
        if (t < 50) {
          return simulator::GreedySimulator::PositionBound{
              .front = 300, .braking_point = 300};
        }
        return std::nullopt;
      });

  const auto result = simulator.simulate(1.0, false, true, true);
  ASSERT_TRUE(result.success);
  for (const auto& [t, pos_vel] : result.train_trajectories.at(0)) {
    if (t < 50) {
      EXPECT_LE(pos_vel.pos, 300 + GRB_EPS) << "at time " << t;
    }
  }
  EXPECT_NEAR(result.train_trajectories.at(0).at(49).pos, 300, GRB_EPS);
  EXPECT_NEAR(result.exit_times.at(0), 135, 1);

  // Without the bound, the train drives through at 10 m/s
  simulator.set_position_bound({});
  EXPECT_NEAR(simulator.simulate(1.0, false, true).exit_times.at(0), 110, 1);
}

TEST(SolutionEvaluation, PositionBoundDelaysEntry) {
  // Entering at 10 m/s, the train needs 50 m to stop. Until time 20, it has to
  // be able to stop within 40 m, hence, it cannot enter before. It then needs
  // 110 s until its rear has passed the exit vertex.
  const auto                 instance = one_edge_instance(1);
  simulator::GreedySimulator simulator(instance, {}, {{0}}, {}, {{0}, {0}},
                                       {{}});
  simulator.set_position_bound(
      [](size_t /*tr*/,
         double t) -> std::optional<simulator::GreedySimulator::PositionBound> {
        if (t < 20) {
          return simulator::GreedySimulator::PositionBound{.front         = 40,
                                                           .braking_point = 40};
        }
        return std::nullopt;
      });

  const auto result = simulator.simulate(1.0, true, true, true);
  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.train_trajectories.at(0).begin()->first, 20);
  EXPECT_NEAR(result.exit_times.at(0), 130, 1);
}

TEST(SolutionEvaluation, GreedySolutionIsReproduced) {
  // A solution obtained by the greedy simulator, as the A* solver does, is
  // simulated identically, including the stop at the station. Since the
  // solution is greedy itself, it makes no difference whether the trains are
  // kept behind it or not.
  const auto                 instance = station_instance();
  simulator::GreedySimulator simulator(instance, {}, {{0, 1}}, {},
                                       {{0}, {}, {0}}, {{500}});
  const auto simulated = simulator.simulate(1.0, false, true, true);
  ASSERT_TRUE(simulated.success);

  instances::SolGeneralPerformanceOptimizationInstance sol(
      instance, SolutionStatus::Feasible, 0, true);
  for (const auto& [t, pos_vel] : simulated.train_trajectories.at(0)) {
    sol.add_train_pos("Train1", t, pos_vel.pos);
    sol.add_train_speed("Train1", t, pos_vel.vel);
  }
  sol.set_exit_times(simulated.exit_times);
  sol.set_stop_times(simulated.stop_times);

  sol.evaluate(1.0);
  ASSERT_TRUE(sol.get_evaluation().has_value());
  for (const auto* delays :
       {&sol.get_evaluation()->trajectory, &sol.get_evaluation()->greedy}) {
    ASSERT_TRUE(delays->success);
    EXPECT_NEAR(delays->exit_times.at(0), simulated.exit_times.at(0), GRB_EPS);
    EXPECT_NEAR(delays->stop_times.at(0).at(0),
                simulated.stop_times.at(0).at(0), GRB_EPS);
  }
}

TEST(SolutionEvaluation, SlowSolutionIsNotOvertaken) {
  // The solution keeps the train at its entry speed of 5 m/s, whereas a greedy
  // driver would accelerate to 10 m/s and arrive at about 102.5. The train's
  // front hence reaches the exit vertex at 200, which is 100 s later than
  // scheduled. Between the two samples, the solution is only approximated,
  // which runs slightly ahead, so that the train has to brake before reaching
  // the second sample. It may hence arrive marginally later, but never
  // earlier. Its rear then needs at least another 12.5 s to pass the exit
  // vertex, since the train must not be faster than its exit speed of 5 m/s
  // there, and at most 16.25 s from a standstill.
  const auto instance = one_edge_instance(1, 5);
  instances::SolGeneralPerformanceOptimizationInstance sol(
      instance, SolutionStatus::Feasible, 0, true);
  add_samples(sol, "Train1", {{0, 0, 5}, {200, 1000, 5}});
  sol.set_exit_times({200});

  sol.evaluate(1.0);
  ASSERT_TRUE(sol.get_evaluation().has_value());
  const auto& evaluation = sol.get_evaluation().value();
  EXPECT_EQ(evaluation.dt, 1.0);
  const auto& trajectory = evaluation.trajectory;
  ASSERT_TRUE(trajectory.success);
  EXPECT_GE(trajectory.exit_times.at(0), 200 + 12.5);
  EXPECT_LE(trajectory.exit_times.at(0), 202 + 16.25);
  EXPECT_NEAR(trajectory.exit_delays.at(0), trajectory.exit_times.at(0) - 100,
              GRB_EPS);
  EXPECT_NEAR(trajectory.obj, trajectory.exit_times.at(0), GRB_EPS);

  // Greedily, the train accelerates to 10 m/s within 5 s, i.e., 37.5 m, and
  // brakes back to its exit speed of 5 m/s within another 5 s and 37.5 m, by
  // the time its rear passes the exit vertex at 1100 m. It covers the 1025 m in
  // between at 10 m/s, so that it leaves at 112.5.
  ASSERT_TRUE(evaluation.greedy.success);
  EXPECT_NEAR(evaluation.greedy.exit_times.at(0), 112.5, 1);
}

TEST(SolutionEvaluation, SlowExitIsReplayed) {
  // The solution lets the train crawl out of the network at 2 m/s, so that it
  // leaves exactly at its exit time. Kept behind it, the train does the same,
  // and it cannot leave any earlier than the train of the solution anyway.
  const auto instance = slow_exit_instance(1);
  auto       sol =
      slow_exit_solution<instances::SolGeneralPerformanceOptimizationInstance>(
          instance);

  for (const double dt : {1.0, 15.0}) {
    sol.evaluate(dt);
    ASSERT_TRUE(sol.get_evaluation().has_value()) << "dt " << dt;
    const auto& trajectory = sol.get_evaluation()->trajectory;
    ASSERT_TRUE(trajectory.success) << "dt " << dt;
    EXPECT_NEAR(trajectory.exit_times.at(0), 154, GRB_EPS) << "dt " << dt;

    // Greedily, the train would leave at 110. In order not to leave before
    // its exit time, it waits at the exit vertex instead and then leaves
    // exactly at its exit time.
    const auto& greedy = sol.get_evaluation()->greedy;
    ASSERT_TRUE(greedy.success) << "dt " << dt;
    EXPECT_NEAR(greedy.exit_times.at(0), 154, GRB_EPS) << "dt " << dt;
  }
}

TEST(SolutionEvaluation, SlowExitDoesNotDelayFollowingTrain) {
  // With a VSS border at 950 m, the second train can only enter once the rear
  // of the first one has passed it, which it does at 129 when crawling out as
  // in the solution. Both trains hence leave as scheduled.
  const auto instance = slow_exit_instance(2);
  auto       sol      = slow_exit_solution<
      instances::SolVSSGeneralPerformanceOptimizationInstance>(instance);
  sol.add_vss_pos(0, 950);

  sol.evaluate(1.0);
  ASSERT_TRUE(sol.get_evaluation().has_value());
  const auto& trajectory = sol.get_evaluation()->trajectory;
  ASSERT_TRUE(trajectory.success);
  EXPECT_NEAR(trajectory.exit_times.at(0), 154, GRB_EPS);
  EXPECT_NEAR(trajectory.exit_times.at(1), 240, GRB_EPS);

  // Greedily, the first train waits at the exit vertex until 139, from where
  // it accelerates, so that it leaves exactly at its exit time. Its rear hence
  // passes the border only at 149, when the second train enters, 19 s later
  // than in the solution.
  const auto& greedy = sol.get_evaluation()->greedy;
  ASSERT_TRUE(greedy.success);
  EXPECT_NEAR(greedy.exit_times.at(0), 154, GRB_EPS);
  EXPECT_NEAR(greedy.exit_times.at(1), 259, GRB_EPS);
}

TEST(SolutionEvaluation, InfeasibleEarlyExitIsNotCounted) {
  // This solution is infeasible: its train leaves at 110, although it may not
  // leave before its exit time of 154. Since the train follows its dynamics,
  // the evaluation does not enforce its exit time but relies on the solution
  // to respect it. Hence, the simulated train follows the solution, and its
  // rear passes the exit vertex at 110 as well. It then waits there and is
  // only counted as leaving at 154, so that the infeasible solution does not
  // get a negative delay.
  const auto instance = slow_exit_instance(1);
  instances::SolGeneralPerformanceOptimizationInstance sol(
      instance, SolutionStatus::Feasible, 0, true);
  add_samples(sol, "Train1", {{0, 0, 10}, {110, 1100, 10}});
  sol.set_exit_times({110});

  sol.evaluate(1.0);
  ASSERT_TRUE(sol.get_evaluation().has_value());
  const auto& trajectory = sol.get_evaluation()->trajectory;
  ASSERT_TRUE(trajectory.success);
  EXPECT_NEAR(trajectory.exit_times.at(0), 154, GRB_EPS);
  EXPECT_NEAR(trajectory.exit_delays.at(0), 0, GRB_EPS);
}

TEST(SolutionEvaluation, StopBeyondDynamicsBeforeExit) {
  // As in a model without train dynamics, the solution lets the train stop at
  // the exit vertex within 40 m from 10 m/s, although it needs 50 m. It then
  // waits there until 139 and leaves at its exit time. Kept behind the
  // solution, the train could no longer stop at the exit vertex but only 10 m
  // beyond it, from where it would leave late. Since its trajectory does not
  // follow its dynamics, its exit time is enforced instead. The train hence
  // brakes in time to wait at the exit vertex and leaves at its exit time.
  const auto instance = slow_exit_instance(1);
  instances::SolGeneralPerformanceOptimizationInstance sol(
      instance, SolutionStatus::Feasible, 0, true);
  add_samples(sol, "Train1",
              {{0, 0, 10},
               {96, 960, 10},
               {100, 1000, 0},
               {139, 1000, 0},
               {149, 1050, 10},
               {154, 1100, 10}});
  sol.set_exit_times({154});

  sol.evaluate(1.0);
  ASSERT_TRUE(sol.get_evaluation().has_value());
  const auto& trajectory = sol.get_evaluation()->trajectory;
  ASSERT_TRUE(trajectory.success);
  EXPECT_NEAR(trajectory.exit_times.at(0), 154, GRB_EPS);
}

TEST(SolutionEvaluation, StationDelay) {
  // The train arrives at the station at 55, i.e., 15 s after its service
  // should have started, and the solution lets it stand there until 120
  // instead of only until 85. It then accelerates to 10 m/s within 10 s, its
  // front reaches the exit vertex at 175, and its rear passes it 10 s later.
  const auto instance = station_instance();
  instances::SolGeneralPerformanceOptimizationInstance sol(
      instance, SolutionStatus::Feasible, 0, true);
  add_samples(sol, "Train1",
              {{0, 0, 10},
               {45, 450, 10},
               {55, 500, 0},
               {120, 500, 0},
               {130, 550, 10},
               {175, 1000, 10}});
  sol.set_exit_times({175});
  sol.set_stop_times({{55}});

  sol.evaluate(1.0);
  ASSERT_TRUE(sol.get_evaluation().has_value());
  const auto& trajectory = sol.get_evaluation()->trajectory;
  ASSERT_TRUE(trajectory.success);
  EXPECT_NEAR(trajectory.stop_times.at(0).at(0), 55, 1);
  EXPECT_NEAR(trajectory.stop_delays.at(0).at(0), 15, 1);
  EXPECT_NEAR(trajectory.exit_times.at(0), 185, 1);
  EXPECT_NEAR(trajectory.exit_delays.at(0), 85, 1);
  // With all weights being 1, the objective is the exit time plus the average
  // station delay
  EXPECT_NEAR(trajectory.obj,
              trajectory.exit_times.at(0) + trajectory.stop_delays.at(0).at(0),
              GRB_EPS);

  // Greedily, the train leaves the station after its service at 85 instead.
  // Within 10 s, it accelerates to 10 m/s up to 550 m, and 55 s later its rear
  // passes the exit vertex at 1100 m.
  const auto& greedy = sol.get_evaluation()->greedy;
  ASSERT_TRUE(greedy.success);
  EXPECT_NEAR(greedy.stop_delays.at(0).at(0), 15, 1);
  EXPECT_NEAR(greedy.exit_times.at(0), 150, 1);
}

TEST(SolutionEvaluation, BlockSectionsDelayFollowingTrain) {
  // The second train follows the first one as closely as moving block allows.
  // A train leaves once its rear has passed the exit vertex, i.e., 10 s after
  // its front has reached it, so that the first train leaves at 110. Under
  // moving block, the second train can follow and leaves at 125. With a VSS
  // border in the middle of the edge, it has to wait until the first train has
  // cleared the first half, i.e., until its front is at 600 m at time 60. It
  // then reaches the border at 110, just when the first train clears the
  // second half, so that it has to brake to 5 m/s before it and leaves at
  // 172.5. Without any VSS border, the whole edge is one block, which the first
  // train only clears when leaving at 110, so that the second one leaves at
  // 220.
  const auto instance = one_edge_instance(2);

  auto mb_sol = following_trains_solution<
      instances::SolGeneralPerformanceOptimizationInstance>(instance);
  auto vss_sol_with_border = following_trains_solution<
      instances::SolVSSGeneralPerformanceOptimizationInstance>(instance);
  vss_sol_with_border.add_vss_pos(0, 500);
  auto vss_sol_without_border = following_trains_solution<
      instances::SolVSSGeneralPerformanceOptimizationInstance>(instance);

  mb_sol.evaluate(1.0);
  vss_sol_with_border.evaluate(1.0);
  vss_sol_without_border.evaluate(1.0);

  for (const auto* sol :
       std::vector<const instances::SolGeneralPerformanceOptimizationInstance*>{
           &mb_sol, &vss_sol_with_border, &vss_sol_without_border}) {
    ASSERT_TRUE(sol->get_evaluation().has_value());
    ASSERT_TRUE(sol->get_evaluation()->trajectory.success);
    EXPECT_NEAR(sol->get_evaluation()->trajectory.exit_times.at(0), 110, 1);
  }
  EXPECT_NEAR(mb_sol.get_evaluation()->trajectory.exit_times.at(1), 125, 1);
  EXPECT_NEAR(vss_sol_with_border.get_evaluation()->trajectory.exit_times.at(1),
              172.5, 1);
  EXPECT_NEAR(
      vss_sol_without_border.get_evaluation()->trajectory.exit_times.at(1), 220,
      1);
}

TEST(SolutionEvaluation, TrainsWithoutTimNotImplementedForVSS) {
  const auto instance = one_edge_instance(2, 10, false);

  auto mb_sol = following_trains_solution<
      instances::SolGeneralPerformanceOptimizationInstance>(instance);
  EXPECT_NO_THROW(mb_sol.evaluate(1.0));

  auto vss_sol = following_trains_solution<
      instances::SolVSSGeneralPerformanceOptimizationInstance>(instance);
  EXPECT_THROW(vss_sol.evaluate(1.0),
               cda_rail::exceptions::InvalidInputException);
}

TEST(SolutionEvaluation, NothingToEvaluate) {
  instances::SolGeneralPerformanceOptimizationInstance sol(
      one_edge_instance(1));
  EXPECT_FALSE(sol.get_evaluation().has_value());
  EXPECT_THROW(sol.evaluate(1.0), cda_rail::exceptions::ConsistencyException);
}

TEST(SolutionEvaluation, ExportAndImport) {
  const auto instance = one_edge_instance(1, 5);
  instances::SolGeneralPerformanceOptimizationInstance sol(
      instance, SolutionStatus::Feasible, 0, true);
  add_samples(sol, "Train1", {{0, 0, 5}, {200, 1000, 5}});
  sol.set_exit_times({200});

  // Without an evaluation, nothing is added
  sol.export_solution("./tmp-solution-evaluation", "test-evaluation");
  {
    std::ifstream  file("./tmp-solution-evaluation/solutions/test-evaluation/"
                        "UnnamedSubdirectory/"
                        "UnnamedInstance/solution_data.json");
    nlohmann::json data = nlohmann::json::parse(file);
    EXPECT_FALSE(data.contains("evaluation"));
  }

  sol.evaluate(1.0);
  sol.export_solution("./tmp-solution-evaluation", "test-evaluation");
  {
    std::ifstream  file("./tmp-solution-evaluation/solutions/test-evaluation/"
                        "UnnamedSubdirectory/"
                        "UnnamedInstance/solution_data.json");
    nlohmann::json data = nlohmann::json::parse(file);
    ASSERT_TRUE(data.contains("evaluation"));
    const auto& evaluation = data.at("evaluation");
    EXPECT_EQ(evaluation.at("dt").get<double>(), 1.0);
    EXPECT_TRUE(evaluation.at("trajectory").at("success").get<bool>());
    EXPECT_EQ(evaluation.at("trajectory").at("obj").get<double>(),
              sol.get_evaluation()->trajectory.obj);
    EXPECT_TRUE(evaluation.at("greedy").at("success").get<bool>());
    EXPECT_EQ(evaluation.at("greedy").at("obj").get<double>(),
              sol.get_evaluation()->greedy.obj);
    EXPECT_EQ(evaluation.at("trajectory")
                  .at("trains")
                  .at("Train1")
                  .at("exit_delay")
                  .get<double>(),
              sol.get_evaluation()->trajectory.exit_delays.at(0));
  }

  instances::SolGeneralPerformanceOptimizationInstance sol_read(instance);
  sol_read.load_solution("./tmp-solution-evaluation", "test-evaluation");
  ASSERT_TRUE(sol_read.get_evaluation().has_value());
  EXPECT_EQ(sol_read.get_evaluation()->dt, sol.get_evaluation()->dt);
  EXPECT_EQ(sol_read.get_evaluation()->trajectory.exit_times,
            sol.get_evaluation()->trajectory.exit_times);
  EXPECT_EQ(sol_read.get_evaluation()->trajectory.exit_delays,
            sol.get_evaluation()->trajectory.exit_delays);
  EXPECT_EQ(sol_read.get_evaluation()->trajectory.obj,
            sol.get_evaluation()->trajectory.obj);
  EXPECT_EQ(sol_read.get_evaluation()->greedy.exit_times,
            sol.get_evaluation()->greedy.exit_times);
  EXPECT_EQ(sol_read.get_evaluation()->greedy.obj,
            sol.get_evaluation()->greedy.obj);

  std::filesystem::remove_all("./tmp-solution-evaluation");
}

TEST(SolutionEvaluation, ContradictoryOrdersEndInADeadlock) {
  // The solution lets the second train leave before the first one, which it
  // cannot, since it follows it on the same track. Hence, both simulations end
  // in a deadlock, which is exported and imported as such.
  const auto instance = one_edge_instance(2);
  auto       sol      = following_trains_solution<
      instances::SolGeneralPerformanceOptimizationInstance>(instance);
  sol.set_exit_times({115, 100});

  sol.evaluate(1.0);
  ASSERT_TRUE(sol.get_evaluation().has_value());
  EXPECT_FALSE(sol.get_evaluation()->trajectory.success);
  EXPECT_FALSE(sol.get_evaluation()->greedy.success);

  sol.export_solution("./tmp-solution-evaluation-deadlock", "test-evaluation");
  instances::SolGeneralPerformanceOptimizationInstance sol_read(instance);
  sol_read.load_solution("./tmp-solution-evaluation-deadlock",
                         "test-evaluation");
  std::filesystem::remove_all("./tmp-solution-evaluation-deadlock");
  ASSERT_TRUE(sol_read.get_evaluation().has_value());
  EXPECT_FALSE(sol_read.get_evaluation()->trajectory.success);
  EXPECT_FALSE(sol_read.get_evaluation()->greedy.success);
}

TEST(SolutionEvaluation, AStarSolutionWithAFinerTimeStep) {
  // The A* search simulates its trains with a time step of 6 s, so that its
  // solution only has a sample every 6 s, in between which the evaluation has
  // to approximate it. It reproduces the exit times up to that time step.
  const instances::GeneralPerformanceOptimizationInstance instance(
      "SimpleStation", "atmos2023", "data");
  solver::astar_based::GenPOMovingBlockAStarSolver solver(instance);
  auto                                             sol = solver.solve();
  ASSERT_TRUE(sol.has_solution());

  sol.evaluate(1.0);
  ASSERT_TRUE(sol.get_evaluation().has_value());
  ASSERT_TRUE(sol.get_evaluation()->greedy.success);
  const auto& evaluation = sol.get_evaluation()->trajectory;
  ASSERT_TRUE(evaluation.success);
  const auto& train_list = instance.get_const_train_list();
  for (size_t tr = 0; tr < train_list.size(); ++tr) {
    const auto& tr_name = train_list.get_train(tr).get_name();
    EXPECT_NEAR(evaluation.exit_times.at(tr), sol.get_exit_time(tr_name), 6)
        << tr_name;
  }
}

TEST(SolutionEvaluation, StoredMovingBlockSolutions) {
  // These solutions were obtained by the moving block MILP, which models the
  // trains in full detail. Hence, simulating them with a time step of 1 s
  // reproduces their exit times up to that time step. Between two vertices,
  // however, the solution only fixes how long a train takes, and the
  // evaluation assumes it to drive at a constant line speed. A train that the
  // solution lets take long for an edge is hence assumed to crawl, which can
  // delay a following train by a few seconds. Since every train of these
  // solutions leaves exactly when scheduled, i.e., without any slack, this can
  // delay the train itself by another second, as can stopping when its rear
  // passes the exit vertex, which only happens at the end of a time step.
  for (const auto* instance_name :
       {"HighSpeedTrack2Trains", "HighSpeedTrack5Trains", "Overtake",
        "SimpleNetwork", "SimpleStation", "SingleTrack",
        "SingleTrackWithStation", "Stammstrecke4Trains", "Stammstrecke8Trains",
        "Stammstrecke16Trains"}) {
    const instances::GeneralPerformanceOptimizationInstance instance(
        instance_name, "atmos2023", "data");
    instances::SolGeneralPerformanceOptimizationInstance sol(instance);
    sol.load_solution("data", "moving-block-solutions");

    sol.evaluate(1.0);
    ASSERT_TRUE(sol.get_evaluation().has_value()) << instance_name;
    ASSERT_TRUE(sol.get_evaluation()->greedy.success) << instance_name;
    const auto& evaluation = sol.get_evaluation()->trajectory;
    ASSERT_TRUE(evaluation.success) << instance_name;
    const auto& train_list = instance.get_const_train_list();
    for (size_t tr = 0; tr < train_list.size(); ++tr) {
      const auto& tr_name = train_list.get_train(tr).get_name();
      EXPECT_NEAR(evaluation.exit_times.at(tr), sol.get_exit_time(tr_name), 2)
          << instance_name << ", " << tr_name;
      for (size_t stop = 0; stop < evaluation.stop_times.at(tr).size();
           ++stop) {
        EXPECT_NEAR(evaluation.stop_times.at(tr).at(stop),
                    sol.get_stop_time(tr_name, stop), 10)
            << instance_name << ", " << tr_name << ", stop " << stop;
      }
    }
  }
}

// NOLINTEND
// (clang-analyzer-deadcode.DeadStores,misc-const-correctness,clang-diagnostic-unused-result)
