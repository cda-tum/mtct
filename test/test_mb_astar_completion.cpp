#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "datastructure/RailwayNetwork.hpp"
#include "datastructure/Route.hpp"
#include "datastructure/Timetable.hpp"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"
#include "simulator/EventSimulator.hpp"
#include "simulator/GeneralSimulator.hpp"
#include "simulator/GreedyHeuristic.hpp"
#include "simulator/GreedySimulator.hpp"
#include "solver/astar-based/CompletionHeuristic.hpp"
#include "solver/astar-based/GenPOMovingBlockAStarSolver.hpp"

#include "gtest/gtest.h"
#include <algorithm>
#include <cstddef>
#include <optional>
#include <vector>

using namespace cda_rail;
// NOLINTBEGIN
// (clang-analyzer-deadcode.DeadStores,misc-const-correctness,clang-diagnostic-unused-result)

namespace {
// The parameters of simulator::GeneralSimulator::simulate that the A* search
// uses by default
constexpr double DT = 6.0;

std::optional<simulator::SimulatorState>
complete(const instances::GeneralPerformanceOptimizationInstance& instance,
         const std::optional<simulator::SimulatorState>&          state = {}) {
  simulator::GreedySimulator simulator(
      instance, instance.get_const_network().unbreakable_sections());
  return solver::astar_based::complete_state(
      simulator, state.value_or(simulator.get_simulator_state()), DT, false,
      true);
}

// Simulates a state as the A* search does by default
simulator::SimulatorResults
simulate(const instances::GeneralPerformanceOptimizationInstance& instance,
         const simulator::SimulatorState&                         state,
         simulator::SimulatorType                                 type) {
  const auto ttd_sections = instance.get_const_network().unbreakable_sections();
  if (type == simulator::SimulatorType::Greedy) {
    simulator::GreedySimulator greedy(instance, ttd_sections);
    greedy.set_simulator_state(state);
    return greedy.simulate(DT, false, true, false, true);
  }
  simulator::EventSimulator event(instance, ttd_sections);
  event.set_simulator_state(state);
  return event.simulate(DT, false, true, false, true);
}

// Whether the state is final and both simulators find it feasible
bool is_feasible(
    const instances::GeneralPerformanceOptimizationInstance& instance,
    const simulator::SimulatorState&                         state) {
  simulator::GreedySimulator greedy(
      instance, instance.get_const_network().unbreakable_sections());
  greedy.set_simulator_state(state);
  return greedy.is_final_state() &&
         simulate(instance, state, simulator::SimulatorType::Greedy).success &&
         simulate(instance, state, simulator::SimulatorType::EventBased)
             .success;
}

size_t ttd_of(const instances::GeneralPerformanceOptimizationInstance& instance,
              size_t                                                   edge) {
  return Network::get_ttd_of_edge(
             edge, instance.get_const_network().unbreakable_sections())
      .value();
}

// A line v0 -> v1 -> v2 -> v3, whose middle edge is the only track of
// station S, at which the single train stops.
struct LineInstance {
  instances::GeneralPerformanceOptimizationInstance instance;
  size_t                                            v0, v3, e01, e12, e23;
};
LineInstance line_instance() {
  Network    network;
  const auto v0  = network.add_vertex("v0", VertexType::TTD);
  const auto v1  = network.add_vertex("v1", VertexType::TTD);
  const auto v2  = network.add_vertex("v2", VertexType::TTD);
  const auto v3  = network.add_vertex("v3", VertexType::TTD);
  const auto e01 = network.add_edge(v0, v1, 1000, 20);
  const auto e12 = network.add_edge(v1, v2, 500, 20);
  const auto e23 = network.add_edge(v2, v3, 1000, 20);
  network.add_successor(e01, e12);
  network.add_successor(e12, e23);

  Timetable timetable;
  timetable.add_empty_station("S");
  timetable.add_track_to_station("S", e12, network);
  const auto tr = timetable.add_train("Train", 100, 20, 2, 4, true, 0, 10, v0,
                                      60, 20, v3, network);
  timetable.insert_stop(tr, "S", 0, 30);

  return {.instance = instances::GeneralPerformanceOptimizationInstance(
              network, timetable, RouteMap()),
          .v0  = v0,
          .v3  = v3,
          .e01 = e01,
          .e12 = e12,
          .e23 = e23};
}

// A line v0 -> v1 -> v3 -> v4, with the platforms a1 and b1 of station S
// between v1 and v3. The early and the late train both stop at S; the one
// with the smaller index enters later.
struct StationInstance {
  instances::GeneralPerformanceOptimizationInstance instance;
  size_t tr_late, tr_early, v0, in, a1, a2, b1, b2, out;
};
StationInstance station_instance() {
  Network    network;
  const auto v0  = network.add_vertex("v0", VertexType::TTD);
  const auto v1  = network.add_vertex("v1", VertexType::TTD);
  const auto v2a = network.add_vertex("v2a", VertexType::TTD);
  const auto v2b = network.add_vertex("v2b", VertexType::TTD);
  const auto v3  = network.add_vertex("v3", VertexType::TTD);
  const auto v4  = network.add_vertex("v4", VertexType::TTD);
  const auto in  = network.add_edge(v0, v1, 1000, 20);
  const auto a1  = network.add_edge(v1, v2a, 500, 20);
  const auto a2  = network.add_edge(v2a, v3, 100, 20);
  const auto b1  = network.add_edge(v1, v2b, 500, 20);
  const auto b2  = network.add_edge(v2b, v3, 100, 20);
  const auto out = network.add_edge(v3, v4, 1000, 20);
  network.add_successor(in, a1);
  network.add_successor(in, b1);
  network.add_successor(a1, a2);
  network.add_successor(b1, b2);
  network.add_successor(a2, out);
  network.add_successor(b2, out);

  Timetable timetable;
  timetable.add_empty_station("S");
  timetable.add_track_to_station("S", a1, network);
  timetable.add_track_to_station("S", b1, network);
  const auto tr_late = timetable.add_train("Late", 100, 20, 2, 4, true, 200, 10,
                                           v0, 300, 20, v4, network);
  const auto tr_early = timetable.add_train("Early", 100, 20, 2, 4, true, 0, 10,
                                            v0, 60, 20, v4, network);
  timetable.insert_stop(tr_late, "S", 0, 30);
  timetable.insert_stop(tr_early, "S", 0, 30);

  return {.instance = instances::GeneralPerformanceOptimizationInstance(
              network, timetable, RouteMap()),
          .tr_late  = tr_late,
          .tr_early = tr_early,
          .v0       = v0,
          .in       = in,
          .a1       = a1,
          .a2       = a2,
          .b1       = b1,
          .b2       = b2,
          .out      = out};
}

// Two trains A and B entering at v0a and v0b, whose lines merge in the switch
// s, after which they continue to v4. The switch is a TTD section.
struct MergeInstance {
  instances::GeneralPerformanceOptimizationInstance instance;
  size_t                                            tr_a, tr_b, merge;
};
MergeInstance merge_instance(double entry_a, double entry_b) {
  Network    network;
  const auto v0a  = network.add_vertex("v0a", VertexType::TTD);
  const auto v0b  = network.add_vertex("v0b", VertexType::TTD);
  const auto m1   = network.add_vertex("m1", VertexType::TTD);
  const auto m2   = network.add_vertex("m2", VertexType::TTD);
  const auto s    = network.add_vertex("s", VertexType::NoBorder);
  const auto m3   = network.add_vertex("m3", VertexType::TTD);
  const auto v4   = network.add_vertex("v4", VertexType::TTD);
  const auto in_a = network.add_edge(v0a, m1, 1000, 20);
  const auto in_b = network.add_edge(v0b, m2, 1000, 20);
  const auto sw_a = network.add_edge(m1, s, 10, 20, false);
  const auto sw_b = network.add_edge(m2, s, 10, 20, false);
  const auto sw_c = network.add_edge(s, m3, 10, 20, false);
  const auto out  = network.add_edge(m3, v4, 1000, 20);
  network.add_successor(in_a, sw_a);
  network.add_successor(in_b, sw_b);
  network.add_successor(sw_a, sw_c);
  network.add_successor(sw_b, sw_c);
  network.add_successor(sw_c, out);

  Timetable  timetable;
  const auto tr_a = timetable.add_train("TrainA", 100, 20, 2, 4, true, entry_a,
                                        10, v0a, 60, 20, v4, network);
  const auto tr_b = timetable.add_train("TrainB", 100, 20, 2, 4, true, entry_b,
                                        10, v0b, 60, 20, v4, network);

  instances::GeneralPerformanceOptimizationInstance instance(network, timetable,
                                                             RouteMap());
  const auto merge = ttd_of(instance, sw_c);
  return {.instance = instance, .tr_a = tr_a, .tr_b = tr_b, .merge = merge};
}

// Train 1 runs from vL_in to vR_out and train 2 from vR_in to vL_out. Both
// use the single track a <-> b in opposite directions, which lies between the
// switches sL and sR, each of which is a TTD section. Every entry is 500 m
// ahead of its switch, so that a train can wait for the other one, unless
// train 2 enters right at its switch.
struct OppositeInstance {
  instances::GeneralPerformanceOptimizationInstance instance;
  size_t                                            tr_1, tr_2, left, right;
};
OppositeInstance opposite_instance(double entry_1, double entry_2,
                                   bool train_2_enters_at_switch = false) {
  Network    network;
  const auto vl_in  = network.add_vertex("vL_in", VertexType::TTD);
  const auto vl_out = network.add_vertex("vL_out", VertexType::TTD);
  const auto vr_in  = network.add_vertex("vR_in", VertexType::TTD);
  const auto vr_out = network.add_vertex("vR_out", VertexType::TTD);
  const auto pl     = network.add_vertex("pL", VertexType::TTD);
  const auto ql     = network.add_vertex("qL", VertexType::TTD);
  const auto qr     = network.add_vertex("qR", VertexType::TTD);
  const auto sl     = network.add_vertex("sL", VertexType::NoBorder);
  const auto sr     = network.add_vertex("sR", VertexType::NoBorder);
  const auto a      = network.add_vertex("a", VertexType::TTD);
  const auto b      = network.add_vertex("b", VertexType::TTD);

  const auto in_l  = network.add_edge(vl_in, pl, 500, 20);
  const auto pl_sl = network.add_edge(pl, sl, 10, 20, false);
  const auto sl_a  = network.add_edge(sl, a, 10, 20, false);
  const auto a_sl  = network.add_edge(a, sl, 10, 20, false);
  const auto sl_ql = network.add_edge(sl, ql, 10, 20, false);
  const auto out_l = network.add_edge(ql, vl_out, 500, 20);
  const auto a_b   = network.add_edge(a, b, 2000, 20);
  const auto b_a   = network.add_edge(b, a, 2000, 20);
  const auto b_sr  = network.add_edge(b, sr, 10, 20, false);
  const auto sr_b  = network.add_edge(sr, b, 10, 20, false);
  const auto sr_qr = network.add_edge(sr, qr, 10, 20, false);
  const auto out_r = network.add_edge(qr, vr_out, 500, 20);
  if (train_2_enters_at_switch) {
    const auto in_r = network.add_edge(vr_in, sr, 10, 20, false);
    network.add_successor(in_r, sr_b);
  } else {
    const auto pr    = network.add_vertex("pR", VertexType::TTD);
    const auto in_r  = network.add_edge(vr_in, pr, 500, 20);
    const auto pr_sr = network.add_edge(pr, sr, 10, 20, false);
    network.add_successor(in_r, pr_sr);
    network.add_successor(pr_sr, sr_b);
  }

  network.add_successor(in_l, pl_sl);
  network.add_successor(pl_sl, sl_a);
  network.add_successor(sl_a, a_b);
  network.add_successor(a_b, b_sr);
  network.add_successor(b_sr, sr_qr);
  network.add_successor(sr_qr, out_r);
  network.add_successor(b_a, a_sl);
  network.add_successor(a_sl, sl_ql);
  network.add_successor(sl_ql, out_l);
  network.add_successor(sr_b, b_a);

  Timetable  timetable;
  const auto tr_1 = timetable.add_train("Train1", 100, 20, 2, 4, true, entry_1,
                                        10, vl_in, 60, 20, vr_out, network);
  const auto tr_2 = timetable.add_train("Train2", 100, 20, 2, 4, true, entry_2,
                                        10, vr_in, 60, 20, vl_out, network);

  instances::GeneralPerformanceOptimizationInstance instance(network, timetable,
                                                             RouteMap());
  const auto left  = ttd_of(instance, sl_a);
  const auto right = ttd_of(instance, b_sr);
  return {.instance = instance,
          .tr_1     = tr_1,
          .tr_2     = tr_2,
          .left     = left,
          .right    = right};
}
} // namespace

// ----------------------
// ROUTES AND STOPS
// ----------------------

TEST(CompletionHeuristic, SingleTrainOnALine) {
  const auto line      = line_instance();
  const auto completed = complete(line.instance);

  ASSERT_TRUE(completed.has_value());
  EXPECT_EQ(completed->train_edges.at(0),
            (index_vector{line.e01, line.e12, line.e23}));
  // At the end of the station track
  EXPECT_EQ(completed->stop_positions.at(0), std::vector<double>{1500});
  // The train is in the orders of its entry and its exit
  EXPECT_EQ(completed->vertex_orders.at(line.v0), index_vector{0});
  EXPECT_EQ(completed->vertex_orders.at(line.v3), index_vector{0});
  EXPECT_TRUE(is_feasible(line.instance, completed.value()));
}

TEST(CompletionHeuristic, KeepsPartialRouteStopsAndOrders) {
  const auto                 line = line_instance();
  simulator::GreedySimulator simulator(
      line.instance, line.instance.get_const_network().unbreakable_sections());

  // A partial route and the entry order are kept
  auto partial                      = simulator.get_simulator_state();
  partial.train_edges.at(0)         = {line.e01};
  partial.vertex_orders.at(line.v0) = {0};
  const auto completed              = complete(line.instance, partial);
  ASSERT_TRUE(completed.has_value());
  EXPECT_EQ(completed->train_edges.at(0),
            (index_vector{line.e01, line.e12, line.e23}));
  EXPECT_EQ(completed->stop_positions.at(0), std::vector<double>{1500});
  EXPECT_EQ(completed->vertex_orders.at(line.v0), index_vector{0});

  // A stop that has already been served is kept
  auto with_stop                 = partial;
  with_stop.train_edges.at(0)    = {line.e01, line.e12};
  with_stop.stop_positions.at(0) = {1500};
  const auto completed_with_stop = complete(line.instance, with_stop);
  ASSERT_TRUE(completed_with_stop.has_value());
  EXPECT_EQ(completed_with_stop->train_edges.at(0),
            (index_vector{line.e01, line.e12, line.e23}));
  EXPECT_EQ(completed_with_stop->stop_positions.at(0),
            std::vector<double>{1500});

  // A train that did not stop at the end of its route cannot stop there
  // later, and there is no further station track
  auto without_stop                 = with_stop;
  without_stop.stop_positions.at(0) = {};
  EXPECT_FALSE(solver::astar_based::complete_routes(line.instance, without_stop)
                   .has_value());
}

TEST(CompletionHeuristic, QuickestRatherThanShortestPath) {
  // From v1, the direct edge to v3 is shorter but slower than the detour via
  // v2
  Network    network;
  const auto v0     = network.add_vertex("v0", VertexType::TTD);
  const auto v1     = network.add_vertex("v1", VertexType::TTD);
  const auto v2     = network.add_vertex("v2", VertexType::TTD);
  const auto v3     = network.add_vertex("v3", VertexType::TTD);
  const auto v4     = network.add_vertex("v4", VertexType::TTD);
  const auto in     = network.add_edge(v0, v1, 1000, 20);
  const auto direct = network.add_edge(v1, v3, 500, 5);
  const auto via_1  = network.add_edge(v1, v2, 600, 20);
  const auto via_2  = network.add_edge(v2, v3, 600, 20);
  const auto out    = network.add_edge(v3, v4, 1000, 20);
  network.add_successor(in, direct);
  network.add_successor(in, via_1);
  network.add_successor(via_1, via_2);
  network.add_successor(direct, out);
  network.add_successor(via_2, out);

  Timetable  timetable;
  const auto tr = timetable.add_train("Train", 100, 20, 2, 4, true, 0, 10, v0,
                                      60, 20, v4, network);
  const instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, RouteMap());

  const auto completed = complete(instance);
  ASSERT_TRUE(completed.has_value());
  EXPECT_EQ(completed->train_edges.at(tr),
            (index_vector{in, via_1, via_2, out}));
}

TEST(CompletionHeuristic, PlatformsInTurnsByEntryTime) {
  const auto station   = station_instance();
  const auto completed = complete(station.instance);
  ASSERT_TRUE(completed.has_value());
  EXPECT_EQ(completed->train_edges.at(station.tr_early),
            (index_vector{station.in, station.a1, station.a2, station.out}));
  EXPECT_EQ(completed->train_edges.at(station.tr_late),
            (index_vector{station.in, station.b1, station.b2, station.out}));
  EXPECT_EQ(completed->stop_positions.at(station.tr_early),
            std::vector<double>{1500});
  EXPECT_EQ(completed->stop_positions.at(station.tr_late),
            std::vector<double>{1500});
  EXPECT_TRUE(is_feasible(station.instance, completed.value()));
}

TEST(CompletionHeuristic, PlatformsOfThePartialStateCountForTheTurns) {
  // The partial state lets the early train stop at platform a1, so that the
  // late one, whose stop the completion chooses first, takes b1
  const auto                 station = station_instance();
  simulator::GreedySimulator simulator(
      station.instance,
      station.instance.get_const_network().unbreakable_sections());
  auto partial                                = simulator.get_simulator_state();
  partial.train_edges.at(station.tr_early)    = {station.in, station.a1};
  partial.stop_positions.at(station.tr_early) = {1500};
  partial.vertex_orders.at(station.v0)        = {station.tr_early};

  const auto completed = complete(station.instance, partial);
  ASSERT_TRUE(completed.has_value());
  EXPECT_EQ(completed->train_edges.at(station.tr_early),
            (index_vector{station.in, station.a1, station.a2, station.out}));
  EXPECT_EQ(completed->train_edges.at(station.tr_late),
            (index_vector{station.in, station.b1, station.b2, station.out}));
  EXPECT_TRUE(is_feasible(station.instance, completed.value()));
}

// ---------------------------------
// FIRST COME, FIRST SERVED ORDERS
// ---------------------------------

TEST(CompletionHeuristic, FirstComeFirstServedSameDirection) {
  // The train entering first gets to the merging switch first
  const auto a_first           = merge_instance(0, 30);
  const auto completed_a_first = complete(a_first.instance);
  ASSERT_TRUE(completed_a_first.has_value());
  EXPECT_EQ(completed_a_first->ttd_orders.at(a_first.merge),
            (index_vector{a_first.tr_a, a_first.tr_b}));
  EXPECT_TRUE(is_feasible(a_first.instance, completed_a_first.value()));

  const auto b_first           = merge_instance(30, 0);
  const auto completed_b_first = complete(b_first.instance);
  ASSERT_TRUE(completed_b_first.has_value());
  EXPECT_EQ(completed_b_first->ttd_orders.at(b_first.merge),
            (index_vector{b_first.tr_b, b_first.tr_a}));
  EXPECT_TRUE(is_feasible(b_first.instance, completed_b_first.value()));
}

TEST(CompletionHeuristic, OrdersOfThePartialStateAreKept) {
  // Train B is to pass the switch first, although train A gets there first
  const auto                 merge = merge_instance(0, 30);
  simulator::GreedySimulator simulator(
      merge.instance,
      merge.instance.get_const_network().unbreakable_sections());
  auto partial                       = simulator.get_simulator_state();
  partial.ttd_orders.at(merge.merge) = {merge.tr_b, merge.tr_a};

  const auto completed = complete(merge.instance, partial);
  ASSERT_TRUE(completed.has_value());
  EXPECT_EQ(completed->ttd_orders.at(merge.merge),
            (index_vector{merge.tr_b, merge.tr_a}));
  EXPECT_TRUE(is_feasible(merge.instance, completed.value()));
}

TEST(CompletionHeuristic, TrainsGetAheadOfTrainsOfThePartialStateComingLater) {
  // Train B is in the order of the switch, but train A gets there first
  const auto                 merge = merge_instance(0, 30);
  simulator::GreedySimulator merge_simulator(
      merge.instance,
      merge.instance.get_const_network().unbreakable_sections());
  auto merge_partial = merge_simulator.get_simulator_state();
  merge_partial.ttd_orders.at(merge.merge) = {merge.tr_b};

  const auto merge_completed = complete(merge.instance, merge_partial);
  ASSERT_TRUE(merge_completed.has_value());
  EXPECT_EQ(merge_completed->ttd_orders.at(merge.merge),
            (index_vector{merge.tr_a, merge.tr_b}));
  EXPECT_TRUE(is_feasible(merge.instance, merge_completed.value()));

  // The late train is in the order of the entry, behind which the early one
  // could not enter in time
  const auto                 station = station_instance();
  simulator::GreedySimulator station_simulator(
      station.instance,
      station.instance.get_const_network().unbreakable_sections());
  auto station_partial = station_simulator.get_simulator_state();
  station_partial.vertex_orders.at(station.v0) = {station.tr_late};

  const auto station_completed = complete(station.instance, station_partial);
  ASSERT_TRUE(station_completed.has_value());
  EXPECT_EQ(station_completed->vertex_orders.at(station.v0),
            (index_vector{station.tr_early, station.tr_late}));
  EXPECT_TRUE(is_feasible(station.instance, station_completed.value()));
}

TEST(CompletionHeuristic, FirstComeFirstServedOppositeDirections) {
  // The train getting to the single track first gets both of its ends, while
  // the other one waits in front of its switch
  const auto first_1     = opposite_instance(0, 20);
  const auto completed_1 = complete(first_1.instance);
  ASSERT_TRUE(completed_1.has_value());
  EXPECT_EQ(completed_1->ttd_orders.at(first_1.left),
            (index_vector{first_1.tr_1, first_1.tr_2}));
  EXPECT_EQ(completed_1->ttd_orders.at(first_1.right),
            (index_vector{first_1.tr_1, first_1.tr_2}));
  EXPECT_TRUE(is_feasible(first_1.instance, completed_1.value()));

  const auto first_2     = opposite_instance(20, 0);
  const auto completed_2 = complete(first_2.instance);
  ASSERT_TRUE(completed_2.has_value());
  EXPECT_EQ(completed_2->ttd_orders.at(first_2.left),
            (index_vector{first_2.tr_2, first_2.tr_1}));
  EXPECT_EQ(completed_2->ttd_orders.at(first_2.right),
            (index_vector{first_2.tr_2, first_2.tr_1}));
  EXPECT_TRUE(is_feasible(first_2.instance, completed_2.value()));
}

TEST(CompletionHeuristic, StretchOfWhichEachTrainHoldsOneEnd) {
  // By the partial state, as if each partial route ended in the switch at its
  // end of the single track, each train is in the order of that switch only.
  // The train getting to the single track first still gets both switches.
  for (const auto& [entry_1, entry_2] :
       {std::pair{0.0, 20.0}, std::pair{20.0, 0.0}}) {
    const auto                 opposite = opposite_instance(entry_1, entry_2);
    simulator::GreedySimulator simulator(
        opposite.instance,
        opposite.instance.get_const_network().unbreakable_sections());
    auto partial                          = simulator.get_simulator_state();
    partial.ttd_orders.at(opposite.left)  = {opposite.tr_1};
    partial.ttd_orders.at(opposite.right) = {opposite.tr_2};

    const auto completed = complete(opposite.instance, partial);
    ASSERT_TRUE(completed.has_value());
    const auto expected_order =
        entry_1 < entry_2 ? index_vector{opposite.tr_1, opposite.tr_2}
                          : index_vector{opposite.tr_2, opposite.tr_1};
    EXPECT_EQ(completed->ttd_orders.at(opposite.left), expected_order);
    EXPECT_EQ(completed->ttd_orders.at(opposite.right), expected_order);
    EXPECT_TRUE(is_feasible(opposite.instance, completed.value()));
  }
}

TEST(CompletionHeuristic, DeadlockYieldsNothing) {
  // By the partial state, each train is the first at the switch it gets to
  // first, so that they meet on the single track
  const auto                 opposite = opposite_instance(0, 20);
  simulator::GreedySimulator simulator(
      opposite.instance,
      opposite.instance.get_const_network().unbreakable_sections());
  auto partial                          = simulator.get_simulator_state();
  partial.ttd_orders.at(opposite.left)  = {opposite.tr_1, opposite.tr_2};
  partial.ttd_orders.at(opposite.right) = {opposite.tr_2, opposite.tr_1};

  EXPECT_TRUE(solver::astar_based::complete_routes(opposite.instance, partial)
                  .has_value());
  EXPECT_FALSE(complete(opposite.instance, partial).has_value());
}

TEST(CompletionHeuristic, OnlyTheGreedySimulatorDecidesOrders) {
  const auto                line = line_instance();
  simulator::EventSimulator simulator(
      line.instance, line.instance.get_const_network().unbreakable_sections());
  EXPECT_THROW(simulator.first_come_first_served_state(DT, false, true),
               exceptions::ConsistencyException);
}

// ----------------------
// A* SEARCH
// ----------------------

// As SOLVER_TEST in test_mb_astar.cpp, for both simulators
#define COMPLETION_SOLVER_TEST(Name)                                           \
  static void Name##_with(simulator::SimulatorType simulator_type);            \
  TEST(GenPOMovingBlockAStarSolverCompletion, Name) {                          \
    Name##_with(simulator::SimulatorType::Greedy);                             \
  }                                                                            \
  TEST(GenPOMovingBlockAStarSolverCompletionWithEventSimulator, Name) {        \
    Name##_with(simulator::SimulatorType::EventBased);                         \
  }                                                                            \
  static void Name##_with(simulator::SimulatorType simulator_type)

COMPLETION_SOLVER_TEST(SameResultAsWithoutCompletions) {
  for (const auto& instance :
       {merge_instance(0, 30).instance, merge_instance(30, 0).instance,
        opposite_instance(0, 20).instance, line_instance().instance}) {
    solver::astar_based::GenPOMovingBlockAStarSolver solver(instance);
    for (const auto next_state_strategy :
         {solver::astar_based::NextStateStrategy::SingleEdge,
          solver::astar_based::NextStateStrategy::NextTTD}) {
      const auto without = solver.solve(
          {.simulator_type = simulator_type},
          {.next_state_strategy = next_state_strategy}, {}, -1, false);
      for (const size_t interval : {1, 3}) {
        const auto with =
            solver.solve({.simulator_type = simulator_type},
                         {.next_state_strategy = next_state_strategy,
                          .completion_interval = interval},
                         {}, -1, false);
        EXPECT_EQ(without.get_status(), SolutionStatus::Optimal);
        EXPECT_EQ(with.get_status(), without.get_status());
        EXPECT_DOUBLE_EQ(with.get_obj(), without.get_obj());
      }
    }
  }
}

COMPLETION_SOLVER_TEST(SearchContinuesAfterFailedCompletion) {
  // Train 1 gets to the single track first, but without late entry, train 2
  // cannot wait for it, since it enters right at its switch
  const auto opposite = opposite_instance(0, 30, true);
  EXPECT_FALSE(complete(opposite.instance).has_value());

  solver::astar_based::GenPOMovingBlockAStarSolver solver(opposite.instance);
  const auto                                       without =
      solver.solve({.simulator_type = simulator_type}, {}, {}, -1, false);
  const auto with = solver.solve({.simulator_type = simulator_type},
                                 {.completion_interval = 1}, {}, -1, false);
  EXPECT_EQ(without.get_status(), SolutionStatus::Optimal);
  EXPECT_EQ(with.get_status(), SolutionStatus::Optimal);
  EXPECT_DOUBLE_EQ(with.get_obj(), without.get_obj());
}

COMPLETION_SOLVER_TEST(IncumbentOfTheInitialCompletion) {
  const instances::GeneralPerformanceOptimizationInstance instance(
      "GeneralStammstrecke6Trains", "gen-po", "./data");
  const auto completed = complete(instance);
  ASSERT_TRUE(completed.has_value());
  const auto results = simulate(instance, completed.value(), simulator_type);
  ASSERT_TRUE(results.success);

  // Within a second, the search finds no solution by itself, but returns the
  // completed initial state, since no other state is completed
  solver::astar_based::GenPOMovingBlockAStarSolver solver(instance);
  const auto                                       without = solver.solve(
      {.simulator_type = simulator_type},
      {.next_state_strategy = solver::astar_based::NextStateStrategy::NextTTD},
      {}, 1, false);
  EXPECT_FALSE(without.has_solution());
  EXPECT_EQ(without.get_status(), SolutionStatus::Timeout);

  const auto with = solver.solve(
      {.simulator_type = simulator_type},
      {.next_state_strategy = solver::astar_based::NextStateStrategy::NextTTD,
       .completion_interval = 1000000},
      {}, 1, false);
  EXPECT_TRUE(with.has_solution());
  EXPECT_EQ(with.get_status(), SolutionStatus::Feasible);
  EXPECT_DOUBLE_EQ(with.get_obj(),
                   simulator::objective_val(instance, results.exit_times,
                                            results.stop_times));
}

// NOLINTEND
// (clang-analyzer-deadcode.DeadStores,misc-const-correctness,clang-diagnostic-unused-result)
