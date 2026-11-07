#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "datastructure/RailwayNetwork.hpp"
#include "datastructure/Route.hpp"
#include "datastructure/Timetable.hpp"
#include "nlohmann/json.hpp"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"
#include "simulator/EventSimulator.hpp"
#include "simulator/GeneralSimulator.hpp"
#include "simulator/GreedyHeuristic.hpp"
#include "simulator/GreedySimulator.hpp"
#include "solver/GeneralSolver.hpp"
#include "solver/astar-based/CompletionHeuristic.hpp"
#include "solver/astar-based/GenPOMovingBlockAStarSolver.hpp"

#include "gtest/gtest.h"
#include <algorithm>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <optional>
#include <vector>

using namespace cda_rail;
// NOLINTBEGIN
// (clang-analyzer-deadcode.DeadStores,misc-const-correctness,clang-diagnostic-unused-result)

namespace {
// The parameters of simulator::GeneralSimulator::simulate that the A* search
// uses by default
constexpr double DT = 6.0;

// Whether the trains of every order of the given state pass in the same order
// in the completed state
bool keeps_orders(const simulator::SimulatorState& given,
                  const simulator::SimulatorState& completed) {
  const auto keeps = [](const index_vector& given_order,
                        const index_vector& order) {
    auto it = order.begin();
    for (const auto tr : given_order) {
      it = std::find(it, order.end(), tr);
      if (it == order.end()) {
        return false;
      }
      ++it;
    }
    return true;
  };
  for (size_t v = 0; v < given.vertex_orders.size(); ++v) {
    if (!keeps(given.vertex_orders.at(v), completed.vertex_orders.at(v))) {
      return false;
    }
  }
  for (size_t ttd = 0; ttd < given.ttd_orders.size(); ++ttd) {
    if (!keeps(given.ttd_orders.at(ttd), completed.ttd_orders.at(ttd))) {
      return false;
    }
  }
  return true;
}

// The completion of the given state, which keeps its orders
std::optional<simulator::SimulatorState>
complete(const instances::GeneralPerformanceOptimizationInstance& instance,
         const std::optional<simulator::SimulatorState>&          state = {}) {
  simulator::GreedySimulator simulator(
      instance, instance.get_const_network().unbreakable_sections());
  const auto given = state.value_or(simulator.get_simulator_state());
  auto       completed =
      solver::astar_based::complete_state(simulator, given, DT, false, true);
  EXPECT_TRUE(!completed.has_value() || keeps_orders(given, completed.value()));
  return completed;
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

// The solver data of a solution exported with the given settings, read as in
// GenPOMovingBlockAStarSolver.SolverDataExport. The export is removed again.
nlohmann::json exported_solver_data(
    const instances::SolGeneralPerformanceOptimizationInstance& solution,
    const solver::GeneralSolutionSettings&                      settings) {
  const auto    path = solution.get_export_path(settings.working_directory,
                                                settings.solution_subdirectory,
                                                settings.parameter_identifier) /
                       "solver_data.json";
  std::ifstream data_file(path);
  if (!data_file.is_open()) {
    throw exceptions::ImportException("Could not open file " + path.string());
  }
  nlohmann::json data;
  data_file >> data;
  data_file.close();
  std::filesystem::remove_all(settings.working_directory);
  return data;
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
// between v1 and v3, of which b1 is 500 m long. With two_track_platforms, the
// tracks a2 and b2 behind them belong to S as well, so that each platform
// consists of two tracks. The early and the late train of 100 m both stop at
// S; the one with the smaller index enters later.
struct StationInstance {
  instances::GeneralPerformanceOptimizationInstance instance;
  size_t tr_late, tr_early, v0, in, a1, a2, b1, b2, out;
};
StationInstance station_instance(double a1_length           = 500,
                                 bool   two_track_platforms = false) {
  Network    network;
  const auto v0  = network.add_vertex("v0", VertexType::TTD);
  const auto v1  = network.add_vertex("v1", VertexType::TTD);
  const auto v2a = network.add_vertex("v2a", VertexType::TTD);
  const auto v2b = network.add_vertex("v2b", VertexType::TTD);
  const auto v3  = network.add_vertex("v3", VertexType::TTD);
  const auto v4  = network.add_vertex("v4", VertexType::TTD);
  const auto in  = network.add_edge(v0, v1, 1000, 20);
  const auto a1  = network.add_edge(v1, v2a, a1_length, 20);
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
  if (two_track_platforms) {
    timetable.add_track_to_station("S", a2, network);
    timetable.add_track_to_station("S", b2, network);
  }
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
// switches sL and sR, each of which is a TTD section. The entry of train 1 is
// approach_1 ahead of its switch, the one of train 2 500 m, so that a train
// can wait for the other one, unless train 2 enters right at its switch.
struct OppositeInstance {
  instances::GeneralPerformanceOptimizationInstance instance;
  size_t                                            tr_1, tr_2, left, right;
};
OppositeInstance opposite_instance(double entry_1, double entry_2,
                                   bool   train_2_enters_at_switch = false,
                                   double approach_1               = 500) {
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

  const auto in_l  = network.add_edge(vl_in, pl, approach_1, 20);
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

// Train 1 runs from vL_in to vR_out and train 2 from vR_in to vL_out, both
// entering at the same time right at their switches sL and sR, so that
// neither can wait for the other. The single tracks a <-> c and d <-> b lead
// to a passing loop between the switches s1 and s2, whose track ux <-> vx is
// shorter than uy <-> vy. Every switch is a TTD section.
struct PassingLoopInstance {
  instances::GeneralPerformanceOptimizationInstance instance;
  size_t                                            tr_1, tr_2;
};
PassingLoopInstance passing_loop_instance() {
  Network    network;
  const auto vl_in  = network.add_vertex("vL_in", VertexType::TTD);
  const auto vl_out = network.add_vertex("vL_out", VertexType::TTD);
  const auto vr_in  = network.add_vertex("vR_in", VertexType::TTD);
  const auto vr_out = network.add_vertex("vR_out", VertexType::TTD);
  const auto ql     = network.add_vertex("qL", VertexType::TTD);
  const auto qr     = network.add_vertex("qR", VertexType::TTD);
  const auto sl     = network.add_vertex("sL", VertexType::NoBorder);
  const auto sr     = network.add_vertex("sR", VertexType::NoBorder);
  const auto s1     = network.add_vertex("s1", VertexType::NoBorder);
  const auto s2     = network.add_vertex("s2", VertexType::NoBorder);
  const auto a      = network.add_vertex("a", VertexType::TTD);
  const auto b      = network.add_vertex("b", VertexType::TTD);
  const auto c      = network.add_vertex("c", VertexType::TTD);
  const auto d      = network.add_vertex("d", VertexType::TTD);
  const auto ux     = network.add_vertex("ux", VertexType::TTD);
  const auto vx     = network.add_vertex("vx", VertexType::TTD);
  const auto uy     = network.add_vertex("uy", VertexType::TTD);
  const auto vy     = network.add_vertex("vy", VertexType::TTD);

  // The edges of train 1
  const auto in_l  = network.add_edge(vl_in, sl, 10, 20, false);
  const auto sl_a  = network.add_edge(sl, a, 10, 20, false);
  const auto a_c   = network.add_edge(a, c, 1000, 20);
  const auto c_s1  = network.add_edge(c, s1, 10, 20, false);
  const auto s1_ux = network.add_edge(s1, ux, 10, 20, false);
  const auto s1_uy = network.add_edge(s1, uy, 10, 20, false);
  const auto ux_vx = network.add_edge(ux, vx, 500, 20);
  const auto uy_vy = network.add_edge(uy, vy, 700, 20);
  const auto vx_s2 = network.add_edge(vx, s2, 10, 20, false);
  const auto vy_s2 = network.add_edge(vy, s2, 10, 20, false);
  const auto s2_d  = network.add_edge(s2, d, 10, 20, false);
  const auto d_b   = network.add_edge(d, b, 1000, 20);
  const auto b_sr  = network.add_edge(b, sr, 10, 20, false);
  const auto sr_qr = network.add_edge(sr, qr, 10, 20, false);
  const auto out_r = network.add_edge(qr, vr_out, 500, 20);
  network.add_successor(in_l, sl_a);
  network.add_successor(sl_a, a_c);
  network.add_successor(a_c, c_s1);
  network.add_successor(c_s1, s1_ux);
  network.add_successor(c_s1, s1_uy);
  network.add_successor(s1_ux, ux_vx);
  network.add_successor(s1_uy, uy_vy);
  network.add_successor(ux_vx, vx_s2);
  network.add_successor(uy_vy, vy_s2);
  network.add_successor(vx_s2, s2_d);
  network.add_successor(vy_s2, s2_d);
  network.add_successor(s2_d, d_b);
  network.add_successor(d_b, b_sr);
  network.add_successor(b_sr, sr_qr);
  network.add_successor(sr_qr, out_r);

  // The edges of train 2
  const auto in_r  = network.add_edge(vr_in, sr, 10, 20, false);
  const auto sr_b  = network.add_edge(sr, b, 10, 20, false);
  const auto b_d   = network.add_edge(b, d, 1000, 20);
  const auto d_s2  = network.add_edge(d, s2, 10, 20, false);
  const auto s2_vx = network.add_edge(s2, vx, 10, 20, false);
  const auto s2_vy = network.add_edge(s2, vy, 10, 20, false);
  const auto vx_ux = network.add_edge(vx, ux, 500, 20);
  const auto vy_uy = network.add_edge(vy, uy, 700, 20);
  const auto ux_s1 = network.add_edge(ux, s1, 10, 20, false);
  const auto uy_s1 = network.add_edge(uy, s1, 10, 20, false);
  const auto s1_c  = network.add_edge(s1, c, 10, 20, false);
  const auto c_a   = network.add_edge(c, a, 1000, 20);
  const auto a_sl  = network.add_edge(a, sl, 10, 20, false);
  const auto sl_ql = network.add_edge(sl, ql, 10, 20, false);
  const auto out_l = network.add_edge(ql, vl_out, 500, 20);
  network.add_successor(in_r, sr_b);
  network.add_successor(sr_b, b_d);
  network.add_successor(b_d, d_s2);
  network.add_successor(d_s2, s2_vx);
  network.add_successor(d_s2, s2_vy);
  network.add_successor(s2_vx, vx_ux);
  network.add_successor(s2_vy, vy_uy);
  network.add_successor(vx_ux, ux_s1);
  network.add_successor(vy_uy, uy_s1);
  network.add_successor(ux_s1, s1_c);
  network.add_successor(uy_s1, s1_c);
  network.add_successor(s1_c, c_a);
  network.add_successor(c_a, a_sl);
  network.add_successor(a_sl, sl_ql);
  network.add_successor(sl_ql, out_l);

  Timetable  timetable;
  const auto tr_1 = timetable.add_train("Train1", 100, 20, 2, 4, true, 0, 10,
                                        vl_in, 60, 20, vr_out, network);
  const auto tr_2 = timetable.add_train("Train2", 100, 20, 2, 4, true, 0, 10,
                                        vr_in, 60, 20, vl_out, network);
  return {.instance = instances::GeneralPerformanceOptimizationInstance(
              network, timetable, RouteMap()),
          .tr_1 = tr_1,
          .tr_2 = tr_2};
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
  EXPECT_FALSE(complete(line.instance, without_stop).has_value());
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

TEST(CompletionHeuristic, PartialRoutesThatCannotBeCompleted) {
  // From the line v0 -> v1 -> v2 -> v4, the loop v2 -> v3 -> v1 branches off,
  // from which a dead end leads to v5
  Network    network;
  const auto v0       = network.add_vertex("v0", VertexType::TTD);
  const auto v1       = network.add_vertex("v1", VertexType::TTD);
  const auto v2       = network.add_vertex("v2", VertexType::TTD);
  const auto v3       = network.add_vertex("v3", VertexType::TTD);
  const auto v4       = network.add_vertex("v4", VertexType::TTD);
  const auto v5       = network.add_vertex("v5", VertexType::TTD);
  const auto in       = network.add_edge(v0, v1, 1000, 20);
  const auto mid      = network.add_edge(v1, v2, 500, 20);
  const auto out      = network.add_edge(v2, v4, 1000, 20);
  const auto loop_1   = network.add_edge(v2, v3, 500, 20);
  const auto loop_2   = network.add_edge(v3, v1, 500, 20);
  const auto dead_end = network.add_edge(v3, v5, 1000, 20);
  network.add_successor(in, mid);
  network.add_successor(mid, out);
  network.add_successor(mid, loop_1);
  network.add_successor(loop_1, loop_2);
  network.add_successor(loop_1, dead_end);
  network.add_successor(loop_2, mid);

  Timetable  timetable;
  const auto tr = timetable.add_train("Train", 100, 20, 2, 4, true, 0, 10, v0,
                                      60, 20, v4, network);
  const instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, RouteMap());
  simulator::GreedySimulator simulator(
      instance, instance.get_const_network().unbreakable_sections());
  const auto initial = simulator.get_simulator_state();

  const auto completed =
      solver::astar_based::complete_routes(instance, initial);
  ASSERT_TRUE(completed.has_value());
  EXPECT_EQ(completed->train_edges.at(tr), (index_vector{in, mid, out}));

  // From the dead end, the exit cannot be reached
  auto in_dead_end               = initial;
  in_dead_end.train_edges.at(tr) = {in, mid, loop_1, dead_end};
  EXPECT_FALSE(
      solver::astar_based::complete_routes(instance, in_dead_end).has_value());

  // Around the loop, the way to the exit uses mid a second time
  auto around_loop               = initial;
  around_loop.train_edges.at(tr) = {in, mid, loop_1, loop_2};
  EXPECT_FALSE(
      solver::astar_based::complete_routes(instance, around_loop).has_value());
}

TEST(CompletionHeuristic, PlatformsInTurnsByEntryTime) {
  // Where a platform consists of two tracks, the late train does not take the
  // track a2 of the platform that the early train used, and both stop at the
  // end of their platform
  for (const bool two_track_platforms : {false, true}) {
    const auto station   = station_instance(500, two_track_platforms);
    const auto completed = complete(station.instance);
    ASSERT_TRUE(completed.has_value());
    EXPECT_EQ(completed->train_edges.at(station.tr_early),
              (index_vector{station.in, station.a1, station.a2, station.out}));
    EXPECT_EQ(completed->train_edges.at(station.tr_late),
              (index_vector{station.in, station.b1, station.b2, station.out}));
    const double stop = two_track_platforms ? 1600 : 1500;
    EXPECT_EQ(completed->stop_positions.at(station.tr_early),
              std::vector<double>{stop});
    EXPECT_EQ(completed->stop_positions.at(station.tr_late),
              std::vector<double>{stop});
    EXPECT_TRUE(is_feasible(station.instance, completed.value()));
  }
}

TEST(CompletionHeuristic, PlatformLeftInTheMiddle) {
  // The platform v1 -> v2 -> v3a of station S can be left at v2 towards v4b.
  // Train A to v4a stops at the end of the platform, while train B to v4b
  // stops at v2, since it cannot continue from the end.
  Network    network;
  const auto v0   = network.add_vertex("v0", VertexType::TTD);
  const auto v1   = network.add_vertex("v1", VertexType::TTD);
  const auto v2   = network.add_vertex("v2", VertexType::TTD);
  const auto v3a  = network.add_vertex("v3a", VertexType::TTD);
  const auto v3b  = network.add_vertex("v3b", VertexType::TTD);
  const auto v4a  = network.add_vertex("v4a", VertexType::TTD);
  const auto v4b  = network.add_vertex("v4b", VertexType::TTD);
  const auto e01  = network.add_edge(v0, v1, 1000, 20);
  const auto e12  = network.add_edge(v1, v2, 500, 20);
  const auto e23a = network.add_edge(v2, v3a, 500, 20);
  const auto e23b = network.add_edge(v2, v3b, 500, 20);
  const auto e34a = network.add_edge(v3a, v4a, 1000, 20);
  const auto e34b = network.add_edge(v3b, v4b, 1000, 20);
  network.add_successor(e01, e12);
  network.add_successor(e12, e23a);
  network.add_successor(e12, e23b);
  network.add_successor(e23a, e34a);
  network.add_successor(e23b, e34b);

  Timetable timetable;
  timetable.add_empty_station("S");
  timetable.add_track_to_station("S", e12, network);
  timetable.add_track_to_station("S", e23a, network);
  const auto tr_a = timetable.add_train("A", 100, 20, 2, 4, true, 0, 10, v0, 60,
                                        20, v4a, network);
  const auto tr_b = timetable.add_train("B", 100, 20, 2, 4, true, 300, 10, v0,
                                        360, 20, v4b, network);
  timetable.insert_stop(tr_a, "S", 0, 30);
  timetable.insert_stop(tr_b, "S", 0, 30);
  const instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, RouteMap());

  const auto completed = complete(instance);
  ASSERT_TRUE(completed.has_value());
  EXPECT_EQ(completed->stop_positions.at(tr_a), std::vector<double>{2000});
  EXPECT_EQ(completed->stop_positions.at(tr_b), std::vector<double>{1500});
  EXPECT_TRUE(is_feasible(instance, completed.value()));
}

TEST(CompletionHeuristic, PlatformsTooShortForTheTrainAreSkipped) {
  // Platform a1 is shorter than the trains, so that both stop at b1, although
  // a1 would be the turn of one of them
  const auto station   = station_instance(50);
  const auto completed = complete(station.instance);
  ASSERT_TRUE(completed.has_value());
  for (const auto tr : {station.tr_early, station.tr_late}) {
    EXPECT_EQ(completed->train_edges.at(tr),
              (index_vector{station.in, station.b1, station.b2, station.out}));
    EXPECT_EQ(completed->stop_positions.at(tr), std::vector<double>{1500});
  }
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

  // Train 1 enters first, but 3000 m ahead of its switch, so that train 2
  // gets to the single track first
  const auto first_far     = opposite_instance(0, 20, false, 3000);
  const auto completed_far = complete(first_far.instance);
  ASSERT_TRUE(completed_far.has_value());
  EXPECT_EQ(completed_far->ttd_orders.at(first_far.left),
            (index_vector{first_far.tr_2, first_far.tr_1}));
  EXPECT_EQ(completed_far->ttd_orders.at(first_far.right),
            (index_vector{first_far.tr_2, first_far.tr_1}));
  EXPECT_TRUE(is_feasible(first_far.instance, completed_far.value()));
}

TEST(CompletionHeuristic, TrainMissingItsEntryPassesFirst) {
  // Train 1 gets to the single track first, but train 2 enters right at its
  // switch, so that it cannot wait for train 1 without entering late. Hence,
  // train 2 passes first, while train 1 waits in front of its switch.
  const auto opposite  = opposite_instance(0, 30, true);
  const auto completed = complete(opposite.instance);
  ASSERT_TRUE(completed.has_value());
  EXPECT_EQ(completed->ttd_orders.at(opposite.left),
            (index_vector{opposite.tr_2, opposite.tr_1}));
  EXPECT_EQ(completed->ttd_orders.at(opposite.right),
            (index_vector{opposite.tr_2, opposite.tr_1}));
  EXPECT_TRUE(is_feasible(opposite.instance, completed.value()));

  // Unless the given state lets train 1 pass first
  simulator::GreedySimulator simulator(
      opposite.instance,
      opposite.instance.get_const_network().unbreakable_sections());
  auto partial                         = simulator.get_simulator_state();
  partial.ttd_orders.at(opposite.left) = {opposite.tr_1, opposite.tr_2};
  EXPECT_FALSE(complete(opposite.instance, partial).has_value());
}

TEST(CompletionHeuristic, TrainMissingItsEntryWithoutTrainAheadFails) {
  // A position bound keeps the only train from entering. No train is ahead of
  // it at its entry vertex, hence there is none that could let it pass first.
  const auto                 line = line_instance();
  simulator::GreedySimulator simulator(
      line.instance, line.instance.get_const_network().unbreakable_sections());
  simulator.set_position_bound(
      [](size_t /*tr*/, double /*t*/)
          -> std::optional<simulator::GreedySimulator::PositionBound> {
        return simulator::GreedySimulator::PositionBound{.front         = 0,
                                                         .braking_point = 0};
      });

  EXPECT_FALSE(solver::astar_based::complete_state(
                   simulator, simulator.get_simulator_state(), DT, false, true)
                   .has_value());
}

TEST(CompletionHeuristic, PlatformsAreUsedInBothDirections) {
  // tr2 and tr3 enter at the same time from opposite ends of the line through
  // station Central, at which they can pass each other only on different
  // platforms. Hence, tr3 does not take the reverse of the platform of tr2.
  const instances::GeneralPerformanceOptimizationInstance instance(
      "SimpleStation", "atmos2023", "./data");
  const auto completed = complete(instance);
  ASSERT_TRUE(completed.has_value());
  const auto& network = instance.get_const_network();
  const auto& platforms =
      instance.get_const_station_list().get_station("Central").tracks;
  const auto& route_2 = completed->train_edges.at(
      instance.get_const_train_list().get_train_index("tr2"));
  for (const auto e : completed->train_edges.at(
           instance.get_const_train_list().get_train_index("tr3"))) {
    const auto reverse = network.get_reverse_edge_index(e);
    EXPECT_FALSE(platforms.contains(e) && reverse.has_value() &&
                 std::ranges::contains(route_2, reverse.value()));
  }
  EXPECT_TRUE(is_feasible(instance, completed.value()));
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

TEST(CompletionHeuristic, FirstComeFirstServedLeavesOutTrainsWithoutRoute) {
  // Train B does not enter, so that only train A is in an order
  const auto                 merge = merge_instance(0, 30);
  simulator::GreedySimulator simulator(
      merge.instance,
      merge.instance.get_const_network().unbreakable_sections());
  auto state = solver::astar_based::complete_routes(
      merge.instance, simulator.get_simulator_state());
  ASSERT_TRUE(state.has_value());
  state->train_edges.at(merge.tr_b).clear();
  simulator.set_simulator_state(state.value());

  const auto decided = simulator.first_come_first_served_state(DT, false, true);
  ASSERT_TRUE(decided.has_value());
  EXPECT_EQ(decided->ttd_orders.at(merge.merge), index_vector{merge.tr_a});
  for (const auto& orders : {decided->ttd_orders, decided->vertex_orders}) {
    for (const auto& order : orders) {
      EXPECT_FALSE(std::ranges::contains(order, merge.tr_b));
    }
  }
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
  // The solver data are exported to a folder of their own per simulator, so
  // that both tests can run in parallel
  const solver::GeneralSolutionSettings export_settings{
      .export_option        = solver::GeneralExportOption::ExportSolution,
      .working_directory    = simulator_type == simulator::SimulatorType::Greedy
                                  ? "tmp_completion_greedy"
                                  : "tmp_completion_event",
      .parameter_identifier = "completion"};
  int most_completions = 0;
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
                         export_settings, -1, false);
        EXPECT_EQ(without.get_status(), SolutionStatus::Optimal);
        EXPECT_EQ(with.get_status(), without.get_status());
        EXPECT_DOUBLE_EQ(with.get_obj(), without.get_obj());

        // Every interval-th state the search expands is completed, starting
        // with the initial one, while the final state that ends the search is
        // not expanded
        const auto data       = exported_solver_data(with, export_settings);
        const auto iterations = data.at("iterations_int").get<int>();
        const auto attempted  = data.at("completions_attempted_int").get<int>();
        EXPECT_EQ(attempted, (iterations - 2) / static_cast<int>(interval) + 1);
        if (interval == 1) {
          most_completions = std::max(most_completions, attempted);
        }
      }
    }
  }
  // Not only the initial state is completed
  EXPECT_GT(most_completions, 1);
}

COMPLETION_SOLVER_TEST(SearchContinuesAfterFailedCompletion) {
  // Both trains take the shorter track of the passing loop, on which they
  // cannot pass each other, while neither of them can wait for the other one
  const auto loop = passing_loop_instance();
  EXPECT_FALSE(complete(loop.instance).has_value());

  solver::astar_based::GenPOMovingBlockAStarSolver solver(loop.instance);
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
