#include <cstdlib>
#define TEST_FRIENDS true

#include "Definitions.hpp"
#include "datastructure/RailwayNetwork.hpp"
#include "datastructure/Route.hpp"
#include "nlohmann/json.hpp"
#include "nlohmann/json_fwd.hpp"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"
#include "solver/astar-based/GenPOMovingBlockAStarSolver.hpp"

#include "gtest/gtest.h"

using namespace cda_rail;
// NOLINTBEGIN
// (clang-analyzer-deadcode.DeadStores,misc-const-correctness,clang-diagnostic-unused-result)

// ----------------------------------
// New Tests for Time-Aware Version
// ----------------------------------

TEST(GenPOMovingBlockAStarSolver, NextTrains) {
  instances::GeneralPerformanceOptimizationInstance instance;
  instance.get_editable_network().add_vertex("v0", VertexType::TTD);
  instance.get_editable_network().add_vertex("v1", VertexType::TTD);
  instance.get_editable_network().add_vertex("v2", VertexType::TTD);
  auto const e1 =
      instance.get_editable_network().add_edge({"v0"}, {"v1"}, 100, 50);
  auto const e2 =
      instance.get_editable_network().add_edge({"v1"}, {"v2"}, 50, 50);

  instance.get_editable_network().add_successor({"v0", "v1"}, {"v1", "v2"});

  auto const tr4 = instance.add_train("Train4", 100, 50, 4, 2, true, 100, 15,
                                      {"v0"}, 300, 40, {"v2"}, 4);
  auto const tr2 = instance.add_train("Train2", 100, 50, 4, 2, true, 60, 15,
                                      {"v0"}, 300, 40, {"v2"}, 3);
  auto const tr1 = instance.add_train("Train1", 100, 50, 4, 2, true, 60, 15,
                                      {"v0"}, 300, 40, {"v2"}, 2);
  auto const tr3 = instance.add_train("Train3", 100, 50, 4, 2, true, 90, 15,
                                      {"v0"}, 300, 40, {"v2"}, 3);

  std::vector<std::vector<size_t>> train_edges{{}, {}, {}, {}};
  std::vector<double>              exit_times{-1, -1, -1, -1};

  EXPECT_EQ(cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::
                get_relevant_trains_for_state_transition(
                    {.train_edges = train_edges}, {.exit_times = exit_times},
                    &instance, {.time_aware_state_transitions = false}),
            cda_rail::index_set({tr1, tr2, tr3, tr4}));
  EXPECT_EQ(cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::
                get_relevant_trains_for_state_transition(
                    {.train_edges = train_edges}, {.exit_times = exit_times},
                    &instance, {.time_aware_state_transitions = true}),
            cda_rail::index_set({tr2}));

  train_edges.at(tr1).push_back(e1);
  train_edges.at(tr2).push_back(e1);
  exit_times.at(tr1) = 100;
  exit_times.at(tr2) = 100;
  EXPECT_EQ(cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::
                get_relevant_trains_for_state_transition(
                    {.train_edges = train_edges}, {.exit_times = exit_times},
                    &instance, {.time_aware_state_transitions = false}),
            cda_rail::index_set({tr1, tr2, tr3, tr4}));
  EXPECT_EQ(cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::
                get_relevant_trains_for_state_transition(
                    {.train_edges = train_edges}, {.exit_times = exit_times},
                    &instance, {.time_aware_state_transitions = true}),
            cda_rail::index_set({tr3}));

  exit_times.at(tr2) = 90;
  EXPECT_EQ(cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::
                get_relevant_trains_for_state_transition(
                    {.train_edges = train_edges}, {.exit_times = exit_times},
                    &instance, {.time_aware_state_transitions = false}),
            cda_rail::index_set({tr1, tr2, tr3, tr4}));
  EXPECT_EQ(cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::
                get_relevant_trains_for_state_transition(
                    {.train_edges = train_edges}, {.exit_times = exit_times},
                    &instance, {.time_aware_state_transitions = true}),
            cda_rail::index_set({tr2}));

  exit_times.at(tr1) = 80;
  EXPECT_EQ(cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::
                get_relevant_trains_for_state_transition(
                    {.train_edges = train_edges}, {.exit_times = exit_times},
                    &instance, {.time_aware_state_transitions = false}),
            cda_rail::index_set({tr1, tr2, tr3, tr4}));
  EXPECT_EQ(cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::
                get_relevant_trains_for_state_transition(
                    {.train_edges = train_edges}, {.exit_times = exit_times},
                    &instance, {.time_aware_state_transitions = true}),
            cda_rail::index_set({tr1}));

  train_edges.at(tr1).push_back(e2);
  train_edges.at(tr2).push_back(e2);
  train_edges.at(tr3).push_back(e1);
  exit_times.at(tr1) = -1;
  exit_times.at(tr2) = -1;
  exit_times.at(tr3) = 100;
  EXPECT_EQ(cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::
                get_relevant_trains_for_state_transition(
                    {.train_edges = train_edges}, {.exit_times = exit_times},
                    &instance, {.time_aware_state_transitions = false}),
            cda_rail::index_set({tr3, tr4}));
  EXPECT_EQ(cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::
                get_relevant_trains_for_state_transition(
                    {.train_edges = train_edges}, {.exit_times = exit_times},
                    &instance, {.time_aware_state_transitions = true}),
            cda_rail::index_set({tr4}));

  train_edges.at(tr4).push_back(e1);
  exit_times.at(tr4) = 120;
  EXPECT_EQ(cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::
                get_relevant_trains_for_state_transition(
                    {.train_edges = train_edges}, {.exit_times = exit_times},
                    &instance, {.time_aware_state_transitions = false}),
            cda_rail::index_set({tr3, tr4}));
  EXPECT_EQ(cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::
                get_relevant_trains_for_state_transition(
                    {.train_edges = train_edges}, {.exit_times = exit_times},
                    &instance, {.time_aware_state_transitions = true}),
            cda_rail::index_set({tr3}));

  exit_times.at(tr3) = -1;
  train_edges.at(tr3).push_back(e2);
  EXPECT_EQ(cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::
                get_relevant_trains_for_state_transition(
                    {.train_edges = train_edges}, {.exit_times = exit_times},
                    &instance, {.time_aware_state_transitions = false}),
            cda_rail::index_set({tr4}));
  EXPECT_EQ(cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::
                get_relevant_trains_for_state_transition(
                    {.train_edges = train_edges}, {.exit_times = exit_times},
                    &instance, {.time_aware_state_transitions = true}),
            cda_rail::index_set({tr4}));

  train_edges.at(tr4).push_back(e2);
  exit_times.at(tr4) = -1;
  EXPECT_EQ(cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::
                get_relevant_trains_for_state_transition(
                    {.train_edges = train_edges}, {.exit_times = exit_times},
                    &instance, {.time_aware_state_transitions = false}),
            cda_rail::index_set({}));
  EXPECT_EQ(cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::
                get_relevant_trains_for_state_transition(
                    {.train_edges = train_edges}, {.exit_times = exit_times},
                    &instance, {.time_aware_state_transitions = true}),
            cda_rail::index_set({}));
}

TEST(GenPOMovingBlockAStarSolver, PathExtensions) {
  // clang-format off
  //                                            - v7c --------------- v10c (exit)
  //                                           /
  //                - v3b --- v4b --- v5b - v6b - v7b --- v8b - v9b - v10b
  //               /                                           /
  // v0 --- v1 - v2 - v3a ----------- v5a --------------- v8a -
  // clang-format on

  // Network
  Network    network;
  const auto v0   = network.add_vertex("v0", VertexType::TTD);
  const auto v1   = network.add_vertex("v1", VertexType::TTD);
  const auto v2   = network.add_vertex("v2", VertexType::NoBorder);
  const auto v3a  = network.add_vertex("v3a", VertexType::TTD);
  const auto v3b  = network.add_vertex("v3b", VertexType::TTD);
  const auto v4b  = network.add_vertex("v4b", VertexType::TTD);
  const auto v5a  = network.add_vertex("v5a", VertexType::TTD);
  const auto v5b  = network.add_vertex("v5b", VertexType::TTD);
  const auto v6b  = network.add_vertex("v6b", VertexType::NoBorder);
  const auto v7b  = network.add_vertex("v7b", VertexType::TTD);
  const auto v7c  = network.add_vertex("v7c", VertexType::TTD);
  const auto v8a  = network.add_vertex("v8a", VertexType::TTD);
  const auto v8b  = network.add_vertex("v8b", VertexType::TTD);
  const auto v9b  = network.add_vertex("v9b", VertexType::NoBorder);
  const auto v10b = network.add_vertex("v10b", VertexType::TTD);
  const auto v10c = network.add_vertex("v10c", VertexType::TTD);

  const auto v0_v1    = network.add_edge(v0, v1, 100, 30, true);
  const auto v1_v2    = network.add_edge(v1, v2, 10, 30, false);
  const auto v2_v3a   = network.add_edge(v2, v3a, 10, 30, false);
  const auto v2_v3b   = network.add_edge(v2, v3b, 10, 30, false);
  const auto v3a_v5a  = network.add_edge(v3a, v5a, 200, 30, true);
  const auto v3b_v4b  = network.add_edge(v3b, v4b, 100, 30, true);
  const auto v4b_v5b  = network.add_edge(v4b, v5b, 100, 30, true);
  const auto v5a_v8a  = network.add_edge(v5a, v8a, 120, 30, true);
  const auto v5b_v6b  = network.add_edge(v5b, v6b, 10, 30, false);
  const auto v6b_v7b  = network.add_edge(v6b, v7b, 10, 30, false);
  const auto v6b_v7c  = network.add_edge(v6b, v7c, 10, 30, false);
  const auto v7b_v8b  = network.add_edge(v7b, v8b, 100, 30, true);
  const auto v7c_v10c = network.add_edge(v7c, v10c, 120, 30, true);
  const auto v8a_v9b  = network.add_edge(v8a, v9b, 10, 30, false);
  const auto v8b_v9b  = network.add_edge(v8b, v9b, 10, 30, false);
  const auto v9b_v10b = network.add_edge(v9b, v10b, 10, 30, false);

  network.add_successor(v0_v1, v1_v2);
  network.add_successor(v1_v2, v2_v3a);
  network.add_successor(v1_v2, v2_v3b);
  network.add_successor(v2_v3a, v3a_v5a);
  network.add_successor(v2_v3b, v3b_v4b);
  network.add_successor(v3a_v5a, v5a_v8a);
  network.add_successor(v3b_v4b, v4b_v5b);
  network.add_successor(v4b_v5b, v5b_v6b);
  network.add_successor(v5a_v8a, v8a_v9b);
  network.add_successor(v5b_v6b, v6b_v7b);
  network.add_successor(v5b_v6b, v6b_v7c);
  network.add_successor(v6b_v7b, v7b_v8b);
  network.add_successor(v6b_v7c, v7c_v10c);
  network.add_successor(v7b_v8b, v8b_v9b);
  network.add_successor(v8a_v9b, v9b_v10b);
  network.add_successor(v8b_v9b, v9b_v10b);

  cda_rail::index_set const              ttd1{v1_v2, v2_v3a, v2_v3b};
  cda_rail::index_set const              ttd2{v5b_v6b, v6b_v7b, v6b_v7c};
  cda_rail::index_set const              ttd3{v8a_v9b, v8b_v9b, v9b_v10b};
  std::vector<cda_rail::index_set> const ttd_sections{ttd1, ttd2, ttd3};

  Timetable  timetable;
  const auto tr1 = timetable.add_train("Train1", 50, 20, 3, 3, true, 0, 20, v0,
                                       100, 20, v10c, network);
  const auto tr2 = timetable.add_train("Train2", 50, 30, 4, 2, true, 100, 30,
                                       v0, 200, 20, v10c, network);
  RouteMap   routes;
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);

  std::vector<cda_rail::index_vector> vertex_orders(
      network.number_of_vertices());
  vertex_orders.at(v0).emplace_back(tr1);
  simulator::SimulatorState state{
      .train_edges    = {{v0_v1, v1_v2, v2_v3b, v3b_v4b, v4b_v5b, v5b_v6b}, {}},
      .ttd_orders     = {{tr1}, {}, {}},
      .vertex_orders  = vertex_orders,
      .stop_positions = {{}, {}}};

  auto print_path_extensions =
      [&network](
          std::vector<cda_rail::solver::astar_based::
                          GenPOMovingBlockAStarSolver::PathExtensionData> const&
                             path_extension,
          std::string const& label) {
        std::cout << label << '\n';
        std::cout << "Path extension size: " << path_extension.size() << '\n';
        for (size_t i = 0; i < path_extension.size(); ++i) {
          std::cout << "Path extension " << i << '\n';
          std::cout << "First stop edge: "
                    << path_extension.at(i).stop_possible_from_idx_onward
                    << '\n';
          std::cout << "Path: ";
          for (size_t j = 0; j < path_extension.at(i).path.size(); ++j) {
            auto const& edge_obj =
                network.get_edge(path_extension.at(i).path.at(j));
            std::cout << network.get_vertex(edge_obj.source).name << "->"
                      << network.get_vertex(edge_obj.target).name << " / ";
          }
          std::cout << '\n';
        }
      };

  auto const path_extensions1a = cda_rail::solver::astar_based::
      GenPOMovingBlockAStarSolver::get_path_extensions(
          tr1, state, solver::astar_based::NextStateStrategy::SingleEdge,
          &instance, ttd_sections);
  print_path_extensions(path_extensions1a, "1A");
  EXPECT_EQ(path_extensions1a.size(), 2);
  cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::PathExtensionData
      expected_1a_1{.path = {v6b_v7b}, .stop_possible_from_idx_onward = 0};
  cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::PathExtensionData
      expected_1a_2{.path = {v6b_v7c}, .stop_possible_from_idx_onward = 0};
  EXPECT_TRUE(std::ranges::contains(path_extensions1a, expected_1a_1));
  EXPECT_TRUE(std::ranges::contains(path_extensions1a, expected_1a_2));

  auto const path_extensions1b =
      cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::
          get_path_extensions(tr1, state,
                              solver::astar_based::NextStateStrategy::NextTTD,
                              &instance, ttd_sections);
  print_path_extensions(path_extensions1b, "1B");
  EXPECT_EQ(path_extensions1b.size(), 2);
  cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::PathExtensionData
      expected_1b_1{.path                          = {v6b_v7b, v7b_v8b},
                    .stop_possible_from_idx_onward = 0};
  cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::PathExtensionData
      expected_1b_2{.path                          = {v6b_v7c, v7c_v10c},
                    .stop_possible_from_idx_onward = 0};
  EXPECT_TRUE(std::ranges::contains(path_extensions1b, expected_1b_1));
  EXPECT_TRUE(std::ranges::contains(path_extensions1b, expected_1b_2));

  auto const path_extensions1c = cda_rail::solver::astar_based::
      GenPOMovingBlockAStarSolver::get_path_extensions(
          tr1, state, solver::astar_based::NextStateStrategy::NextRelevantTTD,
          &instance, ttd_sections);
  print_path_extensions(path_extensions1c, "1C");
  EXPECT_TRUE(std::ranges::contains(path_extensions1c, expected_1b_1));
  EXPECT_TRUE(std::ranges::contains(path_extensions1c, expected_1b_2));

  // tr2 braking distance: 30*30 / 4 = 225
  // hence entry path up to v4b_v5b or v3a_v5a

  auto const path_extensions2a = cda_rail::solver::astar_based::
      GenPOMovingBlockAStarSolver::get_path_extensions(
          tr2, state, solver::astar_based::NextStateStrategy::SingleEdge,
          &instance, ttd_sections);
  print_path_extensions(path_extensions2a, "2A");
  EXPECT_EQ(path_extensions2a.size(), 2);
  cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::PathExtensionData
      expected_2a_1{.path = {v0_v1, v1_v2, v2_v3a, v3a_v5a},
                    .stop_possible_from_idx_onward = 3};
  cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::PathExtensionData
      expected_2a_2{.path = {v0_v1, v1_v2, v2_v3b, v3b_v4b, v4b_v5b},
                    .stop_possible_from_idx_onward = 4};
  EXPECT_TRUE(std::ranges::contains(path_extensions2a, expected_2a_1));
  EXPECT_TRUE(std::ranges::contains(path_extensions2a, expected_2a_2));

  auto const path_extensions2b =
      cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::
          get_path_extensions(tr2, state,
                              solver::astar_based::NextStateStrategy::NextTTD,
                              &instance, ttd_sections);
  print_path_extensions(path_extensions2b, "2B");
  EXPECT_EQ(path_extensions2b.size(), 2);
  cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::PathExtensionData
      expected_2b_1{.path = {v0_v1, v1_v2, v2_v3a, v3a_v5a, v5a_v8a},
                    .stop_possible_from_idx_onward = 3};
  cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::PathExtensionData
      expected_2b_2{.path = {v0_v1, v1_v2, v2_v3b, v3b_v4b, v4b_v5b},
                    .stop_possible_from_idx_onward = 4};
  EXPECT_TRUE(std::ranges::contains(path_extensions2b, expected_2b_1));
  EXPECT_TRUE(std::ranges::contains(path_extensions2b, expected_2b_2));

  auto const path_extensions2c = cda_rail::solver::astar_based::
      GenPOMovingBlockAStarSolver::get_path_extensions(
          tr2, state, solver::astar_based::NextStateStrategy::NextRelevantTTD,
          &instance, ttd_sections);
  print_path_extensions(path_extensions2c, "2C");
  EXPECT_EQ(path_extensions2c.size(), 3);
  // expectd_2b_1
  cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::PathExtensionData
      expected_2c_2{.path = {v0_v1, v1_v2, v2_v3b, v3b_v4b, v4b_v5b, v5b_v6b,
                             v6b_v7b, v7b_v8b},
                    .stop_possible_from_idx_onward = 4};
  cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::PathExtensionData
      expected_2c_3{.path = {v0_v1, v1_v2, v2_v3b, v3b_v4b, v4b_v5b, v5b_v6b,
                             v6b_v7c, v7c_v10c},
                    .stop_possible_from_idx_onward = 4};
  EXPECT_TRUE(std::ranges::contains(path_extensions2c, expected_2b_1));
  EXPECT_TRUE(std::ranges::contains(path_extensions2c, expected_2c_2));
  EXPECT_TRUE(std::ranges::contains(path_extensions2c, expected_2c_3));
}

TEST(GenPOMovingBlockAStarSolver, PathExtensionsCloseToEnd) {
  Network network;

  auto const v0 = network.add_vertex("v0", VertexType::TTD);
  auto const v1 = network.add_vertex("v1", VertexType::TTD);
  auto const v2 = network.add_vertex("v2", VertexType::TTD);

  auto const v0_v1 = network.add_edge(v0, v1, 100, 20, true);
  auto const v1_v2 = network.add_edge(v1, v2, 100, 20, true);

  network.add_successor(v0_v1, v1_v2);

  Timetable  timetable;
  const auto tr1 = timetable.add_train("Train1", 50, 20, 2, 4, true, 0, 20, v0,
                                       500, 20, v2, network);

  RouteMap                                                    routes;
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);

  std::vector<cda_rail::index_vector> vertex_orders(
      network.number_of_vertices());
  vertex_orders.at(v0).emplace_back(tr1);
  simulator::SimulatorState state{.train_edges    = {{v0_v1}},
                                  .ttd_orders     = {},
                                  .vertex_orders  = vertex_orders,
                                  .stop_positions = {{}}};

  auto print_path_extensions =
      [&network](
          std::vector<cda_rail::solver::astar_based::
                          GenPOMovingBlockAStarSolver::PathExtensionData> const&
                             path_extension,
          std::string const& label) {
        std::cout << label << '\n';
        std::cout << "Path extension size: " << path_extension.size() << '\n';
        for (size_t i = 0; i < path_extension.size(); ++i) {
          std::cout << "Path extension " << i << '\n';
          std::cout << "First stop edge: "
                    << path_extension.at(i).stop_possible_from_idx_onward
                    << '\n';
          std::cout << "Path: ";
          for (size_t j = 0; j < path_extension.at(i).path.size(); ++j) {
            auto const& edge_obj =
                network.get_edge(path_extension.at(i).path.at(j));
            std::cout << network.get_vertex(edge_obj.source).name << "->"
                      << network.get_vertex(edge_obj.target).name << " / ";
          }
          std::cout << '\n';
        }
      };

  auto const path_extensions1a = cda_rail::solver::astar_based::
      GenPOMovingBlockAStarSolver::get_path_extensions(
          tr1, state, solver::astar_based::NextStateStrategy::SingleEdge,
          &instance, {});
  print_path_extensions(path_extensions1a, "1A");
  EXPECT_EQ(path_extensions1a.size(), 1);
  cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::PathExtensionData
      expected_1a{.path = {v1_v2}, .stop_possible_from_idx_onward = 0};
  EXPECT_TRUE(std::ranges::contains(path_extensions1a, expected_1a));

  auto const path_extensions1b =
      cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::
          get_path_extensions(tr1, state,
                              solver::astar_based::NextStateStrategy::NextTTD,
                              &instance, {});
  print_path_extensions(path_extensions1b, "1B");
  EXPECT_EQ(path_extensions1b.size(), 1);
  cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::PathExtensionData
      expected_1b{.path = {v1_v2}, .stop_possible_from_idx_onward = 0};
  EXPECT_TRUE(std::ranges::contains(path_extensions1b, expected_1b));

  auto const path_extensions1c = cda_rail::solver::astar_based::
      GenPOMovingBlockAStarSolver::get_path_extensions(
          tr1, state, solver::astar_based::NextStateStrategy::NextRelevantTTD,
          &instance, {});
  print_path_extensions(path_extensions1c, "1C");
  EXPECT_EQ(path_extensions1c.size(), 1);
  cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::PathExtensionData
      expected_1c{.path = {v1_v2}, .stop_possible_from_idx_onward = 0};
  EXPECT_TRUE(std::ranges::contains(path_extensions1c, expected_1c));
}

TEST(GenPOMovingBlockAStarSolver, InferInsertionBounds) {
  EXPECT_EQ(
      cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::
          infer_order_insertion_bounds(2, {3, 9, 4, 2, 5, 7, 10},
                                       {3, 9, 17, 5, 10, 4}, {3, 9, 10}, true),
      cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::IndexBound(
          {.lb = 6, .ub = 6}));
  EXPECT_EQ(
      cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::
          infer_order_insertion_bounds(2, {3, 9, 4, 2, 5, 7, 10},
                                       {3, 9, 17, 5, 10, 4}, {3, 9, 10}, false),
      cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::IndexBound(
          {.lb = 2, .ub = 4}));
}

TEST(GenPOMovingBlockAStarSolver, ExtendStateWithPathExtension) {
  Network    network;
  auto const v1  = network.add_vertex("v1", VertexType::TTD);
  auto const v2  = network.add_vertex("v2", VertexType::TTD);
  auto const v3  = network.add_vertex("v3", VertexType::TTD);
  auto const v4  = network.add_vertex("v4", VertexType::TTD);
  auto const v5  = network.add_vertex("v5", VertexType::TTD);
  auto const v6  = network.add_vertex("v6", VertexType::TTD);
  auto const v7  = network.add_vertex("v7", VertexType::TTD);
  auto const v8  = network.add_vertex("v8", VertexType::TTD);
  auto const v9  = network.add_vertex("v9", VertexType::TTD);
  auto const v10 = network.add_vertex("v10", VertexType::TTD);
  auto const v11 = network.add_vertex("v11", VertexType::TTD);

  auto const v1_v2   = network.add_edge(v1, v2, 100, 20, true);   // 100
  auto const v2_v3   = network.add_edge(v2, v3, 100, 20, true);   // 200
  auto const v3_v4   = network.add_edge(v3, v4, 100, 20, true);   // 300
  auto const v4_v5   = network.add_edge(v4, v5, 100, 20, true);   // 400
  auto const v5_v6   = network.add_edge(v5, v6, 30, 20, true);    // 430
  auto const v6_v7   = network.add_edge(v6, v7, 30, 20, true);    // 460
  auto const v7_v8   = network.add_edge(v7, v8, 30, 20, true);    // 490
  auto const v8_v9   = network.add_edge(v8, v9, 100, 20, true);   // 590
  auto const v9_v10  = network.add_edge(v9, v10, 100, 20, true);  // 690
  auto const v10_v11 = network.add_edge(v10, v11, 100, 20, true); // 790

  network.add_successor(v1_v2, v2_v3);
  network.add_successor(v2_v3, v3_v4);
  network.add_successor(v3_v4, v4_v5);
  network.add_successor(v4_v5, v5_v6);
  network.add_successor(v5_v6, v6_v7);
  network.add_successor(v6_v7, v7_v8);

  Timetable  timetable;
  const auto tr1 = timetable.add_train("Train1", 50, 20, 2, 4, true, 0, 20, v1,
                                       500, 20, v8, network);
  timetable.add_empty_station("Station1");
  timetable.add_empty_station("Station2");
  timetable.add_empty_station("Station3");
  timetable.add_track_to_station("Station1", v2_v3, network);
  timetable.add_track_to_station("Station2", v3_v4, network);
  timetable.add_track_to_station("Station2", v5_v6, network);
  timetable.add_track_to_station("Station2", v6_v7, network);
  timetable.add_track_to_station("Station2", v7_v8, network);
  timetable.add_track_to_station("Station3", v9_v10, network);

  timetable.insert_stop(tr1, "Station1", 100, 60);
  timetable.insert_stop(tr1, "Station2", 200, 60);
  timetable.insert_stop(tr1, "Station3", 300, 60);

  RouteMap                                                    routes;
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);

  simulator::SimulatorState state{.train_edges    = {{v1_v2, v2_v3}},
                                  .ttd_orders     = {},
                                  .vertex_orders  = {},
                                  .stop_positions = {{200}}};
  cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::PathExtensionData
             data{.path = {v3_v4, v4_v5, v5_v6, v6_v7, v7_v8},
                  .stop_possible_from_idx_onward = 1};
  auto const extended_stats =
      cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::
          extend_state_by_path_extension(tr1, state, data, &instance);
  EXPECT_EQ(extended_stats.size(), 2);
  ASSERT_GE(extended_stats.size(), 1);
  EXPECT_EQ(extended_stats.at(0).train_edges.at(tr1),
            cda_rail::index_vector({v1_v2, v2_v3, v3_v4, v4_v5, v5_v6, v6_v7}));
  EXPECT_EQ(extended_stats.at(0).stop_positions.at(tr1),
            std::vector<double>({200, 460}));
  ASSERT_GE(extended_stats.size(), 2);
  EXPECT_EQ(extended_stats.at(1).train_edges.at(tr1),
            cda_rail::index_vector(
                {v1_v2, v2_v3, v3_v4, v4_v5, v5_v6, v6_v7, v7_v8}));
  EXPECT_EQ(extended_stats.at(1).stop_positions.at(tr1),
            std::vector<double>({200, 490}));
}

TEST(GenPOMovingBlockAStarSolver, ExtendTrainOrderEntry) {
  // clang-format off
  //                - v3b -- v4b -
  //               /              \
  // v0 --- v1 - v2 - v3a -- v4a - v5 - v6 -- v7
  // clang-format on

  Network    network;
  auto const v0  = network.add_vertex("v0", VertexType::TTD);
  auto const v1  = network.add_vertex("v1", VertexType::TTD);
  auto const v2  = network.add_vertex("v2", VertexType::NoBorder);
  auto const v3a = network.add_vertex("v3a", VertexType::TTD);
  auto const v3b = network.add_vertex("v3b", VertexType::TTD);
  auto const v4a = network.add_vertex("v4a", VertexType::TTD);
  auto const v4b = network.add_vertex("v4b", VertexType::TTD);
  auto const v5  = network.add_vertex("v5", VertexType::NoBorder);
  auto const v6  = network.add_vertex("v6", VertexType::TTD);
  auto const v7  = network.add_vertex("v7", VertexType::TTD);

  auto const v0_v1   = network.add_edge(v0, v1, 100, 20, true);
  auto const v1_v2   = network.add_edge(v1, v2, 10, 20, false);
  auto const v2_v3a  = network.add_edge(v2, v3a, 10, 20, false);
  auto const v2_v3b  = network.add_edge(v2, v3b, 10, 20, false);
  auto const v3a_v4a = network.add_edge(v3a, v4a, 100, 20, true);
  auto const v3b_v4b = network.add_edge(v3b, v4b, 100, 20, true);
  auto const v4a_v5  = network.add_edge(v4a, v5, 10, 20, false);
  auto const v4b_v5  = network.add_edge(v4b, v5, 10, 20, false);
  auto const v5_v6   = network.add_edge(v5, v6, 10, 20, false);
  auto const v6_v7   = network.add_edge(v6, v7, 100, 20, true);

  network.add_successor(v0_v1, v1_v2);
  network.add_successor(v1_v2, v2_v3a);
  network.add_successor(v1_v2, v2_v3b);
  network.add_successor(v2_v3a, v3a_v4a);
  network.add_successor(v2_v3b, v3b_v4b);
  network.add_successor(v3a_v4a, v4a_v5);
  network.add_successor(v3b_v4b, v4b_v5);
  network.add_successor(v4a_v5, v5_v6);
  network.add_successor(v4b_v5, v5_v6);
  network.add_successor(v5_v6, v6_v7);

  // Add all reverse edges
  auto const          num_edges = network.number_of_edges();
  std::vector<size_t> reverse_edges;
  reverse_edges.resize(num_edges);
  for (size_t i = 0; i < num_edges; ++i) {
    auto const& edge    = network.get_edge(i);
    reverse_edges.at(i) = network.add_edge(
        edge.target, edge.source, edge.length, edge.max_speed, edge.breakable);
  }

  for (size_t i = 0; i < num_edges; ++i) {
    auto const& successors = network.get_successors(i);
    for (auto const& successor : successors) {
      network.add_successor(reverse_edges.at(successor), reverse_edges.at(i));
    }
  }

  Timetable  timetable;
  const auto tr1 = timetable.add_train("Train1", 50, 20, 2, 4, true, 0, 20, v0,
                                       500, 20, v7, network);
  const auto tr2 = timetable.add_train("Train2", 50, 20, 2, 4, true, 100, 20,
                                       v7, 300, 20, v0, network);
  const auto tr3 = timetable.add_train("Train3", 50, 20, 2, 4, true, 200, 20,
                                       v7, 500, 20, v0, network);
  const auto tr4 = timetable.add_train("Train4", 50, 20, 2, 4, true, 400, 20,
                                       v0, 700, 20, v7, network);

  Timetable  timetable2;
  const auto tr1b = timetable2.add_train("Train1", 50, 20, 2, 4, true, 200, 20,
                                         v0, 500, 20, v7, network);
  const auto tr2b = timetable2.add_train("Train2", 50, 20, 2, 4, true, 0, 20,
                                         v7, 50, 20, v0, network);
  const auto tr3b = timetable2.add_train("Train3", 50, 20, 2, 4, true, 50, 20,
                                         v7, 100, 20, v0, network);
  const auto tr4b = timetable2.add_train("Train4", 50, 20, 2, 4, true, 400, 20,
                                         v0, 700, 20, v7, network);

  RouteMap                                                    routes;
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance2(
      network, timetable2, routes);

  cda_rail::index_set              ttd1{v1_v2,
                                        v2_v3a,
                                        v2_v3b,
                                        reverse_edges.at(v1_v2),
                                        reverse_edges.at(v2_v3a),
                                        reverse_edges.at(v2_v3b)};
  cda_rail::index_set              ttd2{v4a_v5,
                                        v4b_v5,
                                        v5_v6,
                                        reverse_edges.at(v4a_v5),
                                        reverse_edges.at(v4b_v5),
                                        reverse_edges.at(v5_v6)};
  std::vector<cda_rail::index_set> ttd_sections{ttd1, ttd2};

  simulator::SimulatorState state{
      .train_edges = std::vector<cda_rail::index_vector>(
          instance.get_const_train_list().get_number_of_trains()),
      .ttd_orders = std::vector<cda_rail::index_vector>(ttd_sections.size()),
      .vertex_orders =
          std::vector<cda_rail::index_vector>(network.number_of_vertices()),
      .stop_positions = std::vector<std::vector<double>>(
          instance.get_const_train_list().get_number_of_trains())};

  state.train_edges.at(tr2) = {
      reverse_edges.at(v6_v7),  reverse_edges.at(v5_v6),
      reverse_edges.at(v4a_v5), reverse_edges.at(v3a_v4a),
      reverse_edges.at(v2_v3a), reverse_edges.at(v1_v2),
      reverse_edges.at(v0_v1)};
  state.train_edges.at(tr3) = state.train_edges.at(tr2);
  state.train_edges.at(tr4) = {v0_v1, v1_v2, v2_v3b, v3b_v4b};

  state.ttd_orders.at(0)     = {tr2, tr4, tr3};
  state.ttd_orders.at(1)     = {tr2, tr3};
  state.vertex_orders.at(v0) = {tr2, tr4, tr3};
  state.vertex_orders.at(v7) = {tr2, tr3};

  auto const extended0_1 = cda_rail::solver::astar_based::
      GenPOMovingBlockAStarSolver::extend_train_orders_of_state(
          tr1, state, {.late_entry_possible = false},
          {.time_aware_state_transitions = false}, ttd_sections, &instance2);
  EXPECT_EQ(extended0_1.size(), 1);
  ASSERT_GE(extended0_1.size(), 1);
  EXPECT_EQ(extended0_1.at(0).train_edges, state.train_edges);
  EXPECT_EQ(extended0_1.at(0).stop_positions, state.stop_positions);
  EXPECT_EQ(extended0_1.at(0).ttd_orders, state.ttd_orders);
  EXPECT_EQ(extended0_1.at(0).vertex_orders, state.vertex_orders);

  auto const extended0_2 = cda_rail::solver::astar_based::
      GenPOMovingBlockAStarSolver::extend_train_orders_of_state(
          tr1, state, {.late_entry_possible = false},
          {.time_aware_state_transitions = true}, ttd_sections, &instance2);
  EXPECT_EQ(extended0_2.size(), 1);
  ASSERT_GE(extended0_2.size(), 1);
  EXPECT_EQ(extended0_2.at(0).train_edges, state.train_edges);
  EXPECT_EQ(extended0_2.at(0).stop_positions, state.stop_positions);
  EXPECT_EQ(extended0_2.at(0).ttd_orders, state.ttd_orders);
  EXPECT_EQ(extended0_2.at(0).vertex_orders, state.vertex_orders);

  auto const extended0_3 = cda_rail::solver::astar_based::
      GenPOMovingBlockAStarSolver::extend_train_orders_of_state(
          tr1, state, {.late_entry_possible = true},
          {.time_aware_state_transitions = true}, ttd_sections, &instance2);
  EXPECT_EQ(extended0_3.size(), 1);
  ASSERT_GE(extended0_3.size(), 1);
  EXPECT_EQ(extended0_3.at(0).train_edges, state.train_edges);
  EXPECT_EQ(extended0_3.at(0).stop_positions, state.stop_positions);
  EXPECT_EQ(extended0_3.at(0).ttd_orders, state.ttd_orders);
  EXPECT_EQ(extended0_3.at(0).vertex_orders, state.vertex_orders);

  state.train_edges.at(tr1) = {v0_v1};

  auto const extended0_1b = cda_rail::solver::astar_based::
      GenPOMovingBlockAStarSolver::extend_train_orders_of_state(
          tr1, state, {.late_entry_possible = false},
          {.time_aware_state_transitions = false}, ttd_sections, &instance2);
  EXPECT_EQ(extended0_1b.size(), 1);
  ASSERT_GE(extended0_1b.size(), 1);
  EXPECT_EQ(extended0_1b.at(0).train_edges, state.train_edges);
  EXPECT_EQ(extended0_1b.at(0).stop_positions, state.stop_positions);
  EXPECT_EQ(extended0_1b.at(0).ttd_orders, state.ttd_orders);
  EXPECT_EQ(extended0_1b.at(0).vertex_orders.at(v0),
            cda_rail::index_vector({tr2, tr4, tr3, tr1}));
  EXPECT_EQ(extended0_1b.at(0).vertex_orders.at(v7),
            state.vertex_orders.at(v7));

  auto const extended0_2b = cda_rail::solver::astar_based::
      GenPOMovingBlockAStarSolver::extend_train_orders_of_state(
          tr1, state, {.late_entry_possible = false},
          {.time_aware_state_transitions = true}, ttd_sections, &instance2);
  EXPECT_EQ(extended0_2b.size(), 2);

  std::vector<cda_rail::index_vector> expected_vertex_orders0_2b{
      {tr1, tr2, tr4, tr3}, {tr2, tr1, tr4, tr3}};
  std::vector<cda_rail::index_vector> actual_vertex_orders02_b{};
  for (auto const& extended_state : extended0_2b) {
    actual_vertex_orders02_b.emplace_back(extended_state.vertex_orders.at(v0));
  }
  EXPECT_TRUE(std::ranges::contains(actual_vertex_orders02_b,
                                    expected_vertex_orders0_2b.at(0)));
  EXPECT_TRUE(std::ranges::contains(actual_vertex_orders02_b,
                                    expected_vertex_orders0_2b.at(1)));
  for (auto const& actual_order : actual_vertex_orders02_b) {
    EXPECT_TRUE(
        std::ranges::contains(expected_vertex_orders0_2b, actual_order));
  }

  for (auto const& extended_state : extended0_2b) {
    EXPECT_EQ(extended_state.train_edges, state.train_edges);
    EXPECT_EQ(extended_state.stop_positions, state.stop_positions);
    EXPECT_EQ(extended_state.ttd_orders, state.ttd_orders);
    EXPECT_EQ(extended_state.vertex_orders.at(v7), state.vertex_orders.at(v7));
  }

  auto const extended0_3b = cda_rail::solver::astar_based::
      GenPOMovingBlockAStarSolver::extend_train_orders_of_state(
          tr1, state, {.late_entry_possible = true},
          {.time_aware_state_transitions = true}, ttd_sections, &instance2);
  EXPECT_EQ(extended0_3b.size(), 4);

  std::vector<cda_rail::index_vector> expected_vertex_orders0_3b{
      {tr1, tr2, tr4, tr3},
      {tr2, tr1, tr4, tr3},
      {tr2, tr4, tr1, tr3},
      {tr2, tr4, tr3, tr1}};
  std::vector<cda_rail::index_vector> actual_vertex_orders03_b{};
  for (auto const& extended_state : extended0_3b) {
    actual_vertex_orders03_b.emplace_back(extended_state.vertex_orders.at(v0));
  }
  EXPECT_TRUE(std::ranges::contains(actual_vertex_orders03_b,
                                    expected_vertex_orders0_3b.at(0)));
  EXPECT_TRUE(std::ranges::contains(actual_vertex_orders03_b,
                                    expected_vertex_orders0_3b.at(1)));
  EXPECT_TRUE(std::ranges::contains(actual_vertex_orders03_b,
                                    expected_vertex_orders0_3b.at(2)));
  EXPECT_TRUE(std::ranges::contains(actual_vertex_orders03_b,
                                    expected_vertex_orders0_3b.at(3)));
  for (auto const& actual_order : actual_vertex_orders03_b) {
    EXPECT_TRUE(
        std::ranges::contains(expected_vertex_orders0_3b, actual_order));
  }

  for (auto const& extended_state : extended0_3b) {
    EXPECT_EQ(extended_state.train_edges, state.train_edges);
    EXPECT_EQ(extended_state.stop_positions, state.stop_positions);
    EXPECT_EQ(extended_state.ttd_orders, state.ttd_orders);
    EXPECT_EQ(extended_state.vertex_orders.at(v7), state.vertex_orders.at(v7));
  }

  state.train_edges.at(tr1)  = {v0_v1, v1_v2, v2_v3b, v3b_v4b, v4b_v5, v5_v6};
  state.ttd_orders.at(0)     = {tr1, tr2, tr3};
  state.ttd_orders.at(1)     = {tr2, tr3, tr1};
  state.vertex_orders.at(v0) = {tr1, tr2, tr3};
  state.vertex_orders.at(v7) = {tr2, tr3};

  auto const extended1 = cda_rail::solver::astar_based::
      GenPOMovingBlockAStarSolver::extend_train_orders_of_state(
          tr4, state, {.late_entry_possible = false},
          {.time_aware_state_transitions = false}, ttd_sections, &instance);
  EXPECT_EQ(extended1.size(), 1);
  ASSERT_GE(extended1.size(), 1);
  EXPECT_EQ(extended1.at(0).train_edges, state.train_edges);
  EXPECT_EQ(extended1.at(0).stop_positions, state.stop_positions);
  EXPECT_EQ(extended1.at(0).vertex_orders.at(v0),
            cda_rail::index_vector({tr1, tr2, tr3, tr4}));
  EXPECT_EQ(extended1.at(0).ttd_orders.at(0),
            cda_rail::index_vector({tr1, tr2, tr3, tr4}));
  EXPECT_EQ(extended1.at(0).ttd_orders.at(1), state.ttd_orders.at(1));
  EXPECT_EQ(extended1.at(0).vertex_orders.at(v7), state.vertex_orders.at(v7));

  auto const extended2 = cda_rail::solver::astar_based::
      GenPOMovingBlockAStarSolver::extend_train_orders_of_state(
          tr4, state, {.late_entry_possible = false},
          {.time_aware_state_transitions = true}, ttd_sections, &instance);
  EXPECT_EQ(extended2.size(), 2);
  ASSERT_GE(extended2.size(), 2);
  EXPECT_EQ(extended2.at(0).train_edges, state.train_edges);
  EXPECT_EQ(extended2.at(0).stop_positions, state.stop_positions);
  EXPECT_EQ(extended2.at(1).train_edges, state.train_edges);
  EXPECT_EQ(extended2.at(1).stop_positions, state.stop_positions);
  EXPECT_EQ(extended2.at(0).vertex_orders.at(v0),
            extended2.at(0).ttd_orders.at(0));
  EXPECT_EQ(extended2.at(1).vertex_orders.at(v0),
            extended2.at(1).ttd_orders.at(0));
  EXPECT_EQ(extended2.at(0).ttd_orders.at(1), state.ttd_orders.at(1));
  EXPECT_EQ(extended2.at(1).ttd_orders.at(1), state.ttd_orders.at(1));
  EXPECT_EQ(extended2.at(0).vertex_orders.at(v7), state.vertex_orders.at(v7));
  EXPECT_EQ(extended2.at(1).vertex_orders.at(v7), state.vertex_orders.at(v7));
  EXPECT_NE(extended2.at(0).vertex_orders.at(v0),
            extended2.at(1).vertex_orders.at(v0));
  std::vector<cda_rail::index_vector> expected_vertex_orders2{
      {tr1, tr4, tr2, tr3}, {tr1, tr2, tr4, tr3}};
  std::vector<cda_rail::index_vector> actual_vertex_orders2{
      extended2.at(0).vertex_orders.at(v0),
      extended2.at(1).vertex_orders.at(v0)};
  EXPECT_TRUE(std::ranges::contains(expected_vertex_orders2,
                                    extended2.at(0).vertex_orders.at(v0)));
  EXPECT_TRUE(std::ranges::contains(expected_vertex_orders2,
                                    extended2.at(1).vertex_orders.at(v0)));
  EXPECT_TRUE(std::ranges::contains(actual_vertex_orders2,
                                    expected_vertex_orders2.at(0)));
  EXPECT_TRUE(std::ranges::contains(actual_vertex_orders2,
                                    expected_vertex_orders2.at(1)));

  auto const extended3 = cda_rail::solver::astar_based::
      GenPOMovingBlockAStarSolver::extend_train_orders_of_state(
          tr4, state, {.late_entry_possible = true},
          {.time_aware_state_transitions = true}, ttd_sections, &instance);
  EXPECT_EQ(extended3.size(), 4);
  ASSERT_GE(extended3.size(), 4);
  EXPECT_EQ(extended3.at(0).train_edges, state.train_edges);
  EXPECT_EQ(extended3.at(0).stop_positions, state.stop_positions);
  EXPECT_EQ(extended3.at(1).train_edges, state.train_edges);
  EXPECT_EQ(extended3.at(1).stop_positions, state.stop_positions);
  EXPECT_EQ(extended3.at(2).train_edges, state.train_edges);
  EXPECT_EQ(extended3.at(2).stop_positions, state.stop_positions);
  EXPECT_EQ(extended3.at(3).train_edges, state.train_edges);
  EXPECT_EQ(extended3.at(3).stop_positions, state.stop_positions);
  EXPECT_EQ(extended3.at(0).vertex_orders.at(v0),
            extended3.at(0).ttd_orders.at(0));
  EXPECT_EQ(extended3.at(1).vertex_orders.at(v0),
            extended3.at(1).ttd_orders.at(0));
  EXPECT_EQ(extended3.at(2).vertex_orders.at(v0),
            extended3.at(2).ttd_orders.at(0));
  EXPECT_EQ(extended3.at(3).vertex_orders.at(v0),
            extended3.at(3).ttd_orders.at(0));
  EXPECT_EQ(extended3.at(0).ttd_orders.at(1), state.ttd_orders.at(1));
  EXPECT_EQ(extended3.at(1).ttd_orders.at(1), state.ttd_orders.at(1));
  EXPECT_EQ(extended3.at(2).ttd_orders.at(1), state.ttd_orders.at(1));
  EXPECT_EQ(extended3.at(3).ttd_orders.at(1), state.ttd_orders.at(1));
  EXPECT_EQ(extended3.at(0).vertex_orders.at(v7), state.vertex_orders.at(v7));
  EXPECT_EQ(extended3.at(1).vertex_orders.at(v7), state.vertex_orders.at(v7));
  EXPECT_EQ(extended3.at(2).vertex_orders.at(v7), state.vertex_orders.at(v7));
  EXPECT_EQ(extended3.at(3).vertex_orders.at(v7), state.vertex_orders.at(v7));
  std::vector<cda_rail::index_vector> expected_vertex_orders3{
      {tr4, tr1, tr2, tr3},
      {tr1, tr4, tr2, tr3},
      {tr1, tr2, tr4, tr3},
      {tr1, tr2, tr3, tr4}};
  std::vector<cda_rail::index_vector> actual_vertex_orders3{
      extended3.at(0).vertex_orders.at(v0),
      extended3.at(1).vertex_orders.at(v0),
      extended3.at(2).vertex_orders.at(v0),
      extended3.at(3).vertex_orders.at(v0)};
  EXPECT_TRUE(std::ranges::contains(expected_vertex_orders3,
                                    extended3.at(0).vertex_orders.at(v0)));
  EXPECT_TRUE(std::ranges::contains(expected_vertex_orders3,
                                    extended3.at(1).vertex_orders.at(v0)));
  EXPECT_TRUE(std::ranges::contains(expected_vertex_orders3,
                                    extended3.at(2).vertex_orders.at(v0)));
  EXPECT_TRUE(std::ranges::contains(expected_vertex_orders3,
                                    extended3.at(3).vertex_orders.at(v0)));
  EXPECT_TRUE(std::ranges::contains(actual_vertex_orders3,
                                    expected_vertex_orders3.at(0)));
  EXPECT_TRUE(std::ranges::contains(actual_vertex_orders3,
                                    expected_vertex_orders3.at(1)));
  EXPECT_TRUE(std::ranges::contains(actual_vertex_orders3,
                                    expected_vertex_orders3.at(2)));
  EXPECT_TRUE(std::ranges::contains(actual_vertex_orders3,
                                    expected_vertex_orders3.at(3)));
}

TEST(GenPOMovingBlockAStarSolver, ExtendTrainOrderExit) {
  // clang-format off
  //                - v3b -- v4b -
  //               /              \
  // v0 --- v1 - v2 - v3a -- v4a - v5 - v6 -- v7
  // clang-format on

  Network    network;
  auto const v0  = network.add_vertex("v0", VertexType::TTD);
  auto const v1  = network.add_vertex("v1", VertexType::TTD);
  auto const v2  = network.add_vertex("v2", VertexType::NoBorder);
  auto const v3a = network.add_vertex("v3a", VertexType::TTD);
  auto const v3b = network.add_vertex("v3b", VertexType::TTD);
  auto const v4a = network.add_vertex("v4a", VertexType::TTD);
  auto const v4b = network.add_vertex("v4b", VertexType::TTD);
  auto const v5  = network.add_vertex("v5", VertexType::NoBorder);
  auto const v6  = network.add_vertex("v6", VertexType::TTD);
  auto const v7  = network.add_vertex("v7", VertexType::TTD);

  auto const v0_v1   = network.add_edge(v0, v1, 100, 20, true);
  auto const v1_v2   = network.add_edge(v1, v2, 10, 20, false);
  auto const v2_v3a  = network.add_edge(v2, v3a, 10, 20, false);
  auto const v2_v3b  = network.add_edge(v2, v3b, 10, 20, false);
  auto const v3a_v4a = network.add_edge(v3a, v4a, 100, 20, true);
  auto const v3b_v4b = network.add_edge(v3b, v4b, 100, 20, true);
  auto const v4a_v5  = network.add_edge(v4a, v5, 10, 20, false);
  auto const v4b_v5  = network.add_edge(v4b, v5, 10, 20, false);
  auto const v5_v6   = network.add_edge(v5, v6, 10, 20, false);
  auto const v6_v7   = network.add_edge(v6, v7, 100, 20, true);

  network.add_successor(v0_v1, v1_v2);
  network.add_successor(v1_v2, v2_v3a);
  network.add_successor(v1_v2, v2_v3b);
  network.add_successor(v2_v3a, v3a_v4a);
  network.add_successor(v2_v3b, v3b_v4b);
  network.add_successor(v3a_v4a, v4a_v5);
  network.add_successor(v3b_v4b, v4b_v5);
  network.add_successor(v4a_v5, v5_v6);
  network.add_successor(v4b_v5, v5_v6);
  network.add_successor(v5_v6, v6_v7);

  // Add all reverse edges
  auto const          num_edges = network.number_of_edges();
  std::vector<size_t> reverse_edges;
  reverse_edges.resize(num_edges);
  for (size_t i = 0; i < num_edges; ++i) {
    auto const& edge    = network.get_edge(i);
    reverse_edges.at(i) = network.add_edge(
        edge.target, edge.source, edge.length, edge.max_speed, edge.breakable);
  }

  for (size_t i = 0; i < num_edges; ++i) {
    auto const& successors = network.get_successors(i);
    for (auto const& successor : successors) {
      network.add_successor(reverse_edges.at(successor), reverse_edges.at(i));
    }
  }

  Timetable  timetable;
  const auto tr1 = timetable.add_train("Train1", 50, 20, 2, 4, true, 0, 20, v0,
                                       100, 20, v7, network);
  const auto tr2 = timetable.add_train("Train2", 50, 20, 2, 4, true, 100, 20,
                                       v7, 300, 20, v0, network);
  const auto tr3 = timetable.add_train("Train3", 50, 20, 2, 4, true, 200, 20,
                                       v7, 500, 20, v0, network);
  const auto tr4 = timetable.add_train("Train4", 50, 20, 2, 4, true, 400, 20,
                                       v0, 700, 20, v7, network);

  RouteMap                                                    routes;
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);

  cda_rail::index_set              ttd1{v1_v2,
                                        v2_v3a,
                                        v2_v3b,
                                        reverse_edges.at(v1_v2),
                                        reverse_edges.at(v2_v3a),
                                        reverse_edges.at(v2_v3b)};
  cda_rail::index_set              ttd2{v4a_v5,
                                        v4b_v5,
                                        v5_v6,
                                        reverse_edges.at(v4a_v5),
                                        reverse_edges.at(v4b_v5),
                                        reverse_edges.at(v5_v6)};
  std::vector<cda_rail::index_set> ttd_sections{ttd1, ttd2};

  simulator::SimulatorState state{
      .train_edges = std::vector<cda_rail::index_vector>(
          instance.get_const_train_list().get_number_of_trains()),
      .ttd_orders = std::vector<cda_rail::index_vector>(ttd_sections.size()),
      .vertex_orders =
          std::vector<cda_rail::index_vector>(network.number_of_vertices()),
      .stop_positions = std::vector<std::vector<double>>(
          instance.get_const_train_list().get_number_of_trains())};
  state.train_edges.at(tr1) = {v0_v1,  v1_v2, v2_v3b, v3b_v4b,
                               v4b_v5, v5_v6, v6_v7};
  state.train_edges.at(tr2) = {
      reverse_edges.at(v6_v7),  reverse_edges.at(v5_v6),
      reverse_edges.at(v4a_v5), reverse_edges.at(v3a_v4a),
      reverse_edges.at(v2_v3a), reverse_edges.at(v1_v2),
      reverse_edges.at(v0_v1)};
  state.train_edges.at(tr3) = state.train_edges.at(tr2);
  state.train_edges.at(tr4) = state.train_edges.at(tr1);

  state.ttd_orders.at(0)     = {tr1, tr2, tr3};
  state.ttd_orders.at(1)     = {tr2, tr1, tr3};
  state.vertex_orders.at(v0) = {tr1, tr4, tr2, tr3};
  state.vertex_orders.at(v7) = {tr2, tr1, tr3};

  auto const extended1 = cda_rail::solver::astar_based::
      GenPOMovingBlockAStarSolver::extend_train_orders_of_state(
          tr4, state, {.late_entry_possible = false},
          {.time_aware_state_transitions = false}, ttd_sections, &instance);
  EXPECT_EQ(extended1.size(), 1);
  ASSERT_GE(extended1.size(), 1);
  EXPECT_EQ(extended1.at(0).train_edges, state.train_edges);
  EXPECT_EQ(extended1.at(0).stop_positions, state.stop_positions);
  EXPECT_EQ(extended1.at(0).vertex_orders.at(v0),
            cda_rail::index_vector({tr1, tr4, tr2, tr3}));
  EXPECT_EQ(extended1.at(0).ttd_orders.at(0),
            cda_rail::index_vector({tr1, tr2, tr3, tr4}));
  EXPECT_EQ(extended1.at(0).ttd_orders.at(1),
            cda_rail::index_vector({tr2, tr1, tr3, tr4}));
  EXPECT_EQ(extended1.at(0).vertex_orders.at(v7),
            cda_rail::index_vector({tr2, tr1, tr3, tr4}));

  auto const extended2 = cda_rail::solver::astar_based::
      GenPOMovingBlockAStarSolver::extend_train_orders_of_state(
          tr4, state, {.late_entry_possible = false},
          {.time_aware_state_transitions = true}, ttd_sections, &instance);
  EXPECT_EQ(extended2.size(), 2);
  ASSERT_GE(extended2.size(), 2);
  EXPECT_EQ(extended2.at(0).train_edges, state.train_edges);
  EXPECT_EQ(extended2.at(0).stop_positions, state.stop_positions);
  EXPECT_EQ(extended2.at(1).train_edges, state.train_edges);
  EXPECT_EQ(extended2.at(1).stop_positions, state.stop_positions);

  EXPECT_EQ(extended2.at(0).vertex_orders.at(v0), state.vertex_orders.at(v0));
  EXPECT_EQ(extended2.at(1).vertex_orders.at(v0), state.vertex_orders.at(v0));

  EXPECT_EQ(extended2.at(0).vertex_orders.at(v0),
            extended2.at(0).ttd_orders.at(0));
  EXPECT_EQ(extended2.at(1).vertex_orders.at(v0),
            extended2.at(1).ttd_orders.at(0));
  EXPECT_EQ(extended2.at(0).vertex_orders.at(v7),
            extended2.at(0).ttd_orders.at(1));
  EXPECT_EQ(extended2.at(1).vertex_orders.at(v7),
            extended2.at(1).ttd_orders.at(1));

  EXPECT_NE(extended2.at(0).ttd_orders.at(1), extended2.at(1).ttd_orders.at(1));
  EXPECT_NE(extended2.at(0).vertex_orders.at(v7),
            extended2.at(1).vertex_orders.at(v7));

  std::vector<cda_rail::index_vector> expected_vertex_orders2{
      {tr2, tr1, tr4, tr3}, {tr2, tr1, tr3, tr4}};
  std::vector<cda_rail::index_vector> actual_vertex_orders2{
      extended2.at(0).vertex_orders.at(v7),
      extended2.at(1).vertex_orders.at(v7)};
  EXPECT_TRUE(std::ranges::contains(expected_vertex_orders2,
                                    extended2.at(0).vertex_orders.at(v7)));
  EXPECT_TRUE(std::ranges::contains(expected_vertex_orders2,
                                    extended2.at(1).vertex_orders.at(v7)));
  EXPECT_TRUE(std::ranges::contains(actual_vertex_orders2,
                                    expected_vertex_orders2.at(0)));
  EXPECT_TRUE(std::ranges::contains(actual_vertex_orders2,
                                    expected_vertex_orders2.at(1)));

  // Test also upper bound on insertion location
  state.ttd_orders.at(0)     = {tr4, tr2, tr3};
  state.ttd_orders.at(1)     = {tr2, tr4, tr3};
  state.vertex_orders.at(v0) = {tr1, tr4, tr2, tr3};
  state.vertex_orders.at(v7) = {tr2, tr4, tr3};
  auto const extended3       = cda_rail::solver::astar_based::
      GenPOMovingBlockAStarSolver::extend_train_orders_of_state(
          tr1, state, {.late_entry_possible = false},
          {.time_aware_state_transitions = true}, ttd_sections, &instance);
  EXPECT_EQ(extended3.size(), 2);
  ASSERT_GE(extended3.size(), 2);
  EXPECT_EQ(extended3.at(0).train_edges, state.train_edges);
  EXPECT_EQ(extended3.at(0).stop_positions, state.stop_positions);
  EXPECT_EQ(extended3.at(1).train_edges, state.train_edges);
  EXPECT_EQ(extended3.at(1).stop_positions, state.stop_positions);

  EXPECT_EQ(extended3.at(0).vertex_orders.at(v0), state.vertex_orders.at(v0));
  EXPECT_EQ(extended3.at(1).vertex_orders.at(v0), state.vertex_orders.at(v0));

  EXPECT_EQ(extended3.at(0).vertex_orders.at(v0),
            extended3.at(0).ttd_orders.at(0));
  EXPECT_EQ(extended3.at(1).vertex_orders.at(v0),
            extended3.at(1).ttd_orders.at(0));
  EXPECT_EQ(extended3.at(0).vertex_orders.at(v7),
            extended3.at(0).ttd_orders.at(1));
  EXPECT_EQ(extended3.at(1).vertex_orders.at(v7),
            extended3.at(1).ttd_orders.at(1));

  EXPECT_NE(extended3.at(0).ttd_orders.at(1), extended3.at(1).ttd_orders.at(1));
  EXPECT_NE(extended3.at(0).vertex_orders.at(v7),
            extended3.at(1).vertex_orders.at(v7));

  std::vector<cda_rail::index_vector> expected_vertex_orders3{
      {tr1, tr2, tr4, tr3}, {tr2, tr1, tr4, tr3}};
  std::vector<cda_rail::index_vector> actual_vertex_orders3{
      extended3.at(0).vertex_orders.at(v7),
      extended3.at(1).vertex_orders.at(v7)};
  EXPECT_TRUE(std::ranges::contains(expected_vertex_orders3,
                                    extended3.at(0).vertex_orders.at(v7)));
  EXPECT_TRUE(std::ranges::contains(expected_vertex_orders3,
                                    extended3.at(1).vertex_orders.at(v7)));
  EXPECT_TRUE(std::ranges::contains(actual_vertex_orders3,
                                    expected_vertex_orders3.at(0)));
  EXPECT_TRUE(std::ranges::contains(actual_vertex_orders3,
                                    expected_vertex_orders3.at(1)));
}

TEST(GenPOMovingBlockAStarSolver, ExtendTrainOrderNothingToChange) {
  // clang-format off
  //                - v3b -- v4b -
  //               /              \
  // v0 --- v1 - v2 - v3a -- v4a - v5 - v6 -- v7
  // clang-format on

  Network    network;
  auto const v0  = network.add_vertex("v0", VertexType::TTD);
  auto const v1  = network.add_vertex("v1", VertexType::TTD);
  auto const v2  = network.add_vertex("v2", VertexType::NoBorder);
  auto const v3a = network.add_vertex("v3a", VertexType::TTD);
  auto const v3b = network.add_vertex("v3b", VertexType::TTD);
  auto const v4a = network.add_vertex("v4a", VertexType::TTD);
  auto const v4b = network.add_vertex("v4b", VertexType::TTD);
  auto const v5  = network.add_vertex("v5", VertexType::NoBorder);
  auto const v6  = network.add_vertex("v6", VertexType::TTD);
  auto const v7  = network.add_vertex("v7", VertexType::TTD);

  auto const v0_v1   = network.add_edge(v0, v1, 100, 20, true);
  auto const v1_v2   = network.add_edge(v1, v2, 10, 20, false);
  auto const v2_v3a  = network.add_edge(v2, v3a, 10, 20, false);
  auto const v2_v3b  = network.add_edge(v2, v3b, 10, 20, false);
  auto const v3a_v4a = network.add_edge(v3a, v4a, 100, 20, true);
  auto const v3b_v4b = network.add_edge(v3b, v4b, 100, 20, true);
  auto const v4a_v5  = network.add_edge(v4a, v5, 10, 20, false);
  auto const v4b_v5  = network.add_edge(v4b, v5, 10, 20, false);
  auto const v5_v6   = network.add_edge(v5, v6, 10, 20, false);
  auto const v6_v7   = network.add_edge(v6, v7, 100, 20, true);

  network.add_successor(v0_v1, v1_v2);
  network.add_successor(v1_v2, v2_v3a);
  network.add_successor(v1_v2, v2_v3b);
  network.add_successor(v2_v3a, v3a_v4a);
  network.add_successor(v2_v3b, v3b_v4b);
  network.add_successor(v3a_v4a, v4a_v5);
  network.add_successor(v3b_v4b, v4b_v5);
  network.add_successor(v4a_v5, v5_v6);
  network.add_successor(v4b_v5, v5_v6);
  network.add_successor(v5_v6, v6_v7);

  // Add all reverse edges
  auto const          num_edges = network.number_of_edges();
  std::vector<size_t> reverse_edges;
  reverse_edges.resize(num_edges);
  for (size_t i = 0; i < num_edges; ++i) {
    auto const& edge    = network.get_edge(i);
    reverse_edges.at(i) = network.add_edge(
        edge.target, edge.source, edge.length, edge.max_speed, edge.breakable);
  }

  for (size_t i = 0; i < num_edges; ++i) {
    auto const& successors = network.get_successors(i);
    for (auto const& successor : successors) {
      network.add_successor(reverse_edges.at(successor), reverse_edges.at(i));
    }
  }

  Timetable  timetable;
  const auto tr1 = timetable.add_train("Train1", 50, 20, 2, 4, true, 0, 20, v0,
                                       100, 20, v7, network);
  const auto tr2 = timetable.add_train("Train2", 50, 20, 2, 4, true, 100, 20,
                                       v7, 300, 20, v0, network);
  const auto tr3 = timetable.add_train("Train3", 50, 20, 2, 4, true, 200, 20,
                                       v7, 500, 20, v0, network);
  const auto tr4 = timetable.add_train("Train4", 50, 20, 2, 4, true, 400, 20,
                                       v0, 700, 20, v7, network);

  RouteMap                                                    routes;
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);

  cda_rail::index_set              ttd1{v1_v2,
                                        v2_v3a,
                                        v2_v3b,
                                        reverse_edges.at(v1_v2),
                                        reverse_edges.at(v2_v3a),
                                        reverse_edges.at(v2_v3b)};
  cda_rail::index_set              ttd2{v4a_v5,
                                        v4b_v5,
                                        v5_v6,
                                        reverse_edges.at(v4a_v5),
                                        reverse_edges.at(v4b_v5),
                                        reverse_edges.at(v5_v6)};
  std::vector<cda_rail::index_set> ttd_sections{ttd1, ttd2};

  simulator::SimulatorState state{
      .train_edges = std::vector<cda_rail::index_vector>(
          instance.get_const_train_list().get_number_of_trains()),
      .ttd_orders = std::vector<cda_rail::index_vector>(ttd_sections.size()),
      .vertex_orders =
          std::vector<cda_rail::index_vector>(network.number_of_vertices()),
      .stop_positions = std::vector<std::vector<double>>(
          instance.get_const_train_list().get_number_of_trains())};
  state.train_edges.at(tr4) = {v0_v1, v1_v2, v2_v3b};

  state.ttd_orders.at(0)     = {tr4};
  state.vertex_orders.at(v0) = {tr4};

  auto const extended1 = cda_rail::solver::astar_based::
      GenPOMovingBlockAStarSolver::extend_train_orders_of_state(
          tr4, state, {.late_entry_possible = false},
          {.time_aware_state_transitions = false}, ttd_sections, &instance);
  // No change
  EXPECT_EQ(extended1.size(), 1);
  ASSERT_GE(extended1.size(), 1);
  EXPECT_EQ(extended1.at(0).train_edges, state.train_edges);
  EXPECT_EQ(extended1.at(0).stop_positions, state.stop_positions);
  EXPECT_EQ(extended1.at(0).vertex_orders, state.vertex_orders);
  EXPECT_EQ(extended1.at(0).ttd_orders, state.ttd_orders);

  auto const extended2 = cda_rail::solver::astar_based::
      GenPOMovingBlockAStarSolver::extend_train_orders_of_state(
          tr4, state, {.late_entry_possible = false},
          {.time_aware_state_transitions = true}, ttd_sections, &instance);
  // No change
  EXPECT_EQ(extended2.size(), 1);
  ASSERT_GE(extended2.size(), 1);
  EXPECT_EQ(extended2.at(0).train_edges, state.train_edges);
  EXPECT_EQ(extended2.at(0).stop_positions, state.stop_positions);
  EXPECT_EQ(extended2.at(0).vertex_orders, state.vertex_orders);
  EXPECT_EQ(extended2.at(0).ttd_orders, state.ttd_orders);
}

TEST(GenPOMovingBlockAStarSolver, DesiredOrderInstanceClassical) {
  Network network;
  network.add_vertex("v0a", VertexType::TTD);
  network.add_vertex("v0b", VertexType::TTD);
  network.add_vertex("v0c", VertexType::TTD);
  network.add_vertex("v1a", VertexType::TTD);
  network.add_vertex("v1b", VertexType::TTD);
  network.add_vertex("v1c", VertexType::TTD);
  network.add_vertex("v2", VertexType::NoBorder);
  network.add_vertex("v3", VertexType::TTD);
  network.add_vertex("v4", VertexType::TTD);

  network.add_edge({"v0a"}, {"v1a"}, 100, 20, true);
  network.add_edge({"v0b"}, {"v1b"}, 100, 20, true);
  network.add_edge({"v0c"}, {"v1c"}, 100, 20, true);

  // Switch
  network.add_edge({"v1a"}, {"v2"}, 10, 20, false);
  network.add_edge({"v1b"}, {"v2"}, 10, 20, false);
  network.add_edge({"v1c"}, {"v2"}, 10, 20, false);

  network.add_edge({"v2"}, {"v3"}, 10, 20, false);
  network.add_edge({"v3"}, {"v4"}, 100, 20, true);

  network.add_successor({"v0a", "v1a"}, {"v1a", "v2"});
  network.add_successor({"v0b", "v1b"}, {"v1b", "v2"});
  network.add_successor({"v0c", "v1c"}, {"v1c", "v2"});
  network.add_successor({"v1a", "v2"}, {"v2", "v3"});
  network.add_successor({"v1b", "v2"}, {"v2", "v3"});
  network.add_successor({"v1c", "v2"}, {"v2", "v3"});
  network.add_successor({"v2", "v3"}, {"v3", "v4"});

  // Trains
  Timetable  timetable;
  const auto tr1 = timetable.add_train("Train1", 50, 20, 2, 4, true, 0, 20,
                                       {"v0a"}, 500, 20, {"v4"}, network);
  const auto tr2 = timetable.add_train("Train2", 50, 20, 2, 4, true, 100, 20,
                                       {"v0b"}, 300, 20, {"v4"}, network);
  const auto tr3 = timetable.add_train("Train3", 50, 20, 2, 4, true, 200, 20,
                                       {"v0c"}, 400, 20, {"v4"}, network);

  RouteMap                                                    routes;
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);

  instance.set_train_weight(tr1, 1);
  instance.set_train_weight(tr2, 2);
  instance.set_train_weight(tr3, 3);

  cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver solver(instance);
  const auto                                                 sol_obj_single =
      solver.solve({.dt = 5},
                   {.next_state_strategy =
                        solver::astar_based::NextStateStrategy::SingleEdge,
                    .time_aware_state_transitions = false},
                   {}, -1, true);
  const auto sol_obj_ttd = solver.solve(
      {.dt = 5},
      {.next_state_strategy = solver::astar_based::NextStateStrategy::NextTTD,
       .time_aware_state_transitions = false},
      {}, -1, true);
  const auto sol_obj_ttd_2 =
      solver.solve({.dt = 5},
                   {.next_state_strategy =
                        solver::astar_based::NextStateStrategy::NextRelevantTTD,
                    .time_aware_state_transitions = false},
                   {}, -1, true);
  const auto sol_obj_ttd_3 =
      solver.solve({.dt = 5},
                   {.next_state_strategy =
                        solver::astar_based::NextStateStrategy::NextRelevantTTD,
                    .time_aware_state_transitions = false,
                    .a_star_weight                = 2},
                   {}, -1, true);

  EXPECT_EQ(sol_obj_single.get_obj(), 1 * 500 + 2 * 300 + 3 * 400);
  EXPECT_EQ(sol_obj_ttd.get_obj(), 1 * 500 + 2 * 300 + 3 * 400);
  EXPECT_EQ(sol_obj_ttd_2.get_obj(), 1 * 500 + 2 * 300 + 3 * 400);
  EXPECT_GE(sol_obj_ttd_3.get_obj(), 1 * 500 + 2 * 300 + 3 * 400);
  EXPECT_EQ(sol_obj_single.get_lower_bound(), sol_obj_single.get_obj());
  EXPECT_EQ(sol_obj_ttd.get_lower_bound(), sol_obj_ttd.get_obj());
  EXPECT_EQ(sol_obj_ttd_2.get_lower_bound(), sol_obj_ttd_2.get_obj());
  EXPECT_LE(sol_obj_ttd_3.get_lower_bound(), sol_obj_ttd_3.get_obj());
  EXPECT_LE(sol_obj_ttd_3.get_obj(), sol_obj_ttd_3.get_lower_bound() * 2);

  auto const instance_weighted_sum = instance.sum_of_weighted_exit_times();
  EXPECT_GE(sol_obj_single.get_lower_bound(), instance_weighted_sum);
  EXPECT_GE(sol_obj_ttd.get_lower_bound(), instance_weighted_sum);
  EXPECT_GE(sol_obj_ttd_2.get_lower_bound(), instance_weighted_sum);
  EXPECT_GE(sol_obj_ttd_3.get_lower_bound(), instance_weighted_sum);

  auto const v2_v3 =
      sol_obj_single.get_instance()->get_const_network().get_edge_index({"v2"},
                                                                        {"v3"});
  auto const tr_order = sol_obj_single.get_train_order(v2_v3);
  EXPECT_EQ(tr_order.size(), 3);
  EXPECT_EQ(tr_order.at(0), tr2);
  EXPECT_EQ(tr_order.at(1), tr3);
  EXPECT_EQ(tr_order.at(2), tr1);

  EXPECT_EQ(sol_obj_ttd.get_train_order(v2_v3), tr_order);
  EXPECT_EQ(sol_obj_ttd_2.get_train_order(v2_v3), tr_order);

  EXPECT_EQ(sol_obj_single.get_exit_time("Train1"), 500);
  EXPECT_EQ(sol_obj_single.get_exit_time("Train2"), 300);
  EXPECT_EQ(sol_obj_single.get_exit_time("Train3"), 400);

  EXPECT_EQ(sol_obj_ttd.get_exit_time("Train1"), 500);
  EXPECT_EQ(sol_obj_ttd.get_exit_time("Train2"), 300);
  EXPECT_EQ(sol_obj_ttd.get_exit_time("Train3"), 400);

  EXPECT_EQ(sol_obj_ttd_2.get_exit_time("Train1"), 500);
  EXPECT_EQ(sol_obj_ttd_2.get_exit_time("Train2"), 300);
  EXPECT_EQ(sol_obj_ttd_2.get_exit_time("Train3"), 400);

  EXPECT_GE(sol_obj_single.get_train_pos(
                "Train1", sol_obj_single.get_exit_time("Train1")),
            100 + 10 + 10 + 100);
  EXPECT_GE(sol_obj_single.get_train_pos(
                "Train2", sol_obj_single.get_exit_time("Train2")),
            100 + 10 + 10 + 100);
  EXPECT_GE(sol_obj_single.get_train_pos(
                "Train3", sol_obj_single.get_exit_time("Train3")),
            100 + 10 + 10 + 100);

  EXPECT_GE(
      sol_obj_ttd.get_train_pos("Train1", sol_obj_ttd.get_exit_time("Train1")),
      100 + 10 + 10 + 100);
  EXPECT_GE(
      sol_obj_ttd.get_train_pos("Train2", sol_obj_ttd.get_exit_time("Train2")),
      100 + 10 + 10 + 100);
  EXPECT_GE(
      sol_obj_ttd.get_train_pos("Train3", sol_obj_ttd.get_exit_time("Train3")),
      100 + 10 + 10 + 100);

  EXPECT_GE(sol_obj_ttd_2.get_train_pos("Train1",
                                        sol_obj_ttd_2.get_exit_time("Train1")),
            100 + 10 + 10 + 100);
  EXPECT_GE(sol_obj_ttd_2.get_train_pos("Train2",
                                        sol_obj_ttd_2.get_exit_time("Train2")),
            100 + 10 + 10 + 100);
  EXPECT_GE(sol_obj_ttd_2.get_train_pos("Train3",
                                        sol_obj_ttd_2.get_exit_time("Train3")),
            100 + 10 + 10 + 100);
}

TEST(GenPOMovingBlockAStarSolver, DesiredOrderInstanceTimeAware) {
  Network network;
  network.add_vertex("v0a", VertexType::TTD);
  network.add_vertex("v0b", VertexType::TTD);
  network.add_vertex("v0c", VertexType::TTD);
  network.add_vertex("v1a", VertexType::TTD);
  network.add_vertex("v1b", VertexType::TTD);
  network.add_vertex("v1c", VertexType::TTD);
  network.add_vertex("v2", VertexType::NoBorder);
  network.add_vertex("v3", VertexType::TTD);
  network.add_vertex("v4", VertexType::TTD);

  network.add_edge({"v0a"}, {"v1a"}, 100, 20, true);
  network.add_edge({"v0b"}, {"v1b"}, 100, 20, true);
  network.add_edge({"v0c"}, {"v1c"}, 100, 20, true);

  // Switch
  network.add_edge({"v1a"}, {"v2"}, 10, 20, false);
  network.add_edge({"v1b"}, {"v2"}, 10, 20, false);
  network.add_edge({"v1c"}, {"v2"}, 10, 20, false);

  network.add_edge({"v2"}, {"v3"}, 10, 20, false);
  network.add_edge({"v3"}, {"v4"}, 100, 20, true);

  network.add_successor({"v0a", "v1a"}, {"v1a", "v2"});
  network.add_successor({"v0b", "v1b"}, {"v1b", "v2"});
  network.add_successor({"v0c", "v1c"}, {"v1c", "v2"});
  network.add_successor({"v1a", "v2"}, {"v2", "v3"});
  network.add_successor({"v1b", "v2"}, {"v2", "v3"});
  network.add_successor({"v1c", "v2"}, {"v2", "v3"});
  network.add_successor({"v2", "v3"}, {"v3", "v4"});

  // Trains
  Timetable  timetable;
  const auto tr1 = timetable.add_train("Train1", 50, 20, 2, 4, true, 0, 20,
                                       {"v0a"}, 500, 20, {"v4"}, network);
  const auto tr2 = timetable.add_train("Train2", 50, 20, 2, 4, true, 100, 20,
                                       {"v0b"}, 300, 20, {"v4"}, network);
  const auto tr3 = timetable.add_train("Train3", 50, 20, 2, 4, true, 200, 20,
                                       {"v0c"}, 400, 20, {"v4"}, network);

  RouteMap                                                    routes;
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);

  instance.set_train_weight(tr1, 1);
  instance.set_train_weight(tr2, 2);
  instance.set_train_weight(tr3, 3);

  cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver solver(instance);
  const auto                                                 sol_obj_single =
      solver.solve({.dt = 5},
                   {.next_state_strategy =
                        solver::astar_based::NextStateStrategy::SingleEdge,
                    .time_aware_state_transitions = true},
                   {}, -1, true);
  const auto sol_obj_ttd = solver.solve(
      {.dt = 5},
      {.next_state_strategy = solver::astar_based::NextStateStrategy::NextTTD,
       .time_aware_state_transitions = true},
      {}, -1, true);
  const auto sol_obj_ttd_2 =
      solver.solve({.dt = 5},
                   {.next_state_strategy =
                        solver::astar_based::NextStateStrategy::NextRelevantTTD,
                    .time_aware_state_transitions = true},
                   {}, -1, true);
  const auto sol_obj_ttd_3 =
      solver.solve({.dt = 5},
                   {.next_state_strategy =
                        solver::astar_based::NextStateStrategy::NextRelevantTTD,
                    .time_aware_state_transitions = true,
                    .a_star_weight                = 3},
                   {}, -1, true);

  EXPECT_EQ(sol_obj_single.get_obj(), 1 * 500 + 2 * 300 + 3 * 400);
  EXPECT_EQ(sol_obj_ttd.get_obj(), 1 * 500 + 2 * 300 + 3 * 400);
  EXPECT_EQ(sol_obj_ttd_2.get_obj(), 1 * 500 + 2 * 300 + 3 * 400);
  EXPECT_GE(sol_obj_ttd_3.get_obj(), 1 * 500 + 2 * 300 + 3 * 400);
  EXPECT_EQ(sol_obj_single.get_lower_bound(), sol_obj_single.get_obj());
  EXPECT_EQ(sol_obj_ttd.get_lower_bound(), sol_obj_ttd.get_obj());
  EXPECT_EQ(sol_obj_ttd_2.get_lower_bound(), sol_obj_ttd_2.get_obj());
  EXPECT_LE(sol_obj_ttd_3.get_lower_bound(), sol_obj_ttd_3.get_obj());
  EXPECT_LE(sol_obj_ttd_3.get_obj(), sol_obj_ttd_3.get_lower_bound() * 3);

  auto const instance_weighted_sum = instance.sum_of_weighted_exit_times();
  EXPECT_GE(sol_obj_single.get_lower_bound(), instance_weighted_sum);
  EXPECT_GE(sol_obj_ttd.get_lower_bound(), instance_weighted_sum);
  EXPECT_GE(sol_obj_ttd_2.get_lower_bound(), instance_weighted_sum);
  EXPECT_GE(sol_obj_ttd_3.get_lower_bound(), instance_weighted_sum);

  auto const v2_v3 =
      sol_obj_single.get_instance()->get_const_network().get_edge_index({"v2"},
                                                                        {"v3"});
  auto const tr_order = sol_obj_single.get_train_order(v2_v3);
  EXPECT_EQ(tr_order.size(), 3);
  EXPECT_EQ(tr_order.at(0), tr2);
  EXPECT_EQ(tr_order.at(1), tr3);
  EXPECT_EQ(tr_order.at(2), tr1);

  EXPECT_EQ(sol_obj_ttd.get_train_order(v2_v3), tr_order);
  EXPECT_EQ(sol_obj_ttd_2.get_train_order(v2_v3), tr_order);

  EXPECT_EQ(sol_obj_single.get_exit_time("Train1"), 500);
  EXPECT_EQ(sol_obj_single.get_exit_time("Train2"), 300);
  EXPECT_EQ(sol_obj_single.get_exit_time("Train3"), 400);

  EXPECT_EQ(sol_obj_ttd.get_exit_time("Train1"), 500);
  EXPECT_EQ(sol_obj_ttd.get_exit_time("Train2"), 300);
  EXPECT_EQ(sol_obj_ttd.get_exit_time("Train3"), 400);

  EXPECT_EQ(sol_obj_ttd_2.get_exit_time("Train1"), 500);
  EXPECT_EQ(sol_obj_ttd_2.get_exit_time("Train2"), 300);
  EXPECT_EQ(sol_obj_ttd_2.get_exit_time("Train3"), 400);

  EXPECT_GE(sol_obj_single.get_train_pos(
                "Train1", sol_obj_single.get_exit_time("Train1")),
            100 + 10 + 10 + 100);
  EXPECT_GE(sol_obj_single.get_train_pos(
                "Train2", sol_obj_single.get_exit_time("Train2")),
            100 + 10 + 10 + 100);
  EXPECT_GE(sol_obj_single.get_train_pos(
                "Train3", sol_obj_single.get_exit_time("Train3")),
            100 + 10 + 10 + 100);

  EXPECT_GE(
      sol_obj_ttd.get_train_pos("Train1", sol_obj_ttd.get_exit_time("Train1")),
      100 + 10 + 10 + 100);
  EXPECT_GE(
      sol_obj_ttd.get_train_pos("Train2", sol_obj_ttd.get_exit_time("Train2")),
      100 + 10 + 10 + 100);
  EXPECT_GE(
      sol_obj_ttd.get_train_pos("Train3", sol_obj_ttd.get_exit_time("Train3")),
      100 + 10 + 10 + 100);

  EXPECT_GE(sol_obj_ttd_2.get_train_pos("Train1",
                                        sol_obj_ttd_2.get_exit_time("Train1")),
            100 + 10 + 10 + 100);
  EXPECT_GE(sol_obj_ttd_2.get_train_pos("Train2",
                                        sol_obj_ttd_2.get_exit_time("Train2")),
            100 + 10 + 10 + 100);
  EXPECT_GE(sol_obj_ttd_2.get_train_pos("Train3",
                                        sol_obj_ttd_2.get_exit_time("Train3")),
            100 + 10 + 10 + 100);
}

TEST(GenPOMovingBlockAStarSolver, DesiredOrderInstanceClassical2) {
  // clang-format off
  // (tr1) -> v0a -- v1a -                    - v6a -- (short) v7a
  //                      \                  /
  //  v0b -- (short) v1b - v2 - v3 -- v4 - v5 - v6b -- v7b <- (tr2)
  // clang-format on

  Network    network;
  auto const v0a = network.add_vertex("v0a", VertexType::TTD);
  auto const v1a = network.add_vertex("v1a", VertexType::TTD);
  auto const v0b = network.add_vertex("v0b", VertexType::TTD);
  auto const v1b = network.add_vertex("v1b", VertexType::TTD);
  auto const v2  = network.add_vertex("v2", VertexType::NoBorder);
  auto const v3  = network.add_vertex("v3", VertexType::TTD);
  auto const v4  = network.add_vertex("v4", VertexType::TTD);
  auto const v5  = network.add_vertex("v5", VertexType::NoBorder);
  auto const v6a = network.add_vertex("v6a", VertexType::TTD);
  auto const v6b = network.add_vertex("v6b", VertexType::TTD);
  auto const v7a = network.add_vertex("v7a", VertexType::TTD);
  auto const v7b = network.add_vertex("v7b", VertexType::TTD);

  auto const v0a_v1a = network.add_edge(v0a, v1a, 100, 20, true);
  auto const v0b_v1b = network.add_edge(v0b, v1b, 10, 20, true);
  auto const v1a_v2  = network.add_edge(v1a, v2, 10, 20, false);
  auto const v1b_v2  = network.add_edge(v1b, v2, 10, 20, false);
  auto const v2_v3   = network.add_edge(v2, v3, 10, 20, false);
  auto const v3_v4   = network.add_edge(v3, v4, 100, 20, true);
  auto const v4_v5   = network.add_edge(v4, v5, 10, 20, false);
  auto const v5_v6a  = network.add_edge(v5, v6a, 10, 20, false);
  auto const v5_v6b  = network.add_edge(v5, v6b, 10, 20, false);
  auto const v6a_v7a = network.add_edge(v6a, v7a, 10, 20, true);
  auto const v6b_v7b = network.add_edge(v6b, v7b, 100, 20, true);

  network.add_successor(v0a_v1a, v1a_v2);
  network.add_successor(v0b_v1b, v1b_v2);
  network.add_successor(v1a_v2, v2_v3);
  network.add_successor(v1b_v2, v2_v3);
  network.add_successor(v2_v3, v3_v4);
  network.add_successor(v3_v4, v4_v5);
  network.add_successor(v4_v5, v5_v6a);
  network.add_successor(v4_v5, v5_v6b);
  network.add_successor(v5_v6a, v6a_v7a);
  network.add_successor(v5_v6b, v6b_v7b);

  // Add all reverse edges
  auto const             num_edges = network.number_of_edges();
  cda_rail::index_vector reverse_edges(num_edges);
  for (size_t i = 0; i < num_edges; ++i) {
    auto const edge     = network.get_edge(i);
    reverse_edges.at(i) = network.add_edge(
        edge.target, edge.source, edge.length, edge.max_speed, edge.breakable);
  }
  for (size_t i = 0; i < num_edges; ++i) {
    auto const& successors = network.get_successors(i);
    for (auto const& successor : successors) {
      network.add_successor(reverse_edges.at(successor), reverse_edges.at(i));
    }
  }

  Timetable  timetable;
  const auto tr1 = timetable.add_train("Train1", 50, 20, 2, 4, true, 0, 10,
                                       {"v0a"}, 500, 20, {"v7a"}, network);
  const auto tr2 = timetable.add_train("Train2", 50, 20, 2, 4, true, 100, 10,
                                       {"v7b"}, 300, 20, {"v0b"}, network);

  RouteMap                                                    routes;
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);

  instance.set_train_weight(tr1, 1);
  instance.set_train_weight(tr2, 2);

  cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver solver(instance);
  const auto                                                 sol_obj_single =
      solver.solve({.dt = 5},
                   {.next_state_strategy =
                        solver::astar_based::NextStateStrategy::SingleEdge,
                    .time_aware_state_transitions = false},
                   {}, -1, true);
  const auto sol_obj_ttd = solver.solve(
      {.dt = 5},
      {.next_state_strategy = solver::astar_based::NextStateStrategy::NextTTD,
       .time_aware_state_transitions = false},
      {}, -1, true);
  const auto sol_obj_ttd_2 =
      solver.solve({.dt = 5},
                   {.next_state_strategy =
                        solver::astar_based::NextStateStrategy::NextRelevantTTD,
                    .time_aware_state_transitions = false},
                   {}, -1, true);
  const auto sol_obj_ttd_3 =
      solver.solve({.dt = 5},
                   {.next_state_strategy =
                        solver::astar_based::NextStateStrategy::NextRelevantTTD,
                    .time_aware_state_transitions = false,
                    .a_star_weight                = 4},
                   {}, -1, true);

  EXPECT_EQ(sol_obj_single.get_obj(), 1 * 500 + 2 * 300);
  EXPECT_EQ(sol_obj_ttd.get_obj(), 1 * 500 + 2 * 300);
  EXPECT_EQ(sol_obj_ttd_2.get_obj(), 1 * 500 + 2 * 300);
  EXPECT_GE(sol_obj_ttd_3.get_obj(), 1 * 500 + 2 * 300);
  EXPECT_EQ(sol_obj_single.get_lower_bound(), sol_obj_single.get_obj());
  EXPECT_EQ(sol_obj_ttd.get_lower_bound(), sol_obj_ttd.get_obj());
  EXPECT_EQ(sol_obj_ttd_2.get_lower_bound(), sol_obj_ttd_2.get_obj());
  EXPECT_LE(sol_obj_ttd_3.get_lower_bound(), sol_obj_ttd_3.get_obj());
  EXPECT_LE(sol_obj_ttd_3.get_obj(), sol_obj_ttd_3.get_lower_bound() * 4);

  auto const instance_weighted_sum = instance.sum_of_weighted_exit_times();
  EXPECT_GE(sol_obj_single.get_lower_bound(), instance_weighted_sum);
  EXPECT_GE(sol_obj_ttd.get_lower_bound(), instance_weighted_sum);
  EXPECT_GE(sol_obj_ttd_2.get_lower_bound(), instance_weighted_sum);
  EXPECT_GE(sol_obj_ttd_3.get_lower_bound(), instance_weighted_sum);

  auto const tr1_time_single =
      sol_obj_single.get_time_at_pos("Train1", 170, true);
  auto const tr2_time_single =
      sol_obj_single.get_time_at_pos("Train2", 170, true);
  EXPECT_GT(tr1_time_single, tr2_time_single);
  auto const tr1_time_ttd = sol_obj_ttd.get_time_at_pos("Train1", 170, true);
  auto const tr2_time_ttd = sol_obj_ttd.get_time_at_pos("Train2", 170, true);
  EXPECT_GT(tr1_time_ttd, tr2_time_ttd);
  auto const tr1_time_ttd_2 =
      sol_obj_ttd_2.get_time_at_pos("Train1", 170, true);
  auto const tr2_time_ttd_2 =
      sol_obj_ttd_2.get_time_at_pos("Train2", 170, true);
  EXPECT_GT(tr1_time_ttd_2, tr2_time_ttd_2);

  EXPECT_EQ(sol_obj_single.get_exit_time("Train1"), 500);
  EXPECT_EQ(sol_obj_single.get_exit_time("Train2"), 300);

  EXPECT_EQ(sol_obj_ttd.get_exit_time("Train1"), 500);
  EXPECT_EQ(sol_obj_ttd.get_exit_time("Train2"), 300);

  EXPECT_EQ(sol_obj_ttd_2.get_exit_time("Train1"), 500);
  EXPECT_EQ(sol_obj_ttd_2.get_exit_time("Train2"), 300);

  EXPECT_GE(sol_obj_single.get_train_pos(
                "Train1", sol_obj_single.get_exit_time("Train1")),
            100 + 10 + 10 + 100 + 10 + 10);
  EXPECT_GE(sol_obj_single.get_train_pos(
                "Train2", sol_obj_single.get_exit_time("Train2")),
            100 + 10 + 10 + 100 + 10 + 10);

  EXPECT_GE(
      sol_obj_ttd.get_train_pos("Train1", sol_obj_ttd.get_exit_time("Train1")),
      100 + 10 + 10 + 100 + 10 + 10);
  EXPECT_GE(
      sol_obj_ttd.get_train_pos("Train2", sol_obj_ttd.get_exit_time("Train2")),
      100 + 10 + 10 + 100 + 10 + 10);

  EXPECT_GE(sol_obj_ttd_2.get_train_pos("Train1",
                                        sol_obj_ttd_2.get_exit_time("Train1")),
            100 + 10 + 10 + 100 + 10 + 10);
  EXPECT_GE(sol_obj_ttd_2.get_train_pos("Train2",
                                        sol_obj_ttd_2.get_exit_time("Train2")),
            100 + 10 + 10 + 100 + 10 + 10);
}

TEST(GenPOMovingBlockAStarSolver, DesiredOrderInstanceTimeAware2) {
  // clang-format off
  // (tr1) -> v0a -- v1a -                    - v6a -- (short) v7a
  //                      \                  /
  //  v0b -- (short) v1b - v2 - v3 -- v4 - v5 - v6b -- v7b <- (tr2)
  // clang-format on

  Network    network;
  auto const v0a = network.add_vertex("v0a", VertexType::TTD);
  auto const v1a = network.add_vertex("v1a", VertexType::TTD);
  auto const v0b = network.add_vertex("v0b", VertexType::TTD);
  auto const v1b = network.add_vertex("v1b", VertexType::TTD);
  auto const v2  = network.add_vertex("v2", VertexType::NoBorder);
  auto const v3  = network.add_vertex("v3", VertexType::TTD);
  auto const v4  = network.add_vertex("v4", VertexType::TTD);
  auto const v5  = network.add_vertex("v5", VertexType::NoBorder);
  auto const v6a = network.add_vertex("v6a", VertexType::TTD);
  auto const v6b = network.add_vertex("v6b", VertexType::TTD);
  auto const v7a = network.add_vertex("v7a", VertexType::TTD);
  auto const v7b = network.add_vertex("v7b", VertexType::TTD);

  auto const v0a_v1a = network.add_edge(v0a, v1a, 100, 20, true);
  auto const v0b_v1b = network.add_edge(v0b, v1b, 10, 20, true);
  auto const v1a_v2  = network.add_edge(v1a, v2, 10, 20, false);
  auto const v1b_v2  = network.add_edge(v1b, v2, 10, 20, false);
  auto const v2_v3   = network.add_edge(v2, v3, 10, 20, false);
  auto const v3_v4   = network.add_edge(v3, v4, 100, 20, true);
  auto const v4_v5   = network.add_edge(v4, v5, 10, 20, false);
  auto const v5_v6a  = network.add_edge(v5, v6a, 10, 20, false);
  auto const v5_v6b  = network.add_edge(v5, v6b, 10, 20, false);
  auto const v6a_v7a = network.add_edge(v6a, v7a, 10, 20, true);
  auto const v6b_v7b = network.add_edge(v6b, v7b, 100, 20, true);

  network.add_successor(v0a_v1a, v1a_v2);
  network.add_successor(v0b_v1b, v1b_v2);
  network.add_successor(v1a_v2, v2_v3);
  network.add_successor(v1b_v2, v2_v3);
  network.add_successor(v2_v3, v3_v4);
  network.add_successor(v3_v4, v4_v5);
  network.add_successor(v4_v5, v5_v6a);
  network.add_successor(v4_v5, v5_v6b);
  network.add_successor(v5_v6a, v6a_v7a);
  network.add_successor(v5_v6b, v6b_v7b);

  // Add all reverse edges
  auto const             num_edges = network.number_of_edges();
  cda_rail::index_vector reverse_edges(num_edges);
  for (size_t i = 0; i < num_edges; ++i) {
    auto const edge     = network.get_edge(i);
    reverse_edges.at(i) = network.add_edge(
        edge.target, edge.source, edge.length, edge.max_speed, edge.breakable);
  }
  for (size_t i = 0; i < num_edges; ++i) {
    auto const& successors = network.get_successors(i);
    for (auto const& successor : successors) {
      network.add_successor(reverse_edges.at(successor), reverse_edges.at(i));
    }
  }

  Timetable  timetable;
  const auto tr1 = timetable.add_train("Train1", 50, 20, 2, 4, true, 0, 10,
                                       {"v0a"}, 500, 20, {"v7a"}, network);
  const auto tr2 = timetable.add_train("Train2", 50, 20, 2, 4, true, 100, 10,
                                       {"v7b"}, 300, 20, {"v0b"}, network);

  RouteMap                                                    routes;
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);

  instance.set_train_weight(tr1, 1);
  instance.set_train_weight(tr2, 2);

  cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver solver(instance);
  const auto                                                 sol_obj_single =
      solver.solve({.dt = 5},
                   {.next_state_strategy =
                        solver::astar_based::NextStateStrategy::SingleEdge,
                    .time_aware_state_transitions = true},
                   {}, -1, true);
  const auto sol_obj_ttd = solver.solve(
      {.dt = 5},
      {.next_state_strategy = solver::astar_based::NextStateStrategy::NextTTD,
       .time_aware_state_transitions = true},
      {}, -1, true);
  const auto sol_obj_ttd_2 =
      solver.solve({.dt = 5},
                   {.next_state_strategy =
                        solver::astar_based::NextStateStrategy::NextRelevantTTD,
                    .time_aware_state_transitions = true},
                   {}, -1, true);
  const auto sol_obj_ttd_3 =
      solver.solve({.dt = 5},
                   {.next_state_strategy =
                        solver::astar_based::NextStateStrategy::NextRelevantTTD,
                    .time_aware_state_transitions = true,
                    .a_star_weight                = 5},
                   {}, -1, true);

  EXPECT_EQ(sol_obj_single.get_obj(), 1 * 500 + 2 * 300);
  EXPECT_EQ(sol_obj_ttd.get_obj(), 1 * 500 + 2 * 300);
  EXPECT_EQ(sol_obj_ttd_2.get_obj(), 1 * 500 + 2 * 300);
  EXPECT_GE(sol_obj_ttd_3.get_obj(), 1 * 500 + 2 * 300);
  EXPECT_EQ(sol_obj_single.get_lower_bound(), sol_obj_single.get_obj());
  EXPECT_EQ(sol_obj_ttd.get_lower_bound(), sol_obj_ttd.get_obj());
  EXPECT_EQ(sol_obj_ttd_2.get_lower_bound(), sol_obj_ttd_2.get_obj());
  EXPECT_LE(sol_obj_ttd_3.get_lower_bound(), sol_obj_ttd_3.get_obj());
  EXPECT_LE(sol_obj_ttd_3.get_obj(), sol_obj_ttd_3.get_lower_bound() * 5);

  auto const instance_weighted_sum = instance.sum_of_weighted_exit_times();
  EXPECT_GE(sol_obj_single.get_lower_bound(), instance_weighted_sum);
  EXPECT_GE(sol_obj_ttd.get_lower_bound(), instance_weighted_sum);
  EXPECT_GE(sol_obj_ttd_2.get_lower_bound(), instance_weighted_sum);
  EXPECT_GE(sol_obj_ttd_3.get_lower_bound(), instance_weighted_sum);

  auto const tr1_time_single =
      sol_obj_single.get_time_at_pos("Train1", 170, true);
  auto const tr2_time_single =
      sol_obj_single.get_time_at_pos("Train2", 170, true);
  EXPECT_GT(tr1_time_single, tr2_time_single);
  auto const tr1_time_ttd = sol_obj_ttd.get_time_at_pos("Train1", 170, true);
  auto const tr2_time_ttd = sol_obj_ttd.get_time_at_pos("Train2", 170, true);
  EXPECT_GT(tr1_time_ttd, tr2_time_ttd);
  auto const tr1_time_ttd_2 =
      sol_obj_ttd_2.get_time_at_pos("Train1", 170, true);
  auto const tr2_time_ttd_2 =
      sol_obj_ttd_2.get_time_at_pos("Train2", 170, true);
  EXPECT_GT(tr1_time_ttd_2, tr2_time_ttd_2);

  EXPECT_EQ(sol_obj_single.get_exit_time("Train1"), 500);
  EXPECT_EQ(sol_obj_single.get_exit_time("Train2"), 300);

  EXPECT_EQ(sol_obj_ttd.get_exit_time("Train1"), 500);
  EXPECT_EQ(sol_obj_ttd.get_exit_time("Train2"), 300);

  EXPECT_EQ(sol_obj_ttd_2.get_exit_time("Train1"), 500);
  EXPECT_EQ(sol_obj_ttd_2.get_exit_time("Train2"), 300);

  EXPECT_GE(sol_obj_single.get_train_pos(
                "Train1", sol_obj_single.get_exit_time("Train1")),
            100 + 10 + 10 + 100 + 10 + 10);
  EXPECT_GE(sol_obj_single.get_train_pos(
                "Train2", sol_obj_single.get_exit_time("Train2")),
            100 + 10 + 10 + 100 + 10 + 10);

  EXPECT_GE(
      sol_obj_ttd.get_train_pos("Train1", sol_obj_ttd.get_exit_time("Train1")),
      100 + 10 + 10 + 100 + 10 + 10);
  EXPECT_GE(
      sol_obj_ttd.get_train_pos("Train2", sol_obj_ttd.get_exit_time("Train2")),
      100 + 10 + 10 + 100 + 10 + 10);

  EXPECT_GE(sol_obj_ttd_2.get_train_pos("Train1",
                                        sol_obj_ttd_2.get_exit_time("Train1")),
            100 + 10 + 10 + 100 + 10 + 10);
  EXPECT_GE(sol_obj_ttd_2.get_train_pos("Train2",
                                        sol_obj_ttd_2.get_exit_time("Train2")),
            100 + 10 + 10 + 100 + 10 + 10);
}

// ----------------
// Original Tests
// ----------------

TEST(GenPOMovingBlockAStarSolver, NextStates) {
  Network    network;
  const auto v0  = network.add_vertex("v0", VertexType::TTD);
  const auto v1  = network.add_vertex("v1", VertexType::TTD);
  const auto v2  = network.add_vertex("v2", VertexType::TTD);
  const auto v3a = network.add_vertex("v3a", VertexType::TTD);
  const auto v3b = network.add_vertex("v3b", VertexType::TTD);
  const auto v4a = network.add_vertex("v4a", VertexType::TTD);
  const auto v4b = network.add_vertex("v4b", VertexType::TTD);
  const auto v5  = network.add_vertex("v5", VertexType::TTD);
  const auto v6  = network.add_vertex("v6", VertexType::TTD);
  const auto v7  = network.add_vertex("v7", VertexType::TTD);

  const auto v0_v1   = network.add_edge(v0, v1, 100, 50);
  const auto v1_v2   = network.add_edge(v1, v2, 10, 50, false);
  const auto v2_v3a  = network.add_edge(v2, v3a, 10, 50, false);
  const auto v2_v3b  = network.add_edge(v2, v3b, 10, 50, false);
  const auto v3a_v4a = network.add_edge(v3a, v4a, 200, 50);
  const auto v3b_v4b = network.add_edge(v3b, v4b, 210, 50);
  const auto v4a_v5  = network.add_edge(v4a, v5, 10, 50, false);
  const auto v4b_v5  = network.add_edge(v4b, v5, 10, 50, false);
  const auto v5_v6   = network.add_edge(v5, v6, 10, 50, false);
  const auto v6_v7   = network.add_edge(v6, v7, 200, 50);

  network.add_successor(v0_v1, v1_v2);
  network.add_successor(v1_v2, v2_v3a);
  network.add_successor(v1_v2, v2_v3b);
  network.add_successor(v2_v3a, v3a_v4a);
  network.add_successor(v2_v3b, v3b_v4b);
  network.add_successor(v3a_v4a, v4a_v5);
  network.add_successor(v3b_v4b, v4b_v5);
  network.add_successor(v4a_v5, v5_v6);
  network.add_successor(v4b_v5, v5_v6);
  network.add_successor(v5_v6, v6_v7);

  Timetable timetable;
  timetable.add_empty_station("Station1");
  timetable.add_track_to_station("Station1", v3a_v4a, network);
  timetable.add_track_to_station("Station1", v3b_v4b, network);
  timetable.add_empty_station("Station2");
  timetable.add_track_to_station("Station2", v6_v7, network);

  const auto tr1 = timetable.add_train("Train1", 100, 50, 4, 2, true, 0, 15, v0,
                                       300, 40, v7, network);
  const auto tr2 = timetable.add_train("Train2", 100, 50, 4, 2, true, 0, 25, v0,
                                       300, 40, v7, network);
  timetable.insert_stop(tr1, "Station1", 20, 30);
  timetable.insert_stop(tr1, "Station2", 200, 30);

  RouteMap                                                    routes;
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);

  std::vector<cda_rail::index_set> ttd_sections{{v1_v2, v2_v3a, v2_v3b},
                                                {v4a_v5, v4b_v5, v5_v6}};

  simulator::SimulatorState simulator_state{
      .train_edges = std::vector<cda_rail::index_vector>(
          instance.get_const_train_list().get_number_of_trains()),
      .ttd_orders = std::vector<cda_rail::index_vector>(ttd_sections.size()),
      .vertex_orders =
          std::vector<cda_rail::index_vector>(network.number_of_vertices()),
      .stop_positions = std::vector<std::vector<double>>(
          instance.get_const_train_list().get_number_of_trains())};

  simulator::SimulatorState expected_state1_1{
      .train_edges    = {{v0_v1}, {}},
      .ttd_orders     = {{}, {}},
      .vertex_orders  = std::vector<std::vector<size_t>>(10),
      .stop_positions = {{}, {}}};
  expected_state1_1.vertex_orders.at(v0).emplace_back(tr1);
  simulator::SimulatorState expected_state1_2{
      .train_edges    = {{}, {v0_v1, v1_v2, v2_v3a, v3a_v4a}},
      .ttd_orders     = {{tr2}, {}},
      .vertex_orders  = std::vector<std::vector<size_t>>(10),
      .stop_positions = {{}, {}}};
  expected_state1_2.vertex_orders.at(v0).emplace_back(tr2);
  simulator::SimulatorState expected_state1_3{
      .train_edges    = {{}, {v0_v1, v1_v2, v2_v3b, v3b_v4b}},
      .ttd_orders     = {{tr2}, {}},
      .vertex_orders  = std::vector<std::vector<size_t>>(10),
      .stop_positions = {{}, {}}};
  expected_state1_3.vertex_orders.at(v0).emplace_back(tr2);
  const auto next_states1 =
      cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::next_states(
          simulator_state, {}, {},
          {.next_state_strategy =
               cda_rail::solver::astar_based::NextStateStrategy::SingleEdge,
           .time_aware_state_transitions = false},
          &instance, ttd_sections);
  EXPECT_EQ(next_states1.size(), 3);
  EXPECT_TRUE(std::ranges::contains(next_states1, expected_state1_1));
  EXPECT_TRUE(std::ranges::contains(next_states1, expected_state1_2));
  EXPECT_TRUE(std::ranges::contains(next_states1, expected_state1_3));

  simulator_state.train_edges.at(tr2)  = {v0_v1, v1_v2};
  simulator_state.vertex_orders.at(v0) = {tr2};
  simulator_state.ttd_orders.at(0)     = {tr2};

  simulator::SimulatorState expected_state2_1{
      .train_edges    = {{v0_v1}, {v0_v1, v1_v2}},
      .ttd_orders     = {{tr2}, {}},
      .vertex_orders  = std::vector<std::vector<size_t>>(10),
      .stop_positions = {{}, {}}};
  expected_state2_1.vertex_orders.at(v0).emplace_back(tr2);
  expected_state2_1.vertex_orders.at(v0).emplace_back(tr1);
  simulator::SimulatorState expected_state2_2 = {
      .train_edges    = {{}, {v0_v1, v1_v2, v2_v3a}},
      .ttd_orders     = {{tr2}, {}},
      .vertex_orders  = std::vector<std::vector<size_t>>(10),
      .stop_positions = {{}, {}}};
  expected_state2_2.vertex_orders.at(v0).emplace_back(tr2);
  simulator::SimulatorState expected_state2_3 = {
      .train_edges    = {{}, {v0_v1, v1_v2, v2_v3b}},
      .ttd_orders     = {{tr2}, {}},
      .vertex_orders  = std::vector<std::vector<size_t>>(10),
      .stop_positions = {{}, {}}};
  expected_state2_3.vertex_orders.at(v0).emplace_back(tr2);
  const auto next_states2 =
      cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::next_states(
          simulator_state, {}, {},
          {.next_state_strategy =
               cda_rail::solver::astar_based::NextStateStrategy::SingleEdge,
           .time_aware_state_transitions = false},
          &instance, ttd_sections);
  EXPECT_EQ(next_states2.size(), 3);
  EXPECT_TRUE(std::ranges::contains(next_states2, expected_state2_1));
  EXPECT_TRUE(std::ranges::contains(next_states2, expected_state2_2));
  EXPECT_TRUE(std::ranges::contains(next_states2, expected_state2_3));

  simulator_state.train_edges.at(tr2)  = {v0_v1,   v1_v2,  v2_v3a,
                                          v3a_v4a, v4a_v5, v5_v6};
  simulator_state.train_edges.at(tr1)  = {v0_v1};
  simulator_state.vertex_orders.at(v0) = {tr2, tr1};
  simulator_state.ttd_orders.at(0)     = {tr2};
  simulator_state.ttd_orders.at(1)     = {tr2};

  simulator::SimulatorState expected_state3_1{
      .train_edges    = {{v0_v1},
                         {v0_v1, v1_v2, v2_v3a, v3a_v4a, v4a_v5, v5_v6, v6_v7}},
      .ttd_orders     = {{tr2}, {tr2}},
      .vertex_orders  = std::vector<std::vector<size_t>>(10),
      .stop_positions = {{}, {}}};
  expected_state3_1.vertex_orders.at(v0) = {tr2, tr1};
  expected_state3_1.vertex_orders.at(v7).emplace_back(tr2);
  simulator::SimulatorState expected_state3_2 = {
      .train_edges    = {{v0_v1, v1_v2},
                         {v0_v1, v1_v2, v2_v3a, v3a_v4a, v4a_v5, v5_v6}},
      .ttd_orders     = {{tr2, tr1}, {tr2}},
      .vertex_orders  = std::vector<std::vector<size_t>>(10),
      .stop_positions = {{}, {}}};
  expected_state3_2.vertex_orders.at(v0) = {tr2, tr1};
  const auto next_states3 =
      cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::next_states(
          simulator_state, {}, {},
          {.next_state_strategy =
               cda_rail::solver::astar_based::NextStateStrategy::SingleEdge,
           .time_aware_state_transitions = false},
          &instance, ttd_sections);
  EXPECT_EQ(next_states3.size(), 2);
  EXPECT_TRUE(std::ranges::contains(next_states3, expected_state3_1));
  EXPECT_TRUE(std::ranges::contains(next_states3, expected_state3_2));

  simulator_state.train_edges.at(tr2)  = {v0_v1,  v1_v2, v2_v3a, v3a_v4a,
                                          v4a_v5, v5_v6, v6_v7};
  simulator_state.vertex_orders.at(v7) = {tr2};
  simulator_state.train_edges.at(tr1)  = {v0_v1, v1_v2, v2_v3b};
  simulator_state.ttd_orders.at(0)     = {tr2, tr1};

  simulator::SimulatorState expected_state4_1{
      .train_edges    = {{v0_v1, v1_v2, v2_v3b, v3b_v4b},
                         {v0_v1, v1_v2, v2_v3a, v3a_v4a, v4a_v5, v5_v6, v6_v7}},
      .ttd_orders     = {{tr2, tr1}, {tr2}},
      .vertex_orders  = std::vector<std::vector<size_t>>(10),
      .stop_positions = {{}, {}}};
  expected_state4_1.vertex_orders.at(v0)      = {tr2, tr1};
  expected_state4_1.vertex_orders.at(v7)      = {tr2};
  simulator::SimulatorState expected_state4_2 = {
      .train_edges    = {{v0_v1, v1_v2, v2_v3b, v3b_v4b},
                         {v0_v1, v1_v2, v2_v3a, v3a_v4a, v4a_v5, v5_v6, v6_v7}},
      .ttd_orders     = {{tr2, tr1}, {tr2}},
      .vertex_orders  = std::vector<std::vector<size_t>>(10),
      .stop_positions = {{330}, {}}};
  expected_state4_2.vertex_orders.at(v0) = {tr2, tr1};
  expected_state4_2.vertex_orders.at(v7) = {tr2};
  const auto next_states4 =
      cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::next_states(
          simulator_state, {}, {},
          {.next_state_strategy =
               cda_rail::solver::astar_based::NextStateStrategy::SingleEdge,
           .time_aware_state_transitions = false},
          &instance, ttd_sections);
  EXPECT_EQ(next_states4.size(), 2);
  EXPECT_TRUE(std::ranges::contains(next_states4, expected_state4_1));
  EXPECT_TRUE(std::ranges::contains(next_states4, expected_state4_2));

  simulator_state = expected_state4_2;

  simulator::SimulatorState expected_state5_1 = expected_state4_2;
  expected_state5_1.train_edges.at(tr1).emplace_back(v4b_v5);
  expected_state5_1.ttd_orders.at(1).emplace_back(tr1);

  const auto next_states5 =
      cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::next_states(
          simulator_state, {}, {},
          {.next_state_strategy =
               cda_rail::solver::astar_based::NextStateStrategy::SingleEdge,
           .time_aware_state_transitions = false},
          &instance, ttd_sections);
  EXPECT_EQ(next_states5.size(), 1);
  EXPECT_TRUE(std::ranges::contains(next_states5, expected_state5_1));

  simulator_state.train_edges.at(tr1) = {v0_v1,   v1_v2,  v2_v3b,
                                         v3b_v4b, v4b_v5, v5_v6};
  simulator_state.ttd_orders.at(1)    = {tr2, tr1};

  simulator::SimulatorState expected_state6_1 = {
      .train_edges    = {{v0_v1, v1_v2, v2_v3b, v3b_v4b, v4b_v5, v5_v6, v6_v7},
                         {v0_v1, v1_v2, v2_v3a, v3a_v4a, v4a_v5, v5_v6, v6_v7}},
      .ttd_orders     = {{tr2, tr1}, {tr2, tr1}},
      .vertex_orders  = std::vector<std::vector<size_t>>(10),
      .stop_positions = {{330, 550}, {}}};
  expected_state6_1.vertex_orders.at(v0) = {tr2, tr1};
  expected_state6_1.vertex_orders.at(v7) = {tr2, tr1};
  const auto next_states6 =
      cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::next_states(
          simulator_state, {}, {},
          {.next_state_strategy =
               cda_rail::solver::astar_based::NextStateStrategy::SingleEdge,
           .time_aware_state_transitions = false},
          &instance, ttd_sections);
  EXPECT_EQ(next_states6.size(), 1);
  EXPECT_TRUE(std::ranges::contains(next_states6, expected_state6_1));

  simulator_state = expected_state6_1;
  const auto next_states7 =
      cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::next_states(
          simulator_state, {}, {},
          {.next_state_strategy =
               cda_rail::solver::astar_based::NextStateStrategy::SingleEdge,
           .time_aware_state_transitions = false},
          &instance, ttd_sections);
  EXPECT_TRUE(next_states7.empty());
}

TEST(GenPOMovingBlockAStarSolver, NextStatesTTD) {
  Network    network;
  const auto v0  = network.add_vertex("v0", VertexType::TTD);
  const auto v1  = network.add_vertex("v1", VertexType::TTD);
  const auto v2  = network.add_vertex("v2", VertexType::NoBorder);
  const auto v3a = network.add_vertex("v3a", VertexType::TTD);
  const auto v3b = network.add_vertex("v3b", VertexType::TTD);
  const auto v4a = network.add_vertex("v4a", VertexType::TTD);
  const auto v5a = network.add_vertex("v5a", VertexType::TTD);
  const auto v5b = network.add_vertex("v5b", VertexType::TTD);
  const auto v6  = network.add_vertex("v6", VertexType::NoBorder);
  const auto v7a = network.add_vertex("v7a", VertexType::TTD);
  const auto v7b = network.add_vertex("v7b", VertexType::TTD);
  const auto v8a = network.add_vertex("v8a", VertexType::TTD);
  const auto v8b = network.add_vertex("v8b", VertexType::TTD);

  const auto v0_v1   = network.add_edge(v0, v1, 100, 50, true);
  const auto v1_v2   = network.add_edge(v1, v2, 10, 50, false);
  const auto v2_v3a  = network.add_edge(v2, v3a, 10, 50, false);
  const auto v2_v3b  = network.add_edge(v2, v3b, 10, 50, false);
  const auto v3a_v4a = network.add_edge(v3a, v4a, 100, 50, true);
  const auto v4a_v5a = network.add_edge(v4a, v5a, 100, 50, true);
  const auto v3b_v5b = network.add_edge(v3b, v5b, 200, 50, true);
  const auto v5a_v6  = network.add_edge(v5a, v6, 10, 50, false);
  const auto v5b_v6  = network.add_edge(v5b, v6, 10, 50, false);
  const auto v6_v7a  = network.add_edge(v6, v7a, 10, 50, false);
  const auto v6_v7b  = network.add_edge(v6, v7b, 10, 50, false);
  const auto v7a_v8a = network.add_edge(v7a, v8a, 200, 50, true);
  const auto v7b_v8b = network.add_edge(v7b, v8b, 200, 50, true);

  network.add_successor(v0_v1, v1_v2);
  network.add_successor(v1_v2, v2_v3a);
  network.add_successor(v1_v2, v2_v3b);
  network.add_successor(v2_v3a, v3a_v4a);
  network.add_successor(v2_v3b, v3b_v5b);
  network.add_successor(v3a_v4a, v4a_v5a);
  network.add_successor(v3b_v5b, v5b_v6);
  network.add_successor(v4a_v5a, v5a_v6);
  network.add_successor(v5a_v6, v6_v7a);
  network.add_successor(v5a_v6, v6_v7b);
  network.add_successor(v5b_v6, v6_v7a);
  network.add_successor(v5b_v6, v6_v7b);
  network.add_successor(v6_v7a, v7a_v8a);
  network.add_successor(v6_v7b, v7b_v8b);

  Timetable timetable;
  timetable.add_empty_station("Station1");
  timetable.add_track_to_station("Station1", v3a_v4a, network);
  timetable.add_track_to_station("Station1", v4a_v5a, network);
  timetable.add_track_to_station("Station1", v3b_v5b, network);
  timetable.add_empty_station("Station2");
  timetable.add_track_to_station("Station2", v7a_v8a, network);
  timetable.add_track_to_station("Station2", v7b_v8b, network);
  timetable.add_empty_station("Station0");
  timetable.add_track_to_station("Station0", v0_v1, network);

  const auto tr1 = timetable.add_train("Train1", 100, 50, 4, 2, true, 0, 15, v0,
                                       300, 40, v8a, network);
  timetable.insert_stop(tr1, "Station0", 20, 30);
  timetable.insert_stop(tr1, "Station1", 100, 30);
  timetable.insert_stop(tr1, "Station2", 200, 30);

  RouteMap                                                    routes;
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);

  const std::vector<cda_rail::index_set> ttd_sections = {
      {v1_v2, v2_v3a, v3a_v4a}, {v5a_v6, v5b_v6, v6_v7a, v7a_v8a}};

  const auto num_vertices = network.number_of_vertices();

  simulator::SimulatorState simulator_state{
      .train_edges = std::vector<std::vector<size_t>>(
          instance.get_const_train_list().get_number_of_trains()),
      .ttd_orders     = std::vector<std::vector<size_t>>(ttd_sections.size()),
      .vertex_orders  = std::vector<std::vector<size_t>>(num_vertices),
      .stop_positions = std::vector<std::vector<double>>(
          instance.get_const_train_list().get_number_of_trains())};

  simulator::SimulatorState expected_state1_1{
      .train_edges    = {{v0_v1}},
      .ttd_orders     = {{}, {}},
      .vertex_orders  = std::vector<std::vector<size_t>>(num_vertices),
      .stop_positions = {{}}};
  expected_state1_1.vertex_orders.at(v0).emplace_back(tr1);
  simulator::SimulatorState expected_state1_2{
      .train_edges    = {{v0_v1}},
      .ttd_orders     = {{}, {}},
      .vertex_orders  = std::vector<std::vector<size_t>>(num_vertices),
      .stop_positions = {{100}}};
  expected_state1_2.vertex_orders.at(v0).emplace_back(tr1);
  const auto next_states1 =
      cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::next_states(
          simulator_state, {}, {},
          {.next_state_strategy =
               cda_rail::solver::astar_based::NextStateStrategy::NextTTD,
           .time_aware_state_transitions = false},
          &instance, ttd_sections);
  EXPECT_EQ(next_states1.size(), 2);
  EXPECT_TRUE(std::ranges::contains(next_states1, expected_state1_1));
  EXPECT_TRUE(std::ranges::contains(next_states1, expected_state1_2));

  simulator_state.train_edges.at(tr1)  = {v0_v1};
  simulator_state.vertex_orders.at(v0) = {tr1};

  simulator::SimulatorState expected_state2_1{
      .train_edges    = {{v0_v1, v1_v2, v2_v3a, v3a_v4a, v4a_v5a}},
      .ttd_orders     = {{tr1}, {}},
      .vertex_orders  = std::vector<std::vector<size_t>>(num_vertices),
      .stop_positions = {{}}};
  expected_state2_1.vertex_orders.at(v0).emplace_back(tr1);
  simulator::SimulatorState expected_state2_2{
      .train_edges    = {{v0_v1, v1_v2, v2_v3b, v3b_v5b}},
      .ttd_orders     = {{tr1}, {}},
      .vertex_orders  = std::vector<std::vector<size_t>>(num_vertices),
      .stop_positions = {{}}};
  expected_state2_2.vertex_orders.at(v0).emplace_back(tr1);
  const auto next_states2 =
      cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::next_states(
          simulator_state, {}, {},
          {.next_state_strategy =
               cda_rail::solver::astar_based::NextStateStrategy::NextTTD,
           .time_aware_state_transitions = false},
          &instance, ttd_sections);
  EXPECT_EQ(next_states2.size(), 2);
  EXPECT_TRUE(std::ranges::contains(next_states2, expected_state2_1));
  EXPECT_TRUE(std::ranges::contains(next_states2, expected_state2_2));

  simulator_state.stop_positions.at(tr1).emplace_back(100);

  simulator::SimulatorState expected_state3_1 = expected_state2_1;
  expected_state3_1.stop_positions.at(0).emplace_back(100);
  simulator::SimulatorState expected_state3_2 = expected_state2_2;
  expected_state3_2.stop_positions.at(0).emplace_back(100);
  simulator::SimulatorState expected_state3_3{
      .train_edges    = {{v0_v1, v1_v2, v2_v3a, v3a_v4a, v4a_v5a}},
      .ttd_orders     = {{tr1}, {}},
      .vertex_orders  = std::vector<std::vector<size_t>>(num_vertices),
      .stop_positions = {{100, 320}}};
  expected_state3_3.vertex_orders.at(v0).emplace_back(tr1);
  simulator::SimulatorState expected_state3_4{
      .train_edges    = {{v0_v1, v1_v2, v2_v3a, v3a_v4a}},
      .ttd_orders     = {{tr1}, {}},
      .vertex_orders  = std::vector<std::vector<size_t>>(num_vertices),
      .stop_positions = {{100, 220}}};
  expected_state3_4.vertex_orders.at(v0).emplace_back(tr1);
  simulator::SimulatorState expected_state3_5{
      .train_edges    = {{v0_v1, v1_v2, v2_v3b, v3b_v5b}},
      .ttd_orders     = {{tr1}, {}},
      .vertex_orders  = std::vector<std::vector<size_t>>(num_vertices),
      .stop_positions = {{100, 320}}};
  expected_state3_5.vertex_orders.at(v0).emplace_back(tr1);
  const auto next_states3 =
      cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::next_states(
          simulator_state, {}, {},
          {.next_state_strategy =
               cda_rail::solver::astar_based::NextStateStrategy::NextTTD,
           .time_aware_state_transitions = false},
          &instance, ttd_sections);
  EXPECT_EQ(next_states3.size(), 5);
  EXPECT_TRUE(std::ranges::contains(next_states3, expected_state3_1));
  EXPECT_TRUE(std::ranges::contains(next_states3, expected_state3_2));
  EXPECT_TRUE(std::ranges::contains(next_states3, expected_state3_3));
  EXPECT_TRUE(std::ranges::contains(next_states3, expected_state3_4));
  EXPECT_TRUE(std::ranges::contains(next_states3, expected_state3_5));

  simulator_state.train_edges.at(tr1) = {v0_v1, v1_v2, v2_v3a, v3a_v4a,
                                         v4a_v5a};
  simulator_state.ttd_orders.at(0)    = {tr1};
  simulator_state.stop_positions.at(tr1).emplace_back(320);

  simulator::SimulatorState expected_state4_1{
      .train_edges   = {{v0_v1, v1_v2, v2_v3a, v3a_v4a, v4a_v5a, v5a_v6, v6_v7a,
                         v7a_v8a}},
      .ttd_orders    = {{tr1}, {tr1}},
      .vertex_orders = std::vector<std::vector<size_t>>(num_vertices),
      .stop_positions = {{100, 320, 540}}};
  expected_state4_1.vertex_orders.at(v0).emplace_back(tr1);
  expected_state4_1.vertex_orders.at(v8a).emplace_back(tr1);
  const auto next_states4 =
      cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver::next_states(
          simulator_state, {}, {},
          {.next_state_strategy =
               cda_rail::solver::astar_based::NextStateStrategy::NextTTD,
           .time_aware_state_transitions = false},
          &instance, ttd_sections);
  EXPECT_EQ(next_states4.size(), 1);
  EXPECT_TRUE(std::ranges::contains(next_states4, expected_state4_1));
}

TEST(GenPOMovingBlockAStarSolver, SimpleInstance) {
  Network    network;
  const auto v0 = network.add_vertex("v0", VertexType::TTD, 60);
  const auto v1 = network.add_vertex("v1", VertexType::TTD, 30);

  const auto v1_v0 = network.add_edge(v1, v0, 500, 20, true);
  const auto v0_v1 = network.add_edge(v0, v1, 5000, 50, true);
  Timetable  timetable;
  const auto tr1 = timetable.add_train("Train1", 100, 50, 4, 2, true, 0, 15, v0,
                                       30, 40, v1, network);
  RouteMap   routes;
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);

  cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver solver(instance);
  const auto sol_obj = solver.solve(-1, true);

  EXPECT_TRUE(sol_obj.has_solution());
  EXPECT_EQ(sol_obj.get_status(), cda_rail::SolutionStatus::Optimal);
}

TEST(GenPOMovingBlockAStarSolver, SimpleInfeasibleInstance) {
  Network    network;
  const auto v0 = network.add_vertex("v0", VertexType::TTD, 60);
  const auto v1 = network.add_vertex("v1", VertexType::TTD, 30);

  const auto v1_v0 = network.add_edge(v1, v0, 500, 20, true);
  const auto v0_v1 = network.add_edge(v0, v1, 5000, 50, true);
  Timetable  timetable;
  const auto tr1 = timetable.add_train("Train1", 100, 50, 4, 2, true, 0, 15, v0,
                                       30, 40, v1, network);
  const auto tr2 = timetable.add_train("Train2", 100, 50, 4, 2, true, 0, 15, v1,
                                       30, 40, v0, network);

  RouteMap                                                    routes;
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);

  cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver solver(instance);
  const auto sol_obj = solver.solve(-1, true);

  EXPECT_FALSE(sol_obj.has_solution());
  EXPECT_EQ(sol_obj.get_status(), cda_rail::SolutionStatus::Infeasible);
}

TEST(GenPOMovingBlockAStarSolver, SimpleSolutionExtraction) {
  Network    network;
  const auto v0 = network.add_vertex("v0", VertexType::TTD, 60);
  const auto v1 = network.add_vertex("v1", VertexType::TTD, 30);

  const auto v0_v1 = network.add_edge(v0, v1, 500, 20, true);
  Timetable  timetable;
  const auto tr1 = timetable.add_train("Train1", 100, 50, 2, 1, true, 10, 0, v0,
                                       10, 20, v1, network);
  RouteMap   routes;
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);

  cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver solver(instance);
  const auto sol_obj = solver.solve({.dt = 5}, {}, {}, -1, false);

  EXPECT_TRUE(sol_obj.has_solution());
  EXPECT_EQ(sol_obj.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(sol_obj.get_const_solution_routes().get_route("Train1").size(), 1);
  EXPECT_EQ(
      sol_obj.get_const_solution_routes().get_route("Train1").get_edge_id(0),
      v0_v1);

  EXPECT_EQ(sol_obj.get_train_times("Train1").size(), 7);
  ASSERT_GE(sol_obj.get_train_times("Train1").size(), 7);
  EXPECT_EQ(sol_obj.get_train_times("Train1").at(0), 10);
  EXPECT_EQ(sol_obj.get_train_times("Train1").at(1), 15);
  EXPECT_EQ(sol_obj.get_train_times("Train1").at(2), 20);
  EXPECT_EQ(sol_obj.get_train_times("Train1").at(3), 25);
  EXPECT_EQ(sol_obj.get_train_times("Train1").at(4), 30);
  EXPECT_EQ(sol_obj.get_train_times("Train1").at(5), 35);
  EXPECT_EQ(sol_obj.get_train_times("Train1").at(6), 40);

  // At time 10, the train enters
  EXPECT_EQ(sol_obj.get_train_pos("Train1", 10), 0);
  EXPECT_EQ(sol_obj.get_train_speed("Train1", 10), 0);

  // t = 15 -> v = 0 + 5*2 = 10 -> s = 0 + (0 + 10)/2 * 5 = 25
  EXPECT_EQ(sol_obj.get_train_pos("Train1", 15), 25);
  EXPECT_EQ(sol_obj.get_train_speed("Train1", 15), 10);

  // t = 20 -> v = 10 + 5*2 = 20 -> s = 25 + (10 + 20)/2 * 5 = 100
  EXPECT_EQ(sol_obj.get_train_pos("Train1", 20), 100);
  EXPECT_EQ(sol_obj.get_train_speed("Train1", 20), 20);

  // t = 25 -> v = 20 -> s = 100 + (20 + 20)/2 * 5 = 200
  EXPECT_EQ(sol_obj.get_train_pos("Train1", 25), 200);
  EXPECT_EQ(sol_obj.get_train_speed("Train1", 25), 20);

  // t = 30 -> v = 20 -> s = 200 + (20 + 20)/2 * 5 = 300
  EXPECT_EQ(sol_obj.get_train_pos("Train1", 30), 300);
  EXPECT_EQ(sol_obj.get_train_speed("Train1", 30), 20);

  // t = 35 -> v = 20 -> s = 300 + (20 + 20)/2 * 5 = 400
  EXPECT_EQ(sol_obj.get_train_pos("Train1", 35), 400);
  EXPECT_EQ(sol_obj.get_train_speed("Train1", 35), 20);

  // t = 40 -> v = 20 -> s = 400 + (20 + 20)/2 * 5 = 500
  EXPECT_EQ(sol_obj.get_train_pos("Train1", 40), 500);
  EXPECT_EQ(sol_obj.get_train_speed("Train1", 40), 20);
}

TEST(GenPOMovingBlockAStarSolver, GeneralSimpleNetwork) {
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      "GeneralSimpleNetwork3Trains", "gen-po", "./data");

  cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver solver(instance);
  const auto sol_obj = solver.solve(
      {},
      {.next_state_strategy =
           cda_rail::solver::astar_based::NextStateStrategy::NextTTD,
       .consider_earliest_exit = true},
      {}, -1, false);

  EXPECT_TRUE(sol_obj.has_solution());
  EXPECT_EQ(sol_obj.get_status(), cda_rail::SolutionStatus::Optimal);
}

TEST(GenPOMovingBlockAStarSolver, Timeout) {
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      "GeneralSimpleNetwork6Trains", "gen-po", "./data");

  cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver solver(instance);
  const auto sol_obj = solver.solve(
      {},
      {.next_state_strategy =
           cda_rail::solver::astar_based::NextStateStrategy::NextTTD,
       .consider_earliest_exit = true},
      {}, 1, false);

  EXPECT_FALSE(sol_obj.has_solution());
  EXPECT_EQ(sol_obj.get_status(), cda_rail::SolutionStatus::Timeout);
}

TEST(GenPOMovingBlockAStarSolver, SimpleSolutionExport) {
  Network network;
  network.set_network_name("testnetwork");

  const auto v0 = network.add_vertex("v0", VertexType::TTD, 60);
  const auto v1 = network.add_vertex("v1", VertexType::TTD, 30);

  const auto v0_v1 = network.add_edge(v0, v1, 500, 20, true);
  Timetable  timetable;
  const auto tr1 = timetable.add_train("Train1", 100, 50, 2, 1, true, 10, 0, v0,
                                       10, 20, v1, network);
  RouteMap   routes;
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      network, timetable, routes);

  instance.set_instance_name("testinstance");
  instance.set_instance_subdirectory("instancesubdirectory");

  std::filesystem::remove_all("tmp1folder");
  std::filesystem::remove_all("tmp2folder");
  std::filesystem::remove_all("tmp3folder");

  cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver solver(instance);
  const auto sol_obj = solver.solve(
      {.dt = 5}, {},
      {.export_option         = cda_rail::solver::GeneralExportOption::NoExport,
       .working_directory     = "tmp1folder",
       .solution_subdirectory = "tmp1subdirectory",
       .parameter_identifier  = "tmp1id"},
      -1, false);
  EXPECT_FALSE(std::filesystem::exists("tmp1folder"));

  const auto sol_obj_2 = solver.solve(
      {.dt = 5}, {},
      {.export_option = cda_rail::solver::GeneralExportOption::ExportSolution,
       .working_directory     = "tmp2folder",
       .solution_subdirectory = "tmp2subdirectory",
       .parameter_identifier  = "tmp2id"},
      -1, false);

  // Check folder structure
  EXPECT_TRUE(std::filesystem::exists("tmp2folder"));
  EXPECT_TRUE(std::filesystem::exists("tmp2folder/solutions"));
  EXPECT_TRUE(std::filesystem::exists("tmp2folder/solutions/tmp2subdirectory"));
  EXPECT_TRUE(std::filesystem::exists(
      "tmp2folder/solutions/tmp2subdirectory/instancesubdirectory"));
  EXPECT_TRUE(
      std::filesystem::exists("tmp2folder/solutions/tmp2subdirectory/"
                              "instancesubdirectory/testinstance-tmp2id"));
  EXPECT_TRUE(std::filesystem::exists(
      "tmp2folder/solutions/tmp2subdirectory/instancesubdirectory/"
      "testinstance-tmp2id/solution_data.json"));
  EXPECT_TRUE(std::filesystem::exists(
      "tmp2folder/solutions/tmp2subdirectory/instancesubdirectory/"
      "testinstance-tmp2id/solver_data.json"));
  EXPECT_TRUE(std::filesystem::exists(
      "tmp2folder/solutions/tmp2subdirectory/instancesubdirectory/"
      "testinstance-tmp2id/routes.json"));
  EXPECT_TRUE(std::filesystem::exists(
      "tmp2folder/solutions/tmp2subdirectory/instancesubdirectory/"
      "testinstance-tmp2id/train_pos.json"));
  EXPECT_TRUE(std::filesystem::exists(
      "tmp2folder/solutions/tmp2subdirectory/instancesubdirectory/"
      "testinstance-tmp2id/train_speed.json"));

  EXPECT_FALSE(std::filesystem::exists("tmp2folder/instances"));
  EXPECT_FALSE(std::filesystem::exists("tmp2folder/networks"));

  // Remove tmp2folder and its contents
  std::filesystem::remove_all("tmp2folder");

  const auto sol_obj_3 = solver.solve(
      {.dt = 5}, {},
      {.export_option =
           cda_rail::solver::GeneralExportOption::ExportSolutionWithInstance,
       .working_directory     = "tmp3folder",
       .solution_subdirectory = "tmp3subdirectory",
       .parameter_identifier  = "tmp3id"},
      -1, false);

  // Check folder structure
  EXPECT_TRUE(std::filesystem::exists("tmp3folder"));
  EXPECT_TRUE(std::filesystem::exists("tmp3folder/solutions"));
  EXPECT_TRUE(std::filesystem::exists("tmp3folder/solutions/tmp3subdirectory"));
  EXPECT_TRUE(std::filesystem::exists(
      "tmp3folder/solutions/tmp3subdirectory/instancesubdirectory"));
  EXPECT_TRUE(
      std::filesystem::exists("tmp3folder/solutions/tmp3subdirectory/"
                              "instancesubdirectory/testinstance-tmp3id"));
  EXPECT_TRUE(std::filesystem::exists(
      "tmp3folder/solutions/tmp3subdirectory/instancesubdirectory/"
      "testinstance-tmp3id/solution_data.json"));
  EXPECT_TRUE(std::filesystem::exists(
      "tmp3folder/solutions/tmp3subdirectory/instancesubdirectory/"
      "testinstance-tmp3id/solver_data.json"));
  EXPECT_TRUE(std::filesystem::exists(
      "tmp3folder/solutions/tmp3subdirectory/instancesubdirectory/"
      "testinstance-tmp3id/routes.json"));
  EXPECT_TRUE(std::filesystem::exists(
      "tmp3folder/solutions/tmp3subdirectory/instancesubdirectory/"
      "testinstance-tmp3id/train_pos.json"));
  EXPECT_TRUE(std::filesystem::exists(
      "tmp3folder/solutions/tmp3subdirectory/instancesubdirectory/"
      "testinstance-tmp3id/train_speed.json"));

  EXPECT_TRUE(std::filesystem::exists("tmp3folder/instances"));
  EXPECT_TRUE(
      std::filesystem::exists("tmp3folder/instances/instancesubdirectory"));
  EXPECT_TRUE(std::filesystem::exists(
      "tmp3folder/instances/instancesubdirectory/testinstance"));
  EXPECT_TRUE(std::filesystem::exists("tmp3folder/networks"));
  EXPECT_TRUE(std::filesystem::exists("tmp3folder/networks/testnetwork"));

  // Remove tmp3folder and its contents
  std::filesystem::remove_all("tmp3folder");
}

TEST(GenPOMovingBlockAStarSolver, SolverDataExport) {
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance;

  std::filesystem::remove_all("tmp_solverdata_folder");

  ASSERT_FALSE(std::filesystem::exists("tmp_solverdata_folder"));

  cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver solver(instance);
  // solver.m_start = now
  // solver.m_model_created = 1.5 seconds later
  // solver.m_model_solved = m_model_created + 10.8 seconds
  solver.m_start         = std::chrono::high_resolution_clock::now();
  solver.m_model_created = solver.m_start + std::chrono::milliseconds(1500);
  solver.m_model_solved =
      solver.m_model_created + std::chrono::milliseconds(10800);

  solver.export_solver_data(
      "tmp_solverdata_folder",
      {.bool_data    = {{"bool_data_0", true}, {"bool_data_1", false}},
       .integer_data = {{"test_data_1", 6}, {"test_data_2", 0}},
       .double_data  = {{"test_data_3", 3.14},
                        {"test_data_4", 0.0},
                        {"test_data_5", 2.71828}},
       .string_data  = {{"test_data_6", "test_string"},
                        {"test_data_7", "something with spaces"}}});

  ASSERT_TRUE(std::filesystem::exists("tmp_solverdata_folder"));
  ASSERT_TRUE(
      std::filesystem::exists("tmp_solverdata_folder/solver_data.json"));

  std::ifstream data_file("tmp_solverdata_folder/solver_data.json");
  if (!data_file.is_open()) {
    throw exceptions::ImportException(
        "Could not open file tmp_solverdata_folder/solver_data.json");
  }
  nlohmann::json data;
  data_file >> data;
  data_file.close();

  std::filesystem::remove_all("tmp_solverdata_folder");
  EXPECT_FALSE(std::filesystem::exists("tmp_solverdata_folder"));

  EXPECT_EQ(data.at("model_creation_time").get<double>(), 1.5);
  EXPECT_EQ(data.at("model_solving_time").get<double>(), 10.8);
  EXPECT_EQ(data.at("total_time").get<double>(), 1.5 + 10.8);

  EXPECT_TRUE(data.at("bool_data_0_bool").get<bool>());
  EXPECT_FALSE(data.at("bool_data_1_bool").get<bool>());
  EXPECT_EQ(data.at("test_data_1_int").get<int>(), 6);
  EXPECT_EQ(data.at("test_data_2_int").get<int>(), 0);
  EXPECT_EQ(data.at("test_data_3_double").get<double>(), 3.14);
  EXPECT_EQ(data.at("test_data_4_double").get<double>(), 0.0);
  EXPECT_EQ(data.at("test_data_5_double").get<double>(), 2.71828);
  EXPECT_EQ(data.at("test_data_6_string").get<std::string>(), "test_string");
  EXPECT_EQ(data.at("test_data_7_string").get<std::string>(),
            "something with spaces");
}

TEST(GenPOMovingBlockAStarSolver, NextStateStrategyToString) {
  EXPECT_EQ(solver::astar_based::next_state_strategy_to_string(
                solver::astar_based::NextStateStrategy::SingleEdge),
            "SingleEdge");
  EXPECT_EQ(solver::astar_based::next_state_strategy_to_string(
                solver::astar_based::NextStateStrategy::NextTTD),
            "NextTTD");
  EXPECT_EQ(solver::astar_based::next_state_strategy_to_string(
                solver::astar_based::NextStateStrategy::NextRelevantTTD),
            "NextRelevantTTD");
  EXPECT_THROW(solver::astar_based::next_state_strategy_to_string(
                   static_cast<solver::astar_based::NextStateStrategy>(255)),
               exceptions::ConsistencyException);
}

TEST(GenPOMovingBlockAStarSolver, SimpleNetwork) {
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      "SimpleNetwork", "atmos2023", "./data");

  cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver solver(instance);
  const auto sol_obj = solver.solve(
      {},
      {.next_state_strategy =
           cda_rail::solver::astar_based::NextStateStrategy::NextRelevantTTD,
       .time_aware_state_transitions = false},
      {}, -1, true);

  const auto sol_obj_2 = solver.solve(
      {},
      {.next_state_strategy =
           cda_rail::solver::astar_based::NextStateStrategy::NextRelevantTTD,
       .time_aware_state_transitions = true},
      {}, -1, true);

  const auto sol_obj_3 = solver.solve(
      {},
      {.next_state_strategy =
           cda_rail::solver::astar_based::NextStateStrategy::NextTTD,
       .time_aware_state_transitions = false},
      {}, -1, true);

  const auto sol_obj_4 = solver.solve(
      {},
      {.next_state_strategy =
           cda_rail::solver::astar_based::NextStateStrategy::NextTTD,
       .time_aware_state_transitions = true},
      {}, -1, true);

  const auto sol_obj_5 = solver.solve(
      {},
      {.next_state_strategy =
           cda_rail::solver::astar_based::NextStateStrategy::SingleEdge,
       .time_aware_state_transitions = false},
      {}, -1, true);

  const auto sol_obj_6 = solver.solve(
      {},
      {.next_state_strategy =
           cda_rail::solver::astar_based::NextStateStrategy::SingleEdge,
       .time_aware_state_transitions = true},
      {}, -1, true);

  const auto sol_obj_7 = solver.solve(
      {},
      {.next_state_strategy =
           cda_rail::solver::astar_based::NextStateStrategy::NextRelevantTTD,
       .time_aware_state_transitions = true,
       .a_star_weight                = 10},
      {}, -1, true);

  EXPECT_TRUE(sol_obj.has_solution());
  EXPECT_TRUE(sol_obj_2.has_solution());
  EXPECT_TRUE(sol_obj_3.has_solution());
  EXPECT_TRUE(sol_obj_4.has_solution());
  EXPECT_TRUE(sol_obj_5.has_solution());
  EXPECT_TRUE(sol_obj_6.has_solution());
  EXPECT_TRUE(sol_obj_7.has_solution());

  EXPECT_EQ(sol_obj.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(sol_obj_2.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(sol_obj_3.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(sol_obj_4.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(sol_obj_5.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(sol_obj_6.get_status(), cda_rail::SolutionStatus::Optimal);
  if (sol_obj_7.get_lower_bound() >= sol_obj_7.get_obj()) {
    EXPECT_EQ(sol_obj_7.get_status(), cda_rail::SolutionStatus::Optimal);
  } else {
    EXPECT_EQ(sol_obj_7.get_status(), cda_rail::SolutionStatus::Feasible);
  }

  EXPECT_EQ(sol_obj.get_obj(), sol_obj_2.get_obj());
  EXPECT_EQ(sol_obj.get_obj(), sol_obj_3.get_obj());
  EXPECT_EQ(sol_obj.get_obj(), sol_obj_4.get_obj());
  EXPECT_EQ(sol_obj.get_obj(), sol_obj_5.get_obj());
  EXPECT_EQ(sol_obj.get_obj(), sol_obj_6.get_obj());

  EXPECT_GE(sol_obj_7.get_obj(), sol_obj.get_obj());
  EXPECT_LE(sol_obj_7.get_lower_bound(), sol_obj.get_obj());
  EXPECT_LE(sol_obj_7.get_obj(), sol_obj_7.get_lower_bound() * 10);

  auto const instance_weighted_sum = instance.sum_of_weighted_exit_times();
  EXPECT_GE(sol_obj.get_lower_bound(), instance_weighted_sum);
  EXPECT_GE(sol_obj_2.get_lower_bound(), instance_weighted_sum);
  EXPECT_GE(sol_obj_3.get_lower_bound(), instance_weighted_sum);
  EXPECT_GE(sol_obj_4.get_lower_bound(), instance_weighted_sum);
  EXPECT_GE(sol_obj_5.get_lower_bound(), instance_weighted_sum);
  EXPECT_GE(sol_obj_6.get_lower_bound(), instance_weighted_sum);
  EXPECT_GE(sol_obj_7.get_lower_bound(), instance_weighted_sum);
}

TEST(GenPOMovingBlockAStarSolver, Stammstrecke4Trains) {
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      "Stammstrecke4Trains", "atmos2023", "./data");

  cda_rail::solver::astar_based::GenPOMovingBlockAStarSolver solver(instance);
  const auto sol_obj = solver.solve(
      {},
      {.next_state_strategy =
           cda_rail::solver::astar_based::NextStateStrategy::NextRelevantTTD,
       .time_aware_state_transitions = true},
      {}, -1, true);

  const auto sol_obj_2 = solver.solve(
      {},
      {.next_state_strategy =
           cda_rail::solver::astar_based::NextStateStrategy::NextTTD,
       .time_aware_state_transitions = true},
      {}, -1, true);

  const auto sol_obj_3 = solver.solve(
      {},
      {.next_state_strategy =
           cda_rail::solver::astar_based::NextStateStrategy::SingleEdge,
       .time_aware_state_transitions = true},
      {}, -1, true);

  const auto sol_obj_4 = solver.solve(
      {},
      {.next_state_strategy =
           cda_rail::solver::astar_based::NextStateStrategy::NextRelevantTTD,
       .time_aware_state_transitions = true,
       .a_star_weight                = 2},
      {}, -1, true);

  EXPECT_TRUE(sol_obj.has_solution());
  EXPECT_TRUE(sol_obj_2.has_solution());
  EXPECT_TRUE(sol_obj_3.has_solution());
  EXPECT_TRUE(sol_obj_4.has_solution());

  EXPECT_EQ(sol_obj.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(sol_obj_2.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(sol_obj_3.get_status(), cda_rail::SolutionStatus::Optimal);
  if (sol_obj_4.get_lower_bound() >= sol_obj_4.get_obj()) {
    EXPECT_EQ(sol_obj_4.get_status(), cda_rail::SolutionStatus::Optimal);
  } else {
    EXPECT_EQ(sol_obj_4.get_status(), cda_rail::SolutionStatus::Feasible);
  }

  EXPECT_EQ(sol_obj.get_obj(), sol_obj_2.get_obj());
  EXPECT_EQ(sol_obj.get_obj(), sol_obj_3.get_obj());
  EXPECT_GE(sol_obj_4.get_obj(), sol_obj.get_obj());
  EXPECT_LE(sol_obj_4.get_lower_bound(), sol_obj.get_obj());
  EXPECT_LE(sol_obj_4.get_obj(), sol_obj_4.get_lower_bound() * 2);

  auto const instance_weighted_sum = instance.sum_of_weighted_exit_times();
  EXPECT_GE(sol_obj.get_lower_bound(), instance_weighted_sum);
  EXPECT_GE(sol_obj_2.get_lower_bound(), instance_weighted_sum);
  EXPECT_GE(sol_obj_3.get_lower_bound(), instance_weighted_sum);
  EXPECT_GE(sol_obj_4.get_lower_bound(), instance_weighted_sum);
}

// NOLINTEND
// (clang-analyzer-deadcode.DeadStores,misc-const-correctness,clang-diagnostic-unused-result)
