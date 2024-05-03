#include <cstdlib>
#define TEST_FRIENDS true

#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"
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

TEST(GenPOMovingBlockMIPSolver, PrivateFillFunctions) {
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance;

  // Vertices
  const auto v1 = instance.n().add_vertex("v1", cda_rail::VertexType::TTD, 30);
  const auto v2 = instance.n().add_vertex("v2", cda_rail::VertexType::TTD);
  const auto v3 = instance.n().add_vertex("v3", cda_rail::VertexType::NoBorder);
  const auto v41 = instance.n().add_vertex("v41", cda_rail::VertexType::TTD);
  const auto v42 = instance.n().add_vertex("v42", cda_rail::VertexType::TTD);
  const auto v51 =
      instance.n().add_vertex("v51", cda_rail::VertexType::NoBorderVSS);
  const auto v61 = instance.n().add_vertex("v61", cda_rail::VertexType::TTD);
  const auto v62 = instance.n().add_vertex("v62", cda_rail::VertexType::TTD);
  const auto v7  = instance.n().add_vertex("v7", cda_rail::VertexType::TTD);
  const auto v8  = instance.n().add_vertex("v8", cda_rail::VertexType::TTD, 60);

  // Edges for simple station
  const auto e_1_2   = instance.n().add_edge(v1, v2, 40, 40);
  const auto e_2_3   = instance.n().add_edge(v2, v3, 5, 40, false);
  const auto e_3_41  = instance.n().add_edge(v3, v41, 10, 10, false);
  const auto e_3_42  = instance.n().add_edge(v3, v42, 10, 40, false);
  const auto e_41_51 = instance.n().add_edge(v41, v51, 50, 30);
  const auto e_51_61 = instance.n().add_edge(v51, v61, 50, 30);
  const auto e_42_62 = instance.n().add_edge(v42, v62, 100, 30);
  const auto e_61_7  = instance.n().add_edge(v61, v7, 10, 10);
  const auto e_62_7  = instance.n().add_edge(v62, v7, 10, 40);
  const auto e_7_8   = instance.n().add_edge(v7, v8, 200, 40);
  // Reverse edges with same properties
  const auto e_2_1   = instance.n().add_edge(v2, v1, 40, 40);
  const auto e_3_2   = instance.n().add_edge(v3, v2, 5, 40, false);
  const auto e_41_3  = instance.n().add_edge(v41, v3, 10, 10, false);
  const auto e_42_3  = instance.n().add_edge(v42, v3, 10, 40, false);
  const auto e_51_41 = instance.n().add_edge(v51, v41, 50, 30);
  const auto e_61_51 = instance.n().add_edge(v61, v51, 50, 30);
  const auto e_62_42 = instance.n().add_edge(v62, v42, 100, 30);
  const auto e_7_61  = instance.n().add_edge(v7, v61, 10, 10);
  const auto e_7_62  = instance.n().add_edge(v7, v62, 10, 40);
  const auto e_8_7   = instance.n().add_edge(v8, v7, 200, 40);

  // Successors
  instance.n().add_successor(e_1_2, e_2_3);
  instance.n().add_successor(e_2_3, e_3_41);
  instance.n().add_successor(e_2_3, e_3_42);
  instance.n().add_successor(e_3_41, e_41_51);
  instance.n().add_successor(e_41_51, e_51_61);
  instance.n().add_successor(e_3_42, e_42_62);
  instance.n().add_successor(e_51_61, e_61_7);
  instance.n().add_successor(e_42_62, e_62_7);
  instance.n().add_successor(e_61_7, e_7_8);
  instance.n().add_successor(e_62_7, e_7_8);
  // Reverse successors
  instance.n().add_successor(e_3_2, e_2_1);
  instance.n().add_successor(e_41_3, e_3_2);
  instance.n().add_successor(e_42_3, e_3_2);
  instance.n().add_successor(e_51_41, e_41_3);
  instance.n().add_successor(e_61_51, e_51_41);
  instance.n().add_successor(e_62_42, e_42_3);
  instance.n().add_successor(e_7_61, e_61_51);
  instance.n().add_successor(e_7_62, e_62_42);
  instance.n().add_successor(e_8_7, e_7_61);
  instance.n().add_successor(e_8_7, e_7_62);

  // Trains
  instance.add_train("Train1", 75, 30, 1, 2, {0, 60}, 10, v1, {300, 360}, 10,
                     v8);
  instance.add_train("Train2", 50, 50, 3, 2, {0, 60}, 10, v8, {300, 360}, 10,
                     v1);

  // Stations
  instance.add_station("Station1");
  instance.add_station("Station2");
  instance.add_track_to_station("Station1", e_41_51);
  instance.add_track_to_station("Station1", e_51_61);
  instance.add_track_to_station("Station1", e_42_62);
  instance.add_track_to_station("Station1", e_51_41);
  instance.add_track_to_station("Station1", e_61_51);
  instance.add_track_to_station("Station1", e_62_42);
  instance.add_track_to_station("Station2", e_7_8);
  instance.add_track_to_station("Station2", e_8_7);

  // Add route to train 1
  instance.add_empty_route("Train1");
  instance.push_back_edge_to_route("Train1", e_1_2);
  instance.push_back_edge_to_route("Train1", e_2_3);
  instance.push_back_edge_to_route("Train1", e_3_41);
  instance.push_back_edge_to_route("Train1", e_41_51);
  instance.push_back_edge_to_route("Train1", e_51_61);
  instance.push_back_edge_to_route("Train1", e_61_7);
  instance.push_back_edge_to_route("Train1", e_7_8);

  // Add stops for trains
  instance.add_stop("Train1", "Station1", std::pair<int, int>(100, 160),
                    std::pair<int, int>(160, 190), 60);
  instance.add_stop("Train1", "Station2", std::pair<int, int>(200, 260),
                    std::pair<int, int>(260, 290), 45);
  instance.add_stop("Train2", "Station1", std::pair<int, int>(100, 160),
                    std::pair<int, int>(160, 220), 90);

  cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver solver(instance);

  // Initialize relevant variables
  solver.initialize_variables(
      {},
      {false, true, true,
       cda_rail::solver::mip_based::LazyConstraintSelectionStrategy::
           OnlyFirstFound},
      {true, 5.55, cda_rail::VelocityRefinementStrategy::None});

  EXPECT_TRUE(solver.model_detail.fix_routes);
  EXPECT_APPROX_EQ(solver.model_detail.max_velocity_delta, 5.55);
  EXPECT_EQ(solver.model_detail.velocity_refinement_strategy,
            cda_rail::VelocityRefinementStrategy::None);
  EXPECT_EQ(solver.num_tr, 2);
  EXPECT_EQ(solver.num_edges, 20);
  EXPECT_EQ(solver.num_vertices, 10);
  EXPECT_EQ(solver.num_ttd, 1);
  EXPECT_EQ(solver.max_t, 360);
  EXPECT_FALSE(solver.solver_strategy.include_timetable_timing_cuts);
  EXPECT_TRUE(solver.solver_strategy.use_lazy_constraints);
  EXPECT_TRUE(solver.solver_strategy.include_reverse_headways);
  EXPECT_EQ(solver.solver_strategy.lazy_constraint_selection_strategy,
            cda_rail::solver::mip_based::LazyConstraintSelectionStrategy::
                OnlyFirstFound);
  EXPECT_EQ(
      solver.solver_strategy.lazy_train_selection_strategy,
      cda_rail::solver::mip_based::LazyTrainSelectionStrategy::OnlyAdjacent);

  // Test if stop data was set correctly

  const auto& tr_stop_data = solver.tr_stop_data;
  EXPECT_EQ(tr_stop_data.size(), 2);
  const auto& tr_1_data = tr_stop_data.at(0);
  const auto& tr_2_data = tr_stop_data.at(1);
  EXPECT_EQ(tr_1_data.size(), 2);
  EXPECT_EQ(tr_2_data.size(), 1);
  const auto& tr_1_1_data = tr_1_data.at(0);
  const auto& tr_1_2_data = tr_1_data.at(1);
  const auto& tr_2_1_data = tr_2_data.at(0);

  // tr_1_1_data
  // Expect 1 possible vertex, namely v61 with edges e_51_61 and e_41_51
  EXPECT_EQ(tr_1_1_data.size(), 1);
  EXPECT_TRUE(std::find(tr_1_1_data.begin(), tr_1_1_data.end(),
                        std::pair<size_t, std::vector<std::vector<size_t>>>(
                            v61, {{e_51_61, e_41_51}})) != tr_1_1_data.end());

  // tr_1_2_data
  // Expect 1 possible vertex, namely v8 with edge e_6_7
  EXPECT_EQ(tr_1_2_data.size(), 1);
  EXPECT_TRUE(std::find(tr_1_2_data.begin(), tr_1_2_data.end(),
                        std::pair<size_t, std::vector<std::vector<size_t>>>(
                            v8, {{e_7_8}})) != tr_1_2_data.end());

  // tr_2_1_data
  // Expect the following pairs:
  // - v_41 with edge e_51_41
  // - v_51 with e_61_51 or e_41_51
  // - v_61 with e_51_61
  // - v42 with e_62_42
  // - v62 with e_42_62
  EXPECT_EQ(tr_2_1_data.size(), 5);

  std::unordered_map<size_t, size_t> tr_2_1_stop_map;
  for (size_t i = 0; i < tr_2_1_data.size(); ++i) {
    tr_2_1_stop_map.insert_or_assign(tr_2_1_data[i].first, i);
  }
  EXPECT_EQ(tr_2_1_stop_map.count(v41), 1);
  EXPECT_EQ(tr_2_1_stop_map.count(v51), 1);
  EXPECT_EQ(tr_2_1_stop_map.count(v61), 1);
  EXPECT_EQ(tr_2_1_stop_map.count(v42), 1);
  EXPECT_EQ(tr_2_1_stop_map.count(v62), 1);

  const auto& [tr_2_1_data_v21_v, tr_2_1_data_v21_p] =
      tr_2_1_data.at(tr_2_1_stop_map.at(v41));
  EXPECT_EQ(tr_2_1_data_v21_v, v41);
  EXPECT_EQ(tr_2_1_data_v21_p.size(), 1);
  EXPECT_TRUE(std::find(tr_2_1_data_v21_p.begin(), tr_2_1_data_v21_p.end(),
                        std::vector<size_t>{e_51_41}) !=
              tr_2_1_data_v21_p.end());

  const auto& [tr_2_1_data_v51_v, tr_2_1_data_v51_p] =
      tr_2_1_data.at(tr_2_1_stop_map.at(v51));
  EXPECT_EQ(tr_2_1_data_v51_v, v51);
  EXPECT_EQ(tr_2_1_data_v51_p.size(), 2);
  EXPECT_TRUE(std::find(tr_2_1_data_v51_p.begin(), tr_2_1_data_v51_p.end(),
                        std::vector<size_t>{e_61_51}) !=
              tr_2_1_data_v51_p.end());
  EXPECT_TRUE(std::find(tr_2_1_data_v51_p.begin(), tr_2_1_data_v51_p.end(),
                        std::vector<size_t>{e_41_51}) !=
              tr_2_1_data_v51_p.end());

  const auto& [tr_2_1_data_v61_v, tr_2_1_data_v61_p] =
      tr_2_1_data.at(tr_2_1_stop_map.at(v61));
  EXPECT_EQ(tr_2_1_data_v61_v, v61);
  EXPECT_EQ(tr_2_1_data_v61_p.size(), 1);
  EXPECT_TRUE(std::find(tr_2_1_data_v61_p.begin(), tr_2_1_data_v61_p.end(),
                        std::vector<size_t>{e_51_61}) !=
              tr_2_1_data_v61_p.end());

  const auto& [tr_2_1_data_v42_v, tr_2_1_data_v42_p] =
      tr_2_1_data.at(tr_2_1_stop_map.at(v42));
  EXPECT_EQ(tr_2_1_data_v42_v, v42);
  EXPECT_EQ(tr_2_1_data_v42_p.size(), 1);
  EXPECT_TRUE(std::find(tr_2_1_data_v42_p.begin(), tr_2_1_data_v42_p.end(),
                        std::vector<size_t>{e_62_42}) !=
              tr_2_1_data_v42_p.end());

  const auto& [tr_2_1_data_v62_v, tr_2_1_data_v62_p] =
      tr_2_1_data.at(tr_2_1_stop_map.at(v62));
  EXPECT_EQ(tr_2_1_data_v62_v, v62);
  EXPECT_EQ(tr_2_1_data_v62_p.size(), 1);
  EXPECT_TRUE(std::find(tr_2_1_data_v62_p.begin(), tr_2_1_data_v62_p.end(),
                        std::vector<size_t>{e_42_62}) !=
              tr_2_1_data_v62_p.end());

  // Test if velocity data was set correctly
  const auto& vel_data = solver.velocity_extensions;

  EXPECT_EQ(vel_data.size(), 2);
  const auto& vel_data_1 = vel_data.at(0);
  const auto& vel_data_2 = vel_data.at(1);

  // Train 1
  EXPECT_EQ(vel_data_1.size(), solver.num_vertices);
  const auto& vel_data_1_v1  = vel_data_1.at(v1);
  const auto& vel_data_1_v2  = vel_data_1.at(v2);
  const auto& vel_data_1_v3  = vel_data_1.at(v3);
  const auto& vel_data_1_v41 = vel_data_1.at(v41);
  const auto& vel_data_1_v42 = vel_data_1.at(v42);
  const auto& vel_data_1_v51 = vel_data_1.at(v51);
  const auto& vel_data_1_v61 = vel_data_1.at(v61);
  const auto& vel_data_1_v62 = vel_data_1.at(v62);
  const auto& vel_data_1_v7  = vel_data_1.at(v7);
  const auto& vel_data_1_v8  = vel_data_1.at(v8);
  // v1 can only have speed 10
  EXPECT_EQ(vel_data_1_v1.size(), 1);
  EXPECT_APPROX_EQ(vel_data_1_v1.at(0), 10);
  // v2 has maximal speed 40, hence only 40 for train 1
  // Thus 0 - 5.55 - 11.1 - 16.65 - 22.2 - 27.75 - 30
  EXPECT_EQ(vel_data_1_v2.size(), 7);
  EXPECT_APPROX_EQ(vel_data_1_v2.at(0), 0);
  EXPECT_APPROX_EQ(vel_data_1_v2.at(1), 5.55);
  EXPECT_APPROX_EQ(vel_data_1_v2.at(2), 11.1);
  EXPECT_APPROX_EQ(vel_data_1_v2.at(3), 16.65);
  EXPECT_APPROX_EQ(vel_data_1_v2.at(4), 22.2);
  EXPECT_APPROX_EQ(vel_data_1_v2.at(5), 27.75);
  EXPECT_APPROX_EQ(vel_data_1_v2.at(6), 30);
  // v3 has maximal speed 10 for train 1
  // Hence 0 - 5.55 - 10
  EXPECT_EQ(vel_data_1_v3.size(), 3);
  EXPECT_APPROX_EQ(vel_data_1_v3.at(0), 0);
  EXPECT_APPROX_EQ(vel_data_1_v3.at(1), 5.55);
  EXPECT_APPROX_EQ(vel_data_1_v3.at(2), 10);
  // v41 has maximal speed 10 for train 1
  // Hence 0 - 5.55 - 10
  EXPECT_EQ(vel_data_1_v41.size(), 3);
  EXPECT_APPROX_EQ(vel_data_1_v41.at(0), 0);
  EXPECT_APPROX_EQ(vel_data_1_v41.at(1), 5.55);
  EXPECT_APPROX_EQ(vel_data_1_v41.at(2), 10);
  // v51 has maximal speed 30 for train 1
  // Hence 0 - 5.55 - 11.1 - 16.65 - 22.2 - 27.75 - 30
  EXPECT_EQ(vel_data_1_v51.size(), 7);
  EXPECT_APPROX_EQ(vel_data_1_v51.at(0), 0);
  EXPECT_APPROX_EQ(vel_data_1_v51.at(1), 5.55);
  EXPECT_APPROX_EQ(vel_data_1_v51.at(2), 11.1);
  EXPECT_APPROX_EQ(vel_data_1_v51.at(3), 16.65);
  EXPECT_APPROX_EQ(vel_data_1_v51.at(4), 22.2);
  EXPECT_APPROX_EQ(vel_data_1_v51.at(5), 27.75);
  EXPECT_APPROX_EQ(vel_data_1_v51.at(6), 30);
  // v61 has maximal speed 10 for train 1
  // Hence 0 - 5.55 - 10
  EXPECT_EQ(vel_data_1_v61.size(), 3);
  EXPECT_APPROX_EQ(vel_data_1_v61.at(0), 0);
  EXPECT_APPROX_EQ(vel_data_1_v61.at(1), 5.55);
  EXPECT_APPROX_EQ(vel_data_1_v61.at(2), 10);
  // v7 has maximal speed 10 for train 1
  // Hence 0 - 5.55 - 10
  EXPECT_EQ(vel_data_1_v7.size(), 3);
  EXPECT_APPROX_EQ(vel_data_1_v7.at(0), 0);
  EXPECT_APPROX_EQ(vel_data_1_v7.at(1), 5.55);
  EXPECT_APPROX_EQ(vel_data_1_v7.at(2), 10);
  // v8 has maximal speed 40, hence only 30 for train 1
  // Hence 0 - 5.55 - 11.1 - 16.65 - 22.2 - 27.75 - 30
  EXPECT_EQ(vel_data_1_v8.size(), 7);
  EXPECT_APPROX_EQ(vel_data_1_v8.at(0), 0);
  EXPECT_APPROX_EQ(vel_data_1_v8.at(1), 5.55);
  EXPECT_APPROX_EQ(vel_data_1_v8.at(2), 11.1);
  EXPECT_APPROX_EQ(vel_data_1_v8.at(3), 16.65);
  EXPECT_APPROX_EQ(vel_data_1_v8.at(4), 22.2);
  EXPECT_APPROX_EQ(vel_data_1_v8.at(5), 27.75);
  EXPECT_APPROX_EQ(vel_data_1_v8.at(6), 30);
  // v42 and v62 are not used, hence only 0
  EXPECT_EQ(vel_data_1_v42.size(), 1);
  EXPECT_APPROX_EQ(vel_data_1_v42.at(0), 0);
  EXPECT_EQ(vel_data_1_v62.size(), 1);
  EXPECT_APPROX_EQ(vel_data_1_v62.at(0), 0);

  // Train 2
  EXPECT_EQ(vel_data_2.size(), solver.num_vertices);
  const auto& vel_data_2_v1  = vel_data_2.at(v1);
  const auto& vel_data_2_v2  = vel_data_2.at(v2);
  const auto& vel_data_2_v3  = vel_data_2.at(v3);
  const auto& vel_data_2_v41 = vel_data_2.at(v41);
  const auto& vel_data_2_v42 = vel_data_2.at(v42);
  const auto& vel_data_2_v51 = vel_data_2.at(v51);
  const auto& vel_data_2_v61 = vel_data_2.at(v61);
  const auto& vel_data_2_v62 = vel_data_2.at(v62);
  const auto& vel_data_2_v7  = vel_data_2.at(v7);
  const auto& vel_data_2_v8  = vel_data_2.at(v8);

  // v1 has maximal speed 40
  // Hence 0 - 5.55 - 11.1 - 16.65 - 22.2 - 27.75 - 33.3 - 38.85 - 40
  EXPECT_EQ(vel_data_2_v1.size(), 9);
  EXPECT_APPROX_EQ(vel_data_2_v1.at(0), 0);
  EXPECT_APPROX_EQ(vel_data_2_v1.at(1), 5.55);
  EXPECT_APPROX_EQ(vel_data_2_v1.at(2), 11.1);
  EXPECT_APPROX_EQ(vel_data_2_v1.at(3), 16.65);
  EXPECT_APPROX_EQ(vel_data_2_v1.at(4), 22.2);
  EXPECT_APPROX_EQ(vel_data_2_v1.at(5), 27.75);
  EXPECT_APPROX_EQ(vel_data_2_v1.at(6), 33.3);
  EXPECT_APPROX_EQ(vel_data_2_v1.at(7), 38.85);
  EXPECT_APPROX_EQ(vel_data_2_v1.at(8), 40);

  // v2 has maximal speed 40
  // Hence 0 - 5.55 - 11.1 - 16.65 - 22.2 - 27.75 - 33.3 - 38.85 - 40
  EXPECT_EQ(vel_data_2_v2.size(), 9);
  EXPECT_APPROX_EQ(vel_data_2_v2.at(0), 0);
  EXPECT_APPROX_EQ(vel_data_2_v2.at(1), 5.55);
  EXPECT_APPROX_EQ(vel_data_2_v2.at(2), 11.1);
  EXPECT_APPROX_EQ(vel_data_2_v2.at(3), 16.65);
  EXPECT_APPROX_EQ(vel_data_2_v2.at(4), 22.2);
  EXPECT_APPROX_EQ(vel_data_2_v2.at(5), 27.75);
  EXPECT_APPROX_EQ(vel_data_2_v2.at(6), 33.3);
  EXPECT_APPROX_EQ(vel_data_2_v2.at(7), 38.85);
  EXPECT_APPROX_EQ(vel_data_2_v2.at(8), 40);

  // v3 has maximal speed 40
  // Hence 0 - 5.55 - 11.1 - 16.65 - 22.2 - 27.75 - 33.3 - 38.85 - 40
  EXPECT_EQ(vel_data_2_v3.size(), 9);
  EXPECT_APPROX_EQ(vel_data_2_v3.at(0), 0);
  EXPECT_APPROX_EQ(vel_data_2_v3.at(1), 5.55);
  EXPECT_APPROX_EQ(vel_data_2_v3.at(2), 11.1);
  EXPECT_APPROX_EQ(vel_data_2_v3.at(3), 16.65);
  EXPECT_APPROX_EQ(vel_data_2_v3.at(4), 22.2);
  EXPECT_APPROX_EQ(vel_data_2_v3.at(5), 27.75);
  EXPECT_APPROX_EQ(vel_data_2_v3.at(6), 33.3);
  EXPECT_APPROX_EQ(vel_data_2_v3.at(7), 38.85);
  EXPECT_APPROX_EQ(vel_data_2_v3.at(8), 40);

  // v41 has maximal speed 10
  // Hence 0 - 5.55 - 10
  EXPECT_EQ(vel_data_2_v41.size(), 3);
  EXPECT_APPROX_EQ(vel_data_2_v41.at(0), 0);
  EXPECT_APPROX_EQ(vel_data_2_v41.at(1), 5.55);
  EXPECT_APPROX_EQ(vel_data_2_v41.at(2), 10);

  // v51 has maximal speed 30
  // Hence 0 - 5.55 - 11.1 - 16.65 - 22.2 - 27.75 - 30
  EXPECT_EQ(vel_data_2_v51.size(), 7);
  EXPECT_APPROX_EQ(vel_data_2_v51.at(0), 0);
  EXPECT_APPROX_EQ(vel_data_2_v51.at(1), 5.55);
  EXPECT_APPROX_EQ(vel_data_2_v51.at(2), 11.1);
  EXPECT_APPROX_EQ(vel_data_2_v51.at(3), 16.65);
  EXPECT_APPROX_EQ(vel_data_2_v51.at(4), 22.2);
  EXPECT_APPROX_EQ(vel_data_2_v51.at(5), 27.75);
  EXPECT_APPROX_EQ(vel_data_2_v51.at(6), 30);

  // v61 has maximal speed 10
  // Hence 0 - 5.55 - 10
  EXPECT_EQ(vel_data_2_v61.size(), 3);
  EXPECT_APPROX_EQ(vel_data_2_v61.at(0), 0);
  EXPECT_APPROX_EQ(vel_data_2_v61.at(1), 5.55);
  EXPECT_APPROX_EQ(vel_data_2_v61.at(2), 10);

  // v42 has maximal speed 30
  // Hence 0 - 5.55 - 11.1 - 16.65 - 22.2 - 27.75 - 30
  EXPECT_EQ(vel_data_2_v42.size(), 7);
  EXPECT_APPROX_EQ(vel_data_2_v42.at(0), 0);
  EXPECT_APPROX_EQ(vel_data_2_v42.at(1), 5.55);
  EXPECT_APPROX_EQ(vel_data_2_v42.at(2), 11.1);
  EXPECT_APPROX_EQ(vel_data_2_v42.at(3), 16.65);
  EXPECT_APPROX_EQ(vel_data_2_v42.at(4), 22.2);
  EXPECT_APPROX_EQ(vel_data_2_v42.at(5), 27.75);
  EXPECT_APPROX_EQ(vel_data_2_v42.at(6), 30);

  // v62 has maximal speed 30
  // Hence 0 - 5.55 - 11.1 - 16.65 - 22.2 - 27.75 - 30
  EXPECT_EQ(vel_data_2_v62.size(), 7);
  EXPECT_APPROX_EQ(vel_data_2_v62.at(0), 0);
  EXPECT_APPROX_EQ(vel_data_2_v62.at(1), 5.55);
  EXPECT_APPROX_EQ(vel_data_2_v62.at(2), 11.1);
  EXPECT_APPROX_EQ(vel_data_2_v62.at(3), 16.65);
  EXPECT_APPROX_EQ(vel_data_2_v62.at(4), 22.2);
  EXPECT_APPROX_EQ(vel_data_2_v62.at(5), 27.75);
  EXPECT_APPROX_EQ(vel_data_2_v62.at(6), 30);

  // v7 has maximal speed 40
  // Hence 0 - 5.55 - 11.1 - 16.65 - 22.2 - 27.75 - 33.3 - 38.85 - 40
  EXPECT_EQ(vel_data_2_v7.size(), 9);
  EXPECT_APPROX_EQ(vel_data_2_v7.at(0), 0);
  EXPECT_APPROX_EQ(vel_data_2_v7.at(1), 5.55);
  EXPECT_APPROX_EQ(vel_data_2_v7.at(2), 11.1);
  EXPECT_APPROX_EQ(vel_data_2_v7.at(3), 16.65);
  EXPECT_APPROX_EQ(vel_data_2_v7.at(4), 22.2);
  EXPECT_APPROX_EQ(vel_data_2_v7.at(5), 27.75);
  EXPECT_APPROX_EQ(vel_data_2_v7.at(6), 33.3);
  EXPECT_APPROX_EQ(vel_data_2_v7.at(7), 38.85);
  EXPECT_APPROX_EQ(vel_data_2_v7.at(8), 40);

  // v8 is entered at exactly speed 10
  EXPECT_EQ(vel_data_2_v8.size(), 1);
  EXPECT_APPROX_EQ(vel_data_2_v8.at(0), 10);

  // Test with minimum one change refinement
  solver.model_detail.velocity_refinement_strategy =
      cda_rail::VelocityRefinementStrategy::MinOneStep;
  solver.model_detail.max_velocity_delta = 10;

  solver.fill_velocity_extensions();

  EXPECT_TRUE(solver.model_detail.fix_routes);
  EXPECT_APPROX_EQ(solver.model_detail.max_velocity_delta, 10);
  EXPECT_EQ(solver.model_detail.velocity_refinement_strategy,
            cda_rail::VelocityRefinementStrategy::MinOneStep);
  EXPECT_EQ(solver.num_tr, 2);
  EXPECT_EQ(solver.num_edges, 20);
  EXPECT_EQ(solver.num_vertices, 10);
  EXPECT_EQ(solver.num_ttd, 1);
  EXPECT_EQ(solver.max_t, 360);

  // Test new velocity extensions
  const auto& vel_data_new = solver.velocity_extensions;

  EXPECT_EQ(vel_data_new.size(), 2);
  const auto& vel_data_new_1 = vel_data_new.at(0);
  const auto& vel_data_new_2 = vel_data_new.at(1);

  // Train 1
  EXPECT_EQ(vel_data_new_1.size(), solver.num_vertices);
  const auto& vel_data_new_1_v1  = vel_data_new_1.at(v1);
  const auto& vel_data_new_1_v2  = vel_data_new_1.at(v2);
  const auto& vel_data_new_1_v3  = vel_data_new_1.at(v3);
  const auto& vel_data_new_1_v41 = vel_data_new_1.at(v41);
  const auto& vel_data_new_1_v42 = vel_data_new_1.at(v42);
  const auto& vel_data_new_1_v51 = vel_data_new_1.at(v51);
  const auto& vel_data_new_1_v61 = vel_data_new_1.at(v61);
  const auto& vel_data_new_1_v62 = vel_data_new_1.at(v62);
  const auto& vel_data_new_1_v7  = vel_data_new_1.at(v7);
  const auto& vel_data_new_1_v8  = vel_data_new_1.at(v8);

  // Note that train 1 has minimal velocity change of 1

  // v1 is entered at exactly speed 10
  EXPECT_EQ(vel_data_new_1_v1.size(), 1);
  EXPECT_APPROX_EQ(vel_data_new_1_v1.at(0), 10);

  // v2 has maximal speed 40, hence only 30 for train 1
  // v2 has minimal length of 5 -> 2*1*5 = 10
  std::vector<double> expected_speeds_1_v2 = {0};
  while (expected_speeds_1_v2.back() < 30) {
    const auto new_speed = std::sqrt(
        expected_speeds_1_v2.back() * expected_speeds_1_v2.back() + 10);
    expected_speeds_1_v2.push_back(new_speed > 30 ? 30 : new_speed);
  }
  EXPECT_EQ(vel_data_new_1_v2.size(), expected_speeds_1_v2.size());
  for (size_t i = 0; i < vel_data_new_1_v2.size(); ++i) {
    EXPECT_APPROX_EQ(vel_data_new_1_v2.at(i), expected_speeds_1_v2.at(i));
  }

  // v3 has maximal speed 10 for train 1
  // v3 has minimal length of 5 -> 2*1*5 = 10
  std::vector<double> expected_speeds_1_v3 = {0};
  while (expected_speeds_1_v3.back() < 10) {
    const auto new_speed = std::sqrt(
        expected_speeds_1_v3.back() * expected_speeds_1_v3.back() + 10);
    expected_speeds_1_v3.push_back(new_speed > 10 ? 10 : new_speed);
  }
  EXPECT_EQ(vel_data_new_1_v3.size(), expected_speeds_1_v3.size());
  for (size_t i = 0; i < vel_data_new_1_v3.size(); ++i) {
    EXPECT_APPROX_EQ(vel_data_new_1_v3.at(i), expected_speeds_1_v3.at(i));
  }

  // v41 has maximal speed 10 for train 1
  // v41 has minimal length of 10 -> 2*1*10 = 20
  std::vector<double> expected_speeds_1_v41 = {0};
  while (expected_speeds_1_v41.back() < 10) {
    const auto new_speed = std::sqrt(
        expected_speeds_1_v41.back() * expected_speeds_1_v41.back() + 20);
    expected_speeds_1_v41.push_back(new_speed > 10 ? 10 : new_speed);
  }
  EXPECT_EQ(vel_data_new_1_v41.size(), expected_speeds_1_v41.size());
  for (size_t i = 0; i < vel_data_new_1_v41.size(); ++i) {
    EXPECT_APPROX_EQ(vel_data_new_1_v41.at(i), expected_speeds_1_v41.at(i));
  }

  // v51 has maximal speed 30
  // v51 has minimal length of 50 -> 2*1*50 = 100
  // Initial speed is 0
  // sqrt(0^2+100) = 10
  // sqrt(10^2+100) = 14.1421
  // ...
  std::vector<double> expected_speeds_1_v51 = {0};
  while (expected_speeds_1_v51.back() < 30) {
    const auto new_speed = std::sqrt(
        expected_speeds_1_v51.back() * expected_speeds_1_v51.back() + 100);
    expected_speeds_1_v51.push_back(new_speed > 30 ? 30 : new_speed);
  }
  EXPECT_EQ(vel_data_new_1_v51.size(), expected_speeds_1_v51.size());
  for (size_t i = 0; i < vel_data_new_1_v51.size(); ++i) {
    EXPECT_APPROX_EQ(vel_data_new_1_v51.at(i), expected_speeds_1_v51.at(i));
  }

  // v61 has maximal speed 10
  // v61 has minimal length of 10 -> 2*1*10 = 20
  std::vector<double> expected_speeds_1_v61 = {0};
  while (expected_speeds_1_v61.back() < 10) {
    const auto new_speed = std::sqrt(
        expected_speeds_1_v61.back() * expected_speeds_1_v61.back() + 20);
    expected_speeds_1_v61.push_back(new_speed > 10 ? 10 : new_speed);
  }
  EXPECT_EQ(vel_data_new_1_v61.size(), expected_speeds_1_v61.size());
  for (size_t i = 0; i < vel_data_new_1_v61.size(); ++i) {
    EXPECT_APPROX_EQ(vel_data_new_1_v61.at(i), expected_speeds_1_v61.at(i));
  }

  // v7 has maximal speed 10
  // v7 has minimal length of 10 -> 2*1*10 = 20
  std::vector<double> expected_speeds_1_v7 = {0};
  while (expected_speeds_1_v7.back() < 10) {
    const auto new_speed = std::sqrt(
        expected_speeds_1_v7.back() * expected_speeds_1_v7.back() + 20);
    expected_speeds_1_v7.push_back(new_speed > 10 ? 10 : new_speed);
  }
  EXPECT_EQ(vel_data_new_1_v7.size(), expected_speeds_1_v7.size());
  for (size_t i = 0; i < vel_data_new_1_v7.size(); ++i) {
    EXPECT_APPROX_EQ(vel_data_new_1_v7.at(i), expected_speeds_1_v7.at(i));
  }

  // v8 has maximal speed 40, hence only 30 for train 1
  // v8 has minimal length of tr length, namely 75 -> 2*1*75 = 150
  // Initial speed is 0
  // sqrt(0^2+150) = 12.2474 > 10 -> 10
  // sqrt(10^2+150) = 15.8114 < 20 -> ...
  std::vector<double> expected_speeds_1_v8 = {0, 10};
  while (expected_speeds_1_v8.back() < 30) {
    const auto new_speed = std::sqrt(
        expected_speeds_1_v8.back() * expected_speeds_1_v8.back() + 150);
    expected_speeds_1_v8.push_back(new_speed > 30 ? 30 : new_speed);
  }
  EXPECT_EQ(vel_data_new_1_v8.size(), expected_speeds_1_v8.size());
  for (size_t i = 0; i < vel_data_new_1_v8.size(); ++i) {
    EXPECT_APPROX_EQ(vel_data_new_1_v8.at(i), expected_speeds_1_v8.at(i));
  }

  // v42 and v62 are not used, hence only 0
  EXPECT_EQ(vel_data_new_1_v42.size(), 1);
  EXPECT_APPROX_EQ(vel_data_new_1_v42.at(0), 0);
  EXPECT_EQ(vel_data_new_1_v62.size(), 1);
  EXPECT_APPROX_EQ(vel_data_new_1_v62.at(0), 0);

  // Train 2
  EXPECT_EQ(vel_data_new_2.size(), solver.num_vertices);
  const auto& vel_data_new_2_v1  = vel_data_new_2.at(v1);
  const auto& vel_data_new_2_v2  = vel_data_new_2.at(v2);
  const auto& vel_data_new_2_v3  = vel_data_new_2.at(v3);
  const auto& vel_data_new_2_v41 = vel_data_new_2.at(v41);
  const auto& vel_data_new_2_v42 = vel_data_new_2.at(v42);
  const auto& vel_data_new_2_v51 = vel_data_new_2.at(v51);
  const auto& vel_data_new_2_v61 = vel_data_new_2.at(v61);
  const auto& vel_data_new_2_v62 = vel_data_new_2.at(v62);
  const auto& vel_data_new_2_v7  = vel_data_new_2.at(v7);
  const auto& vel_data_new_2_v8  = vel_data_new_2.at(v8);

  // Note that train 2 has minimal velocity change of 2 and length of 50

  // v1 has maximal speed 40
  // v1 has minimal length of 40 -> 2*2*40 = 160
  // Initial speed is 0
  // sqrt(0^2+160) = 12.6491 > 10 -> 10
  // sqrt(10^2+160) = 16.0312 < 20 -> ...
  std::vector<double> expected_speeds_2_v1 = {0, 10};
  while (expected_speeds_2_v1.back() < 40) {
    const auto new_speed = std::sqrt(
        expected_speeds_2_v1.back() * expected_speeds_2_v1.back() + 160);
    expected_speeds_2_v1.push_back(new_speed > 40 ? 40 : new_speed);
  }
  EXPECT_EQ(vel_data_new_2_v1.size(), expected_speeds_2_v1.size());
  for (size_t i = 0; i < vel_data_new_2_v1.size(); ++i) {
    EXPECT_APPROX_EQ(vel_data_new_2_v1.at(i), expected_speeds_2_v1.at(i));
  }

  // v2 has maximal speed 40
  // v2 has minimal length of 5 -> 2*2*5 = 20
  // Initial speed is 0
  // sqrt(0^2+20) = 4.47214 < 10 -> ...
  std::vector<double> expected_speeds_2_v2 = {0};
  while (expected_speeds_2_v2.back() < 40) {
    const auto new_speed = std::sqrt(
        expected_speeds_2_v2.back() * expected_speeds_2_v2.back() + 20);
    expected_speeds_2_v2.push_back(new_speed > 40 ? 40 : new_speed);
  }
  EXPECT_EQ(vel_data_new_2_v2.size(), expected_speeds_2_v2.size());
  for (size_t i = 0; i < vel_data_new_2_v2.size(); ++i) {
    EXPECT_APPROX_EQ(vel_data_new_2_v2.at(i), expected_speeds_2_v2.at(i));
  }

  // v3 has maximal speed 40
  // v3 has minimal length of 5 -> 2*2*5 = 20
  // Initial speed is 0
  // sqrt(0^2+20) = 4.47214 < 10 -> ...
  std::vector<double> expected_speeds_2_v3 = {0};
  while (expected_speeds_2_v3.back() < 40) {
    const auto new_speed = std::sqrt(
        expected_speeds_2_v3.back() * expected_speeds_2_v3.back() + 20);
    expected_speeds_2_v3.push_back(new_speed > 40 ? 40 : new_speed);
  }
  EXPECT_EQ(vel_data_new_2_v3.size(), expected_speeds_2_v3.size());
  for (size_t i = 0; i < vel_data_new_2_v3.size(); ++i) {
    EXPECT_APPROX_EQ(vel_data_new_2_v3.at(i), expected_speeds_2_v3.at(i));
  }

  // v41 has maximal speed 10
  // v41 has minimal length of 10 -> 2*2*10 = 40
  // Initial speed is 0
  // sqrt(0^2+40) = 6.32456 < 10 -> ...
  std::vector<double> expected_speeds_2_v41 = {0};
  while (expected_speeds_2_v41.back() < 10) {
    const auto new_speed = std::sqrt(
        expected_speeds_2_v41.back() * expected_speeds_2_v41.back() + 40);
    expected_speeds_2_v41.push_back(new_speed > 10 ? 10 : new_speed);
  }
  EXPECT_EQ(vel_data_new_2_v41.size(), expected_speeds_2_v41.size());
  for (size_t i = 0; i < vel_data_new_2_v41.size(); ++i) {
    EXPECT_APPROX_EQ(vel_data_new_2_v41.at(i), expected_speeds_2_v41.at(i));
  }

  // v51 has maximal speed 30
  // v51 has minimal length of 50 -> 2*2*50 = 200
  // Initial speed is 0
  // sqrt(0^2+200) = 14.1421 > 10 -> 10
  // sqrt(10^2+200) = 14.1421 < 20 -> ...
  std::vector<double> expected_speeds_2_v51 = {0, 10};
  while (expected_speeds_2_v51.back() < 30) {
    const auto new_speed = std::sqrt(
        expected_speeds_2_v51.back() * expected_speeds_2_v51.back() + 200);
    expected_speeds_2_v51.push_back(new_speed > 30 ? 30 : new_speed);
  }
  EXPECT_EQ(vel_data_new_2_v51.size(), expected_speeds_2_v51.size());
  for (size_t i = 0; i < vel_data_new_2_v51.size(); ++i) {
    EXPECT_APPROX_EQ(vel_data_new_2_v51.at(i), expected_speeds_2_v51.at(i));
  }

  // v61 has maximal speed 10
  // v61 has minimal length of 10 -> 2*2*10 = 40
  // Initial speed is 0
  // sqrt(0^2+40) = 6.32456 < 10 -> ...
  std::vector<double> expected_speeds_2_v61 = {0};
  while (expected_speeds_2_v61.back() < 10) {
    const auto new_speed = std::sqrt(
        expected_speeds_2_v61.back() * expected_speeds_2_v61.back() + 40);
    expected_speeds_2_v61.push_back(new_speed > 10 ? 10 : new_speed);
  }
  EXPECT_EQ(vel_data_new_2_v61.size(), expected_speeds_2_v61.size());
  for (size_t i = 0; i < vel_data_new_2_v61.size(); ++i) {
    EXPECT_APPROX_EQ(vel_data_new_2_v61.at(i), expected_speeds_2_v61.at(i));
  }

  // v42 has maximal speed 30
  // v42 has minimal length of 10 -> 2*2*10 = 40
  // Initial speed is 0
  // sqrt(0^2+40) = 6.32456 < 10 -> ...
  std::vector<double> expected_speeds_2_v42 = {0};
  while (expected_speeds_2_v42.back() < 30) {
    const auto new_speed = std::sqrt(
        expected_speeds_2_v42.back() * expected_speeds_2_v42.back() + 40);
    expected_speeds_2_v42.push_back(new_speed > 30 ? 30 : new_speed);
  }
  EXPECT_EQ(vel_data_new_2_v42.size(), expected_speeds_2_v42.size());
  for (size_t i = 0; i < vel_data_new_2_v42.size(); ++i) {
    EXPECT_APPROX_EQ(vel_data_new_2_v42.at(i), expected_speeds_2_v42.at(i));
  }

  // v62 has maximal speed 30
  // v62 has minimal length of 10 -> 2*2*10 = 40
  // Initial speed is 0
  // sqrt(0^2+40) = 6.32456 < 10 -> ...
  std::vector<double> expected_speeds_2_v62 = {0};
  while (expected_speeds_2_v62.back() < 30) {
    const auto new_speed = std::sqrt(
        expected_speeds_2_v62.back() * expected_speeds_2_v62.back() + 40);
    expected_speeds_2_v62.push_back(new_speed > 30 ? 30 : new_speed);
  }
  EXPECT_EQ(vel_data_new_2_v62.size(), expected_speeds_2_v62.size());
  for (size_t i = 0; i < vel_data_new_2_v62.size(); ++i) {
    EXPECT_APPROX_EQ(vel_data_new_2_v62.at(i), expected_speeds_2_v62.at(i));
  }

  // v7 has maximal speed 40
  // v7 has minimal length of 10 -> 2*2*10 = 40
  // Initial speed is 0
  // sqrt(0^2+40) = 6.32456 < 10 -> ...
  std::vector<double> expected_speeds_2_v7 = {0};
  while (expected_speeds_2_v7.back() < 40) {
    const auto new_speed = std::sqrt(
        expected_speeds_2_v7.back() * expected_speeds_2_v7.back() + 40);
    expected_speeds_2_v7.push_back(new_speed > 40 ? 40 : new_speed);
  }
  EXPECT_EQ(vel_data_new_2_v7.size(), expected_speeds_2_v7.size());
  for (size_t i = 0; i < vel_data_new_2_v7.size(); ++i) {
    EXPECT_APPROX_EQ(vel_data_new_2_v7.at(i), expected_speeds_2_v7.at(i));
  }

  // v8 is entered at exactly speed 10
  EXPECT_EQ(vel_data_new_2_v8.size(), 1);
  EXPECT_APPROX_EQ(vel_data_new_2_v8.at(0), 10);
}

// NOLINTEND (clang-analyzer-deadcode.DeadStores)
