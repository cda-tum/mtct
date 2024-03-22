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
  EXPECT_TRUE(std::abs(a - b) < 1e-2) << a << " !=(approx.) " << b;

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
      instance.n().add_vertex("v51", cda_rail::VertexType::NoBorder);
  const auto v61 = instance.n().add_vertex("v61", cda_rail::VertexType::TTD);
  const auto v62 = instance.n().add_vertex("v62", cda_rail::VertexType::TTD);
  const auto v7  = instance.n().add_vertex("v7", cda_rail::VertexType::TTD);
  const auto v8  = instance.n().add_vertex("v8", cda_rail::VertexType::TTD, 60);

  // Edges for simple station
  const auto e_1_2   = instance.n().add_edge(v1, v2, 60, 40);
  const auto e_2_3   = instance.n().add_edge(v2, v3, 5, 40);
  const auto e_3_41  = instance.n().add_edge(v3, v41, 10, 10);
  const auto e_3_42  = instance.n().add_edge(v3, v42, 10, 40);
  const auto e_41_51 = instance.n().add_edge(v41, v51, 50, 30);
  const auto e_51_61 = instance.n().add_edge(v51, v61, 50, 30);
  const auto e_42_62 = instance.n().add_edge(v42, v62, 100, 30);
  const auto e_61_7  = instance.n().add_edge(v61, v7, 10, 10);
  const auto e_62_7  = instance.n().add_edge(v62, v7, 10, 40);
  const auto e_7_8   = instance.n().add_edge(v7, v8, 200, 40);
  // Reverse edges with same properties
  const auto e_2_1   = instance.n().add_edge(v2, v1, 60, 40);
  const auto e_3_2   = instance.n().add_edge(v3, v2, 5, 40);
  const auto e_41_3  = instance.n().add_edge(v41, v3, 10, 10);
  const auto e_42_3  = instance.n().add_edge(v42, v3, 10, 40);
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
  instance.add_train("Train2", 50, 30, 1, 2, {0, 60}, 10, v8, {300, 360}, 10,
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
  EXPECT_APPROX_EQ(solver.model_detail.max_velocity_delta, 5.55);
  EXPECT_EQ(solver.model_detail.velocity_refinement_strategy,
            cda_rail::VelocityRefinementStrategy::MinOneStep);
}

// NOLINTEND (clang-analyzer-deadcode.DeadStores)
