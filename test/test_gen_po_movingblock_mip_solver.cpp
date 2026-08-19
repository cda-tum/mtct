#include <cstdlib>
#define TEST_FRIENDS true

#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"
#include "solver/mip-based/GenPOMovingBlockMIPSolver.hpp"

#include "gtest/gtest.h"
#include <filesystem>
#include <string>
#include <utility>
#include <vector>

#define EXPECT_APPROX_EQ(a, b)                                                 \
  EXPECT_TRUE(std::abs((a) - (b)) < 1e-2) << (a) << " !=(approx.) " << (b)

void cleanup_export_dirs() {
  std::filesystem::remove_all("tmp1folder");
  std::filesystem::remove_all("tmp2folder");
  std::filesystem::remove_all("tmp3folder");
  std::filesystem::remove_all("tmp4folder");
  std::filesystem::remove_all("tmp5folder");
  std::filesystem::remove_all("tmp6folder");
  std::filesystem::remove_all("instances");
  std::filesystem::remove_all("networks");
  std::filesystem::remove_all("solutions");
  std::filesystem::remove("model.mps");
  std::filesystem::remove("model.json");
}

void check_last_train_pos(
    const cda_rail::instances::GeneralPerformanceOptimizationInstance& instance,
    const cda_rail::instances::SolGeneralPerformanceOptimizationInstance& sol,
    const std::string& instance_path) {
  const auto num_tr = instance.get_const_train_list().size();
  for (size_t tr = 0; tr < num_tr; tr++) {
    const auto tr_object = instance.get_const_train_list().get_train(tr);
    const auto t_n       = instance.get_const_schedule(tr).get_exit_time();
    const auto route_len = sol.get_instance()
                               ->get_const_routes()
                               .get_route(tr_object.get_name())
                               .length(instance.get_const_network());

    const auto tr_times = sol.get_train_times(tr_object.get_name());

    EXPECT_GE(tr_times.size(), 2);

    EXPECT_APPROX_EQ(tr_times.at(tr_times.size() - 1), t_n)
        << " for train " << tr_object.get_name() << " in " << instance_path;

    EXPECT_APPROX_EQ(sol.get_train_pos(tr_object.get_name(),
                                       tr_times.at(tr_times.size() - 2)),
                     route_len)
        << " for train " << tr_object.get_name() << " in " << instance_path;

    EXPECT_APPROX_EQ(sol.get_train_pos(tr_object.get_name(),
                                       tr_times.at(tr_times.size() - 1)),
                     route_len + tr_object.get_length())
        << " for train " << tr_object.get_name() << " in " << instance_path;
  }
}

// NOLINTBEGIN (clang-analyzer-deadcode.DeadStores)

TEST(GenPOMovingBlockMIPSolver, PrivateFillFunctions) {
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance;

  // Vertices
  const auto v1 = instance.get_editable_network().add_vertex(
      "v1", cda_rail::VertexType::TTD, 30);
  const auto v2 = instance.get_editable_network().add_vertex(
      "v2", cda_rail::VertexType::TTD);
  const auto v3 = instance.get_editable_network().add_vertex(
      "v3", cda_rail::VertexType::NoBorder);
  const auto v41 = instance.get_editable_network().add_vertex(
      "v41", cda_rail::VertexType::TTD);
  const auto v42 = instance.get_editable_network().add_vertex(
      "v42", cda_rail::VertexType::TTD);
  const auto v51 = instance.get_editable_network().add_vertex(
      "v51", cda_rail::VertexType::NoBorderVSS);
  const auto v61 = instance.get_editable_network().add_vertex(
      "v61", cda_rail::VertexType::TTD);
  const auto v62 = instance.get_editable_network().add_vertex(
      "v62", cda_rail::VertexType::TTD);
  const auto v7 = instance.get_editable_network().add_vertex(
      "v7", cda_rail::VertexType::TTD);
  const auto v8 = instance.get_editable_network().add_vertex(
      "v8", cda_rail::VertexType::TTD, 60);

  // Edges for simple station
  const auto e_1_2 = instance.get_editable_network().add_edge(v1, v2, 40, 40);
  const auto e_2_3 =
      instance.get_editable_network().add_edge(v2, v3, 5, 40, false);
  const auto e_3_41 =
      instance.get_editable_network().add_edge(v3, v41, 10, 10, false);
  const auto e_3_42 =
      instance.get_editable_network().add_edge(v3, v42, 10, 40, false);
  const auto e_41_51 =
      instance.get_editable_network().add_edge(v41, v51, 50, 30);
  const auto e_51_61 =
      instance.get_editable_network().add_edge(v51, v61, 50, 30);
  const auto e_42_62 =
      instance.get_editable_network().add_edge(v42, v62, 100, 30);
  const auto e_61_7 = instance.get_editable_network().add_edge(v61, v7, 10, 10);
  const auto e_62_7 = instance.get_editable_network().add_edge(v62, v7, 10, 40);
  const auto e_7_8  = instance.get_editable_network().add_edge(v7, v8, 200, 40);
  // Reverse edges with same properties
  const auto e_2_1 = instance.get_editable_network().add_edge(v2, v1, 40, 40);
  const auto e_3_2 =
      instance.get_editable_network().add_edge(v3, v2, 5, 40, false);
  const auto e_41_3 =
      instance.get_editable_network().add_edge(v41, v3, 10, 10, false);
  const auto e_42_3 =
      instance.get_editable_network().add_edge(v42, v3, 10, 40, false);
  const auto e_51_41 =
      instance.get_editable_network().add_edge(v51, v41, 50, 30);
  const auto e_61_51 =
      instance.get_editable_network().add_edge(v61, v51, 50, 30);
  const auto e_62_42 =
      instance.get_editable_network().add_edge(v62, v42, 100, 30);
  const auto e_7_61 = instance.get_editable_network().add_edge(v7, v61, 10, 10);
  const auto e_7_62 = instance.get_editable_network().add_edge(v7, v62, 10, 40);
  const auto e_8_7  = instance.get_editable_network().add_edge(v8, v7, 200, 40);

  // Successors
  instance.get_editable_network().add_successor(e_1_2, e_2_3);
  instance.get_editable_network().add_successor(e_2_3, e_3_41);
  instance.get_editable_network().add_successor(e_2_3, e_3_42);
  instance.get_editable_network().add_successor(e_3_41, e_41_51);
  instance.get_editable_network().add_successor(e_41_51, e_51_61);
  instance.get_editable_network().add_successor(e_3_42, e_42_62);
  instance.get_editable_network().add_successor(e_51_61, e_61_7);
  instance.get_editable_network().add_successor(e_42_62, e_62_7);
  instance.get_editable_network().add_successor(e_61_7, e_7_8);
  instance.get_editable_network().add_successor(e_62_7, e_7_8);
  // Reverse successors
  instance.get_editable_network().add_successor(e_3_2, e_2_1);
  instance.get_editable_network().add_successor(e_41_3, e_3_2);
  instance.get_editable_network().add_successor(e_42_3, e_3_2);
  instance.get_editable_network().add_successor(e_51_41, e_41_3);
  instance.get_editable_network().add_successor(e_61_51, e_51_41);
  instance.get_editable_network().add_successor(e_62_42, e_42_3);
  instance.get_editable_network().add_successor(e_7_61, e_61_51);
  instance.get_editable_network().add_successor(e_7_62, e_62_42);
  instance.get_editable_network().add_successor(e_8_7, e_7_61);
  instance.get_editable_network().add_successor(e_8_7, e_7_62);

  // Trains
  instance.add_train("Train1", 75, 30, 1, 2, 0, 10, v1, 300, 10, v8);
  instance.add_train("Train2", 50, 50, 3, 2, 0, 10, v8, 300, 10, v1);

  // Stations
  instance.add_empty_station("Station1");
  instance.add_empty_station("Station2");
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
  instance.insert_stop("Train1", "Station1", 100, 60);
  instance.insert_stop("Train1", "Station2", 200, 45);
  instance.insert_stop("Train2", "Station1", 100, 90);

  cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver solver(instance);

  // Initialize relevant variables
  EXPECT_THROW(solver.initialize_variables(
                   {},
                   {true, true, true,
                    cda_rail::solver::mip_based::
                        LazyConstraintSelectionStrategy::OnlyFirstFound},
                   {true, 5.55, cda_rail::VelocityRefinementStrategy::None}),
               cda_rail::exceptions::InvalidInputException);

  EXPECT_THROW(solver.initialize_variables(
                   {},
                   {false, true, true,
                    cda_rail::solver::mip_based::
                        LazyConstraintSelectionStrategy::AllChecked},
                   {true, 5.55, cda_rail::VelocityRefinementStrategy::None}),
               cda_rail::exceptions::InvalidInputException);

  solver.initialize_variables(
      {},
      {true, false, true,
       cda_rail::solver::mip_based::LazyConstraintSelectionStrategy::
           OnlyFirstFound},
      {true, 5.55, cda_rail::VelocityRefinementStrategy::None});

  EXPECT_TRUE(solver.m_model_detail.fix_routes);
  EXPECT_APPROX_EQ(solver.m_model_detail.max_velocity_delta, 5.55);
  EXPECT_EQ(solver.m_model_detail.velocity_refinement_strategy,
            cda_rail::VelocityRefinementStrategy::None);
  EXPECT_EQ(solver.m_num_tr, 2);
  EXPECT_EQ(solver.m_num_edges, 20);
  EXPECT_EQ(solver.m_num_vertices, 10);
  EXPECT_EQ(solver.m_num_ttd, 1);
  EXPECT_TRUE(solver.m_solver_strategy.use_lazy_constraints);
  EXPECT_FALSE(solver.m_solver_strategy.include_reverse_headways);
  EXPECT_TRUE(solver.m_solver_strategy.include_higher_velocities_in_edge_expr);
  EXPECT_EQ(solver.m_solver_strategy.lazy_constraint_selection_strategy,
            cda_rail::solver::mip_based::LazyConstraintSelectionStrategy::
                OnlyFirstFound);
  EXPECT_EQ(
      solver.m_solver_strategy.lazy_train_selection_strategy,
      cda_rail::solver::mip_based::LazyTrainSelectionStrategy::OnlyAdjacent);

  // Test if stop data was set correctly

  const auto& tr_stop_data = solver.m_tr_stop_data;
  ASSERT_EQ(tr_stop_data.size(), 2);
  const auto& tr_1_data = tr_stop_data.at(0);
  const auto& tr_2_data = tr_stop_data.at(1);
  ASSERT_EQ(tr_1_data.size(), 2);
  ASSERT_EQ(tr_2_data.size(), 1);
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
  const auto& vel_data = solver.m_velocity_extensions;

  ASSERT_EQ(vel_data.size(), 2);
  const auto& vel_data_1 = vel_data.at(0);
  const auto& vel_data_2 = vel_data.at(1);

  // Train 1
  EXPECT_EQ(vel_data_1.size(), solver.m_num_vertices);
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
  EXPECT_EQ(vel_data_2.size(), solver.m_num_vertices);
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
  solver.m_model_detail.velocity_refinement_strategy =
      cda_rail::VelocityRefinementStrategy::MinOneStep;
  solver.m_model_detail.max_velocity_delta = 10;

  solver.fill_velocity_extensions();

  EXPECT_TRUE(solver.m_model_detail.fix_routes);
  EXPECT_APPROX_EQ(solver.m_model_detail.max_velocity_delta, 10);
  EXPECT_EQ(solver.m_model_detail.velocity_refinement_strategy,
            cda_rail::VelocityRefinementStrategy::MinOneStep);
  EXPECT_EQ(solver.m_num_tr, 2);
  EXPECT_EQ(solver.m_num_edges, 20);
  EXPECT_EQ(solver.m_num_vertices, 10);
  EXPECT_EQ(solver.m_num_ttd, 1);

  // Test new velocity extensions
  const auto& vel_data_new = solver.m_velocity_extensions;

  EXPECT_EQ(vel_data_new.size(), 2);
  const auto& vel_data_new_1 = vel_data_new.at(0);
  const auto& vel_data_new_2 = vel_data_new.at(1);

  // Train 1
  EXPECT_EQ(vel_data_new_1.size(), solver.m_num_vertices);
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
    const auto new_speed = std::max(
        std::sqrt(expected_speeds_1_v2.back() * expected_speeds_1_v2.back() +
                  10) -
            cda_rail::V_MIN,
        expected_speeds_1_v2.back() + cda_rail::V_MIN);
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
    const auto new_speed = std::max(
        std::sqrt(expected_speeds_1_v3.back() * expected_speeds_1_v3.back() +
                  10) -
            cda_rail::V_MIN,
        expected_speeds_1_v3.back() + cda_rail::V_MIN);
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
    const auto new_speed = std::max(
        std::sqrt(expected_speeds_1_v41.back() * expected_speeds_1_v41.back() +
                  20) -
            cda_rail::V_MIN,
        expected_speeds_1_v41.back() + cda_rail::V_MIN);
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
    const auto new_speed = std::max(
        std::sqrt(expected_speeds_1_v51.back() * expected_speeds_1_v51.back() +
                  100) -
            cda_rail::V_MIN,
        expected_speeds_1_v51.back() + cda_rail::V_MIN);
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
    const auto new_speed = std::max(
        std::sqrt(expected_speeds_1_v61.back() * expected_speeds_1_v61.back() +
                  20) -
            cda_rail::V_MIN,
        expected_speeds_1_v61.back() + cda_rail::V_MIN);
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
    const auto new_speed = std::max(
        std::sqrt(expected_speeds_1_v7.back() * expected_speeds_1_v7.back() +
                  20) -
            cda_rail::V_MIN,
        expected_speeds_1_v7.back() + cda_rail::V_MIN);
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
    const auto new_speed = std::max(
        std::sqrt(expected_speeds_1_v8.back() * expected_speeds_1_v8.back() +
                  150) -
            cda_rail::V_MIN,
        expected_speeds_1_v8.back() + cda_rail::V_MIN);
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
  EXPECT_EQ(vel_data_new_2.size(), solver.m_num_vertices);
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
    const auto new_speed = std::max(
        std::sqrt(expected_speeds_2_v1.back() * expected_speeds_2_v1.back() +
                  160) -
            cda_rail::V_MIN,
        expected_speeds_2_v1.back() + cda_rail::V_MIN);
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
    const auto new_speed = std::max(
        std::sqrt(expected_speeds_2_v2.back() * expected_speeds_2_v2.back() +
                  20) -
            cda_rail::V_MIN,
        expected_speeds_2_v2.back() + cda_rail::V_MIN);
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
    const auto new_speed = std::max(
        std::sqrt(expected_speeds_2_v3.back() * expected_speeds_2_v3.back() +
                  20) -
            cda_rail::V_MIN,
        expected_speeds_2_v3.back() + cda_rail::V_MIN);
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
    const auto new_speed = std::max(
        std::sqrt(expected_speeds_2_v41.back() * expected_speeds_2_v41.back() +
                  40) -
            cda_rail::V_MIN,
        expected_speeds_2_v41.back() + cda_rail::V_MIN);
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
    const auto new_speed = std::max(
        std::sqrt(expected_speeds_2_v51.back() * expected_speeds_2_v51.back() +
                  200) -
            cda_rail::V_MIN,
        expected_speeds_2_v51.back() + cda_rail::V_MIN);
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
    const auto new_speed = std::max(
        std::sqrt(expected_speeds_2_v61.back() * expected_speeds_2_v61.back() +
                  40) -
            cda_rail::V_MIN,
        expected_speeds_2_v61.back() + cda_rail::V_MIN);
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
    const auto new_speed = std::max(
        std::sqrt(expected_speeds_2_v42.back() * expected_speeds_2_v42.back() +
                  40) -
            cda_rail::V_MIN,
        expected_speeds_2_v42.back() + cda_rail::V_MIN);
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
    const auto new_speed = std::max(
        std::sqrt(expected_speeds_2_v62.back() * expected_speeds_2_v62.back() +
                  40) -
            cda_rail::V_MIN,
        expected_speeds_2_v62.back() + cda_rail::V_MIN);
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
    const auto new_speed = std::max(
        std::sqrt(expected_speeds_2_v7.back() * expected_speeds_2_v7.back() +
                  40) -
            cda_rail::V_MIN,
        expected_speeds_2_v7.back() + cda_rail::V_MIN);
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

TEST(GenPOMovingBlockMIPSolver, Default1) {
  const std::vector<std::string> paths{"HighSpeedTrack2Trains",
                                       "HighSpeedTrack5Trains"};

  for (const auto& p : paths) {
    const auto instance =
        cda_rail::instances::GeneralPerformanceOptimizationInstance(
            p, "atmos2023", "data");
    cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver solver(instance);
    const auto                                             sol =
        solver.solve({.max_exit_delay = 0.0}, {.abs_mip_gap = 10}, {}, 350);

    EXPECT_TRUE(sol.has_solution()) << "No solution found for instance " << p;
    EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal)
        << "Solution status is not optimal for instance " << p;
    EXPECT_LE(sol.get_obj(), 10)
        << "Objective value is not 0 for instance " << p;

    check_last_train_pos(instance, sol, p);
  }
}

TEST(GenPOMovingBlockMIPSolver, Default2) {
  const std::vector<std::string> paths{"SimpleStation", "SingleTrack",
                                       "SingleTrackWithStation"};

  for (const auto& p : paths) {
    const auto instance =
        cda_rail::instances::GeneralPerformanceOptimizationInstance(
            p, "atmos2023", "data");
    cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver solver(instance);
    const auto                                             sol =
        solver.solve({.max_exit_delay = 0.0}, {.abs_mip_gap = 10}, {}, 240);

    EXPECT_TRUE(sol.has_solution()) << "No solution found for instance " << p;
    EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal)
        << "Solution status is not optimal for instance " << p;
    EXPECT_LE(sol.get_obj(), 10)
        << "Objective value is not 0 for instance " << p;

    check_last_train_pos(instance, sol, p);
  }
}

TEST(GenPOMovingBlockMIPSolver, Default3) {
  const std::vector<std::string> paths{"Stammstrecke4Trains"};

  for (const auto& p : paths) {
    const auto instance =
        cda_rail::instances::GeneralPerformanceOptimizationInstance(
            p, "atmos2023", "data");
    cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver solver(instance);
    const auto                                             sol =
        solver.solve({.max_exit_delay = 0.0}, {.abs_mip_gap = 5}, {}, 700);

    EXPECT_TRUE(sol.has_solution()) << "No solution found for instance " << p;
    EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal)
        << "Solution status is not optimal for instance " << p;
    EXPECT_LE(sol.get_obj(), 5)
        << "Objective value is not 0 for instance " << p;

    check_last_train_pos(instance, sol, p);
  }
}

TEST(GenPOMovingBlockMIPSolver, Default4) {
  const std::vector<std::string> paths{"Stammstrecke8Trains"};

  for (const auto& p : paths) {
    const auto instance =
        cda_rail::instances::GeneralPerformanceOptimizationInstance(
            p, "atmos2023", "data");
    cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver solver(instance);
    const auto                                             sol =
        solver.solve({.max_exit_delay = 0.0}, {.abs_mip_gap = 5}, {}, 700);

    EXPECT_TRUE(sol.has_solution()) << "No solution found for instance " << p;
    EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal)
        << "Solution status is not optimal for instance " << p;
    EXPECT_LE(sol.get_obj(), 5)
        << "Objective value is not 0 for instance " << p;

    check_last_train_pos(instance, sol, p);
  }
}

TEST(GenPOMovingBlockMIPSolver, Default5) {
  const std::vector<std::string> paths{"Stammstrecke16Trains"};

  for (const auto& p : paths) {
    const auto instance =
        cda_rail::instances::GeneralPerformanceOptimizationInstance(
            p, "atmos2023", "data");
    cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver solver(instance);
    const auto                                             sol =
        solver.solve({.max_exit_delay = 0.0}, {.abs_mip_gap = 5}, {}, 700);

    EXPECT_TRUE(sol.has_solution()) << "No solution found for instance " << p;
    EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal)
        << "Solution status is not optimal for instance " << p;
    EXPECT_LE(sol.get_obj(), 5)
        << "Objective value is not 0 for instance " << p;

    check_last_train_pos(instance, sol, p);
  }
}

TEST(GenPOMovingBlockMIPSolver, OnlyFirstWithHigherVelocities1) {
  const std::vector<std::string> paths{
      "HighSpeedTrack2Trains", "HighSpeedTrack5Trains", "SimpleNetwork"};

  for (const auto& p : paths) {
    const auto instance =
        cda_rail::instances::GeneralPerformanceOptimizationInstance(
            p, "atmos2023", "data");
    cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver solver(instance);
    const auto                                             sol = solver.solve(
        {.fix_routes         = false,
         .max_velocity_delta = 5.55,
         .velocity_refinement_strategy =
             cda_rail::VelocityRefinementStrategy::None,
         .max_exit_delay = 0.0},
        {.use_lazy_constraints                   = true,
         .include_reverse_headways               = false,
         .include_higher_velocities_in_edge_expr = true,
         .lazy_constraint_selection_strategy     = cda_rail::solver::mip_based::
             LazyConstraintSelectionStrategy::OnlyFirstFound,
         .abs_mip_gap = 10},
        {}, 240);

    EXPECT_TRUE(sol.has_solution()) << "No solution found for instance " << p;
    EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal)
        << "Solution status is not optimal for instance " << p;
    EXPECT_LE(sol.get_obj(), 10)
        << "Objective value is not 0 for instance " << p;

    check_last_train_pos(instance, sol, p);
  }
}

TEST(GenPOMovingBlockMIPSolver, OnlyFirstWithHigherVelocities2) {
  const std::vector<std::string> paths{"SimpleStation", "SingleTrack",
                                       "SingleTrackWithStation"};

  for (const auto& p : paths) {
    const auto instance =
        cda_rail::instances::GeneralPerformanceOptimizationInstance(
            p, "atmos2023", "data");
    cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver solver(instance);
    const auto                                             sol = solver.solve(
        {.max_exit_delay = 0.0},
        {.use_lazy_constraints                   = true,
         .include_reverse_headways               = false,
         .include_higher_velocities_in_edge_expr = true,
         .lazy_constraint_selection_strategy     = cda_rail::solver::mip_based::
             LazyConstraintSelectionStrategy::OnlyFirstFound,
         .abs_mip_gap = 10},
        {}, 240);

    EXPECT_TRUE(sol.has_solution()) << "No solution found for instance " << p;
    EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal)
        << "Solution status is not optimal for instance " << p;
    EXPECT_LE(sol.get_obj(), 10)
        << "Objective value is not 0 for instance " << p;

    check_last_train_pos(instance, sol, p);
  }
}

TEST(GenPOMovingBlockMIPSolver, OnlyFirstWithHigherVelocities3_4Trains) {
  const std::vector<std::string> paths{"Stammstrecke4Trains"};

  for (const auto& p : paths) {
    const auto instance =
        cda_rail::instances::GeneralPerformanceOptimizationInstance(
            p, "atmos2023", "data");
    cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver solver(instance);
    const auto                                             sol = solver.solve(
        {.max_exit_delay = 0.0},
        {.use_lazy_constraints                   = true,
         .include_reverse_headways               = false,
         .include_higher_velocities_in_edge_expr = true,
         .lazy_constraint_selection_strategy     = cda_rail::solver::mip_based::
             LazyConstraintSelectionStrategy::OnlyFirstFound,
         .abs_mip_gap = 10},
        {}, 700);

    EXPECT_TRUE(sol.has_solution()) << "No solution found for instance " << p;
    EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal)
        << "Solution status is not optimal for instance " << p;
    EXPECT_LE(sol.get_obj(), 10)
        << "Objective value is not 0 for instance " << p;

    check_last_train_pos(instance, sol, p);
  }
}

TEST(GenPOMovingBlockMIPSolver, OnlyFirstWithHigherVelocities3_8Trains) {
  const std::vector<std::string> paths{"Stammstrecke8Trains"};

  for (const auto& p : paths) {
    const auto instance =
        cda_rail::instances::GeneralPerformanceOptimizationInstance(
            p, "atmos2023", "data");
    cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver solver(instance);
    const auto                                             sol = solver.solve(
        {.max_exit_delay = 0.0},
        {.use_lazy_constraints                   = true,
         .include_reverse_headways               = false,
         .include_higher_velocities_in_edge_expr = true,
         .lazy_constraint_selection_strategy     = cda_rail::solver::mip_based::
             LazyConstraintSelectionStrategy::OnlyFirstFound,
         .abs_mip_gap = 10},
        {}, 700);

    EXPECT_TRUE(sol.has_solution()) << "No solution found for instance " << p;
    EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal)
        << "Solution status is not optimal for instance " << p;
    EXPECT_LE(sol.get_obj(), 10)
        << "Objective value is not 0 for instance " << p;

    check_last_train_pos(instance, sol, p);
  }
}

TEST(GenPOMovingBlockMIPSolver, OnlyFirstWithHigherVelocities3_16Trains) {
  const std::vector<std::string> paths{"Stammstrecke16Trains"};

  for (const auto& p : paths) {
    const auto instance =
        cda_rail::instances::GeneralPerformanceOptimizationInstance(
            p, "atmos2023", "data");
    cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver solver(instance);
    const auto                                             sol = solver.solve(
        {.max_exit_delay = 0.0},
        {.use_lazy_constraints                   = true,
         .include_reverse_headways               = false,
         .include_higher_velocities_in_edge_expr = true,
         .lazy_constraint_selection_strategy     = cda_rail::solver::mip_based::
             LazyConstraintSelectionStrategy::OnlyFirstFound,
         .abs_mip_gap = 10},
        {}, 700);

    EXPECT_TRUE(sol.has_solution()) << "No solution found for instance " << p;
    EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal)
        << "Solution status is not optimal for instance " << p;
    EXPECT_LE(sol.get_obj(), 10)
        << "Objective value is not 0 for instance " << p;

    check_last_train_pos(instance, sol, p);
  }
}

TEST(GenPOMovingBlockMIPSolver, All1) {
  const std::vector<std::string> paths{"HighSpeedTrack2Trains",
                                       "HighSpeedTrack5Trains"};

  for (const auto& p : paths) {
    const auto instance =
        cda_rail::instances::GeneralPerformanceOptimizationInstance(
            p, "atmos2023", "data");
    cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver solver(instance);
    const auto                                             sol = solver.solve(
        {.fix_routes         = false,
         .max_velocity_delta = 5.55,
         .velocity_refinement_strategy =
             cda_rail::VelocityRefinementStrategy::None,
         .max_exit_delay = 0.0},
        {.use_lazy_constraints                   = true,
         .include_reverse_headways               = true,
         .include_higher_velocities_in_edge_expr = false,
         .lazy_constraint_selection_strategy     = cda_rail::solver::mip_based::
             LazyConstraintSelectionStrategy::AllChecked,
         .lazy_train_selection_strategy =
             cda_rail::solver::mip_based::LazyTrainSelectionStrategy::All,
         .abs_mip_gap = 10},
        {}, 350);

    EXPECT_TRUE(sol.has_solution()) << "No solution found for instance " << p;
    EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal)
        << "Solution status is not optimal for instance " << p;
    EXPECT_LE(sol.get_obj(), 10)
        << "Objective value is not 0 for instance " << p;

    check_last_train_pos(instance, sol, p);
  }
}

TEST(GenPOMovingBlockMIPSolver, All1b) {
  const std::vector<std::string> paths{"Overtake"};

  for (const auto& p : paths) {
    const auto instance =
        cda_rail::instances::GeneralPerformanceOptimizationInstance(
            p, "atmos2023", "data");
    cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver solver(instance);
    const auto                                             sol = solver.solve(
        {.fix_routes         = true,
         .max_velocity_delta = 5.55,
         .velocity_refinement_strategy =
             cda_rail::VelocityRefinementStrategy::None,
         .max_exit_delay = 0.0},
        {.use_lazy_constraints                   = true,
         .include_reverse_headways               = true,
         .include_higher_velocities_in_edge_expr = false,
         .lazy_constraint_selection_strategy     = cda_rail::solver::mip_based::
             LazyConstraintSelectionStrategy::AllChecked,
         .lazy_train_selection_strategy =
             cda_rail::solver::mip_based::LazyTrainSelectionStrategy::All,
         .abs_mip_gap = 10},
        {}, 700);

    EXPECT_TRUE(sol.has_solution()) << "No solution found for instance " << p;
    EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal)
        << "Solution status is not optimal for instance " << p;
    EXPECT_LE(sol.get_obj(), 10)
        << "Objective value is not 0 for instance " << p;

    check_last_train_pos(instance, sol, p);
  }
}

TEST(GenPOMovingBlockMIPSolver, All2) {
  const std::vector<std::string> paths{"SimpleStation", "SingleTrack",
                                       "SingleTrackWithStation"};

  for (const auto& p : paths) {
    const auto instance =
        cda_rail::instances::GeneralPerformanceOptimizationInstance(
            p, "atmos2023", "data");
    cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver solver(instance);
    const auto                                             sol = solver.solve(
        {.max_exit_delay = 0.0},
        {.use_lazy_constraints                   = true,
         .include_reverse_headways               = true,
         .include_higher_velocities_in_edge_expr = false,
         .lazy_constraint_selection_strategy     = cda_rail::solver::mip_based::
             LazyConstraintSelectionStrategy::AllChecked,
         .lazy_train_selection_strategy =
             cda_rail::solver::mip_based::LazyTrainSelectionStrategy::All,
         .abs_mip_gap = 10},
        {}, 240);

    EXPECT_TRUE(sol.has_solution()) << "No solution found for instance " << p;
    EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal)
        << "Solution status is not optimal for instance " << p;
    EXPECT_LE(sol.get_obj(), 10)
        << "Objective value is not 0 for instance " << p;

    check_last_train_pos(instance, sol, p);
  }
}

TEST(GenPOMovingBlockMIPSolver, All3_4Trains) {
  const std::vector<std::string> paths{"Stammstrecke4Trains"};

  for (const auto& p : paths) {
    const auto instance =
        cda_rail::instances::GeneralPerformanceOptimizationInstance(
            p, "atmos2023", "data");
    cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver solver(instance);
    const auto                                             sol = solver.solve(
        {.max_exit_delay = 0.0},
        {.use_lazy_constraints                   = true,
         .include_reverse_headways               = true,
         .include_higher_velocities_in_edge_expr = false,
         .lazy_constraint_selection_strategy     = cda_rail::solver::mip_based::
             LazyConstraintSelectionStrategy::AllChecked,
         .lazy_train_selection_strategy =
             cda_rail::solver::mip_based::LazyTrainSelectionStrategy::All,
         .abs_mip_gap = 10},
        {}, 700);

    EXPECT_TRUE(sol.has_solution()) << "No solution found for instance " << p;
    EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal)
        << "Solution status is not optimal for instance " << p;
    EXPECT_LE(sol.get_obj(), 10)
        << "Objective value is not 0 for instance " << p;

    check_last_train_pos(instance, sol, p);
  }
}

TEST(GenPOMovingBlockMIPSolver, All3_8Trains) {
  const std::vector<std::string> paths{"Stammstrecke8Trains"};

  for (const auto& p : paths) {
    const auto instance =
        cda_rail::instances::GeneralPerformanceOptimizationInstance(
            p, "atmos2023", "data");
    cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver solver(instance);
    const auto                                             sol = solver.solve(
        {.max_exit_delay = 0.0},
        {.use_lazy_constraints                   = true,
         .include_reverse_headways               = true,
         .include_higher_velocities_in_edge_expr = false,
         .lazy_constraint_selection_strategy     = cda_rail::solver::mip_based::
             LazyConstraintSelectionStrategy::AllChecked,
         .lazy_train_selection_strategy =
             cda_rail::solver::mip_based::LazyTrainSelectionStrategy::All,
         .abs_mip_gap = 10},
        {}, 700);

    EXPECT_TRUE(sol.has_solution()) << "No solution found for instance " << p;
    EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal)
        << "Solution status is not optimal for instance " << p;
    EXPECT_LE(sol.get_obj(), 10)
        << "Objective value is not 0 for instance " << p;

    check_last_train_pos(instance, sol, p);
  }
}

TEST(GenPOMovingBlockMIPSolver, All3_16Trains) {
  const std::vector<std::string> paths{"Stammstrecke16Trains"};

  for (const auto& p : paths) {
    const auto instance =
        cda_rail::instances::GeneralPerformanceOptimizationInstance(
            p, "atmos2023", "data");
    cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver solver(instance);
    const auto                                             sol = solver.solve(
        {.max_exit_delay = 0.0},
        {.use_lazy_constraints                   = true,
         .include_reverse_headways               = true,
         .include_higher_velocities_in_edge_expr = false,
         .lazy_constraint_selection_strategy     = cda_rail::solver::mip_based::
             LazyConstraintSelectionStrategy::AllChecked,
         .lazy_train_selection_strategy =
             cda_rail::solver::mip_based::LazyTrainSelectionStrategy::All,
         .abs_mip_gap = 10},
        {}, 700);

    EXPECT_TRUE(sol.has_solution()) << "No solution found for instance " << p;
    EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal)
        << "Solution status is not optimal for instance " << p;
    EXPECT_LE(sol.get_obj(), 10)
        << "Objective value is not 0 for instance " << p;

    check_last_train_pos(instance, sol, p);
  }
}

TEST(GenPOMovingBlockMIPSolver, NoLazy1) {
  const std::vector<std::string> paths{"SimpleStation"};

  for (const auto& p : paths) {
    const auto instance =
        cda_rail::instances::GeneralPerformanceOptimizationInstance(
            p, "atmos2023", "data");
    cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver solver(instance);
    const auto                                             sol = solver.solve(
        {.max_exit_delay = 0.0},
        {.use_lazy_constraints = false, .abs_mip_gap = 10}, {}, 700);

    EXPECT_TRUE(sol.has_solution()) << "No solution found for instance " << p;
    EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal)
        << "Solution status is not optimal for instance " << p;
    EXPECT_LE(sol.get_obj(), 10)
        << "Objective value is not 0 for instance " << p;

    check_last_train_pos(instance, sol, p);
  }
}

TEST(GenPOMovingBlockMIPSolver, NoLazy2) {
  const std::vector<std::string> paths{"SimpleNetwork"};

  for (const auto& p : paths) {
    const auto instance =
        cda_rail::instances::GeneralPerformanceOptimizationInstance(
            p, "atmos2023", "data");
    cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver solver(instance);
    const auto                                             sol = solver.solve(
        {.fix_routes         = false,
         .max_velocity_delta = 5.55,
         .velocity_refinement_strategy =
             cda_rail::VelocityRefinementStrategy::None,
         .max_exit_delay = 0.0},
        {.use_lazy_constraints = false, .abs_mip_gap = 10}, {}, 700);

    EXPECT_TRUE(sol.has_solution()) << "No solution found for instance " << p;
    EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal)
        << "Solution status is not optimal for instance " << p;
    EXPECT_LE(sol.get_obj(), 10)
        << "Objective value is not 0 for instance " << p;

    check_last_train_pos(instance, sol, p);
  }
}

TEST(GenPOMovingBlockMIPSolver, NoLazy3) {
  const std::vector<std::string> paths{"SingleTrack"};

  for (const auto& p : paths) {
    const auto instance =
        cda_rail::instances::GeneralPerformanceOptimizationInstance(
            p, "atmos2023", "data");
    cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver solver(instance);
    const auto                                             sol = solver.solve(
        {.max_exit_delay = 0.0},
        {.use_lazy_constraints = false, .abs_mip_gap = 10}, {}, 700);

    EXPECT_TRUE(sol.has_solution()) << "No solution found for instance " << p;
    EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal)
        << "Solution status is not optimal for instance " << p;
    EXPECT_LE(sol.get_obj(), 10)
        << "Objective value is not 0 for instance " << p;

    check_last_train_pos(instance, sol, p);
  }
}

TEST(GenPOMovingBlockMIPSolver, NoLazySimplified1) {
  const std::vector<std::string> paths{"SimpleStation"};

  for (const auto& p : paths) {
    const auto instance =
        cda_rail::instances::GeneralPerformanceOptimizationInstance(
            p, "atmos2023", "data");
    cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver solver(instance);
    const auto                                             sol = solver.solve(
        {.fix_routes         = false,
         .max_velocity_delta = 5.55,
         .velocity_refinement_strategy =
             cda_rail::VelocityRefinementStrategy::MinOneStep,
         .simplify_headway_constraints          = true,
         .strengthen_vertex_headway_constraints = true,
         .max_exit_delay                        = 0.0},
        {.use_lazy_constraints = false, .abs_mip_gap = 10}, {}, 700, true);

    EXPECT_TRUE(sol.has_solution()) << "No solution found for instance " << p;
    EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal)
        << "Solution status is not optimal for instance " << p;
    EXPECT_LE(sol.get_obj(), 10)
        << "Objective value is not 0 for instance " << p;

    check_last_train_pos(instance, sol, p);
  }
}

TEST(GenPOMovingBlockMIPSolver, NoLazySimplified2) {
  const std::vector<std::string> paths{"SimpleNetwork"};

  for (const auto& p : paths) {
    const auto instance =
        cda_rail::instances::GeneralPerformanceOptimizationInstance(
            p, "atmos2023", "data");
    cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver solver(instance);
    const auto                                             sol = solver.solve(
        {.fix_routes         = false,
         .max_velocity_delta = 5.55,
         .velocity_refinement_strategy =
             cda_rail::VelocityRefinementStrategy::MinOneStep,
         .simplify_headway_constraints          = true,
         .strengthen_vertex_headway_constraints = true,
         .max_exit_delay                        = 0.0},
        {.use_lazy_constraints = false, .abs_mip_gap = 10}, {}, 700, true);

    EXPECT_TRUE(sol.has_solution()) << "No solution found for instance " << p;
    EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal)
        << "Solution status is not optimal for instance " << p;
    EXPECT_LE(sol.get_obj(), 10)
        << "Objective value is not 0 for instance " << p;

    check_last_train_pos(instance, sol, p);
  }
}

TEST(GenPOMovingBlockMIPSolver, NoLazySimplified3) {
  const std::vector<std::string> paths{"SingleTrack"};

  for (const auto& p : paths) {
    const auto instance =
        cda_rail::instances::GeneralPerformanceOptimizationInstance(
            p, "atmos2023", "data");
    cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver solver(instance);
    const auto                                             sol = solver.solve(
        {.fix_routes         = false,
         .max_velocity_delta = 5.55,
         .velocity_refinement_strategy =
             cda_rail::VelocityRefinementStrategy::MinOneStep,
         .simplify_headway_constraints          = true,
         .strengthen_vertex_headway_constraints = true,
         .max_exit_delay                        = 0.0},
        {.use_lazy_constraints = false, .abs_mip_gap = 10}, {}, 700, true);

    EXPECT_TRUE(sol.has_solution()) << "No solution found for instance " << p;
    EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal)
        << "Solution status is not optimal for instance " << p;
    EXPECT_LE(sol.get_obj(), 10)
        << "Objective value is not 0 for instance " << p;

    check_last_train_pos(instance, sol, p);
  }
}

TEST(GenPOMovingBlockMIPSolver, StandardLazySimplified1) {
  const std::vector<std::string> paths{"SimpleStation"};

  for (const auto& p : paths) {
    const auto instance =
        cda_rail::instances::GeneralPerformanceOptimizationInstance(
            p, "atmos2023", "data");
    cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver solver(instance);
    const auto                                             sol = solver.solve(
        {.fix_routes         = false,
         .max_velocity_delta = 5.55,
         .velocity_refinement_strategy =
             cda_rail::VelocityRefinementStrategy::MinOneStep,
         .simplify_headway_constraints          = true,
         .strengthen_vertex_headway_constraints = true,
         .max_exit_delay                        = 0.0},
        {.use_lazy_constraints = true, .abs_mip_gap = 10}, {}, 700, true);

    EXPECT_TRUE(sol.has_solution()) << "No solution found for instance " << p;
    EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal)
        << "Solution status is not optimal for instance " << p;
    EXPECT_LE(sol.get_obj(), 10)
        << "Objective value is not 0 for instance " << p;

    check_last_train_pos(instance, sol, p);
  }
}

TEST(GenPOMovingBlockMIPSolver, StandardLazySimplified2) {
  const std::vector<std::string> paths{"SimpleNetwork"};

  for (const auto& p : paths) {
    const auto instance =
        cda_rail::instances::GeneralPerformanceOptimizationInstance(
            p, "atmos2023", "data");
    cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver solver(instance);
    const auto                                             sol = solver.solve(
        {.fix_routes         = false,
         .max_velocity_delta = 5.55,
         .velocity_refinement_strategy =
             cda_rail::VelocityRefinementStrategy::None,
         .simplify_headway_constraints          = true,
         .strengthen_vertex_headway_constraints = true,
         .max_exit_delay                        = 0.0},
        {.use_lazy_constraints = true, .abs_mip_gap = 10}, {}, 700, true);

    EXPECT_TRUE(sol.has_solution()) << "No solution found for instance " << p;
    EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal)
        << "Solution status is not optimal for instance " << p;
    EXPECT_LE(sol.get_obj(), 10)
        << "Objective value is not 0 for instance " << p;

    check_last_train_pos(instance, sol, p);
  }
}

TEST(GenPOMovingBlockMIPSolver, StandardLazySimplified3) {
  const std::vector<std::string> paths{"SingleTrack"};

  for (const auto& p : paths) {
    const auto instance =
        cda_rail::instances::GeneralPerformanceOptimizationInstance(
            p, "atmos2023", "data");
    cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver solver(instance);
    const auto                                             sol = solver.solve(
        {.fix_routes         = false,
         .max_velocity_delta = 5.55,
         .velocity_refinement_strategy =
             cda_rail::VelocityRefinementStrategy::MinOneStep,
         .simplify_headway_constraints          = true,
         .strengthen_vertex_headway_constraints = true,
         .max_exit_delay                        = 0.0},
        {.use_lazy_constraints = true, .abs_mip_gap = 10}, {}, 700, true);

    EXPECT_TRUE(sol.has_solution()) << "No solution found for instance " << p;
    EXPECT_EQ(sol.get_status(), cda_rail::SolutionStatus::Optimal)
        << "Solution status is not optimal for instance " << p;
    EXPECT_LE(sol.get_obj(), 10)
        << "Objective value is not 0 for instance " << p;

    check_last_train_pos(instance, sol, p);
  }
}

TEST(GenPOMovingBlockMIPSolver, SimpleStationExportOptions) {
  const auto instance =
      cda_rail::instances::GeneralPerformanceOptimizationInstance(
          "SimpleStation", "atmos2023", "data");
  cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver solver(instance);

  cleanup_export_dirs();

  std::error_code ec;

  std::cout << "Starting first solve" << std::endl;
  const auto obj_val = solver.solve(
      {.fix_routes         = false,
       .max_velocity_delta = 5.55,
       .velocity_refinement_strategy =
           cda_rail::VelocityRefinementStrategy::None,
       .max_exit_delay = 0.0},
      {.abs_mip_gap = 10},
      {cda_rail::ExportOption::ExportLP, "tmp1file", "tmp1folder"}, 30, true);

  // Expect optimal value of 0
  EXPECT_EQ(obj_val.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_LE(obj_val.get_obj(), 10);
  // Check that tmp1folder and tmp1folder/tmp1file.mps and
  // tmp1folder/tmp1file.sol exist
  EXPECT_TRUE(std::filesystem::exists("tmp1folder"));
  EXPECT_TRUE(std::filesystem::exists("tmp1folder/tmp1file.mps"));
  EXPECT_TRUE(std::filesystem::exists("tmp1folder/tmp1file.json"));

  cleanup_export_dirs();

  std::cout << "Starting second solve" << std::endl;
  const auto obj_val2 = solver.solve(
      {.fix_routes         = false,
       .max_velocity_delta = 5.55,
       .velocity_refinement_strategy =
           cda_rail::VelocityRefinementStrategy::None,
       .max_exit_delay = 0.0},
      {.abs_mip_gap = 10},
      {cda_rail::ExportOption::ExportSolution, "tmp2file", "tmp2folder"}, 30,
      true);

  // Expect optimal value of 0
  EXPECT_EQ(obj_val2.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_LE(obj_val2.get_obj(), 10);
  // Check that tmp2folder and solutions structure exists
  EXPECT_TRUE(std::filesystem::exists("tmp2folder"));
  EXPECT_TRUE(std::filesystem::exists(
      "tmp2folder/solutions/tmp2file/atmos2023/SimpleStation"));
  // Expect solution JSON files to exist
  EXPECT_TRUE(std::filesystem::exists(
      "tmp2folder/solutions/tmp2file/atmos2023/SimpleStation/routes.json"));
  EXPECT_TRUE(std::filesystem::exists("tmp2folder/solutions/tmp2file/atmos2023/"
                                      "SimpleStation/solution_data.json"));
  EXPECT_TRUE(std::filesystem::exists("tmp2folder/solutions/tmp2file/atmos2023/"
                                      "SimpleStation/train_exit_times.json"));
  EXPECT_TRUE(std::filesystem::exists(
      "tmp2folder/solutions/tmp2file/atmos2023/SimpleStation/train_pos.json"));
  EXPECT_TRUE(std::filesystem::exists("tmp2folder/solutions/tmp2file/atmos2023/"
                                      "SimpleStation/train_speed.json"));
  EXPECT_TRUE(std::filesystem::exists("tmp2folder/solutions/tmp2file/atmos2023/"
                                      "SimpleStation/train_stop_times.json"));
  // Expect no instance or network folders to exist
  EXPECT_FALSE(std::filesystem::exists("tmp2folder/instances"));
  EXPECT_FALSE(std::filesystem::exists("tmp2folder/networks"));

  cleanup_export_dirs();

  std::cout << "Starting third solve" << std::endl;
  const auto obj_val3 =
      solver.solve({.fix_routes         = false,
                    .max_velocity_delta = 5.55,
                    .velocity_refinement_strategy =
                        cda_rail::VelocityRefinementStrategy::None,
                    .max_exit_delay = 0.0},
                   {.abs_mip_gap = 10},
                   {cda_rail::ExportOption::ExportSolutionWithInstance,
                    "tmp3file", "tmp3folder"},
                   30, true);

  // Expect optimal value of 0
  EXPECT_EQ(obj_val3.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_LE(obj_val3.get_obj(), 10);
  // Check that corresponding folders exist
  EXPECT_TRUE(std::filesystem::exists("tmp3folder"));
  EXPECT_TRUE(
      std::filesystem::exists("tmp3folder/instances/atmos2023/SimpleStation"));
  EXPECT_TRUE(std::filesystem::exists("tmp3folder/networks/SimpleStation"));
  EXPECT_TRUE(std::filesystem::exists(
      "tmp3folder/solutions/tmp3file/atmos2023/SimpleStation"));
  // Expect instance files to exist
  EXPECT_TRUE(std::filesystem::exists(
      "tmp3folder/instances/atmos2023/SimpleStation/network.json"));
  EXPECT_TRUE(std::filesystem::exists(
      "tmp3folder/instances/atmos2023/SimpleStation/problem_data.json"));
  EXPECT_TRUE(std::filesystem::exists(
      "tmp3folder/instances/atmos2023/SimpleStation/routes/routes.json"));
  EXPECT_TRUE(std::filesystem::exists(
      "tmp3folder/instances/atmos2023/SimpleStation/timetable/schedules.json"));
  EXPECT_TRUE(std::filesystem::exists(
      "tmp3folder/instances/atmos2023/SimpleStation/timetable/stations.json"));
  EXPECT_TRUE(std::filesystem::exists(
      "tmp3folder/instances/atmos2023/SimpleStation/timetable/trains.json"));
  // Expect network files to exist
  EXPECT_TRUE(std::filesystem::exists(
      "tmp3folder/networks/SimpleStation/successors.txt"));
  EXPECT_TRUE(std::filesystem::exists(
      "tmp3folder/networks/SimpleStation/successors_cpp.json"));
  EXPECT_TRUE(std::filesystem::exists(
      "tmp3folder/networks/SimpleStation/tracks.graphml"));
  // Expect solution JSON files to exist
  EXPECT_TRUE(std::filesystem::exists(
      "tmp3folder/solutions/tmp3file/atmos2023/SimpleStation/routes.json"));
  EXPECT_TRUE(std::filesystem::exists("tmp3folder/solutions/tmp3file/atmos2023/"
                                      "SimpleStation/solution_data.json"));
  EXPECT_TRUE(std::filesystem::exists("tmp3folder/solutions/tmp3file/atmos2023/"
                                      "SimpleStation/train_exit_times.json"));
  EXPECT_TRUE(std::filesystem::exists(
      "tmp3folder/solutions/tmp3file/atmos2023/SimpleStation/train_pos.json"));
  EXPECT_TRUE(std::filesystem::exists("tmp3folder/solutions/tmp3file/atmos2023/"
                                      "SimpleStation/train_speed.json"));
  EXPECT_TRUE(std::filesystem::exists("tmp3folder/solutions/tmp3file/atmos2023/"
                                      "SimpleStation/train_stop_times.json"));

  cleanup_export_dirs();

  std::cout << "Starting fourth solve" << std::endl;
  const auto obj_val4 = solver.solve(
      {.fix_routes         = false,
       .max_velocity_delta = 5.55,
       .velocity_refinement_strategy =
           cda_rail::VelocityRefinementStrategy::None,
       .max_exit_delay = 0.0},
      {.abs_mip_gap = 10},
      {cda_rail::ExportOption::NoExport, "tmp4file", "tmp4folder"}, 30, true);

  // Expect optimal value of 0
  EXPECT_EQ(obj_val4.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_LE(obj_val4.get_obj(), 10);
  // Expect no folder tmp4folder to exist
  EXPECT_FALSE(std::filesystem::exists("tmp4folder"));

  std::cout << "Starting fifth solve" << std::endl;
  const auto obj_val5 = solver.solve(
      {.fix_routes         = false,
       .max_velocity_delta = 5.55,
       .velocity_refinement_strategy =
           cda_rail::VelocityRefinementStrategy::None,
       .max_exit_delay = 0.0},
      {.abs_mip_gap = 10},
      {cda_rail::ExportOption::ExportSolutionAndLP, "tmp5file", "tmp5folder"},
      30, false);

  // Expect optimal value of 0
  EXPECT_EQ(obj_val5.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_LE(obj_val5.get_obj(), 10);
  // Check that tmp5folder exists with LP files and solutions
  EXPECT_TRUE(std::filesystem::exists("tmp5folder"));
  EXPECT_TRUE(std::filesystem::exists("tmp5folder/tmp5file.mps"));
  EXPECT_TRUE(std::filesystem::exists("tmp5folder/tmp5file.json"));
  EXPECT_TRUE(std::filesystem::exists(
      "tmp5folder/solutions/tmp5file/atmos2023/SimpleStation"));
  // Expect solution JSON files to exist
  EXPECT_TRUE(std::filesystem::exists(
      "tmp5folder/solutions/tmp5file/atmos2023/SimpleStation/routes.json"));
  EXPECT_TRUE(std::filesystem::exists("tmp5folder/solutions/tmp5file/atmos2023/"
                                      "SimpleStation/solution_data.json"));
  EXPECT_TRUE(std::filesystem::exists("tmp5folder/solutions/tmp5file/atmos2023/"
                                      "SimpleStation/train_exit_times.json"));
  EXPECT_TRUE(std::filesystem::exists(
      "tmp5folder/solutions/tmp5file/atmos2023/SimpleStation/train_pos.json"));
  EXPECT_TRUE(std::filesystem::exists("tmp5folder/solutions/tmp5file/atmos2023/"
                                      "SimpleStation/train_speed.json"));
  EXPECT_TRUE(std::filesystem::exists("tmp5folder/solutions/tmp5file/atmos2023/"
                                      "SimpleStation/train_stop_times.json"));
  // Expect no instance or network folders to exist
  EXPECT_FALSE(std::filesystem::exists("tmp5folder/instances"));
  EXPECT_FALSE(std::filesystem::exists("tmp5folder/networks"));

  cleanup_export_dirs();

  std::cout << "Starting sixth solve" << std::endl;
  const auto obj_val6 =
      solver.solve({.fix_routes         = false,
                    .max_velocity_delta = 5.55,
                    .velocity_refinement_strategy =
                        cda_rail::VelocityRefinementStrategy::None,
                    .max_exit_delay = 0.0},
                   {.abs_mip_gap = 10},
                   {cda_rail::ExportOption::ExportSolutionWithInstanceAndLP,
                    "tmp6file", "tmp6folder"},
                   30, false);

  // Expect optimal value of 0
  EXPECT_EQ(obj_val6.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_LE(obj_val6.get_obj(), 10);
  // Check that tmp6folder exists with LP files, instance, networks, and
  // solutions
  EXPECT_TRUE(std::filesystem::exists("tmp6folder"));
  EXPECT_TRUE(std::filesystem::exists("tmp6folder/tmp6file.mps"));
  EXPECT_TRUE(std::filesystem::exists("tmp6folder/tmp6file.json"));
  EXPECT_TRUE(
      std::filesystem::exists("tmp6folder/instances/atmos2023/SimpleStation"));
  EXPECT_TRUE(std::filesystem::exists("tmp6folder/networks/SimpleStation"));
  EXPECT_TRUE(std::filesystem::exists(
      "tmp6folder/solutions/tmp6file/atmos2023/SimpleStation"));
  // Expect instance files to exist
  EXPECT_TRUE(std::filesystem::exists(
      "tmp6folder/instances/atmos2023/SimpleStation/network.json"));
  EXPECT_TRUE(std::filesystem::exists(
      "tmp6folder/instances/atmos2023/SimpleStation/problem_data.json"));
  EXPECT_TRUE(std::filesystem::exists(
      "tmp6folder/instances/atmos2023/SimpleStation/routes/routes.json"));
  EXPECT_TRUE(std::filesystem::exists(
      "tmp6folder/instances/atmos2023/SimpleStation/timetable/schedules.json"));
  EXPECT_TRUE(std::filesystem::exists(
      "tmp6folder/instances/atmos2023/SimpleStation/timetable/stations.json"));
  EXPECT_TRUE(std::filesystem::exists(
      "tmp6folder/instances/atmos2023/SimpleStation/timetable/trains.json"));
  // Expect network files to exist
  EXPECT_TRUE(std::filesystem::exists(
      "tmp6folder/networks/SimpleStation/successors.txt"));
  EXPECT_TRUE(std::filesystem::exists(
      "tmp6folder/networks/SimpleStation/successors_cpp.json"));
  EXPECT_TRUE(std::filesystem::exists(
      "tmp6folder/networks/SimpleStation/tracks.graphml"));
  // Expect solution JSON files to exist
  EXPECT_TRUE(std::filesystem::exists(
      "tmp6folder/solutions/tmp6file/atmos2023/SimpleStation/routes.json"));
  EXPECT_TRUE(std::filesystem::exists("tmp6folder/solutions/tmp6file/atmos2023/"
                                      "SimpleStation/solution_data.json"));
  EXPECT_TRUE(std::filesystem::exists("tmp6folder/solutions/tmp6file/atmos2023/"
                                      "SimpleStation/train_exit_times.json"));
  EXPECT_TRUE(std::filesystem::exists(
      "tmp6folder/solutions/tmp6file/atmos2023/SimpleStation/train_pos.json"));
  EXPECT_TRUE(std::filesystem::exists("tmp6folder/solutions/tmp6file/atmos2023/"
                                      "SimpleStation/train_speed.json"));
  EXPECT_TRUE(std::filesystem::exists("tmp6folder/solutions/tmp6file/atmos2023/"
                                      "SimpleStation/train_stop_times.json"));

  cleanup_export_dirs();

  std::cout << "Starting seventh solve" << std::endl;
  const auto obj_val7 = solver.solve(
      {.fix_routes         = false,
       .max_velocity_delta = 5.55,
       .velocity_refinement_strategy =
           cda_rail::VelocityRefinementStrategy::None,
       .max_exit_delay = 0.0},
      {}, {cda_rail::ExportOption::ExportSolutionWithInstanceAndLP}, 30, false);

  // Expect optimal value of 0
  EXPECT_EQ(obj_val7.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_LE(obj_val7.get_obj(), 10);
  // Expect LP files to exist
  EXPECT_TRUE(std::filesystem::exists("model.mps"));
  EXPECT_TRUE(std::filesystem::exists("model.json"));
  // Expect instance files to exist
  EXPECT_TRUE(std::filesystem::exists(
      "instances/atmos2023/SimpleStation/network.json"));
  EXPECT_TRUE(std::filesystem::exists(
      "instances/atmos2023/SimpleStation/problem_data.json"));
  EXPECT_TRUE(std::filesystem::exists(
      "instances/atmos2023/SimpleStation/routes/routes.json"));
  EXPECT_TRUE(std::filesystem::exists(
      "instances/atmos2023/SimpleStation/timetable/schedules.json"));
  EXPECT_TRUE(std::filesystem::exists(
      "instances/atmos2023/SimpleStation/timetable/stations.json"));
  EXPECT_TRUE(std::filesystem::exists(
      "instances/atmos2023/SimpleStation/timetable/trains.json"));
  // Expect network files to exist
  EXPECT_TRUE(std::filesystem::exists("networks/SimpleStation/successors.txt"));
  EXPECT_TRUE(
      std::filesystem::exists("networks/SimpleStation/successors_cpp.json"));
  EXPECT_TRUE(std::filesystem::exists("networks/SimpleStation/tracks.graphml"));
  // Expect solution JSON files to exist
  EXPECT_TRUE(std::filesystem::exists(
      "solutions/model/atmos2023/SimpleStation/routes.json"));
  EXPECT_TRUE(std::filesystem::exists(
      "solutions/model/atmos2023/SimpleStation/solution_data.json"));
  EXPECT_TRUE(std::filesystem::exists(
      "solutions/model/atmos2023/SimpleStation/train_exit_times.json"));
  EXPECT_TRUE(std::filesystem::exists(
      "solutions/model/atmos2023/SimpleStation/train_pos.json"));
  EXPECT_TRUE(std::filesystem::exists(
      "solutions/model/atmos2023/SimpleStation/train_speed.json"));
  EXPECT_TRUE(std::filesystem::exists(
      "solutions/model/atmos2023/SimpleStation/train_stop_times.json"));

  cleanup_export_dirs();
}

TEST(GenPOMovingBlockMIPSolver, RASToy) {
  const std::vector<std::string> paths{"toy"};

  for (const auto& p : paths) {
    const auto instance =
        cda_rail::instances::GeneralPerformanceOptimizationInstance(
            "toy", "ras", "data");
    cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver solver(instance);
    const auto                                             sol = solver.solve(
        {.fix_routes         = false,
         .max_velocity_delta = 10,
         .velocity_refinement_strategy =
             cda_rail::VelocityRefinementStrategy::MinOneStep,
         .simplify_headway_constraints          = true,
         .strengthen_vertex_headway_constraints = true},
        {.use_lazy_constraints = true, .abs_mip_gap = 10}, {}, 700, true);

    EXPECT_TRUE(sol.has_solution()) << "No solution found for instance " << p;
  }
}

// NOLINTEND (clang-analyzer-deadcode.DeadStores)
