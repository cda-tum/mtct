#include "Definitions.hpp"
#include "VSSModel.hpp"
#include "solver/mip-based/VSSGenTimetableSolver.hpp"

#include "gtest/gtest.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <iostream>
#include <map>
#include <string>
#include <system_error>
#include <vector>

using std::size_t;

namespace {
using TrainDirection = cda_rail::instances::
    SolVSSGeneralPerformanceOptimizationInstance::TrainDirection;

// Only usable within this translation unit.
std::string order_to_string(cda_rail::TrainList const&    train_list,
                            cda_rail::index_vector const& order) {
  std::string ret;
  for (auto const& tr_id : order) {
    if (!ret.empty()) {
      ret += ", ";
    }
    ret += train_list.get_train(tr_id).get_name();
  }
  return ret;
}

// Only usable within this translation unit.
std::string order_to_string(cda_rail::TrainList const&         train_list,
                            std::vector<TrainDirection> const& order) {
  std::string ret;
  for (auto const& tr : order) {
    if (!ret.empty()) {
      ret += ", ";
    }
    ret += train_list.get_train(tr.train_id).get_name();
    ret += tr.original_direction ? " (forward)" : " (reverse)";
  }
  return ret;
}

// Only usable within this translation unit.
void check_exit_times_within_dt_and_order(
    const cda_rail::instances::SolVSSGeneralPerformanceOptimizationInstance&
                 sol,
    double const dt, const std::string& label) {
  // Solutions of the discrete VSS model do not contain any train data, hence
  // there is nothing to check
  if (!sol.has_solution()) {
    return;
  }

  auto const& instance   = *sol.get_instance();
  auto const& network    = instance.get_const_network();
  auto const& train_list = instance.get_const_train_list();

  for (auto const& tr : train_list) {
    auto const& tr_schedule = instance.get_const_schedule(tr.get_name());
    EXPECT_GE(sol.get_exit_time(tr.get_name()), tr_schedule.get_exit_time())
        << label << ", Train: " << tr.get_name()
        << ", Exit time: " << sol.get_exit_time(tr.get_name())
        << ", Schedule exit time: " << tr_schedule.get_exit_time();
    EXPECT_LE(sol.get_exit_time(tr.get_name()),
              tr_schedule.get_exit_time() + dt)
        << label << ", Train: " << tr.get_name()
        << ", Exit time: " << sol.get_exit_time(tr.get_name())
        << ", Schedule exit time: " << tr_schedule.get_exit_time();
  }

  // Every train enters the network on the unique outgoing edge of its entry
  // vertex and leaves it on the unique incoming edge of its exit vertex. Hence,
  // the train order on such an edge has to coincide with the order induced by
  // the scheduled entry respectively exit times at that vertex. Taking the
  // reverse edge into account, both entering and exiting trains share one
  // order, where the direction is given by whether a train enters or exits.
  struct BoundaryTrain {
    size_t tr_id;
    bool   entering;
    double time;
  };
  std::map<size_t, std::vector<BoundaryTrain>> boundary_trains;
  for (size_t tr_id = 0; tr_id < train_list.size(); ++tr_id) {
    auto const& tr_schedule = instance.get_const_schedule(tr_id);
    boundary_trains[tr_schedule.get_entry_vertex()].emplace_back(
        tr_id, true, tr_schedule.get_entry_time());
    boundary_trains[tr_schedule.get_exit_vertex()].emplace_back(
        tr_id, false, tr_schedule.get_exit_time());
  }

  for (auto& [v_id, v_trains] : boundary_trains) {
    std::ranges::sort(v_trains,
                      [](BoundaryTrain const& tr1, BoundaryTrain const& tr2) {
                        return tr1.time < tr2.time;
                      });

    auto const& v_name = network.get_vertex(v_id).name;
    auto const  out_e  = network.out_edges(v_id);
    auto const  in_e   = network.in_edges(v_id);

    cda_rail::index_vector expected_entry_order;
    cda_rail::index_vector expected_exit_order;
    for (auto const& tr : v_trains) {
      if (tr.entering) {
        expected_entry_order.push_back(tr.tr_id);
      } else {
        expected_exit_order.push_back(tr.tr_id);
      }
    }

    // Entering trains are ordered by their entry times
    if (!expected_entry_order.empty()) {
      EXPECT_EQ(out_e.size(), 1) << label << ", Entry vertex: " << v_name
                                 << " has no unique outgoing edge";
      if (out_e.size() == 1) {
        auto const entry_order = sol.get_train_order(*out_e.begin());
        EXPECT_EQ(entry_order, expected_entry_order)
            << label << ", Entry vertex: " << v_name << ", Expected order: "
            << order_to_string(train_list, expected_entry_order)
            << ", Actual order: " << order_to_string(train_list, entry_order);
      }
    }

    // Exiting trains are ordered by their exit times
    if (!expected_exit_order.empty()) {
      EXPECT_EQ(in_e.size(), 1) << label << ", Exit vertex: " << v_name
                                << " has no unique incoming edge";
      if (in_e.size() == 1) {
        auto const exit_order = sol.get_train_order(*in_e.begin());
        EXPECT_EQ(exit_order, expected_exit_order)
            << label << ", Exit vertex: " << v_name << ", Expected order: "
            << order_to_string(train_list, expected_exit_order)
            << ", Actual order: " << order_to_string(train_list, exit_order);
      }
    }

    // Entering and exiting trains share one order on the edge and its reverse
    if (out_e.size() != 1 && in_e.size() != 1) {
      continue;
    }
    bool const   entering_is_forward = out_e.size() == 1;
    size_t const reference_edge =
        entering_is_forward ? *out_e.begin() : *in_e.begin();
    std::vector<TrainDirection> expected_order;
    expected_order.reserve(v_trains.size());
    for (auto const& tr : v_trains) {
      expected_order.emplace_back(tr.tr_id, tr.entering == entering_is_forward);
    }
    auto const order_with_reverse =
        sol.get_train_order_with_reverse(reference_edge);
    std::string const order_message =
        label + ", Vertex: " + v_name +
        ", Expected order: " + order_to_string(train_list, expected_order) +
        ", Actual order: " + order_to_string(train_list, order_with_reverse);
    EXPECT_EQ(order_with_reverse.size(), expected_order.size())
        << order_message;
    for (size_t i = 0;
         i < std::min(order_with_reverse.size(), expected_order.size()); ++i) {
      EXPECT_EQ(order_with_reverse.at(i).train_id,
                expected_order.at(i).train_id)
          << order_message;
      EXPECT_EQ(order_with_reverse.at(i).original_direction,
                expected_order.at(i).original_direction)
          << order_message;
    }
  }
}
} // namespace

TEST(VSSGenSolver, GurobiVSSDiscretizeInstanceWithoutChange) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "SimpleStation", "atmos2023", "data");

  const auto num_vertices =
      solver.get_instance().get_const_network().number_of_vertices();
  // NOLINTNEXTLINE(clang-diagnostic-unused-result)
  solver.solve({30, true},
               {cda_rail::vss::Model(cda_rail::vss::ModelType::Discrete,
                                     {cda_rail::vss::functions::uniform})});

  EXPECT_EQ(num_vertices,
            solver.get_instance().get_const_network().number_of_vertices());
}

TEST(VSSGenSolver, GurobiVSSGenDeltaTDefault) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "SimpleStation", "atmos2023", "data");

  std::cout << "--------------------- TEST 1 ---------------------------"
            << '\n';
  auto obj_val_6 = solver.solve({6});
  std::cout << "--------------------- TEST 2 ---------------------------"
            << '\n';
  auto obj_val_15 = solver.solve({15});
  std::cout << "--------------------- TEST 3 ---------------------------"
            << '\n';
  auto obj_val_11 = solver.solve({11});
  std::cout << "--------------------- TEST 4 ---------------------------"
            << '\n';
  auto obj_val_18 = solver.solve({18});
  std::cout << "--------------------- TEST 5 ---------------------------"
            << '\n';
  auto obj_val_30 = solver.solve({30});

  EXPECT_EQ(obj_val_6.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_15.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_11.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_18.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_30.get_status(), cda_rail::SolutionStatus::Optimal);

  EXPECT_EQ(obj_val_6.get_obj(), 1);
  EXPECT_EQ(obj_val_15.get_obj(), 1);
  EXPECT_EQ(obj_val_11.get_obj(), 1);
  EXPECT_EQ(obj_val_18.get_obj(), 1);
  EXPECT_EQ(obj_val_30.get_obj(), 1);

  check_exit_times_within_dt_and_order(obj_val_6, 6, "obj_val_6");
  check_exit_times_within_dt_and_order(obj_val_15, 15, "obj_val_15");
  check_exit_times_within_dt_and_order(obj_val_11, 11, "obj_val_11");
  check_exit_times_within_dt_and_order(obj_val_18, 18, "obj_val_18");
  check_exit_times_within_dt_and_order(obj_val_30, 30, "obj_val_30");
}

TEST(VSSGenSolver, GurobiVSSGenDeltaT) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "SimpleStation", "atmos2023", "data");

  std::cout << "--------------------- TEST 1 ---------------------------"
            << '\n';
  const auto obj_val_2 = solver.solve({30, false});
  std::cout << "--------------------- TEST 2 ---------------------------"
            << '\n';
  const auto obj_val_1 = solver.solve({30, true});
  std::cout << "--------------------- TEST 3 ---------------------------"
            << '\n';
  const auto obj_val_3 = solver.solve(
      {30, true, false, false},
      {cda_rail::vss::Model(cda_rail::vss::ModelType::Discrete,
                            {&cda_rail::vss::functions::uniform})});

  EXPECT_EQ(obj_val_1.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_2.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_3.get_status(), cda_rail::SolutionStatus::Optimal);

  EXPECT_EQ(obj_val_1.get_obj(), 1);
  EXPECT_EQ(obj_val_2.get_obj(), 1);
  EXPECT_EQ(obj_val_3.get_obj(), 1);

  check_exit_times_within_dt_and_order(obj_val_1, 30, "obj_val_1");
  check_exit_times_within_dt_and_order(obj_val_2, 30, "obj_val_2");
  check_exit_times_within_dt_and_order(obj_val_3, 30, "obj_val_3");
}

TEST(VSSGenSolver, GurobiVSSGenDefault) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "SimpleStation", "atmos2023", "data");

  // Test various options
  std::cout << "--------------------- DEFAULT ---------------------------"
            << '\n';
  const auto obj_val_default = solver.solve();
  EXPECT_EQ(obj_val_default.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_default.get_obj(), 1);

  check_exit_times_within_dt_and_order(obj_val_default, 15, "obj_val_default");
}

TEST(VSSGenSolver, GurobiVSSGenDefaultInstance) {
  const cda_rail::instances::GeneralPerformanceOptimizationInstance instance(
      "SimpleStation", "atmos2023", "data");
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(instance);

  // Test various options
  std::cout << "--------------------- DEFAULT ---------------------------"
            << '\n';
  const auto obj_val_default = solver.solve();
  EXPECT_EQ(obj_val_default.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_default.get_obj(), 1);

  check_exit_times_within_dt_and_order(obj_val_default, 15, "obj_val_default");
}

TEST(VSSGenSolver, GurobiVSSGenDefaultInstanceForward) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "SimpleStation", "atmos2023", "data");

  // Test various options
  std::cout << "--------------------- DEFAULT ---------------------------"
            << '\n';
  const auto obj_val_default = solver.solve();
  EXPECT_EQ(obj_val_default.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_default.get_obj(), 1);

  check_exit_times_within_dt_and_order(obj_val_default, 15, "obj_val_default");
}

TEST(VSSGenSolver, GurobiVSSGenModelDetailFixed) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "SimpleStation", "atmos2023", "data");

  std::cout << "--------------------- TEST 1 ---------------------------"
            << '\n';
  const auto obj_val_1 = solver.solve({}, {}, {}, {}, 60, true);

  std::cout << "--------------------- TEST 2 ---------------------------"
            << '\n';
  const auto obj_val_2 = solver.solve({}, {}, {}, {}, 60, true);

  std::cout << "--------------------- TEST 3 ---------------------------"
            << '\n';
  const auto obj_val_3 =
      solver.solve({15, true, true, false},
                   {cda_rail::vss::Model(cda_rail::vss::ModelType::Continuous),
                    false, false},
                   {}, {}, 60, true);

  std::cout << "--------------------- TEST 4 ---------------------------"
            << '\n';
  const auto obj_val_4 = solver.solve(
      {},
      {cda_rail::vss::Model(cda_rail::vss::ModelType::Continuous), true, true},
      {}, {}, 60, true);

  std::cout << "--------------------- TEST 5 ---------------------------"
            << '\n';
  const auto obj_val_5 =
      solver.solve({15, true, true, false}, {}, {}, {}, 60, true);

  std::cout << "--------------------- TEST 6 ---------------------------"
            << '\n';
  const auto obj_val_6 =
      solver.solve({15, true, false, false}, {}, {}, {}, 60, true);

  // Check if all objective values are 1
  EXPECT_EQ(obj_val_1.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_2.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_3.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_4.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_5.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_6.get_status(), cda_rail::SolutionStatus::Optimal);

  EXPECT_EQ(obj_val_1.get_obj(), 1);
  EXPECT_EQ(obj_val_2.get_obj(), 1);
  EXPECT_EQ(obj_val_3.get_obj(), 1);
  EXPECT_EQ(obj_val_4.get_obj(), 1);
  EXPECT_EQ(obj_val_5.get_obj(), 1);
  EXPECT_EQ(obj_val_6.get_obj(), 1);

  check_exit_times_within_dt_and_order(obj_val_1, 15, "obj_val_1");
  check_exit_times_within_dt_and_order(obj_val_2, 15, "obj_val_2");
  check_exit_times_within_dt_and_order(obj_val_3, 15, "obj_val_3");
  check_exit_times_within_dt_and_order(obj_val_4, 15, "obj_val_4");
  check_exit_times_within_dt_and_order(obj_val_5, 15, "obj_val_5");
  check_exit_times_within_dt_and_order(obj_val_6, 15, "obj_val_6");
}

TEST(VSSGenSolver, GurobiVSSGenModelDetailFree1) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "SimpleStation", "atmos2023", "data");

  std::cout << "--------------------- TEST 1 ---------------------------"
            << '\n';
  const auto obj_val_1 = solver.solve({15, false}, {}, {}, {}, 280, true);

  EXPECT_EQ(obj_val_1.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_1.get_obj(), 1);

  check_exit_times_within_dt_and_order(obj_val_1, 15, "obj_val_1");
}

TEST(VSSGenSolver, GurobiVSSGenModelDetailFree2) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "SimpleStation", "atmos2023", "data");

  std::cout << "--------------------- TEST 2 ---------------------------"
            << '\n';
  const auto obj_val_2 = solver.solve({15, false}, {}, {}, {}, 280, true);

  EXPECT_EQ(obj_val_2.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_2.get_obj(), 1);

  check_exit_times_within_dt_and_order(obj_val_2, 15, "obj_val_2");
}

TEST(VSSGenSolver, GurobiVSSGenModelDetailFree3) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "SimpleStation", "atmos2023", "data");

  std::cout << "--------------------- TEST 3 ---------------------------"
            << '\n';
  const auto obj_val_3 = solver.solve(
      {15, false}, {cda_rail::vss::Model(), true, true}, {}, {}, 280, true);

  EXPECT_EQ(obj_val_3.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_3.get_obj(), 1);

  check_exit_times_within_dt_and_order(obj_val_3, 15, "obj_val_3");
}

TEST(VSSGenSolver, GurobiVSSGenModelDetailFree4) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "SimpleStation", "atmos2023", "data");

  std::cout << "--------------------- TEST 4 ---------------------------"
            << '\n';
  const auto obj_val_4 =
      solver.solve({15, false, true, false}, {}, {}, {}, 280, true);

  EXPECT_EQ(obj_val_4.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_4.get_obj(), 1);

  check_exit_times_within_dt_and_order(obj_val_4, 15, "obj_val_4");
}

TEST(VSSGenSolver, GurobiVSSGenModelDetailFree5) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "SimpleStation", "atmos2023", "data");

  std::cout << "--------------------- TEST 5 ---------------------------"
            << '\n';
  const auto obj_val_5 =
      solver.solve({15, false, false, false}, {}, {}, {}, 280, true);

  EXPECT_EQ(obj_val_5.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_5.get_obj(), 1);

  check_exit_times_within_dt_and_order(obj_val_5, 15, "obj_val_5");
}

TEST(VSSGenSolver, GurobiVSSGenVSSDiscrete) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "SimpleStation", "atmos2023", "data");

  const auto obj_val =
      solver.solve({15, true, false, false},
                   {cda_rail::vss::Model(cda_rail::vss::ModelType::Discrete,
                                         {&cda_rail::vss::functions::uniform})},
                   {}, {}, 600, true);

  // Check if all objective values are 1
  EXPECT_EQ(obj_val.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val.get_obj(), 1);

  check_exit_times_within_dt_and_order(obj_val, 15, "obj_val");
}

TEST(VSSGenSolver, GurobiVSSGenTim) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "SimpleStation", "atmos2023", "data");

  std::cout << "--------------------- TEST 1 ---------------------------"
            << '\n';

  EXPECT_TRUE(
      solver.get_instance().get_const_train_list().get_train("tr1").has_tim());
  EXPECT_TRUE(
      solver.get_instance().get_const_train_list().get_train("tr2").has_tim());
  EXPECT_TRUE(
      solver.get_instance().get_const_train_list().get_train("tr3").has_tim());

  const auto obj_val_1 = solver.solve({15, false});
  std::cout << "--------------------- TEST 2 ---------------------------"
            << '\n';

  solver.editable_instance().editable_train("tr1").set_no_tim();

  EXPECT_FALSE(
      solver.get_instance().get_const_train_list().get_train("tr1").has_tim());
  EXPECT_TRUE(
      solver.get_instance().get_const_train_list().get_train("tr2").has_tim());
  EXPECT_TRUE(
      solver.get_instance().get_const_train_list().get_train("tr3").has_tim());

  const auto obj_val_2 = solver.solve({15, false});
  std::cout << "--------------------- TEST 3 ---------------------------"
            << '\n';

  solver.editable_instance().editable_train("tr1").set_tim();
  solver.editable_instance().editable_train("tr2").set_no_tim();

  EXPECT_TRUE(
      solver.get_instance().get_const_train_list().get_train("tr1").has_tim());
  EXPECT_FALSE(
      solver.get_instance().get_const_train_list().get_train("tr2").has_tim());
  EXPECT_TRUE(
      solver.get_instance().get_const_train_list().get_train("tr3").has_tim());

  const auto obj_val_3 = solver.solve({15, false});

  EXPECT_EQ(obj_val_1.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_2.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_3.get_status(), cda_rail::SolutionStatus::Infeasible);

  EXPECT_EQ(obj_val_1.get_obj(), 1);
  EXPECT_EQ(obj_val_2.get_obj(), 1);

  check_exit_times_within_dt_and_order(obj_val_1, 15, "obj_val_1");
  check_exit_times_within_dt_and_order(obj_val_2, 15, "obj_val_2");
}

TEST(VSSGenSolver, GurobiVSSGenTimFixed) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "SimpleStation", "atmos2023", "data");

  std::cout << "--------------------- TEST 1 ---------------------------"
            << '\n';

  EXPECT_TRUE(
      solver.get_instance().get_const_train_list().get_train("tr1").has_tim());
  EXPECT_TRUE(
      solver.get_instance().get_const_train_list().get_train("tr2").has_tim());
  EXPECT_TRUE(
      solver.get_instance().get_const_train_list().get_train("tr3").has_tim());

  const auto obj_val_1 = solver.solve();
  std::cout << "--------------------- TEST 2 ---------------------------"
            << '\n';

  solver.editable_instance().editable_train("tr1").set_no_tim();

  EXPECT_FALSE(
      solver.get_instance().get_const_train_list().get_train("tr1").has_tim());
  EXPECT_TRUE(
      solver.get_instance().get_const_train_list().get_train("tr2").has_tim());
  EXPECT_TRUE(
      solver.get_instance().get_const_train_list().get_train("tr3").has_tim());

  const auto obj_val_2 = solver.solve();
  std::cout << "--------------------- TEST 3 ---------------------------"
            << '\n';

  solver.editable_instance().editable_train("tr1").set_tim();
  solver.editable_instance().editable_train("tr2").set_no_tim();

  EXPECT_TRUE(
      solver.get_instance().get_const_train_list().get_train("tr1").has_tim());
  EXPECT_FALSE(
      solver.get_instance().get_const_train_list().get_train("tr2").has_tim());
  EXPECT_TRUE(
      solver.get_instance().get_const_train_list().get_train("tr3").has_tim());

  const auto obj_val_3 = solver.solve();

  EXPECT_EQ(obj_val_1.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_2.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_3.get_status(), cda_rail::SolutionStatus::Infeasible);

  EXPECT_EQ(obj_val_1.get_obj(), 1);
  EXPECT_EQ(obj_val_2.get_obj(), 1);

  check_exit_times_within_dt_and_order(obj_val_1, 15, "obj_val_1");
  check_exit_times_within_dt_and_order(obj_val_2, 15, "obj_val_2");
}

TEST(VSSGenSolver, GurobiVSSGenTimDiscrete1) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "SimpleStation", "atmos2023", "data");

  std::cout << "--------------------- TEST 1 ---------------------------"
            << '\n';

  EXPECT_TRUE(
      solver.get_instance().get_const_train_list().get_train("tr1").has_tim());
  EXPECT_TRUE(
      solver.get_instance().get_const_train_list().get_train("tr2").has_tim());
  EXPECT_TRUE(
      solver.get_instance().get_const_train_list().get_train("tr3").has_tim());

  const auto obj_val_1 =
      solver.solve({15, true, false, false},
                   {cda_rail::vss::Model(cda_rail::vss::ModelType::Discrete,
                                         {&cda_rail::vss::functions::uniform})},
                   {}, {}, 600, true);

  EXPECT_EQ(obj_val_1.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_1.get_obj(), 1);

  check_exit_times_within_dt_and_order(obj_val_1, 15, "obj_val_1");
}

TEST(VSSGenSolver, GurobiVSSGenTimDiscrete2) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "SimpleStation", "atmos2023", "data");

  std::cout << "--------------------- TEST 2 ---------------------------"
            << '\n';

  solver.editable_instance().editable_train("tr1").set_no_tim();

  EXPECT_FALSE(
      solver.get_instance().get_const_train_list().get_train("tr1").has_tim());
  EXPECT_TRUE(
      solver.get_instance().get_const_train_list().get_train("tr2").has_tim());
  EXPECT_TRUE(
      solver.get_instance().get_const_train_list().get_train("tr3").has_tim());

  const auto obj_val_2 =
      solver.solve({15, true, false, false},
                   {cda_rail::vss::Model(cda_rail::vss::ModelType::Discrete,
                                         {&cda_rail::vss::functions::uniform})},
                   {}, {}, 600, true);

  EXPECT_EQ(obj_val_2.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_2.get_obj(), 1);

  check_exit_times_within_dt_and_order(obj_val_2, 15, "obj_val_2");
}

TEST(VSSGenSolver, GurobiVSSGenTimDiscrete3) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "SimpleStation", "atmos2023", "data");

  std::cout << "--------------------- TEST 3 ---------------------------"
            << '\n';

  solver.editable_instance().editable_train("tr2").set_no_tim();

  EXPECT_TRUE(
      solver.get_instance().get_const_train_list().get_train("tr1").has_tim());
  EXPECT_FALSE(
      solver.get_instance().get_const_train_list().get_train("tr2").has_tim());
  EXPECT_TRUE(
      solver.get_instance().get_const_train_list().get_train("tr3").has_tim());

  const auto obj_val_3 =
      solver.solve({15, true, false, false},
                   {cda_rail::vss::Model(cda_rail::vss::ModelType::Discrete,
                                         {&cda_rail::vss::functions::uniform})},
                   {}, {}, 600, true);

  EXPECT_EQ(obj_val_3.get_status(), cda_rail::SolutionStatus::Infeasible);
}

TEST(VSSGenSolver, OvertakeFixedContinuous) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "Overtake", "atmos2023", "data");

  const auto obj_val_base =
      solver.solve({15, true, false, false}, {}, {}, {}, 120);
  const auto obj_val_dynamics =
      solver.solve({15, true, true, false}, {}, {}, {}, 120);
  const auto obj_val_braking =
      solver.solve({15, true, true, true}, {}, {}, {}, 120);

  EXPECT_EQ(obj_val_base.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_dynamics.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_braking.get_status(), cda_rail::SolutionStatus::Optimal);

  EXPECT_EQ(obj_val_base.get_obj(), 4);
  EXPECT_EQ(obj_val_dynamics.get_obj(), 4);
  EXPECT_EQ(obj_val_braking.get_obj(), 7);

  check_exit_times_within_dt_and_order(obj_val_base, 15, "obj_val_base");
  check_exit_times_within_dt_and_order(obj_val_dynamics, 15,
                                       "obj_val_dynamics");
  check_exit_times_within_dt_and_order(obj_val_braking, 15, "obj_val_braking");
}

TEST(VSSGenSolver, OvertakeFreeContinuous) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "Overtake", "atmos2023", "data");

  const auto obj_val_base =
      solver.solve({15, false, false, false}, {}, {}, {}, 100);
  const auto obj_val_dynamics =
      solver.solve({15, false, true, false}, {}, {}, {}, 200);
  const auto obj_val_braking =
      solver.solve({15, false, true, true}, {}, {}, {}, 400);

  EXPECT_EQ(obj_val_base.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_dynamics.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_braking.get_status(), cda_rail::SolutionStatus::Optimal);

  EXPECT_EQ(obj_val_base.get_obj(), 4);
  EXPECT_EQ(obj_val_dynamics.get_obj(), 4);
  EXPECT_EQ(obj_val_braking.get_obj(), 7);

  check_exit_times_within_dt_and_order(obj_val_base, 15, "obj_val_base");
  check_exit_times_within_dt_and_order(obj_val_dynamics, 15,
                                       "obj_val_dynamics");
  check_exit_times_within_dt_and_order(obj_val_braking, 15, "obj_val_braking");
}

TEST(VSSGenSolver, Stammstrecke4FixedContinuous) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "Stammstrecke4Trains", "atmos2023", "data");

  const auto obj_val_base =
      solver.solve({15, true, false, false}, {}, {}, {}, 120);
  const auto obj_val_dynamics =
      solver.solve({15, true, true, false}, {}, {}, {}, 120);
  const auto obj_val_braking =
      solver.solve({15, true, true, true}, {}, {}, {}, 120);

  EXPECT_EQ(obj_val_base.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_dynamics.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_braking.get_status(), cda_rail::SolutionStatus::Optimal);

  EXPECT_EQ(obj_val_base.get_obj(), 0);
  EXPECT_EQ(obj_val_dynamics.get_obj(), 6);
  EXPECT_EQ(obj_val_braking.get_obj(), 6);

  check_exit_times_within_dt_and_order(obj_val_base, 15, "obj_val_base");
  check_exit_times_within_dt_and_order(obj_val_dynamics, 15,
                                       "obj_val_dynamics");
  check_exit_times_within_dt_and_order(obj_val_braking, 15, "obj_val_braking");
}

TEST(VSSGenSolver, Stammstrecke8FixedContinuous) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "Stammstrecke8Trains", "atmos2023", "data");

  const auto obj_val_base =
      solver.solve({15, true, false, false}, {}, {}, {}, 120);
  const auto obj_val_dynamics =
      solver.solve({15, true, true, false}, {}, {}, {}, 120);
  const auto obj_val_braking =
      solver.solve({15, true, true, true}, {}, {}, {}, 120);

  EXPECT_EQ(obj_val_base.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_dynamics.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_braking.get_status(), cda_rail::SolutionStatus::Optimal);

  EXPECT_EQ(obj_val_base.get_obj(), 0);
  EXPECT_EQ(obj_val_dynamics.get_obj(), 14);
  EXPECT_EQ(obj_val_braking.get_obj(), 14);

  check_exit_times_within_dt_and_order(obj_val_base, 15, "obj_val_base");
  check_exit_times_within_dt_and_order(obj_val_dynamics, 15,
                                       "obj_val_dynamics");
  check_exit_times_within_dt_and_order(obj_val_braking, 15, "obj_val_braking");
}

TEST(VSSGenSolver, Stammstrecke16FixedContinuousBase) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "Stammstrecke16Trains", "atmos2023", "data");

  const auto obj_val_base =
      solver.solve({15, true, false, false}, {}, {}, {}, 600);

  EXPECT_EQ(obj_val_base.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_base.get_obj(), 0);

  check_exit_times_within_dt_and_order(obj_val_base, 15, "obj_val_base");
}

TEST(VSSGenSolver, Stammstrecke16FixedContinuousDynamics) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "Stammstrecke16Trains", "atmos2023", "data");

  const auto obj_val_dynamics =
      solver.solve({15, true, true, false}, {}, {}, {}, 600);

  EXPECT_EQ(obj_val_dynamics.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_dynamics.get_obj(), 15);

  check_exit_times_within_dt_and_order(obj_val_dynamics, 15,
                                       "obj_val_dynamics");
}

TEST(VSSGenSolver, Stammstrecke16FixedContinuousBraking) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "Stammstrecke16Trains", "atmos2023", "data");

  const auto obj_val_braking =
      solver.solve({15, true, true, true}, {}, {}, {}, 600);

  EXPECT_EQ(obj_val_braking.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_braking.get_obj(), 15);

  check_exit_times_within_dt_and_order(obj_val_braking, 15, "obj_val_braking");
}

TEST(VSSGenSolver, SimpleStationInferredUniform) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "SimpleStation", "atmos2023", "data");

  const auto obj_val =
      solver.solve({},
                   {cda_rail::vss::Model(cda_rail::vss::ModelType::Inferred,
                                         {&cda_rail::vss::functions::uniform})},
                   {}, {}, 60, true);

  // Check if all objective values are 1
  EXPECT_EQ(obj_val.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val.get_obj(), 1);

  check_exit_times_within_dt_and_order(obj_val, 15, "obj_val");
}

TEST(VSSGenSolver, SimpleStationInferredUniformPostprocess) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "SimpleStation", "atmos2023", "data");

  const auto obj_val =
      solver.solve({},
                   {cda_rail::vss::Model(cda_rail::vss::ModelType::Inferred,
                                         {&cda_rail::vss::functions::uniform})},
                   {}, {true}, 60, true);

  // Check if all objective values are 1
  EXPECT_EQ(obj_val.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val.get_obj(), 1);

  check_exit_times_within_dt_and_order(obj_val, 15, "obj_val");
}

TEST(VSSGenSolver, SimpleStationInferredAltUniformPostprocess) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "SimpleStation", "atmos2023", "data");

  const auto obj_val =
      solver.solve({},
                   {cda_rail::vss::Model(cda_rail::vss::ModelType::InferredAlt,
                                         {&cda_rail::vss::functions::uniform})},
                   {}, {true}, 60, true);

  // Check if all objective values are 1
  EXPECT_EQ(obj_val.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val.get_obj(), 1);

  check_exit_times_within_dt_and_order(obj_val, 15, "obj_val");
}

TEST(VSSGenSolver, SimpleStationContinuousPostprocess) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "SimpleStation", "atmos2023", "data");

  const auto obj_val = solver.solve({15, false}, {}, {}, {true}, 240, true);

  // Check if all objective values are 1
  EXPECT_EQ(obj_val.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val.get_obj(), 1);

  check_exit_times_within_dt_and_order(obj_val, 15, "obj_val");
}

TEST(VSSGenSolver, SimpleStationContinuousFixedPostprocess) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "SimpleStation", "atmos2023", "data");

  const auto obj_val = solver.solve({15, true}, {}, {}, {true}, 60, true);

  // Check if all objective values are 1
  EXPECT_EQ(obj_val.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val.get_obj(), 1);

  check_exit_times_within_dt_and_order(obj_val, 15, "obj_val");
}

TEST(VSSGenSolver, SimpleStationInferredChebychev) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "SimpleStation", "atmos2023", "data");

  const auto obj_val = solver.solve(
      {},
      {cda_rail::vss::Model(cda_rail::vss::ModelType::Inferred,
                            {&cda_rail::vss::functions::chebyshev})},
      {}, {}, 60, true);

  // Check if all objective values are 1
  EXPECT_EQ(obj_val.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val.get_obj(), 1);

  check_exit_times_within_dt_and_order(obj_val, 15, "obj_val");
}

TEST(VSSGenSolver, SimpleStationInferredBoth) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "SimpleStation", "atmos2023", "data");

  const auto obj_val = solver.solve(
      {},
      {cda_rail::vss::Model(cda_rail::vss::ModelType::Inferred,
                            {&cda_rail::vss::functions::uniform,
                             &cda_rail::vss::functions::chebyshev})},
      {}, {}, 60, true);

  // Check if all objective values are 1
  EXPECT_EQ(obj_val.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val.get_obj(), 1);

  check_exit_times_within_dt_and_order(obj_val, 15, "obj_val");
}

TEST(VSSGenSolver, SimpleStationInferredAltBoth) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "SimpleStation", "atmos2023", "data");

  const auto obj_val = solver.solve(
      {},
      {cda_rail::vss::Model(cda_rail::vss::ModelType::InferredAlt,
                            {&cda_rail::vss::functions::uniform,
                             &cda_rail::vss::functions::chebyshev})},
      {}, {}, 60, true);

  // Check if all objective values are 1
  EXPECT_EQ(obj_val.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val.get_obj(), 1);

  check_exit_times_within_dt_and_order(obj_val, 15, "obj_val");
}

TEST(VSSGenSolver, IterativeContinuousSingleTrack) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "SingleTrack", "atmos2023", "data");

  const auto obj_val = solver.solve(
      {}, {}, {true, cda_rail::OptimalityStrategy::Optimal}, {}, 60, true);

  EXPECT_EQ(obj_val.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val.get_obj(), 9);

  check_exit_times_within_dt_and_order(obj_val, 15, "obj_val");
}

TEST(VSSGenSolver, IterativeContinuousSingleTrackCuts) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "SingleTrack", "atmos2023", "data");

  const auto obj_val = solver.solve(
      {}, {},
      {true, cda_rail::OptimalityStrategy::Optimal,
       cda_rail::solver::mip_based::UpdateStrategy::Fixed, 1, 2, true},
      {}, 60, true);

  EXPECT_EQ(obj_val.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val.get_obj(), 9);

  check_exit_times_within_dt_and_order(obj_val, 15, "obj_val");
}

TEST(VSSGenSolver, IterativeContinuousSingleRelative) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "SingleTrack", "atmos2023", "data");

  const auto obj_val =
      solver.solve({}, {},
                   {true, cda_rail::OptimalityStrategy::Optimal,
                    cda_rail::solver::mip_based::UpdateStrategy::Relative,
                    0.025, 0.05, true},
                   {}, 60, true);

  EXPECT_EQ(obj_val.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val.get_obj(), 9);

  check_exit_times_within_dt_and_order(obj_val, 15, "obj_val");
}

TEST(VSSGenSolver, IterativeContinuousSimpleStationInferredCuts) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "SimpleStation", "atmos2023", "data");

  const auto obj_val = solver.solve(
      {15, true, true, false},
      {cda_rail::vss::Model(cda_rail::vss::ModelType::Inferred,
                            {&cda_rail::vss::functions::uniform})},
      {true, cda_rail::OptimalityStrategy::Optimal,
       cda_rail::solver::mip_based::UpdateStrategy::Fixed, 0, 2, true},
      {}, 60, true);

  check_exit_times_within_dt_and_order(obj_val, 15, "obj_val");
}

TEST(VSSGenSolver, IterativeContinuousSimpleStationCuts) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "SimpleStation", "atmos2023", "data");

  const auto obj_val = solver.solve(
      {15, true, true, false}, {},
      {true, cda_rail::OptimalityStrategy::Optimal,
       cda_rail::solver::mip_based::UpdateStrategy::Fixed, 0, 2, true},
      {}, 60, true);

  check_exit_times_within_dt_and_order(obj_val, 15, "obj_val");
}

TEST(VSSGenSolver, IterativeContinuousFeasible) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "Stammstrecke4Trains", "atmos2023", "data");

  const auto obj_val = solver.solve(
      {}, {}, {true, cda_rail::OptimalityStrategy::Feasible}, {}, 60, true);

  EXPECT_EQ(obj_val.get_status(), cda_rail::SolutionStatus::Feasible);
  EXPECT_GE(obj_val.get_obj(), 6);

  check_exit_times_within_dt_and_order(obj_val, 15, "obj_val");
}

TEST(VSSGenSolver, IterativeTimeout1) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "SimpleNetwork", "atmos2023", "data");

  const auto obj_val = solver.solve({15, false}, {}, {true}, {}, 30, true);

  EXPECT_EQ(obj_val.get_status(), cda_rail::SolutionStatus::Timeout);
}

TEST(VSSGenSolver, IterativeTimeout2) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "SimpleNetwork", "atmos2023", "data");

  const auto obj_val = solver.solve({}, {}, {true}, {}, 1, true);

  EXPECT_EQ(obj_val.get_status(), cda_rail::SolutionStatus::Timeout);
}

TEST(VSSGenSolver, IterativeContinuousSimpleStationInferredAlt) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "SimpleStation", "atmos2023", "data");

  const auto obj_val = solver.solve(
      {15, true, true, false},
      {cda_rail::vss::Model(cda_rail::vss::ModelType::InferredAlt,
                            {&cda_rail::vss::functions::uniform})},
      {true, cda_rail::OptimalityStrategy::Optimal,
       cda_rail::solver::mip_based::UpdateStrategy::Fixed, 0, 2, true},
      {}, 60, true);

  EXPECT_EQ(obj_val.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val.get_obj(), 1);

  check_exit_times_within_dt_and_order(obj_val, 15, "obj_val");
}

TEST(VSSGenSolver, IterativeContinuousStammstrecke4) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "Stammstrecke4Trains", "atmos2023", "data");

  const auto obj_val = solver.solve({}, {}, {true}, {}, 60, true);

  EXPECT_EQ(obj_val.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val.get_obj(), 6);

  check_exit_times_within_dt_and_order(obj_val, 15, "obj_val");
}

TEST(VSSGenSolver, IterativeContinuousStammstrecke4Cuts) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "Stammstrecke4Trains", "atmos2023", "data");

  const auto obj_val = solver.solve(
      {}, {},
      {true, cda_rail::OptimalityStrategy::Optimal,
       cda_rail::solver::mip_based::UpdateStrategy::Fixed, 0, 2, true},
      {}, 60, true);

  EXPECT_EQ(obj_val.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val.get_obj(), 6);

  check_exit_times_within_dt_and_order(obj_val, 15, "obj_val");
}

TEST(VSSGenSolver, IterativeContinuousOvertakeRelative) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "Overtake", "atmos2023", "data");

  const auto obj_val = solver.solve(
      {}, {},
      {true, cda_rail::OptimalityStrategy::Optimal,
       cda_rail::solver::mip_based::UpdateStrategy::Relative, 0.05, 0.05, true},
      {}, 60, true);

  EXPECT_EQ(obj_val.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val.get_obj(), 7);

  check_exit_times_within_dt_and_order(obj_val, 15, "obj_val");
}

TEST(VSSGenSolver, OnlyStopAtBoundariesContinuousFixed1) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "SimpleStation", "atmos2023", "data");

  const auto obj_val = solver.solve(
      {15, true, false, false},
      {cda_rail::vss::Model(cda_rail::vss::ModelType::Continuous, {}, true)},
      {}, {}, 240, true);

  // Check if all objective values are 1
  EXPECT_EQ(obj_val.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val.get_obj(), 1);

  check_exit_times_within_dt_and_order(obj_val, 15, "obj_val");

  for (size_t tr = 0;
       tr < obj_val.get_instance()->get_const_train_list().size(); ++tr) {
    const auto& train_name =
        obj_val.get_instance()->get_const_train_list().get_train(tr).get_name();
    const auto& allowed_stops = obj_val.get_valid_border_stops(train_name);

    // put values of allowed_stops into string separated by comma
    std::string allowed_stops_str;
    for (const auto& stop : allowed_stops) {
      allowed_stops_str += std::to_string(stop) + ", ";
    }
    // remove last comma
    allowed_stops_str =
        allowed_stops_str.substr(0, allowed_stops_str.size() - 2);

    constexpr double dt = 15.0;
    const auto& [t0, tn] =
        obj_val.get_instance()->time_index_interval(tr, dt, false);
    for (int t = static_cast<int>(t0) + 1; t <= static_cast<int>(tn); ++t) {
      const auto& train_speed = obj_val.get_train_speed(train_name, t * dt);
      if (train_speed > cda_rail::GRB_EPS) {
        continue;
      }
      const auto& tr_pos = obj_val.get_train_pos(train_name, t * dt);
      // Expect any of allowed_stops to be within EPS of tr_pos
      bool found = false;
      for (const auto& stop : allowed_stops) {
        if (std::abs(stop - tr_pos) <
            cda_rail::GRB_EPS + cda_rail::STOP_TOLERANCE) {
          found = true;
          break;
        }
      }
      EXPECT_TRUE(found) << "Error on train " << train_name << " (id=" << tr
                         << ") at time " << t * dt << " with speed "
                         << train_speed << " and position " << tr_pos
                         << ". Allowed stops: " << allowed_stops_str;
    }
  }
}

TEST(VSSGenSolver, OnlyStopAtBoundariesContinuousFixed2) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "SimpleStation", "atmos2023", "data");

  const auto obj_val = solver.solve(
      {15, true, true, false},
      {cda_rail::vss::Model(cda_rail::vss::ModelType::Continuous, {}, true)},
      {}, {}, 240, true);

  // Check if all objective values are 1
  EXPECT_EQ(obj_val.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val.get_obj(), 1);

  check_exit_times_within_dt_and_order(obj_val, 15, "obj_val");

  for (size_t tr = 0;
       tr < obj_val.get_instance()->get_const_train_list().size(); ++tr) {
    const auto& train_name =
        obj_val.get_instance()->get_const_train_list().get_train(tr).get_name();
    const auto& allowed_stops = obj_val.get_valid_border_stops(train_name);

    // put values of allowed_stops into string separated by comma
    std::string allowed_stops_str;
    for (const auto& stop : allowed_stops) {
      allowed_stops_str += std::to_string(stop) + ", ";
    }
    // remove last comma
    allowed_stops_str =
        allowed_stops_str.substr(0, allowed_stops_str.size() - 2);

    constexpr double dt = 15.0;
    const auto& [t0, tn] =
        obj_val.get_instance()->time_index_interval(tr, dt, false);
    for (int t = static_cast<int>(t0) + 1; t <= static_cast<int>(tn); ++t) {
      const auto& train_speed = obj_val.get_train_speed(train_name, t * dt);
      if (train_speed > cda_rail::GRB_EPS) {
        continue;
      }
      const auto& tr_pos = obj_val.get_train_pos(train_name, t * dt);
      // Expect any of allowed_stops to be within EPS of tr_pos
      bool found = false;
      for (const auto& stop : allowed_stops) {
        if (std::abs(stop - tr_pos) <
            cda_rail::GRB_EPS + cda_rail::STOP_TOLERANCE) {
          found = true;
          break;
        }
      }
      EXPECT_TRUE(found) << "Error on train " << train_name << " (id=" << tr
                         << ") at time " << t * dt << " with speed "
                         << train_speed << " and position " << tr_pos
                         << ". Allowed stops: " << allowed_stops_str;
    }
  }
}

TEST(VSSGenSolver, OnlyStopAtBoundariesContinuousFixed3) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "SimpleStation", "atmos2023", "data");

  const auto obj_val = solver.solve(
      {15, true, true, true},
      {cda_rail::vss::Model(cda_rail::vss::ModelType::Continuous, {}, true)},
      {}, {}, 240, true);

  // Check if all objective values are 1
  EXPECT_EQ(obj_val.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val.get_obj(), 1);

  check_exit_times_within_dt_and_order(obj_val, 15, "obj_val");

  for (size_t tr = 0;
       tr < obj_val.get_instance()->get_const_train_list().size(); ++tr) {
    const auto& train_name =
        obj_val.get_instance()->get_const_train_list().get_train(tr).get_name();
    const auto& allowed_stops = obj_val.get_valid_border_stops(train_name);

    // put values of allowed_stops into string separated by comma
    std::string allowed_stops_str;
    for (const auto& stop : allowed_stops) {
      allowed_stops_str += std::to_string(stop) + ", ";
    }
    // remove last comma
    allowed_stops_str =
        allowed_stops_str.substr(0, allowed_stops_str.size() - 2);

    constexpr double dt = 15.0;
    const auto& [t0, tn] =
        obj_val.get_instance()->time_index_interval(tr, dt, false);
    for (int t = static_cast<int>(t0) + 1; t <= static_cast<int>(tn); ++t) {
      const auto& train_speed = obj_val.get_train_speed(train_name, t * dt);
      if (train_speed > cda_rail::GRB_EPS) {
        continue;
      }
      const auto& tr_pos = obj_val.get_train_pos(train_name, t * dt);
      // Expect any of allowed_stops to be within EPS of tr_pos
      bool found = false;
      for (const auto& stop : allowed_stops) {
        if (std::abs(stop - tr_pos) <
            cda_rail::GRB_EPS + cda_rail::STOP_TOLERANCE) {
          found = true;
          break;
        }
      }
      EXPECT_TRUE(found) << "Error on train " << train_name << " (id=" << tr
                         << ") at time " << t * dt << " with speed "
                         << train_speed << " and position " << tr_pos
                         << ". Allowed stops: " << allowed_stops_str;
    }
  }
}

TEST(VSSGenSolver, OnlyStopAtBoundariesContinuousFree1) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "SimpleStation", "atmos2023", "data");

  const auto obj_val = solver.solve(
      {15, false, false, false},
      {cda_rail::vss::Model(cda_rail::vss::ModelType::Continuous, {}, true)},
      {}, {}, 240, true);

  // Check if all objective values are 1
  EXPECT_EQ(obj_val.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val.get_obj(), 1);

  check_exit_times_within_dt_and_order(obj_val, 15, "obj_val");

  for (size_t tr = 0;
       tr < obj_val.get_instance()->get_const_train_list().size(); ++tr) {
    const auto& train_name =
        obj_val.get_instance()->get_const_train_list().get_train(tr).get_name();
    const auto& allowed_stops = obj_val.get_valid_border_stops(train_name);

    // put values of allowed_stops into string separated by comma
    std::string allowed_stops_str;
    for (const auto& stop : allowed_stops) {
      allowed_stops_str += std::to_string(stop) + ", ";
    }
    // remove last comma
    allowed_stops_str =
        allowed_stops_str.substr(0, allowed_stops_str.size() - 2);

    constexpr double dt = 15.0;
    const auto& [t0, tn] =
        obj_val.get_instance()->time_index_interval(tr, dt, false);
    for (int t = static_cast<int>(t0) + 1; t <= static_cast<int>(tn); ++t) {
      const auto& train_speed = obj_val.get_train_speed(train_name, t * dt);
      if (train_speed > cda_rail::GRB_EPS) {
        continue;
      }
      const auto& tr_pos = obj_val.get_train_pos(train_name, t * dt);
      // Expect any of allowed_stops to be within EPS of tr_pos
      bool found = false;
      for (const auto& stop : allowed_stops) {
        if (std::abs(stop - tr_pos) <
            cda_rail::GRB_EPS + cda_rail::STOP_TOLERANCE) {
          found = true;
          break;
        }
      }
      EXPECT_TRUE(found) << "Error on train " << train_name << " (id=" << tr
                         << ") at time " << t * dt << " with speed "
                         << train_speed << " and position " << tr_pos
                         << ". Allowed stops: " << allowed_stops_str;
    }
  }
}

TEST(VSSGenSolver, OnlyStopAtBoundariesContinuousFree2) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "SimpleStation", "atmos2023", "data");

  const auto obj_val = solver.solve(
      {15, false, true, false},
      {cda_rail::vss::Model(cda_rail::vss::ModelType::Continuous, {}, true)},
      {}, {}, 240, true);

  // Check if all objective values are 1
  EXPECT_EQ(obj_val.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val.get_obj(), 1);

  check_exit_times_within_dt_and_order(obj_val, 15, "obj_val");

  for (size_t tr = 0;
       tr < obj_val.get_instance()->get_const_train_list().size(); ++tr) {
    const auto& train_name =
        obj_val.get_instance()->get_const_train_list().get_train(tr).get_name();
    const auto& allowed_stops = obj_val.get_valid_border_stops(train_name);

    // put values of allowed_stops into string separated by comma
    std::string allowed_stops_str;
    for (const auto& stop : allowed_stops) {
      allowed_stops_str += std::to_string(stop) + ", ";
    }
    // remove last comma
    allowed_stops_str =
        allowed_stops_str.substr(0, allowed_stops_str.size() - 2);

    constexpr double dt = 15.0;
    const auto& [t0, tn] =
        obj_val.get_instance()->time_index_interval(tr, dt, false);
    for (int t = static_cast<int>(t0) + 1; t <= static_cast<int>(tn); ++t) {
      const auto& train_speed = obj_val.get_train_speed(train_name, t * dt);
      if (train_speed > cda_rail::GRB_EPS) {
        continue;
      }
      const auto& tr_pos = obj_val.get_train_pos(train_name, t * dt);
      // Expect any of allowed_stops to be within EPS of tr_pos
      bool found = false;
      for (const auto& stop : allowed_stops) {
        if (std::abs(stop - tr_pos) <
            cda_rail::GRB_EPS + cda_rail::STOP_TOLERANCE) {
          found = true;
          break;
        }
      }
      EXPECT_TRUE(found) << "Error on train " << train_name << " (id=" << tr
                         << ") at time " << t * dt << " with speed "
                         << train_speed << " and position " << tr_pos
                         << ". Allowed stops: " << allowed_stops_str;
    }
  }
}

TEST(VSSGenSolver, OnlyStopAtBoundariesContinuousFree3) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "SimpleStation", "atmos2023", "data");

  const auto obj_val = solver.solve(
      {15, false, true, true},
      {cda_rail::vss::Model(cda_rail::vss::ModelType::Continuous, {}, true)},
      {}, {}, 240, true);

  // Check if all objective values are 1
  EXPECT_EQ(obj_val.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val.get_obj(), 1);

  check_exit_times_within_dt_and_order(obj_val, 15, "obj_val");

  for (size_t tr = 0;
       tr < obj_val.get_instance()->get_const_train_list().size(); ++tr) {
    const auto& train_name =
        obj_val.get_instance()->get_const_train_list().get_train(tr).get_name();
    const auto& allowed_stops = obj_val.get_valid_border_stops(train_name);

    // put values of allowed_stops into string separated by comma
    std::string allowed_stops_str;
    for (const auto& stop : allowed_stops) {
      allowed_stops_str += std::to_string(stop) + ", ";
    }
    // remove last comma
    allowed_stops_str =
        allowed_stops_str.substr(0, allowed_stops_str.size() - 2);

    constexpr double dt = 15.0;
    const auto& [t0, tn] =
        obj_val.get_instance()->time_index_interval(tr, dt, false);
    for (int t = static_cast<int>(t0) + 1; t <= static_cast<int>(tn); ++t) {
      const auto& train_speed = obj_val.get_train_speed(train_name, t * dt);
      if (train_speed > cda_rail::GRB_EPS) {
        continue;
      }
      const auto& tr_pos = obj_val.get_train_pos(train_name, t * dt);
      // Expect any of allowed_stops to be within EPS of tr_pos
      bool found = false;
      for (const auto& stop : allowed_stops) {
        if (std::abs(stop - tr_pos) <
            cda_rail::GRB_EPS + cda_rail::STOP_TOLERANCE) {
          found = true;
          break;
        }
      }
      EXPECT_TRUE(found) << "Error on train " << train_name << " (id=" << tr
                         << ") at time " << t * dt << " with speed "
                         << train_speed << " and position " << tr_pos
                         << ". Allowed stops: " << allowed_stops_str;
    }
  }
}

TEST(VSSGenSolver, InfeasibleFree) {
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance;
  instance.get_editable_network().add_vertex("v0", cda_rail::VertexType::TTD);
  instance.get_editable_network().add_vertex("v1", cda_rail::VertexType::TTD);
  instance.get_editable_network().add_edge({"v0"}, {"v1"}, 350, 30,
                                           true); // Train needs 450m to stop

  instance.add_train("Train1", 100, 30, 1, 1, true, 0, 30, {"v0"}, 60, 0,
                     {"v1"}, 1);
  instance.add_empty_route("Train1");
  instance.push_back_edge_to_route("Train1", {"v0", "v1"});

  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(instance);

  auto const sol_obj =
      solver.solve({5, false, true, false}, {}, {}, {}, 100, true);
  EXPECT_FALSE(sol_obj.has_solution());
  EXPECT_EQ(sol_obj.get_status(), cda_rail::SolutionStatus::Infeasible);
}
TEST(VSSGenSolver, BarelyInfeasibleFree) {
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance;
  instance.get_editable_network().add_vertex("v0", cda_rail::VertexType::TTD);
  instance.get_editable_network().add_vertex("v1", cda_rail::VertexType::TTD);
  instance.get_editable_network().add_edge({"v0"}, {"v1"}, 350, 30,
                                           true); // Train needs 450m to stop

  instance.add_train("Train1", 100, 30, 1, 1, true, 0, 30, {"v0"}, 35, 0,
                     {"v1"}, 1);
  instance.add_empty_route("Train1");
  instance.push_back_edge_to_route("Train1", {"v0", "v1"});

  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(instance);

  auto const sol_obj =
      solver.solve({5, false, true, false}, {}, {}, {}, 100, true);
  EXPECT_FALSE(sol_obj.has_solution());
  EXPECT_EQ(sol_obj.get_status(), cda_rail::SolutionStatus::Infeasible);
}
TEST(VSSGenSolver, FeasibleFree) {
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance;
  instance.get_editable_network().add_vertex("v0", cda_rail::VertexType::TTD);
  instance.get_editable_network().add_vertex("v1", cda_rail::VertexType::TTD);
  instance.get_editable_network().add_edge({"v0"}, {"v1"}, 350, 30,
                                           true); // Train needs 450m to stop

  instance.add_train("Train1", 100, 30, 1, 1, true, 0, 30, {"v0"}, 30, 0,
                     {"v1"}, 1);
  instance.add_empty_route("Train1");
  instance.push_back_edge_to_route("Train1", {"v0", "v1"});

  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(instance);

  auto const sol_obj =
      solver.solve({5, false, true, false}, {}, {}, {}, 100, true);
  EXPECT_TRUE(sol_obj.has_solution());
  EXPECT_EQ(sol_obj.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(sol_obj.get_obj(), 0);
  EXPECT_EQ(sol_obj.get_exit_time("Train1"), 30);
}

TEST(VSSGenSolver, InfeasibleFixed) {
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance;
  instance.get_editable_network().add_vertex("v0", cda_rail::VertexType::TTD);
  instance.get_editable_network().add_vertex("v1", cda_rail::VertexType::TTD);
  instance.get_editable_network().add_edge({"v0"}, {"v1"}, 350, 30,
                                           true); // Train needs 450m to stop

  instance.add_train("Train1", 100, 30, 1, 1, true, 0, 30, {"v0"}, 60, 0,
                     {"v1"}, 1);
  instance.add_empty_route("Train1");
  instance.push_back_edge_to_route("Train1", {"v0", "v1"});

  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(instance);

  auto const sol_obj =
      solver.solve({5, true, true, false}, {}, {}, {}, 100, true);
  EXPECT_FALSE(sol_obj.has_solution());
  EXPECT_EQ(sol_obj.get_status(), cda_rail::SolutionStatus::Infeasible);
}
TEST(VSSGenSolver, BarelyInfeasibleFixed) {
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance;
  instance.get_editable_network().add_vertex("v0", cda_rail::VertexType::TTD);
  instance.get_editable_network().add_vertex("v1", cda_rail::VertexType::TTD);
  instance.get_editable_network().add_edge({"v0"}, {"v1"}, 350, 30,
                                           true); // Train needs 450m to stop

  instance.add_train("Train1", 100, 30, 1, 1, true, 0, 30, {"v0"}, 35, 0,
                     {"v1"}, 1);
  instance.add_empty_route("Train1");
  instance.push_back_edge_to_route("Train1", {"v0", "v1"});

  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(instance);

  auto const sol_obj =
      solver.solve({5, true, true, false}, {}, {}, {}, 100, true);
  EXPECT_FALSE(sol_obj.has_solution());
  EXPECT_EQ(sol_obj.get_status(), cda_rail::SolutionStatus::Infeasible);
}
TEST(VSSGenSolver, FeasibleFixed) {
  cda_rail::instances::GeneralPerformanceOptimizationInstance instance;
  instance.get_editable_network().add_vertex("v0", cda_rail::VertexType::TTD);
  instance.get_editable_network().add_vertex("v1", cda_rail::VertexType::TTD);
  instance.get_editable_network().add_edge({"v0"}, {"v1"}, 350, 30,
                                           true); // Train needs 450m to stop

  instance.add_train("Train1", 100, 30, 1, 1, true, 0, 30, {"v0"}, 30, 0,
                     {"v1"}, 1);
  instance.add_empty_route("Train1");
  instance.push_back_edge_to_route("Train1", {"v0", "v1"});

  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(instance);

  auto const sol_obj =
      solver.solve({5, true, true, false}, {}, {}, {}, 100, true);
  EXPECT_TRUE(sol_obj.has_solution());
  EXPECT_EQ(sol_obj.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(sol_obj.get_obj(), 0);
  EXPECT_EQ(sol_obj.get_exit_time("Train1"), 30);
}

TEST(VSSGenSolver, SimpleStationExportOptions) {
  // The instance is read relative to the current working directory, hence the
  // solver has to be created before switching to the temporary directory.
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "SimpleStation", "atmos2023", "data");

  // All exports happen relative to the current working directory. Hence, the
  // test is executed within a temporary directory that is removed afterwards,
  // even if an expectation fails in between.
  struct ScopedTempWorkingDirectory {
    std::filesystem::path original_directory;
    std::filesystem::path temporary_directory;
    ~ScopedTempWorkingDirectory() {
      std::error_code ignored;
      std::filesystem::current_path(original_directory, ignored);
      std::filesystem::remove_all(temporary_directory, ignored);
    }
  };

  const std::filesystem::path temp_dir =
      std::filesystem::temp_directory_path() /
      "cda_rail_test_vss_gen_simple_station_export";
  std::filesystem::remove_all(temp_dir);
  ASSERT_TRUE(std::filesystem::create_directories(temp_dir));
  const ScopedTempWorkingDirectory temp_working_directory{
      std::filesystem::current_path(), temp_dir};
  std::filesystem::current_path(temp_dir);

  std::error_code ec;

  // Names of the exported instance as given to the solver constructor above.
  const std::filesystem::path instance_subdirectory = "atmos2023";
  const std::filesystem::path instance_name         = "SimpleStation";
  // The network of the SimpleStation instance carries the same name.
  const std::filesystem::path network_name = "SimpleStation";

  const auto expect_non_empty_file = [&ec](const std::filesystem::path& p) {
    EXPECT_TRUE(std::filesystem::exists(p)) << "Missing file " << p;
    EXPECT_GT(std::filesystem::file_size(p, ec), 0) << "Empty file " << p;
  };

  // A solution is exported to
  // <export_dir>/solutions/<solution_subdir>/<instance_subdir>/<instance_name>
  const auto solution_dir =
      [&](const std::filesystem::path& export_dir,
          const std::filesystem::path& solution_subdirectory) {
        return export_dir / "solutions" / solution_subdirectory /
               instance_subdirectory / instance_name;
      };
  const auto expect_solution_files = [&](const std::filesystem::path& p) {
    EXPECT_TRUE(std::filesystem::is_directory(p)) << "Missing directory " << p;
    expect_non_empty_file(p / "solution_data.json");
    expect_non_empty_file(p / "routes.json");
    expect_non_empty_file(p / "train_pos.json");
    expect_non_empty_file(p / "train_speed.json");
    expect_non_empty_file(p / "train_exit_times.json");
    expect_non_empty_file(p / "train_stop_times.json");
    expect_non_empty_file(p / "vss_pos.json");
  };

  // The instance is exported to <export_dir>/instances/... and the
  // corresponding network to <export_dir>/networks/...
  const auto expect_instance_files =
      [&](const std::filesystem::path& export_dir) {
        const auto instance_dir =
            export_dir / "instances" / instance_subdirectory / instance_name;
        const auto network_dir = export_dir / "networks" / network_name;
        EXPECT_TRUE(std::filesystem::is_directory(instance_dir))
            << "Missing directory " << instance_dir;
        EXPECT_TRUE(std::filesystem::is_directory(network_dir))
            << "Missing directory " << network_dir;
        expect_non_empty_file(instance_dir / "network.json");
        expect_non_empty_file(instance_dir / "problem_data.json");
        expect_non_empty_file(instance_dir / "routes" / "routes.json");
        expect_non_empty_file(instance_dir / "timetable" / "schedules.json");
        expect_non_empty_file(instance_dir / "timetable" / "stations.json");
        expect_non_empty_file(instance_dir / "timetable" / "trains.json");
        expect_non_empty_file(network_dir / "successors.txt");
        expect_non_empty_file(network_dir / "successors_cpp.json");
        expect_non_empty_file(network_dir / "tracks.graphml");
      };
  const auto expect_no_instance_files =
      [](const std::filesystem::path& export_dir) {
        EXPECT_FALSE(std::filesystem::exists(export_dir / "instances"));
        EXPECT_FALSE(std::filesystem::exists(export_dir / "networks"));
      };

  const auto obj_val = solver.solve(
      {15, true, true, false}, {}, {},
      {false, cda_rail::ExportOption::ExportLP, "tmp1file", "tmp1folder"}, 20,
      true);

  // Expect optimal value of 1
  EXPECT_EQ(obj_val.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val.get_obj(), 1);

  check_exit_times_within_dt_and_order(obj_val, 15, "obj_val");
  // Check that tmp1folder and tmp1folder/tmp1file.mps and
  // tmp1folder/tmp1file.sol exist and are not empty
  EXPECT_TRUE(std::filesystem::exists("tmp1folder"));
  expect_non_empty_file("tmp1folder/tmp1file.mps");
  expect_non_empty_file("tmp1folder/tmp1file.sol");
  // Expect no solution and no instance to be exported
  EXPECT_FALSE(std::filesystem::exists("tmp1folder/solutions"));
  expect_no_instance_files("tmp1folder");
  // Remove tmp1folder and its contents
  std::filesystem::remove_all("tmp1folder");

  const auto obj_val2 = solver.solve(
      {15, true, true, false}, {}, {},
      {false, cda_rail::ExportOption::ExportSolution, "tmp2file", "tmp2folder"},
      20, true);

  // Expect optimal value of 1
  EXPECT_EQ(obj_val2.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val2.get_obj(), 1);

  check_exit_times_within_dt_and_order(obj_val2, 15, "obj_val2");
  // Check that tmp2folder and the solution directory exist
  EXPECT_TRUE(std::filesystem::exists("tmp2folder"));
  // Expect all solution files to exist and be not empty
  expect_solution_files(solution_dir("tmp2folder", "tmp2file"));
  // Expect neither the instance nor the network nor the model to be exported
  expect_no_instance_files("tmp2folder");
  EXPECT_FALSE(std::filesystem::exists("tmp2folder/tmp2file.mps"));
  EXPECT_FALSE(std::filesystem::exists("tmp2folder/tmp2file.sol"));
  // Remove tmp2folder and its contents
  std::filesystem::remove_all("tmp2folder");

  const auto obj_val3 =
      solver.solve({15, true, true, false}, {}, {},
                   {false, cda_rail::ExportOption::ExportSolutionWithInstance,
                    "tmp3file", "tmp3folder"},
                   20, true);

  // Expect optimal value of 1
  EXPECT_EQ(obj_val3.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val3.get_obj(), 1);

  check_exit_times_within_dt_and_order(obj_val3, 15, "obj_val3");
  // Check that corresponding folders and files exist and are not empty
  EXPECT_TRUE(std::filesystem::exists("tmp3folder"));
  expect_solution_files(solution_dir("tmp3folder", "tmp3file"));
  expect_instance_files("tmp3folder");
  // Expect the model to not be exported
  EXPECT_FALSE(std::filesystem::exists("tmp3folder/tmp3file.mps"));
  EXPECT_FALSE(std::filesystem::exists("tmp3folder/tmp3file.sol"));
  // Remove tmp3folder and its contents
  std::filesystem::remove_all("tmp3folder");

  const auto obj_val4 = solver.solve(
      {15, true, true, false}, {}, {},
      {false, cda_rail::ExportOption::NoExport, "tmp4file", "tmp4folder"}, 20,
      true);

  // Expect optimal value of 1
  EXPECT_EQ(obj_val4.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val4.get_obj(), 1);

  check_exit_times_within_dt_and_order(obj_val4, 15, "obj_val4");
  // Expect no folder tmp4folder to exist
  EXPECT_FALSE(std::filesystem::exists("tmp4folder"));

  const auto obj_val5 =
      solver.solve({15, true, true, false}, {}, {},
                   {false, cda_rail::ExportOption::ExportSolutionAndLP,
                    "tmp5file", "tmp5folder"},
                   20, false);

  // Expect optimal value of 1
  EXPECT_EQ(obj_val5.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val5.get_obj(), 1);

  check_exit_times_within_dt_and_order(obj_val5, 15, "obj_val5");
  // Expect relevant folders and files to exist and be not empty
  EXPECT_TRUE(std::filesystem::exists("tmp5folder"));
  expect_solution_files(solution_dir("tmp5folder", "tmp5file"));
  expect_non_empty_file("tmp5folder/tmp5file.mps");
  expect_non_empty_file("tmp5folder/tmp5file.sol");
  // Expect neither the instance nor the network to be exported
  expect_no_instance_files("tmp5folder");
  // Remove tmp5folder and its contents
  std::filesystem::remove_all("tmp5folder");

  const auto obj_val6 = solver.solve(
      {15, true, true, false}, {}, {},
      {false, cda_rail::ExportOption::ExportSolutionWithInstanceAndLP,
       "tmp6file", "tmp6folder"},
      20, false);

  // Expect optimal value of 1
  EXPECT_EQ(obj_val6.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val6.get_obj(), 1);

  check_exit_times_within_dt_and_order(obj_val6, 15, "obj_val6");
  // Expect relevant folders and files to exist and be not empty
  EXPECT_TRUE(std::filesystem::exists("tmp6folder"));
  expect_solution_files(solution_dir("tmp6folder", "tmp6file"));
  expect_instance_files("tmp6folder");
  expect_non_empty_file("tmp6folder/tmp6file.mps");
  expect_non_empty_file("tmp6folder/tmp6file.sol");
  // Remove tmp6folder and its contents
  std::filesystem::remove_all("tmp6folder");

  const auto obj_val7 = solver.solve(
      {15, true, true, false}, {}, {},
      {false, cda_rail::ExportOption::ExportSolutionWithInstanceAndLP}, 20,
      false);

  // Expect optimal value of 1
  EXPECT_EQ(obj_val7.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val7.get_obj(), 1);

  check_exit_times_within_dt_and_order(obj_val7, 15, "obj_val7");
  // By default everything is exported to the current directory using the name
  // "model"
  expect_solution_files(solution_dir(".", "model"));
  expect_instance_files(".");
  expect_non_empty_file("model.mps");
  expect_non_empty_file("model.sol");
  // Remove files and folders
  std::filesystem::remove_all("solutions");
  std::filesystem::remove_all("instances");
  std::filesystem::remove_all("networks");
  std::filesystem::remove("model.mps");
  std::filesystem::remove("model.sol");
}
