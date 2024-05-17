#include "VSSModel.hpp"
#include "solver/mip-based/VSSGenTimetableSolver.hpp"

#include "gtest/gtest.h"
#include <filesystem>
#include <iostream>
#include <string>

TEST(Solver, GurobiVSSDiscretizeInstanceWithoutChange) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  const auto num_vertices =
      solver.get_instance().const_n().number_of_vertices();
  solver.solve({30, true},
               {cda_rail::vss::Model(cda_rail::vss::ModelType::Discrete,
                                     {cda_rail::vss::functions::uniform})});

  EXPECT_EQ(num_vertices, solver.get_instance().const_n().number_of_vertices());
}

TEST(Solver, GurobiVSSGenDeltaTDefault) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  std::cout << "--------------------- TEST 1 ---------------------------"
            << std::endl;
  auto obj_val_6 = solver.solve({6});
  std::cout << "--------------------- TEST 2 ---------------------------"
            << std::endl;
  auto obj_val_15 = solver.solve({15});
  std::cout << "--------------------- TEST 3 ---------------------------"
            << std::endl;
  auto obj_val_11 = solver.solve({11});
  std::cout << "--------------------- TEST 4 ---------------------------"
            << std::endl;
  auto obj_val_18 = solver.solve({18});
  std::cout << "--------------------- TEST 5 ---------------------------"
            << std::endl;
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

  EXPECT_EQ(obj_val_6.get_mip_obj(), 1);
  EXPECT_EQ(obj_val_15.get_mip_obj(), 1);
  EXPECT_EQ(obj_val_11.get_mip_obj(), 1);
  EXPECT_EQ(obj_val_18.get_mip_obj(), 1);
  EXPECT_EQ(obj_val_30.get_mip_obj(), 1);
}

TEST(Solver, GurobiVSSGenDeltaT) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  std::cout << "--------------------- TEST 1 ---------------------------"
            << std::endl;
  const auto obj_val_2 = solver.solve({30, false});
  std::cout << "--------------------- TEST 2 ---------------------------"
            << std::endl;
  const auto obj_val_1 = solver.solve({30, true});
  std::cout << "--------------------- TEST 3 ---------------------------"
            << std::endl;
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

  EXPECT_EQ(obj_val_1.get_mip_obj(), 1);
  EXPECT_EQ(obj_val_2.get_mip_obj(), 1);
  EXPECT_EQ(obj_val_3.get_mip_obj(), 1);
}

TEST(Solver, GurobiVSSGenDefault) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  // Test various options
  std::cout << "--------------------- DEFAULT ---------------------------"
            << std::endl;
  const auto obj_val_default = solver.solve();
  EXPECT_EQ(obj_val_default.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_default.get_obj(), 1);
  EXPECT_EQ(obj_val_default.get_mip_obj(), 1);
}

TEST(Solver, GurobiVSSGenDefaultInstance) {
  cda_rail::instances::VSSGenerationTimetable instance(
      "./example-networks/SimpleStation/");
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(instance);

  // Test various options
  std::cout << "--------------------- DEFAULT ---------------------------"
            << std::endl;
  const auto obj_val_default = solver.solve();
  EXPECT_EQ(obj_val_default.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_default.get_obj(), 1);
  EXPECT_EQ(obj_val_default.get_mip_obj(), 1);
}

TEST(Solver, GurobiVSSGenDefaultInstancePath) {
  std::filesystem::path instance_path("./example-networks/SimpleStation/");
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(instance_path);

  // Test various options
  std::cout << "--------------------- DEFAULT ---------------------------"
            << std::endl;
  const auto obj_val_default = solver.solve();
  EXPECT_EQ(obj_val_default.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_default.get_obj(), 1);
  EXPECT_EQ(obj_val_default.get_mip_obj(), 1);
}

TEST(Solver, GurobiVSSGenDefaultInstanceString) {
  std::string instance_path("./example-networks/SimpleStation/");
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(instance_path);

  // Test various options
  std::cout << "--------------------- DEFAULT ---------------------------"
            << std::endl;
  const auto obj_val_default = solver.solve();
  EXPECT_EQ(obj_val_default.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_default.get_obj(), 1);
  EXPECT_EQ(obj_val_default.get_mip_obj(), 1);
}

TEST(Solver, GurobiVSSGenModelDetailFixed) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  std::cout << "--------------------- TEST 1 ---------------------------"
            << std::endl;
  const auto obj_val_1 = solver.solve({}, {}, {}, {}, 60, true);

  std::cout << "--------------------- TEST 2 ---------------------------"
            << std::endl;
  const auto obj_val_2 = solver.solve({}, {}, {}, {}, 60, true);

  std::cout << "--------------------- TEST 3 ---------------------------"
            << std::endl;
  const auto obj_val_3 =
      solver.solve({15, true, true, false},
                   {cda_rail::vss::Model(cda_rail::vss::ModelType::Continuous),
                    false, false},
                   {}, {}, 60, true);

  std::cout << "--------------------- TEST 4 ---------------------------"
            << std::endl;
  const auto obj_val_4 = solver.solve(
      {},
      {cda_rail::vss::Model(cda_rail::vss::ModelType::Continuous), true, true},
      {}, {}, 60, true);

  std::cout << "--------------------- TEST 5 ---------------------------"
            << std::endl;
  const auto obj_val_5 =
      solver.solve({15, true, true, false}, {}, {}, {}, 60, true);

  std::cout << "--------------------- TEST 6 ---------------------------"
            << std::endl;
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

  EXPECT_EQ(obj_val_1.get_mip_obj(), 1);
  EXPECT_EQ(obj_val_2.get_mip_obj(), 1);
  EXPECT_EQ(obj_val_3.get_mip_obj(), 1);
  EXPECT_EQ(obj_val_4.get_mip_obj(), 1);
  EXPECT_EQ(obj_val_5.get_mip_obj(), 1);
  EXPECT_EQ(obj_val_6.get_mip_obj(), 1);
}

TEST(Solver, GurobiVSSGenModelDetailFree1) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  std::cout << "--------------------- TEST 1 ---------------------------"
            << std::endl;
  const auto obj_val_1 = solver.solve({15, false}, {}, {}, {}, 280, true);

  EXPECT_EQ(obj_val_1.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_1.get_obj(), 1);
  EXPECT_EQ(obj_val_1.get_mip_obj(), 1);
}

TEST(Solver, GurobiVSSGenModelDetailFree2) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  std::cout << "--------------------- TEST 2 ---------------------------"
            << std::endl;
  const auto obj_val_2 = solver.solve({15, false}, {}, {}, {}, 280, true);

  EXPECT_EQ(obj_val_2.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_2.get_obj(), 1);
  EXPECT_EQ(obj_val_2.get_mip_obj(), 1);
}

TEST(Solver, GurobiVSSGenModelDetailFree3) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  std::cout << "--------------------- TEST 3 ---------------------------"
            << std::endl;
  const auto obj_val_3 = solver.solve(
      {15, false}, {cda_rail::vss::Model(), true, true}, {}, {}, 280, true);

  EXPECT_EQ(obj_val_3.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_3.get_obj(), 1);
  EXPECT_EQ(obj_val_3.get_mip_obj(), 1);
}

TEST(Solver, GurobiVSSGenModelDetailFree4) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  std::cout << "--------------------- TEST 4 ---------------------------"
            << std::endl;
  const auto obj_val_4 =
      solver.solve({15, false, true, false}, {}, {}, {}, 280, true);

  EXPECT_EQ(obj_val_4.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_4.get_obj(), 1);
  EXPECT_EQ(obj_val_4.get_mip_obj(), 1);
}

TEST(Solver, GurobiVSSGenModelDetailFree5) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  std::cout << "--------------------- TEST 5 ---------------------------"
            << std::endl;
  const auto obj_val_5 =
      solver.solve({15, false, false, false}, {}, {}, {}, 280, true);

  EXPECT_EQ(obj_val_5.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_5.get_obj(), 1);
  EXPECT_EQ(obj_val_5.get_mip_obj(), 1);
}

TEST(Solver, GurobiVSSGenVSSDiscrete) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  const auto obj_val =
      solver.solve({15, true, false, false},
                   {cda_rail::vss::Model(cda_rail::vss::ModelType::Discrete,
                                         {&cda_rail::vss::functions::uniform})},
                   {}, {}, 375, true);

  // Check if all objective values are 1
  EXPECT_EQ(obj_val.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val.get_obj(), 1);
  EXPECT_EQ(obj_val.get_mip_obj(), 1);
}

TEST(Solver, GurobiVSSGenTim) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  std::cout << "--------------------- TEST 1 ---------------------------"
            << std::endl;

  EXPECT_TRUE(solver.get_instance().get_train_list().get_train("tr1").tim);
  EXPECT_TRUE(solver.get_instance().get_train_list().get_train("tr2").tim);
  EXPECT_TRUE(solver.get_instance().get_train_list().get_train("tr3").tim);

  const auto obj_val_1 = solver.solve({15, false});
  std::cout << "--------------------- TEST 2 ---------------------------"
            << std::endl;

  solver.editable_instance().editable_tr("tr1").tim = false;

  EXPECT_FALSE(solver.get_instance().get_train_list().get_train("tr1").tim);
  EXPECT_TRUE(solver.get_instance().get_train_list().get_train("tr2").tim);
  EXPECT_TRUE(solver.get_instance().get_train_list().get_train("tr3").tim);

  const auto obj_val_2 = solver.solve({15, false});
  std::cout << "--------------------- TEST 3 ---------------------------"
            << std::endl;

  solver.editable_instance().editable_tr("tr1").tim = true;
  solver.editable_instance().editable_tr("tr2").tim = false;

  EXPECT_TRUE(solver.get_instance().get_train_list().get_train("tr1").tim);
  EXPECT_FALSE(solver.get_instance().get_train_list().get_train("tr2").tim);
  EXPECT_TRUE(solver.get_instance().get_train_list().get_train("tr3").tim);

  const auto obj_val_3 = solver.solve({15, false});

  EXPECT_EQ(obj_val_1.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_2.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_3.get_status(), cda_rail::SolutionStatus::Infeasible);

  EXPECT_EQ(obj_val_1.get_obj(), 1);
  EXPECT_EQ(obj_val_2.get_obj(), 1);

  EXPECT_EQ(obj_val_1.get_mip_obj(), 1);
  EXPECT_EQ(obj_val_2.get_mip_obj(), 1);
}

TEST(Solver, GurobiVSSGenTimFixed) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  std::cout << "--------------------- TEST 1 ---------------------------"
            << std::endl;

  EXPECT_TRUE(solver.get_instance().get_train_list().get_train("tr1").tim);
  EXPECT_TRUE(solver.get_instance().get_train_list().get_train("tr2").tim);
  EXPECT_TRUE(solver.get_instance().get_train_list().get_train("tr3").tim);

  const auto obj_val_1 = solver.solve();
  std::cout << "--------------------- TEST 2 ---------------------------"
            << std::endl;

  solver.editable_instance().editable_tr("tr1").tim = false;

  EXPECT_FALSE(solver.get_instance().get_train_list().get_train("tr1").tim);
  EXPECT_TRUE(solver.get_instance().get_train_list().get_train("tr2").tim);
  EXPECT_TRUE(solver.get_instance().get_train_list().get_train("tr3").tim);

  const auto obj_val_2 = solver.solve();
  std::cout << "--------------------- TEST 3 ---------------------------"
            << std::endl;

  solver.editable_instance().editable_tr("tr1").tim = true;
  solver.editable_instance().editable_tr("tr2").tim = false;

  EXPECT_TRUE(solver.get_instance().get_train_list().get_train("tr1").tim);
  EXPECT_FALSE(solver.get_instance().get_train_list().get_train("tr2").tim);
  EXPECT_TRUE(solver.get_instance().get_train_list().get_train("tr3").tim);

  const auto obj_val_3 = solver.solve();

  EXPECT_EQ(obj_val_1.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_2.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_3.get_status(), cda_rail::SolutionStatus::Infeasible);

  EXPECT_EQ(obj_val_1.get_obj(), 1);
  EXPECT_EQ(obj_val_2.get_obj(), 1);

  EXPECT_EQ(obj_val_1.get_mip_obj(), 1);
  EXPECT_EQ(obj_val_2.get_mip_obj(), 1);
}

TEST(Solver, GurobiVSSGenTimDiscrete1) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  std::cout << "--------------------- TEST 1 ---------------------------"
            << std::endl;

  EXPECT_TRUE(solver.get_instance().get_train_list().get_train("tr1").tim);
  EXPECT_TRUE(solver.get_instance().get_train_list().get_train("tr2").tim);
  EXPECT_TRUE(solver.get_instance().get_train_list().get_train("tr3").tim);

  const auto obj_val_1 =
      solver.solve({15, true, false, false},
                   {cda_rail::vss::Model(cda_rail::vss::ModelType::Discrete,
                                         {&cda_rail::vss::functions::uniform})},
                   {}, {}, 375, true);

  EXPECT_EQ(obj_val_1.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_1.get_obj(), 1);
  EXPECT_EQ(obj_val_1.get_mip_obj(), 1);
}

TEST(Solver, GurobiVSSGenTimDiscrete2) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  std::cout << "--------------------- TEST 2 ---------------------------"
            << std::endl;

  solver.editable_instance().editable_tr("tr1").tim = false;

  EXPECT_FALSE(solver.get_instance().get_train_list().get_train("tr1").tim);
  EXPECT_TRUE(solver.get_instance().get_train_list().get_train("tr2").tim);
  EXPECT_TRUE(solver.get_instance().get_train_list().get_train("tr3").tim);

  const auto obj_val_2 =
      solver.solve({15, true, false, false},
                   {cda_rail::vss::Model(cda_rail::vss::ModelType::Discrete,
                                         {&cda_rail::vss::functions::uniform})},
                   {}, {}, 375, true);

  EXPECT_EQ(obj_val_2.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_2.get_obj(), 1);
  EXPECT_EQ(obj_val_2.get_mip_obj(), 1);
}

TEST(Solver, GurobiVSSGenTimDiscrete3) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  std::cout << "--------------------- TEST 3 ---------------------------"
            << std::endl;

  solver.editable_instance().editable_tr("tr2").tim = false;

  EXPECT_TRUE(solver.get_instance().get_train_list().get_train("tr1").tim);
  EXPECT_FALSE(solver.get_instance().get_train_list().get_train("tr2").tim);
  EXPECT_TRUE(solver.get_instance().get_train_list().get_train("tr3").tim);

  const auto obj_val_3 =
      solver.solve({15, true, false, false},
                   {cda_rail::vss::Model(cda_rail::vss::ModelType::Discrete,
                                         {&cda_rail::vss::functions::uniform})},
                   {}, {}, 375, true);

  EXPECT_EQ(obj_val_3.get_status(), cda_rail::SolutionStatus::Infeasible);
}

TEST(Solver, OvertakeFixedContinuous) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/Overtake/");

  const auto obj_val_base =
      solver.solve({15, true, false, false}, {}, {}, {}, 120);
  const auto obj_val_dynamics =
      solver.solve({15, true, true, false}, {}, {}, {}, 120);
  const auto obj_val_braking =
      solver.solve({15, true, true, true}, {}, {}, {}, 120);

  EXPECT_EQ(obj_val_base.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_dynamics.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_braking.get_status(), cda_rail::SolutionStatus::Optimal);

  EXPECT_EQ(obj_val_base.get_obj(), 8);
  EXPECT_EQ(obj_val_dynamics.get_obj(), 8);
  EXPECT_EQ(obj_val_braking.get_obj(), 14);

  EXPECT_EQ(obj_val_base.get_mip_obj(), 8);
  EXPECT_EQ(obj_val_dynamics.get_mip_obj(), 8);
  EXPECT_EQ(obj_val_braking.get_mip_obj(), 14);
}

TEST(Solver, OvertakeFreeContinuous) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/Overtake/");

  const auto obj_val_base =
      solver.solve({15, false, false, false}, {}, {}, {}, 120);
  const auto obj_val_dynamics =
      solver.solve({15, false, true, false}, {}, {}, {}, 120);
  const auto obj_val_braking =
      solver.solve({15, false, true, true}, {}, {}, {}, 120);

  EXPECT_EQ(obj_val_base.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_dynamics.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_braking.get_status(), cda_rail::SolutionStatus::Optimal);

  EXPECT_EQ(obj_val_base.get_obj(), 8);
  EXPECT_EQ(obj_val_dynamics.get_obj(), 8);
  EXPECT_EQ(obj_val_braking.get_obj(), 14);

  EXPECT_EQ(obj_val_base.get_mip_obj(), 8);
  EXPECT_EQ(obj_val_dynamics.get_mip_obj(), 8);
  EXPECT_EQ(obj_val_braking.get_mip_obj(), 14);
}

TEST(Solver, Stammstrecke4FixedContinuous) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/Stammstrecke4Trains/");

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

  EXPECT_EQ(obj_val_base.get_mip_obj(), 0);
  EXPECT_EQ(obj_val_dynamics.get_mip_obj(), 6);
  EXPECT_EQ(obj_val_braking.get_mip_obj(), 6);
}

TEST(Solver, Stammstrecke8FixedContinuous) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/Stammstrecke8Trains/");

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

  EXPECT_EQ(obj_val_base.get_mip_obj(), 0);
  EXPECT_EQ(obj_val_dynamics.get_mip_obj(), 14);
  EXPECT_EQ(obj_val_braking.get_mip_obj(), 14);
}

TEST(Solver, Stammstrecke16FixedContinuous) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/Stammstrecke16Trains/");

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
  EXPECT_EQ(obj_val_dynamics.get_obj(), 16);
  EXPECT_EQ(obj_val_braking.get_obj(), 16);

  EXPECT_EQ(obj_val_base.get_mip_obj(), 0);
  EXPECT_EQ(obj_val_dynamics.get_mip_obj(), 16);
  EXPECT_EQ(obj_val_braking.get_mip_obj(), 16);
}

TEST(Solver, SimpleStationInferredUniform) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  const auto obj_val =
      solver.solve({},
                   {cda_rail::vss::Model(cda_rail::vss::ModelType::Inferred,
                                         {&cda_rail::vss::functions::uniform})},
                   {}, {}, 60, true);

  // Check if all objective values are 1
  EXPECT_EQ(obj_val.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val.get_obj(), 1);
  EXPECT_EQ(obj_val.get_mip_obj(), 1);
}

TEST(Solver, SimpleStationInferredUniformPostprocess) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  const auto obj_val =
      solver.solve({},
                   {cda_rail::vss::Model(cda_rail::vss::ModelType::Inferred,
                                         {&cda_rail::vss::functions::uniform})},
                   {}, {true}, 60, true);

  // Check if all objective values are 1
  EXPECT_EQ(obj_val.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val.get_obj(), 1);
  EXPECT_EQ(obj_val.get_mip_obj(), 1);
}

TEST(Solver, SimpleStationInferredAltUniformPostprocess) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  const auto obj_val =
      solver.solve({},
                   {cda_rail::vss::Model(cda_rail::vss::ModelType::InferredAlt,
                                         {&cda_rail::vss::functions::uniform})},
                   {}, {true}, 60, true);

  // Check if all objective values are 1
  EXPECT_EQ(obj_val.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val.get_obj(), 1);
  EXPECT_EQ(obj_val.get_mip_obj(), 1);
}

TEST(Solver, SimpleStationContinuousPostprocess) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  const auto obj_val = solver.solve({15, false}, {}, {}, {true}, 240, true);

  // Check if all objective values are 1
  EXPECT_EQ(obj_val.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val.get_obj(), 1);
  EXPECT_EQ(obj_val.get_mip_obj(), 1);
}

TEST(Solver, SimpleStationContinuousFixedPostprocess) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  const auto obj_val = solver.solve({15, true}, {}, {}, {true}, 60, true);

  // Check if all objective values are 1
  EXPECT_EQ(obj_val.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val.get_obj(), 1);
  EXPECT_EQ(obj_val.get_mip_obj(), 1);
}

TEST(Solver, SimpleStationInferredChebychev) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  const auto obj_val = solver.solve(
      {},
      {cda_rail::vss::Model(cda_rail::vss::ModelType::Inferred,
                            {&cda_rail::vss::functions::chebyshev})},
      {}, {}, 60, true);

  // Check if all objective values are 1
  EXPECT_EQ(obj_val.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val.get_obj(), 1);
  EXPECT_EQ(obj_val.get_mip_obj(), 1);
}

TEST(Solver, SimpleStationInferredBoth) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  const auto obj_val = solver.solve(
      {},
      {cda_rail::vss::Model(cda_rail::vss::ModelType::Inferred,
                            {&cda_rail::vss::functions::uniform,
                             &cda_rail::vss::functions::chebyshev})},
      {}, {}, 60, true);

  // Check if all objective values are 1
  EXPECT_EQ(obj_val.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val.get_obj(), 1);
  EXPECT_EQ(obj_val.get_mip_obj(), 1);
}

TEST(Solver, SimpleStationInferredAltBoth) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  const auto obj_val = solver.solve(
      {},
      {cda_rail::vss::Model(cda_rail::vss::ModelType::InferredAlt,
                            {&cda_rail::vss::functions::uniform,
                             &cda_rail::vss::functions::chebyshev})},
      {}, {}, 60, true);

  // Check if all objective values are 1
  EXPECT_EQ(obj_val.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val.get_obj(), 1);
  EXPECT_EQ(obj_val.get_mip_obj(), 1);
}

TEST(Solver, IterativeContinuousSingleTrack) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SingleTrack/");

  const auto obj_val = solver.solve(
      {}, {}, {true, cda_rail::OptimalityStrategy::Optimal}, {}, 60, true);

  EXPECT_EQ(obj_val.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val.get_obj(), 9);
  EXPECT_EQ(obj_val.get_mip_obj(), 9);
}

TEST(Solver, IterativeContinuousSingleTrackCuts) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SingleTrack/");

  const auto obj_val = solver.solve(
      {}, {},
      {true, cda_rail::OptimalityStrategy::Optimal,
       cda_rail::solver::mip_based::UpdateStrategy::Fixed, 1, 2, true},
      {}, 60, true);

  EXPECT_EQ(obj_val.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val.get_obj(), 9);
  EXPECT_EQ(obj_val.get_mip_obj(), 9);
}

TEST(Solver, IterativeContinuousSingleRelative) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SingleTrack/");

  const auto obj_val =
      solver.solve({}, {},
                   {true, cda_rail::OptimalityStrategy::Optimal,
                    cda_rail::solver::mip_based::UpdateStrategy::Relative,
                    0.025, 0.05, true},
                   {}, 60, true);

  EXPECT_EQ(obj_val.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val.get_obj(), 9);
  EXPECT_EQ(obj_val.get_mip_obj(), 9);
}

TEST(Solver, IterativeContinuousSimpleStationInferredCuts) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  const auto obj_val = solver.solve(
      {15, true, true, false},
      {cda_rail::vss::Model(cda_rail::vss::ModelType::Inferred,
                            {&cda_rail::vss::functions::uniform})},
      {true, cda_rail::OptimalityStrategy::Optimal,
       cda_rail::solver::mip_based::UpdateStrategy::Fixed, 0, 2, true},
      {}, 60, true);
}

TEST(Solver, IterativeContinuousSimpleStationCuts) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  const auto obj_val = solver.solve(
      {15, true, true, false}, {},
      {true, cda_rail::OptimalityStrategy::Optimal,
       cda_rail::solver::mip_based::UpdateStrategy::Fixed, 0, 2, true},
      {}, 60, true);
}

TEST(Solver, IterativeContinuousFeasible) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/Stammstrecke4Trains/");

  const auto obj_val = solver.solve(
      {}, {}, {true, cda_rail::OptimalityStrategy::Feasible}, {}, 60, true);

  EXPECT_EQ(obj_val.get_status(), cda_rail::SolutionStatus::Feasible);
  EXPECT_GE(obj_val.get_obj(), 6);
  EXPECT_GE(obj_val.get_mip_obj(), 6);
}

TEST(Solver, IterativeTimeout1) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleNetwork/");

  const auto obj_val = solver.solve({15, false}, {}, {true}, {}, 30, true);

  EXPECT_EQ(obj_val.get_status(), cda_rail::SolutionStatus::Timeout);
}

TEST(Solver, IterativeTimeout2) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleNetwork/");

  const auto obj_val = solver.solve({}, {}, {true}, {}, 1, true);

  EXPECT_EQ(obj_val.get_status(), cda_rail::SolutionStatus::Timeout);
}

TEST(Solver, IterativeContinuousSimpleStationInferredAlt) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  const auto obj_val = solver.solve(
      {15, true, true, false},
      {cda_rail::vss::Model(cda_rail::vss::ModelType::InferredAlt,
                            {&cda_rail::vss::functions::uniform})},
      {true, cda_rail::OptimalityStrategy::Optimal,
       cda_rail::solver::mip_based::UpdateStrategy::Fixed, 0, 2, true},
      {}, 60, true);

  EXPECT_EQ(obj_val.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val.get_obj(), 1);
  EXPECT_EQ(obj_val.get_mip_obj(), 1);
}

TEST(Solver, IterativeContinuousStammstrecke4) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/Stammstrecke4Trains/");

  const auto obj_val = solver.solve({}, {}, {true}, {}, 60, true);

  EXPECT_EQ(obj_val.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val.get_obj(), 6);
  EXPECT_EQ(obj_val.get_mip_obj(), 6);
}

TEST(Solver, IterativeContinuousStammstrecke4Cuts) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/Stammstrecke4Trains/");

  const auto obj_val = solver.solve(
      {}, {},
      {true, cda_rail::OptimalityStrategy::Optimal,
       cda_rail::solver::mip_based::UpdateStrategy::Fixed, 0, 2, true},
      {}, 60, true);

  EXPECT_EQ(obj_val.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val.get_obj(), 6);
  EXPECT_EQ(obj_val.get_mip_obj(), 6);
}

TEST(Solver, IterativeContinuousOvertakeRelative) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/Overtake/");

  const auto obj_val = solver.solve(
      {}, {},
      {true, cda_rail::OptimalityStrategy::Optimal,
       cda_rail::solver::mip_based::UpdateStrategy::Relative, 0.05, 0.05, true},
      {}, 60, true);

  EXPECT_EQ(obj_val.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val.get_obj(), 14);
  EXPECT_EQ(obj_val.get_mip_obj(), 14);
}

TEST(Solver, OnlyStopAtBoundariesContinuousFixed1) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  const auto obj_val = solver.solve(
      {15, true, false, false},
      {cda_rail::vss::Model(cda_rail::vss::ModelType::Continuous, {}, true)},
      {}, {}, 240, true);

  // Check if all objective values are 1
  EXPECT_EQ(obj_val.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val.get_obj(), 1);
  EXPECT_EQ(obj_val.get_mip_obj(), 1);

  for (size_t tr = 0; tr < obj_val.get_instance().get_train_list().size();
       ++tr) {
    const auto& allowed_stops = obj_val.get_valid_border_stops(tr);
    const auto& tr_name =
        obj_val.get_instance().get_train_list().get_train(tr).name;

    // put values of allowed_stops into string separated by comma
    std::string allowed_stops_str;
    for (const auto& stop : allowed_stops) {
      allowed_stops_str += std::to_string(stop) + ", ";
    }
    // remove last comma
    allowed_stops_str =
        allowed_stops_str.substr(0, allowed_stops_str.size() - 2);

    const auto& [t0, tn] =
        obj_val.get_instance().time_index_interval(tr, obj_val.get_dt(), false);
    for (int t = static_cast<int>(t0) + 1; t <= static_cast<int>(tn); ++t) {
      const auto& train_speed =
          obj_val.get_train_speed(tr, t * obj_val.get_dt());
      if (train_speed > cda_rail::GRB_EPS) {
        continue;
      }
      const auto& tr_pos = obj_val.get_train_pos(tr, t * obj_val.get_dt());
      // Expect any of allowed_stops to be within EPS of tr_pos
      bool found = false;
      for (const auto& stop : allowed_stops) {
        if (std::abs(stop - tr_pos) <
            cda_rail::GRB_EPS + cda_rail::STOP_TOLERANCE) {
          found = true;
          break;
        }
      }
      EXPECT_TRUE(found) << "Error on train " << tr_name << " (id=" << tr
                         << ") at time " << t * obj_val.get_dt()
                         << " with speed " << train_speed << " and position "
                         << tr_pos << ". Allowed stops: " << allowed_stops_str;
    }
  }
}

TEST(Solver, OnlyStopAtBoundariesContinuousFixed2) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  const auto obj_val = solver.solve(
      {15, true, true, false},
      {cda_rail::vss::Model(cda_rail::vss::ModelType::Continuous, {}, true)},
      {}, {}, 240, true);

  // Check if all objective values are 1
  EXPECT_EQ(obj_val.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val.get_obj(), 1);
  EXPECT_EQ(obj_val.get_mip_obj(), 1);

  for (size_t tr = 0; tr < obj_val.get_instance().get_train_list().size();
       ++tr) {
    const auto& allowed_stops = obj_val.get_valid_border_stops(tr);
    const auto& tr_name =
        obj_val.get_instance().get_train_list().get_train(tr).name;

    // put values of allowed_stops into string separated by comma
    std::string allowed_stops_str;
    for (const auto& stop : allowed_stops) {
      allowed_stops_str += std::to_string(stop) + ", ";
    }
    // remove last comma
    allowed_stops_str =
        allowed_stops_str.substr(0, allowed_stops_str.size() - 2);

    const auto& [t0, tn] =
        obj_val.get_instance().time_index_interval(tr, obj_val.get_dt(), false);
    for (int t = static_cast<int>(t0) + 1; t <= static_cast<int>(tn); ++t) {
      const auto& train_speed =
          obj_val.get_train_speed(tr, t * obj_val.get_dt());
      if (train_speed > cda_rail::GRB_EPS) {
        continue;
      }
      const auto& tr_pos = obj_val.get_train_pos(tr, t * obj_val.get_dt());
      // Expect any of allowed_stops to be within EPS of tr_pos
      bool found = false;
      for (const auto& stop : allowed_stops) {
        if (std::abs(stop - tr_pos) <
            cda_rail::GRB_EPS + cda_rail::STOP_TOLERANCE) {
          found = true;
          break;
        }
      }
      EXPECT_TRUE(found) << "Error on train " << tr_name << " (id=" << tr
                         << ") at time " << t * obj_val.get_dt()
                         << " with speed " << train_speed << " and position "
                         << tr_pos << ". Allowed stops: " << allowed_stops_str;
    }
  }
}

TEST(Solver, OnlyStopAtBoundariesContinuousFixed3) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  const auto obj_val = solver.solve(
      {15, true, true, true},
      {cda_rail::vss::Model(cda_rail::vss::ModelType::Continuous, {}, true)},
      {}, {}, 240, true);

  // Check if all objective values are 1
  EXPECT_EQ(obj_val.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val.get_obj(), 1);
  EXPECT_EQ(obj_val.get_mip_obj(), 1);

  for (size_t tr = 0; tr < obj_val.get_instance().get_train_list().size();
       ++tr) {
    const auto& allowed_stops = obj_val.get_valid_border_stops(tr);
    const auto& tr_name =
        obj_val.get_instance().get_train_list().get_train(tr).name;

    // put values of allowed_stops into string separated by comma
    std::string allowed_stops_str;
    for (const auto& stop : allowed_stops) {
      allowed_stops_str += std::to_string(stop) + ", ";
    }
    // remove last comma
    allowed_stops_str =
        allowed_stops_str.substr(0, allowed_stops_str.size() - 2);

    const auto& [t0, tn] =
        obj_val.get_instance().time_index_interval(tr, obj_val.get_dt(), false);
    for (int t = static_cast<int>(t0) + 1; t <= static_cast<int>(tn); ++t) {
      const auto& train_speed =
          obj_val.get_train_speed(tr, t * obj_val.get_dt());
      if (train_speed > cda_rail::GRB_EPS) {
        continue;
      }
      const auto& tr_pos = obj_val.get_train_pos(tr, t * obj_val.get_dt());
      // Expect any of allowed_stops to be within EPS of tr_pos
      bool found = false;
      for (const auto& stop : allowed_stops) {
        if (std::abs(stop - tr_pos) <
            cda_rail::GRB_EPS + cda_rail::STOP_TOLERANCE) {
          found = true;
          break;
        }
      }
      EXPECT_TRUE(found) << "Error on train " << tr_name << " (id=" << tr
                         << ") at time " << t * obj_val.get_dt()
                         << " with speed " << train_speed << " and position "
                         << tr_pos << ". Allowed stops: " << allowed_stops_str;
    }
  }
}

TEST(Solver, OnlyStopAtBoundariesContinuousFree1) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  const auto obj_val = solver.solve(
      {15, false, false, false},
      {cda_rail::vss::Model(cda_rail::vss::ModelType::Continuous, {}, true)},
      {}, {}, 240, true);

  // Check if all objective values are 1
  EXPECT_EQ(obj_val.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val.get_obj(), 1);
  EXPECT_EQ(obj_val.get_mip_obj(), 1);

  for (size_t tr = 0; tr < obj_val.get_instance().get_train_list().size();
       ++tr) {
    const auto& allowed_stops = obj_val.get_valid_border_stops(tr);
    const auto& tr_name =
        obj_val.get_instance().get_train_list().get_train(tr).name;

    // put values of allowed_stops into string separated by comma
    std::string allowed_stops_str;
    for (const auto& stop : allowed_stops) {
      allowed_stops_str += std::to_string(stop) + ", ";
    }
    // remove last comma
    allowed_stops_str =
        allowed_stops_str.substr(0, allowed_stops_str.size() - 2);

    const auto& [t0, tn] =
        obj_val.get_instance().time_index_interval(tr, obj_val.get_dt(), false);
    for (int t = static_cast<int>(t0) + 1; t <= static_cast<int>(tn); ++t) {
      const auto& train_speed =
          obj_val.get_train_speed(tr, t * obj_val.get_dt());
      if (train_speed > cda_rail::GRB_EPS) {
        continue;
      }
      const auto& tr_pos = obj_val.get_train_pos(tr, t * obj_val.get_dt());
      // Expect any of allowed_stops to be within EPS of tr_pos
      bool found = false;
      for (const auto& stop : allowed_stops) {
        if (std::abs(stop - tr_pos) <
            cda_rail::GRB_EPS + cda_rail::STOP_TOLERANCE) {
          found = true;
          break;
        }
      }
      EXPECT_TRUE(found) << "Error on train " << tr_name << " (id=" << tr
                         << ") at time " << t * obj_val.get_dt()
                         << " with speed " << train_speed << " and position "
                         << tr_pos << ". Allowed stops: " << allowed_stops_str;
    }
  }
}

TEST(Solver, OnlyStopAtBoundariesContinuousFree2) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  const auto obj_val = solver.solve(
      {15, false, true, false},
      {cda_rail::vss::Model(cda_rail::vss::ModelType::Continuous, {}, true)},
      {}, {}, 240, true);

  // Check if all objective values are 1
  EXPECT_EQ(obj_val.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val.get_obj(), 1);
  EXPECT_EQ(obj_val.get_mip_obj(), 1);

  for (size_t tr = 0; tr < obj_val.get_instance().get_train_list().size();
       ++tr) {
    const auto& allowed_stops = obj_val.get_valid_border_stops(tr);
    const auto& tr_name =
        obj_val.get_instance().get_train_list().get_train(tr).name;

    // put values of allowed_stops into string separated by comma
    std::string allowed_stops_str;
    for (const auto& stop : allowed_stops) {
      allowed_stops_str += std::to_string(stop) + ", ";
    }
    // remove last comma
    allowed_stops_str =
        allowed_stops_str.substr(0, allowed_stops_str.size() - 2);

    const auto& [t0, tn] =
        obj_val.get_instance().time_index_interval(tr, obj_val.get_dt(), false);
    for (int t = static_cast<int>(t0) + 1; t <= static_cast<int>(tn); ++t) {
      const auto& train_speed =
          obj_val.get_train_speed(tr, t * obj_val.get_dt());
      if (train_speed > cda_rail::GRB_EPS) {
        continue;
      }
      const auto& tr_pos = obj_val.get_train_pos(tr, t * obj_val.get_dt());
      // Expect any of allowed_stops to be within EPS of tr_pos
      bool found = false;
      for (const auto& stop : allowed_stops) {
        if (std::abs(stop - tr_pos) <
            cda_rail::GRB_EPS + cda_rail::STOP_TOLERANCE) {
          found = true;
          break;
        }
      }
      EXPECT_TRUE(found) << "Error on train " << tr_name << " (id=" << tr
                         << ") at time " << t * obj_val.get_dt()
                         << " with speed " << train_speed << " and position "
                         << tr_pos << ". Allowed stops: " << allowed_stops_str;
    }
  }
}

TEST(Solver, OnlyStopAtBoundariesContinuousFree3) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  const auto obj_val = solver.solve(
      {15, false, true, true},
      {cda_rail::vss::Model(cda_rail::vss::ModelType::Continuous, {}, true)},
      {}, {}, 240, true);

  // Check if all objective values are 1
  EXPECT_EQ(obj_val.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val.get_obj(), 1);
  EXPECT_EQ(obj_val.get_mip_obj(), 1);

  for (size_t tr = 0; tr < obj_val.get_instance().get_train_list().size();
       ++tr) {
    const auto& allowed_stops = obj_val.get_valid_border_stops(tr);
    const auto& tr_name =
        obj_val.get_instance().get_train_list().get_train(tr).name;

    // put values of allowed_stops into string separated by comma
    std::string allowed_stops_str;
    for (const auto& stop : allowed_stops) {
      allowed_stops_str += std::to_string(stop) + ", ";
    }
    // remove last comma
    allowed_stops_str =
        allowed_stops_str.substr(0, allowed_stops_str.size() - 2);

    const auto& [t0, tn] =
        obj_val.get_instance().time_index_interval(tr, obj_val.get_dt(), false);
    for (int t = static_cast<int>(t0) + 1; t <= static_cast<int>(tn); ++t) {
      const auto& train_speed =
          obj_val.get_train_speed(tr, t * obj_val.get_dt());
      if (train_speed > cda_rail::GRB_EPS) {
        continue;
      }
      const auto& tr_pos = obj_val.get_train_pos(tr, t * obj_val.get_dt());
      // Expect any of allowed_stops to be within EPS of tr_pos
      bool found = false;
      for (const auto& stop : allowed_stops) {
        if (std::abs(stop - tr_pos) <
            cda_rail::GRB_EPS + cda_rail::STOP_TOLERANCE) {
          found = true;
          break;
        }
      }
      EXPECT_TRUE(found) << "Error on train " << tr_name << " (id=" << tr
                         << ") at time " << t * obj_val.get_dt()
                         << " with speed " << train_speed << " and position "
                         << tr_pos << ". Allowed stops: " << allowed_stops_str;
    }
  }
}

TEST(Solver, SimpleStationExportOptions) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  std::filesystem::remove_all("tmp1folder");
  std::filesystem::remove_all("tmp2folder");
  std::filesystem::remove_all("tmp3folder");
  std::filesystem::remove_all("tmp4folder");
  std::filesystem::remove_all("tmp5folder");
  std::filesystem::remove_all("tmp6folder");
  std::filesystem::remove_all("model");
  std::filesystem::remove("model.mps");
  std::filesystem::remove("model.sol");

  std::error_code ec;

  const auto obj_val = solver.solve(
      {15, true, true, false}, {}, {},
      {false, cda_rail::ExportOption::ExportLP, "tmp1file", "tmp1folder"}, 20,
      true);

  // Expect optimal value of 1
  EXPECT_EQ(obj_val.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val.get_obj(), 1);
  EXPECT_EQ(obj_val.get_mip_obj(), 1);
  // Check that tmp1folder and tmp1folder/tmp1file.mps and
  // tmp1folder/tmp1file.sol exist
  EXPECT_TRUE(std::filesystem::exists("tmp1folder"));
  EXPECT_TRUE(std::filesystem::exists("tmp1folder/tmp1file.mps"));
  EXPECT_TRUE(std::filesystem::exists("tmp1folder/tmp1file.sol"));
  // Check that both files are not empty
  EXPECT_GT(std::filesystem::file_size("tmp1folder/tmp1file.mps", ec), 0);
  EXPECT_GT(std::filesystem::file_size("tmp1folder/tmp1file.sol", ec), 0);
  // Remove tmp1folder and its contents
  std::filesystem::remove_all("tmp1folder");

  const auto obj_val2 = solver.solve(
      {15, true, true, false}, {}, {},
      {false, cda_rail::ExportOption::ExportSolution, "tmp2file", "tmp2folder"},
      20, true);

  // Expect optimal value of 1
  EXPECT_EQ(obj_val2.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val2.get_obj(), 1);
  EXPECT_EQ(obj_val2.get_mip_obj(), 1);
  // Check that tmp2folder and tmp2folder/tmp2file exist
  EXPECT_TRUE(std::filesystem::exists("tmp2folder"));
  EXPECT_TRUE(std::filesystem::exists("tmp2folder/tmp2file"));
  // Expect that .../instance and .../solution exist
  EXPECT_TRUE(std::filesystem::exists("tmp2folder/tmp2file/instance"));
  EXPECT_TRUE(std::filesystem::exists("tmp2folder/tmp2file/solution"));
  // Expect that .../instance/routes exists
  EXPECT_TRUE(std::filesystem::exists("tmp2folder/tmp2file/instance/routes"));
  // Expect that .../instance/routes/routes.json exists and is not empty
  EXPECT_TRUE(std::filesystem::exists(
      "tmp2folder/tmp2file/instance/routes/routes.json"));
  EXPECT_GT(std::filesystem::file_size(
                "tmp2folder/tmp2file/instance/routes/routes.json", ec),
            0);
  // Within .../solution expect data.json, train_pos.json, train_speed.json,
  // vss_pos.json and all not empty
  EXPECT_TRUE(
      std::filesystem::exists("tmp2folder/tmp2file/solution/data.json"));
  EXPECT_TRUE(
      std::filesystem::exists("tmp2folder/tmp2file/solution/train_pos.json"));
  EXPECT_TRUE(
      std::filesystem::exists("tmp2folder/tmp2file/solution/train_speed.json"));
  EXPECT_TRUE(
      std::filesystem::exists("tmp2folder/tmp2file/solution/vss_pos.json"));
  EXPECT_GT(
      std::filesystem::file_size("tmp2folder/tmp2file/solution/data.json", ec),
      0);
  EXPECT_GT(std::filesystem::file_size(
                "tmp2folder/tmp2file/solution/train_pos.json", ec),
            0);
  EXPECT_GT(std::filesystem::file_size(
                "tmp2folder/tmp2file/solution/train_speed.json", ec),
            0);
  EXPECT_GT(std::filesystem::file_size(
                "tmp2folder/tmp2file/solution/vss_pos.json", ec),
            0);
  // Expect folders .../instance/network and .../instance/timetable to not exist
  EXPECT_FALSE(std::filesystem::exists("tmp2folder/tmp2file/instance/network"));
  EXPECT_FALSE(
      std::filesystem::exists("tmp2folder/tmp2file/instance/timetable"));
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
  EXPECT_EQ(obj_val3.get_mip_obj(), 1);
  // Check that corresponding folders exist
  EXPECT_TRUE(std::filesystem::exists("tmp3folder"));
  EXPECT_TRUE(std::filesystem::exists("tmp3folder/tmp3file"));
  EXPECT_TRUE(std::filesystem::exists("tmp3folder/tmp3file/instance"));
  EXPECT_TRUE(std::filesystem::exists("tmp3folder/tmp3file/solution"));
  EXPECT_TRUE(std::filesystem::exists("tmp3folder/tmp3file/instance/routes"));
  EXPECT_TRUE(std::filesystem::exists("tmp3folder/tmp3file/instance/network"));
  EXPECT_TRUE(
      std::filesystem::exists("tmp3folder/tmp3file/instance/timetable"));
  // Expect relevant files to exist and be not empty
  EXPECT_TRUE(std::filesystem::exists(
      "tmp3folder/tmp3file/instance/routes/routes.json"));
  EXPECT_TRUE(std::filesystem::exists(
      "tmp3folder/tmp3file/instance/network/successors.txt"));
  EXPECT_TRUE(std::filesystem::exists(
      "tmp3folder/tmp3file/instance/network/successors_cpp.json"));
  EXPECT_TRUE(std::filesystem::exists(
      "tmp3folder/tmp3file/instance/network/tracks.graphml"));
  EXPECT_TRUE(std::filesystem::exists(
      "tmp3folder/tmp3file/instance/timetable/schedules.json"));
  EXPECT_TRUE(std::filesystem::exists(
      "tmp3folder/tmp3file/instance/timetable/stations.json"));
  EXPECT_TRUE(std::filesystem::exists(
      "tmp3folder/tmp3file/instance/timetable/trains.json"));
  EXPECT_TRUE(
      std::filesystem::exists("tmp3folder/tmp3file/solution/data.json"));
  EXPECT_TRUE(
      std::filesystem::exists("tmp3folder/tmp3file/solution/train_pos.json"));
  EXPECT_TRUE(
      std::filesystem::exists("tmp3folder/tmp3file/solution/train_speed.json"));
  EXPECT_TRUE(
      std::filesystem::exists("tmp3folder/tmp3file/solution/vss_pos.json"));
  EXPECT_GT(std::filesystem::file_size(
                "tmp3folder/tmp3file/instance/routes/routes.json", ec),
            0);
  EXPECT_GT(std::filesystem::file_size(
                "tmp3folder/tmp3file/instance/network/successors.txt", ec),
            0);
  EXPECT_GT(std::filesystem::file_size(
                "tmp3folder/tmp3file/instance/network/successors_cpp.json", ec),
            0);
  EXPECT_GT(std::filesystem::file_size(
                "tmp3folder/tmp3file/instance/network/tracks.graphml", ec),
            0);
  EXPECT_GT(std::filesystem::file_size(
                "tmp3folder/tmp3file/instance/timetable/schedules.json", ec),
            0);
  EXPECT_GT(std::filesystem::file_size(
                "tmp3folder/tmp3file/instance/timetable/stations.json", ec),
            0);
  EXPECT_GT(std::filesystem::file_size(
                "tmp3folder/tmp3file/instance/timetable/trains.json", ec),
            0);
  EXPECT_GT(
      std::filesystem::file_size("tmp3folder/tmp3file/solution/data.json", ec),
      0);
  EXPECT_GT(std::filesystem::file_size(
                "tmp3folder/tmp3file/solution/train_pos.json", ec),
            0);
  EXPECT_GT(std::filesystem::file_size(
                "tmp3folder/tmp3file/solution/train_speed.json", ec),
            0);
  EXPECT_GT(std::filesystem::file_size(
                "tmp3folder/tmp3file/solution/vss_pos.json", ec),
            0);
  // Remove tmp3folder and its contents
  std::filesystem::remove_all("tmp3folder");

  const auto obj_val4 = solver.solve(
      {15, true, true, false}, {}, {},
      {false, cda_rail::ExportOption::NoExport, "tmp4file", "tmp4folder"}, 20,
      true);

  // Expect optimal value of 1
  EXPECT_EQ(obj_val4.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val4.get_obj(), 1);
  EXPECT_EQ(obj_val4.get_mip_obj(), 1);
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
  EXPECT_EQ(obj_val5.get_mip_obj(), 1);
  // Expect relevant folders to exist
  EXPECT_TRUE(std::filesystem::exists("tmp5folder"));
  EXPECT_TRUE(std::filesystem::exists("tmp5folder/tmp5file"));
  EXPECT_TRUE(std::filesystem::exists("tmp5folder/tmp5file/solution"));
  EXPECT_TRUE(std::filesystem::exists("tmp5folder/tmp5file/instance"));
  EXPECT_TRUE(std::filesystem::exists("tmp5folder/tmp5file/instance/routes"));
  // Expect non-relevant folders to not exist
  EXPECT_FALSE(std::filesystem::exists("tmp5folder/tmp5file/instance/network"));
  EXPECT_FALSE(
      std::filesystem::exists("tmp5folder/tmp5file/instance/timetable"));
  // Expect relevant files to exist and be not empty
  EXPECT_TRUE(std::filesystem::exists(
      "tmp5folder/tmp5file/instance/routes/routes.json"));
  EXPECT_TRUE(
      std::filesystem::exists("tmp5folder/tmp5file/solution/data.json"));
  EXPECT_TRUE(
      std::filesystem::exists("tmp5folder/tmp5file/solution/train_pos.json"));
  EXPECT_TRUE(
      std::filesystem::exists("tmp5folder/tmp5file/solution/train_speed.json"));
  EXPECT_TRUE(
      std::filesystem::exists("tmp5folder/tmp5file/solution/vss_pos.json"));
  EXPECT_TRUE(std::filesystem::exists("tmp5folder/tmp5file.mps"));
  EXPECT_TRUE(std::filesystem::exists("tmp5folder/tmp5file.sol"));
  EXPECT_GT(std::filesystem::file_size(
                "tmp5folder/tmp5file/instance/routes/routes.json", ec),
            0);
  EXPECT_GT(
      std::filesystem::file_size("tmp5folder/tmp5file/solution/data.json", ec),
      0);
  EXPECT_GT(std::filesystem::file_size(
                "tmp5folder/tmp5file/solution/train_pos.json", ec),
            0);
  EXPECT_GT(std::filesystem::file_size(
                "tmp5folder/tmp5file/solution/train_speed.json", ec),
            0);
  EXPECT_GT(std::filesystem::file_size(
                "tmp5folder/tmp5file/solution/vss_pos.json", ec),
            0);
  EXPECT_GT(std::filesystem::file_size("tmp5folder/tmp5file.mps", ec), 0);
  EXPECT_GT(std::filesystem::file_size("tmp5folder/tmp5file.sol", ec), 0);
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
  EXPECT_EQ(obj_val6.get_mip_obj(), 1);
  // Expect relevant folders to exist
  EXPECT_TRUE(std::filesystem::exists("tmp6folder"));
  EXPECT_TRUE(std::filesystem::exists("tmp6folder/tmp6file"));
  EXPECT_TRUE(std::filesystem::exists("tmp6folder/tmp6file/solution"));
  EXPECT_TRUE(std::filesystem::exists("tmp6folder/tmp6file/instance"));
  EXPECT_TRUE(std::filesystem::exists("tmp6folder/tmp6file/instance/routes"));
  EXPECT_TRUE(std::filesystem::exists("tmp6folder/tmp6file/instance/network"));
  EXPECT_TRUE(
      std::filesystem::exists("tmp6folder/tmp6file/instance/timetable"));
  // Expect relevant files to exist and be not empty
  EXPECT_TRUE(std::filesystem::exists(
      "tmp6folder/tmp6file/instance/routes/routes.json"));
  EXPECT_TRUE(std::filesystem::exists(
      "tmp6folder/tmp6file/instance/network/successors.txt"));
  EXPECT_TRUE(std::filesystem::exists(
      "tmp6folder/tmp6file/instance/network/successors_cpp.json"));
  EXPECT_TRUE(std::filesystem::exists(
      "tmp6folder/tmp6file/instance/network/tracks.graphml"));
  EXPECT_TRUE(std::filesystem::exists(
      "tmp6folder/tmp6file/instance/timetable/schedules.json"));
  EXPECT_TRUE(std::filesystem::exists(
      "tmp6folder/tmp6file/instance/timetable/stations.json"));
  EXPECT_TRUE(std::filesystem::exists(
      "tmp6folder/tmp6file/instance/timetable/trains.json"));
  EXPECT_TRUE(
      std::filesystem::exists("tmp6folder/tmp6file/solution/data.json"));
  EXPECT_TRUE(
      std::filesystem::exists("tmp6folder/tmp6file/solution/train_pos.json"));
  EXPECT_TRUE(
      std::filesystem::exists("tmp6folder/tmp6file/solution/train_speed.json"));
  EXPECT_TRUE(
      std::filesystem::exists("tmp6folder/tmp6file/solution/vss_pos.json"));
  EXPECT_TRUE(std::filesystem::exists("tmp6folder/tmp6file.mps"));
  EXPECT_TRUE(std::filesystem::exists("tmp6folder/tmp6file.sol"));
  EXPECT_GT(std::filesystem::file_size(
                "tmp6folder/tmp6file/instance/routes/routes.json", ec),
            0);
  EXPECT_GT(std::filesystem::file_size(
                "tmp6folder/tmp6file/instance/network/successors.txt", ec),
            0);
  EXPECT_GT(std::filesystem::file_size(
                "tmp6folder/tmp6file/instance/network/successors_cpp.json", ec),
            0);
  EXPECT_GT(std::filesystem::file_size(
                "tmp6folder/tmp6file/instance/network/tracks.graphml", ec),
            0);
  EXPECT_GT(std::filesystem::file_size(
                "tmp6folder/tmp6file/instance/timetable/schedules.json", ec),
            0);
  EXPECT_GT(std::filesystem::file_size(
                "tmp6folder/tmp6file/instance/timetable/stations.json", ec),
            0);
  EXPECT_GT(std::filesystem::file_size(
                "tmp6folder/tmp6file/instance/timetable/trains.json", ec),
            0);
  EXPECT_GT(
      std::filesystem::file_size("tmp6folder/tmp6file/solution/data.json", ec),
      0);
  EXPECT_GT(std::filesystem::file_size(
                "tmp6folder/tmp6file/solution/train_pos.json", ec),
            0);
  EXPECT_GT(std::filesystem::file_size(
                "tmp6folder/tmp6file/solution/train_speed.json", ec),
            0);
  EXPECT_GT(std::filesystem::file_size(
                "tmp6folder/tmp6file/solution/vss_pos.json", ec),
            0);
  EXPECT_GT(std::filesystem::file_size("tmp6folder/tmp6file.mps", ec), 0);
  EXPECT_GT(std::filesystem::file_size("tmp6folder/tmp6file.sol", ec), 0);
  // Remove tmp6folder and its contents
  std::filesystem::remove_all("tmp6folder");

  const auto obj_val7 = solver.solve(
      {15, true, true, false}, {}, {},
      {false, cda_rail::ExportOption::ExportSolutionWithInstanceAndLP}, 20,
      false);

  // Expect optimal value of 1
  EXPECT_EQ(obj_val7.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val7.get_obj(), 1);
  EXPECT_EQ(obj_val7.get_mip_obj(), 1);
  // Expect relevant folders to exist
  EXPECT_TRUE(std::filesystem::exists("model/instance"));
  EXPECT_TRUE(std::filesystem::exists("model/solution"));
  EXPECT_TRUE(std::filesystem::exists("model/instance/routes"));
  EXPECT_TRUE(std::filesystem::exists("model/instance/network"));
  EXPECT_TRUE(std::filesystem::exists("model/instance/timetable"));
  // Expect relevant files to exist and be not empty
  EXPECT_TRUE(std::filesystem::exists("model/instance/routes/routes.json"));
  EXPECT_TRUE(std::filesystem::exists("model/instance/network/successors.txt"));
  EXPECT_TRUE(
      std::filesystem::exists("model/instance/network/successors_cpp.json"));
  EXPECT_TRUE(std::filesystem::exists("model/instance/network/tracks.graphml"));
  EXPECT_TRUE(
      std::filesystem::exists("model/instance/timetable/schedules.json"));
  EXPECT_TRUE(
      std::filesystem::exists("model/instance/timetable/stations.json"));
  EXPECT_TRUE(std::filesystem::exists("model/instance/timetable/trains.json"));
  EXPECT_TRUE(std::filesystem::exists("model/solution/data.json"));
  EXPECT_TRUE(std::filesystem::exists("model/solution/train_pos.json"));
  EXPECT_TRUE(std::filesystem::exists("model/solution/train_speed.json"));
  EXPECT_TRUE(std::filesystem::exists("model/solution/vss_pos.json"));
  EXPECT_TRUE(std::filesystem::exists("model.mps"));
  EXPECT_TRUE(std::filesystem::exists("model.sol"));
  EXPECT_GT(std::filesystem::file_size("model/instance/routes/routes.json", ec),
            0);
  EXPECT_GT(
      std::filesystem::file_size("model/instance/network/successors.txt", ec),
      0);
  EXPECT_GT(std::filesystem::file_size(
                "model/instance/network/successors_cpp.json", ec),
            0);
  EXPECT_GT(
      std::filesystem::file_size("model/instance/network/tracks.graphml", ec),
      0);
  EXPECT_GT(
      std::filesystem::file_size("model/instance/timetable/schedules.json", ec),
      0);
  EXPECT_GT(
      std::filesystem::file_size("model/instance/timetable/stations.json", ec),
      0);
  EXPECT_GT(
      std::filesystem::file_size("model/instance/timetable/trains.json", ec),
      0);
  EXPECT_GT(std::filesystem::file_size("model/solution/data.json", ec), 0);
  EXPECT_GT(std::filesystem::file_size("model/solution/train_pos.json", ec), 0);
  EXPECT_GT(std::filesystem::file_size("model/solution/train_speed.json", ec),
            0);
  EXPECT_GT(std::filesystem::file_size("model/solution/vss_pos.json", ec), 0);
  EXPECT_GT(std::filesystem::file_size("model.mps", ec), 0);
  EXPECT_GT(std::filesystem::file_size("model.sol", ec), 0);
  // Remove files and folders
  std::filesystem::remove_all("model");
  std::filesystem::remove("model.mps");
  std::filesystem::remove("model.sol");
}
