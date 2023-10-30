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
  solver.solve(30, true,
               cda_rail::vss::Model(cda_rail::vss::ModelType::Discrete,
                                    {cda_rail::vss::functions::uniform}));
  EXPECT_EQ(num_vertices, solver.get_instance().const_n().number_of_vertices());
}

TEST(Solver, GurobiVSSGenDeltaTDefault) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  std::cout << "--------------------- TEST 1 ---------------------------"
            << std::endl;
  auto obj_val_6 = solver.solve(6);
  std::cout << "--------------------- TEST 2 ---------------------------"
            << std::endl;
  auto obj_val_15 = solver.solve(15);
  std::cout << "--------------------- TEST 3 ---------------------------"
            << std::endl;
  auto obj_val_11 = solver.solve(11);
  std::cout << "--------------------- TEST 4 ---------------------------"
            << std::endl;
  auto obj_val_18 = solver.solve(18);
  std::cout << "--------------------- TEST 5 ---------------------------"
            << std::endl;
  auto obj_val_30 = solver.solve(30);

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
  const auto obj_val_1 = solver.solve(
      30, true, cda_rail::vss::Model(cda_rail::vss::ModelType::Continuous));
  std::cout << "--------------------- TEST 2 ---------------------------"
            << std::endl;
  const auto obj_val_2 = solver.solve(
      30, false, cda_rail::vss::Model(cda_rail::vss::ModelType::Continuous));
  std::cout << "--------------------- TEST 3 ---------------------------"
            << std::endl;
  const auto obj_val_3 =
      solver.solve(30, true,
                   cda_rail::vss::Model(cda_rail::vss::ModelType::Discrete,
                                        {&cda_rail::vss::functions::uniform}),
                   false, false);

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

TEST(Solver, GurobiVSSGenModelDetailFixed) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  std::cout << "--------------------- TEST 1 ---------------------------"
            << std::endl;
  const auto obj_val_1 = solver.solve(
      15, true, cda_rail::vss::Model(cda_rail::vss::ModelType::Continuous),
      true, true, false, true, {false, cda_rail::OptimalityStrategy::Optimal},
      false, 60, true, cda_rail::ExportOption::ExportLP, "test_1");
  EXPECT_TRUE(std::filesystem::exists("test_1.mps"));
  EXPECT_TRUE(std::filesystem::exists("test_1.sol"));
  std::filesystem::remove("test_1.mps");
  std::filesystem::remove("test_1.sol");
  EXPECT_FALSE(std::filesystem::exists("test_1.mps"));
  EXPECT_FALSE(std::filesystem::exists("test_1.sol"));

  std::cout << "--------------------- TEST 2 ---------------------------"
            << std::endl;
  const auto obj_val_2 = solver.solve(
      15, true, cda_rail::vss::Model(cda_rail::vss::ModelType::Continuous),
      true, true, false, true, {false, cda_rail::OptimalityStrategy::Optimal},
      false, 60, true, cda_rail::ExportOption::NoExport);

  std::cout << "--------------------- TEST 3 ---------------------------"
            << std::endl;
  const auto obj_val_3 = solver.solve(
      15, true, cda_rail::vss::Model(cda_rail::vss::ModelType::Continuous),
      true, false, false, false, {false, cda_rail::OptimalityStrategy::Optimal},
      false, 60, true, cda_rail::ExportOption::NoExport);

  std::cout << "--------------------- TEST 4 ---------------------------"
            << std::endl;
  const auto obj_val_4 = solver.solve(
      15, true, cda_rail::vss::Model(cda_rail::vss::ModelType::Continuous),
      true, true, true, true, {false, cda_rail::OptimalityStrategy::Optimal},
      false, 60, true, cda_rail::ExportOption::NoExport);

  std::cout << "--------------------- TEST 5 ---------------------------"
            << std::endl;
  const auto obj_val_5 = solver.solve(
      15, true, cda_rail::vss::Model(cda_rail::vss::ModelType::Continuous),
      true, false, false, true, {false, cda_rail::OptimalityStrategy::Optimal},
      false, 60, true, cda_rail::ExportOption::NoExport);

  std::cout << "--------------------- TEST 6 ---------------------------"
            << std::endl;
  const auto obj_val_6 = solver.solve(
      15, true, cda_rail::vss::Model(cda_rail::vss::ModelType::Continuous),
      false, false, false, true, {false, cda_rail::OptimalityStrategy::Optimal},
      false, 60, true, cda_rail::ExportOption::NoExport);

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
  const auto obj_val_1 = solver.solve(
      15, false, cda_rail::vss::Model(cda_rail::vss::ModelType::Continuous),
      true, true, false, true, {false, cda_rail::OptimalityStrategy::Optimal},
      false, 280, true, cda_rail::ExportOption::ExportLP, "test_1");
  EXPECT_TRUE(std::filesystem::exists("test_1.mps"));
  EXPECT_TRUE(std::filesystem::exists("test_1.sol"));
  std::filesystem::remove("test_1.mps");
  std::filesystem::remove("test_1.sol");
  EXPECT_FALSE(std::filesystem::exists("test_1.mps"));
  EXPECT_FALSE(std::filesystem::exists("test_1.sol"));

  EXPECT_EQ(obj_val_1.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_1.get_obj(), 1);
  EXPECT_EQ(obj_val_1.get_mip_obj(), 1);
}

TEST(Solver, GurobiVSSGenModelDetailFree2) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  std::cout << "--------------------- TEST 2 ---------------------------"
            << std::endl;
  const auto obj_val_2 = solver.solve(
      15, false, cda_rail::vss::Model(cda_rail::vss::ModelType::Continuous),
      true, true, false, true, {false, cda_rail::OptimalityStrategy::Optimal},
      false, 280, true, cda_rail::ExportOption::NoExport);

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
      15, false, cda_rail::vss::Model(cda_rail::vss::ModelType::Continuous),
      true, true, true, true, {false, cda_rail::OptimalityStrategy::Optimal},
      false, 280, true, cda_rail::ExportOption::NoExport);

  EXPECT_EQ(obj_val_3.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_3.get_obj(), 1);
  EXPECT_EQ(obj_val_3.get_mip_obj(), 1);
}

TEST(Solver, GurobiVSSGenModelDetailFree4) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  std::cout << "--------------------- TEST 4 ---------------------------"
            << std::endl;
  const auto obj_val_4 = solver.solve(
      15, false, cda_rail::vss::Model(cda_rail::vss::ModelType::Continuous),
      true, false, false, true, {false, cda_rail::OptimalityStrategy::Optimal},
      false, 280, true, cda_rail::ExportOption::NoExport);

  EXPECT_EQ(obj_val_4.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_4.get_obj(), 1);
  EXPECT_EQ(obj_val_4.get_mip_obj(), 1);
}

TEST(Solver, GurobiVSSGenModelDetailFree5) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  std::cout << "--------------------- TEST 5 ---------------------------"
            << std::endl;
  const auto obj_val_5 = solver.solve(
      15, false, cda_rail::vss::Model(cda_rail::vss::ModelType::Continuous),
      false, false, false, true, {false, cda_rail::OptimalityStrategy::Optimal},
      false, 280, true, cda_rail::ExportOption::NoExport);

  EXPECT_EQ(obj_val_5.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_5.get_obj(), 1);
  EXPECT_EQ(obj_val_5.get_mip_obj(), 1);
}

TEST(Solver, GurobiVSSGenVSSDiscrete) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  const auto obj_val = solver.solve(
      15, true,
      cda_rail::vss::Model(cda_rail::vss::ModelType::Discrete,
                           {&cda_rail::vss::functions::uniform}),
      false, false, false, true, {false, cda_rail::OptimalityStrategy::Optimal},
      false, 375, true, cda_rail::ExportOption::NoExport);

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

  const auto obj_val_1 = solver.solve(15);
  std::cout << "--------------------- TEST 2 ---------------------------"
            << std::endl;

  solver.editable_instance().editable_tr("tr1").tim = false;

  EXPECT_FALSE(solver.get_instance().get_train_list().get_train("tr1").tim);
  EXPECT_TRUE(solver.get_instance().get_train_list().get_train("tr2").tim);
  EXPECT_TRUE(solver.get_instance().get_train_list().get_train("tr3").tim);

  const auto obj_val_2 = solver.solve(15);
  std::cout << "--------------------- TEST 3 ---------------------------"
            << std::endl;

  solver.editable_instance().editable_tr("tr1").tim = true;
  solver.editable_instance().editable_tr("tr2").tim = false;

  EXPECT_TRUE(solver.get_instance().get_train_list().get_train("tr1").tim);
  EXPECT_FALSE(solver.get_instance().get_train_list().get_train("tr2").tim);
  EXPECT_TRUE(solver.get_instance().get_train_list().get_train("tr3").tim);

  const auto obj_val_3 = solver.solve(15);

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

  const auto obj_val_1 = solver.solve(15, true);
  std::cout << "--------------------- TEST 2 ---------------------------"
            << std::endl;

  solver.editable_instance().editable_tr("tr1").tim = false;

  EXPECT_FALSE(solver.get_instance().get_train_list().get_train("tr1").tim);
  EXPECT_TRUE(solver.get_instance().get_train_list().get_train("tr2").tim);
  EXPECT_TRUE(solver.get_instance().get_train_list().get_train("tr3").tim);

  const auto obj_val_2 = solver.solve(15, true);
  std::cout << "--------------------- TEST 3 ---------------------------"
            << std::endl;

  solver.editable_instance().editable_tr("tr1").tim = true;
  solver.editable_instance().editable_tr("tr2").tim = false;

  EXPECT_TRUE(solver.get_instance().get_train_list().get_train("tr1").tim);
  EXPECT_FALSE(solver.get_instance().get_train_list().get_train("tr2").tim);
  EXPECT_TRUE(solver.get_instance().get_train_list().get_train("tr3").tim);

  const auto obj_val_3 = solver.solve(15, true);

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

  const auto obj_val_1 = solver.solve(
      15, true,
      cda_rail::vss::Model(cda_rail::vss::ModelType::Discrete,
                           {&cda_rail::vss::functions::uniform}),
      false, false, false, true, {false, cda_rail::OptimalityStrategy::Optimal},
      false, 375, true, cda_rail::ExportOption::NoExport);

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

  const auto obj_val_2 = solver.solve(
      15, true,
      cda_rail::vss::Model(cda_rail::vss::ModelType::Discrete,
                           {&cda_rail::vss::functions::uniform}),
      false, false, false, true, {false, cda_rail::OptimalityStrategy::Optimal},
      false, 375, true, cda_rail::ExportOption::NoExport);

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

  const auto obj_val_3 = solver.solve(
      15, true,
      cda_rail::vss::Model(cda_rail::vss::ModelType::Discrete,
                           {&cda_rail::vss::functions::uniform}),
      false, false, false, true, {false, cda_rail::OptimalityStrategy::Optimal},
      false, 375, true, cda_rail::ExportOption::NoExport);

  EXPECT_EQ(obj_val_3.get_status(), cda_rail::SolutionStatus::Infeasible);
}

TEST(Solver, OvertakeFixedContinuous) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/Overtake/");

  const auto obj_val_base = solver.solve(
      15, true, cda_rail::vss::Model(cda_rail::vss::ModelType::Continuous),
      false, false, false, true, {false, cda_rail::OptimalityStrategy::Optimal},
      false, 120, false, cda_rail::ExportOption::NoExport);
  const auto obj_val_dynamics = solver.solve(
      15, true, cda_rail::vss::Model(cda_rail::vss::ModelType::Continuous),
      true, false, false, true, {false, cda_rail::OptimalityStrategy::Optimal},
      false, 120, false, cda_rail::ExportOption::NoExport);
  const auto obj_val_braking = solver.solve(
      15, true, cda_rail::vss::Model(cda_rail::vss::ModelType::Continuous),
      true, true, false, true, {false, cda_rail::OptimalityStrategy::Optimal},
      false, 120, false, cda_rail::ExportOption::NoExport);

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

  const auto obj_val_base = solver.solve(
      15, false, cda_rail::vss::Model(cda_rail::vss::ModelType::Continuous),
      false, false, false, true, {false, cda_rail::OptimalityStrategy::Optimal},
      false, 120, false, cda_rail::ExportOption::NoExport);
  const auto obj_val_dynamics = solver.solve(
      15, false, cda_rail::vss::Model(cda_rail::vss::ModelType::Continuous),
      true, false, false, true, {false, cda_rail::OptimalityStrategy::Optimal},
      false, 120, false, cda_rail::ExportOption::NoExport);
  const auto obj_val_braking = solver.solve(
      15, false, cda_rail::vss::Model(cda_rail::vss::ModelType::Continuous),
      true, true, false, true, {false, cda_rail::OptimalityStrategy::Optimal},
      false, 120, false, cda_rail::ExportOption::NoExport);

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

  const auto obj_val_base = solver.solve(
      15, true, cda_rail::vss::Model(cda_rail::vss::ModelType::Continuous),
      false, false, false, true, {false, cda_rail::OptimalityStrategy::Optimal},
      false, 120, false, cda_rail::ExportOption::NoExport);
  const auto obj_val_dynamics = solver.solve(
      15, true, cda_rail::vss::Model(cda_rail::vss::ModelType::Continuous),
      true, false, false, true, {false, cda_rail::OptimalityStrategy::Optimal},
      false, 120, false, cda_rail::ExportOption::NoExport);
  const auto obj_val_braking = solver.solve(
      15, true, cda_rail::vss::Model(cda_rail::vss::ModelType::Continuous),
      true, true, false, true, {false, cda_rail::OptimalityStrategy::Optimal},
      false, 120, false, cda_rail::ExportOption::NoExport);

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

  const auto obj_val_base = solver.solve(
      15, true, cda_rail::vss::Model(cda_rail::vss::ModelType::Continuous),
      false, false, false, true, {false, cda_rail::OptimalityStrategy::Optimal},
      false, 120, false, cda_rail::ExportOption::NoExport);
  const auto obj_val_dynamics = solver.solve(
      15, true, cda_rail::vss::Model(cda_rail::vss::ModelType::Continuous),
      true, false, false, true, {false, cda_rail::OptimalityStrategy::Optimal},
      false, 120, false, cda_rail::ExportOption::NoExport);
  const auto obj_val_braking = solver.solve(
      15, true, cda_rail::vss::Model(cda_rail::vss::ModelType::Continuous),
      true, true, false, true, {false, cda_rail::OptimalityStrategy::Optimal},
      false, 120, false, cda_rail::ExportOption::NoExport);

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

  const auto obj_val_base = solver.solve(
      15, true, cda_rail::vss::Model(cda_rail::vss::ModelType::Continuous),
      false, false, false, true, {false, cda_rail::OptimalityStrategy::Optimal},
      false, 120, false, cda_rail::ExportOption::NoExport);
  const auto obj_val_dynamics = solver.solve(
      15, true, cda_rail::vss::Model(cda_rail::vss::ModelType::Continuous),
      true, false, false, true, {false, cda_rail::OptimalityStrategy::Optimal},
      false, 120, false, cda_rail::ExportOption::NoExport);
  const auto obj_val_braking = solver.solve(
      15, true, cda_rail::vss::Model(cda_rail::vss::ModelType::Continuous),
      true, true, false, true, {false, cda_rail::OptimalityStrategy::Optimal},
      false, 120, false, cda_rail::ExportOption::NoExport);

  EXPECT_EQ(obj_val_base.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_dynamics.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val_braking.get_status(), cda_rail::SolutionStatus::Optimal);

  EXPECT_EQ(obj_val_base.get_obj(), 0);
  EXPECT_EQ(obj_val_dynamics.get_obj(), 15);
  EXPECT_EQ(obj_val_braking.get_obj(), 15);

  EXPECT_EQ(obj_val_base.get_mip_obj(), 0);
  EXPECT_EQ(obj_val_dynamics.get_mip_obj(), 15);
  EXPECT_EQ(obj_val_braking.get_mip_obj(), 15);
}

TEST(Solver, SimpleStationInferredUniform) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  const auto obj_val = solver.solve(
      15, true,
      cda_rail::vss::Model(cda_rail::vss::ModelType::Inferred,
                           {&cda_rail::vss::functions::uniform}),
      true, true, false, true, {false, cda_rail::OptimalityStrategy::Optimal},
      false, 60, true, cda_rail::ExportOption::NoExport);

  // Check if all objective values are 1
  EXPECT_EQ(obj_val.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val.get_obj(), 1);
  EXPECT_EQ(obj_val.get_mip_obj(), 1);
}

TEST(Solver, SimpleStationInferredChebychev) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  const auto obj_val = solver.solve(
      15, true,
      cda_rail::vss::Model(cda_rail::vss::ModelType::Inferred,
                           {&cda_rail::vss::functions::chebyshev}),
      true, true, false, true, {false, cda_rail::OptimalityStrategy::Optimal},
      false, 60, true, cda_rail::ExportOption::NoExport);

  // Check if all objective values are 1
  EXPECT_EQ(obj_val.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val.get_obj(), 1);
  EXPECT_EQ(obj_val.get_mip_obj(), 1);
}

TEST(Solver, SimpleStationInferredBoth) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  const auto obj_val = solver.solve(
      15, true,
      cda_rail::vss::Model(cda_rail::vss::ModelType::Inferred,
                           {&cda_rail::vss::functions::uniform,
                            &cda_rail::vss::functions::chebyshev}),
      true, true, false, true, {false, cda_rail::OptimalityStrategy::Optimal},
      false, 60, true, cda_rail::ExportOption::NoExport);

  // Check if all objective values are 1
  EXPECT_EQ(obj_val.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val.get_obj(), 1);
  EXPECT_EQ(obj_val.get_mip_obj(), 1);
}

TEST(Solver, SimpleStationInferredAltBoth) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  const auto obj_val = solver.solve(
      15, true,
      cda_rail::vss::Model(cda_rail::vss::ModelType::InferredAlt,
                           {&cda_rail::vss::functions::uniform,
                            &cda_rail::vss::functions::chebyshev}),
      true, true, false, true, {false, cda_rail::OptimalityStrategy::Optimal},
      false, 60, true, cda_rail::ExportOption::NoExport);

  // Check if all objective values are 1
  EXPECT_EQ(obj_val.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val.get_obj(), 1);
  EXPECT_EQ(obj_val.get_mip_obj(), 1);
}

TEST(Solver, IterativeContinuousSingleTrack) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SingleTrack/");

  const auto obj_val = solver.solve(
      15, true, cda_rail::vss::Model(cda_rail::vss::ModelType::Continuous),
      true, true, false, true, {true, cda_rail::OptimalityStrategy::Optimal},
      false, 60, true, cda_rail::ExportOption::NoExport);

  EXPECT_EQ(obj_val.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val.get_obj(), 9);
  EXPECT_EQ(obj_val.get_mip_obj(), 9);
}

TEST(Solver, IterativeContinuousStammstrecke4) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/Stammstrecke4Trains/");

  const auto obj_val = solver.solve(
      15, true, cda_rail::vss::Model(cda_rail::vss::ModelType::Continuous),
      true, true, false, true, {true, cda_rail::OptimalityStrategy::Optimal},
      false, 60, true, cda_rail::ExportOption::NoExport);

  EXPECT_EQ(obj_val.get_status(), cda_rail::SolutionStatus::Optimal);
  EXPECT_EQ(obj_val.get_obj(), 6);
  EXPECT_EQ(obj_val.get_mip_obj(), 6);
}

TEST(Solver, OnlyStopAtBoundariesContinuousFixed1) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  const auto obj_val = solver.solve(
      15, true,
      cda_rail::vss::Model(cda_rail::vss::ModelType::Continuous, {}, true),
      false, false, false, true, {false, cda_rail::OptimalityStrategy::Optimal},
      false, 240, true);

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
      15, true,
      cda_rail::vss::Model(cda_rail::vss::ModelType::Continuous, {}, true),
      true, false, false, true, {false, cda_rail::OptimalityStrategy::Optimal},
      false, 240, true);

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
      15, true,
      cda_rail::vss::Model(cda_rail::vss::ModelType::Continuous, {}, true),
      true, true, false, true, {false, cda_rail::OptimalityStrategy::Optimal},
      false, 240, true);

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
      15, false,
      cda_rail::vss::Model(cda_rail::vss::ModelType::Continuous, {}, true),
      false, false, false, true, {false, cda_rail::OptimalityStrategy::Optimal},
      false, 240, true);

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
      15, false,
      cda_rail::vss::Model(cda_rail::vss::ModelType::Continuous, {}, true),
      true, false, false, true, {false, cda_rail::OptimalityStrategy::Optimal},
      false, 240, true);

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
      15, false,
      cda_rail::vss::Model(cda_rail::vss::ModelType::Continuous, {}, true),
      true, true, false, true, {true, cda_rail::OptimalityStrategy::Optimal},
      false, 240, true);

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
