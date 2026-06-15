#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "EOMHelper.hpp"
#include "VSSModel.hpp"
#include "solver/astar-based/GenPOMovingBlockAStarSolver.hpp"

#include "gtest/gtest.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#define EXPECT_APPROX_EQ(a, b)                                                 \
  EXPECT_TRUE(std::abs((a) - (b)) < 1e-6) << (a) << " !=(approx.) " << (b)

// NOLINTBEGIN(clang-diagnostic-unused-result)

TEST(VSSModel, Consistency) {
  const auto& f = cda_rail::vss::functions::uniform;

  auto model = cda_rail::vss::Model(cda_rail::vss::ModelType::Discrete);
  EXPECT_FALSE(model.check_consistency());
  model = cda_rail::vss::Model(cda_rail::vss::ModelType::Discrete, {&f, &f});
  EXPECT_FALSE(model.check_consistency());
  model = cda_rail::vss::Model(cda_rail::vss::ModelType::Continuous, {&f});
  EXPECT_FALSE(model.check_consistency());
  model = cda_rail::vss::Model(cda_rail::vss::ModelType::Continuous, {&f, &f});
  EXPECT_FALSE(model.check_consistency());
  model = cda_rail::vss::Model(cda_rail::vss::ModelType::Inferred);
  EXPECT_FALSE(model.check_consistency());
  model = cda_rail::vss::Model(cda_rail::vss::ModelType::InferredAlt);
  EXPECT_FALSE(model.check_consistency());

  model = cda_rail::vss::Model(cda_rail::vss::ModelType::Discrete, {&f});
  EXPECT_TRUE(model.check_consistency());
  model = cda_rail::vss::Model(cda_rail::vss::ModelType::Continuous);
  EXPECT_TRUE(model.check_consistency());
  model = cda_rail::vss::Model(cda_rail::vss::ModelType::Inferred, {&f});
  EXPECT_TRUE(model.check_consistency());
  model = cda_rail::vss::Model(cda_rail::vss::ModelType::Inferred, {&f, &f});
  EXPECT_TRUE(model.check_consistency());
  model = cda_rail::vss::Model(cda_rail::vss::ModelType::InferredAlt, {&f});
  EXPECT_TRUE(model.check_consistency());
  model = cda_rail::vss::Model(cda_rail::vss::ModelType::InferredAlt, {&f, &f});
  EXPECT_TRUE(model.check_consistency());
}

TEST(VSSModel, Functions) {
  const auto& f1 = cda_rail::vss::functions::uniform;
  const auto& f2 = cda_rail::vss::functions::chebyshev;

  EXPECT_EQ(f1(0, 1), 1);
  EXPECT_EQ(f1(1, 1), 1);
  EXPECT_EQ(f1(0, 2), 0.5);
  EXPECT_EQ(f1(1, 2), 1);
  EXPECT_EQ(f1(2, 2), 1);
  EXPECT_EQ(f1(0, 3), 1.0 / 3.0);
  EXPECT_EQ(f1(1, 3), 2.0 / 3.0);
  EXPECT_EQ(f1(2, 3), 1);
  EXPECT_EQ(f1(3, 3), 1);
  EXPECT_EQ(f1(0, 4), 0.25);
  EXPECT_EQ(f1(1, 4), 0.5);
  EXPECT_EQ(f1(2, 4), 0.75);
  EXPECT_EQ(f1(3, 4), 1);
  EXPECT_EQ(f1(4, 4), 1);

  EXPECT_EQ(f2(0, 1), 1);
  EXPECT_EQ(f2(1, 1), 1);
  EXPECT_EQ(f2(0, 2), 0.5);
  EXPECT_EQ(f2(1, 2), 1);
  EXPECT_EQ(f2(2, 2), 1);
  EXPECT_EQ(cda_rail::round_to(f2(0, 3), 1e-5), 0.14645);
  EXPECT_EQ(cda_rail::round_to(f2(1, 3), 1e-5), 0.85355);
  EXPECT_EQ(f2(2, 3), 1);
  EXPECT_EQ(f2(3, 3), 1);
  EXPECT_EQ(cda_rail::round_to(f2(0, 4), 1e-5), 0.06699);
  EXPECT_EQ(cda_rail::round_to(f2(1, 4), 1e-5), 0.5);
  EXPECT_EQ(cda_rail::round_to(f2(2, 4), 1e-5), 0.93301);
  EXPECT_EQ(f2(3, 4), 1);
  EXPECT_EQ(f2(4, 4), 1);

  EXPECT_EQ(cda_rail::vss::functions::max_n_blocks(f1, 0.1), 10);
  EXPECT_EQ(cda_rail::vss::functions::max_n_blocks(f1, 1), 1);
  EXPECT_EQ(cda_rail::vss::functions::max_n_blocks(f2, 0.1), 3);

  EXPECT_THROW(cda_rail::vss::functions::max_n_blocks(f1, -0.1),
               std::invalid_argument);
  EXPECT_THROW(cda_rail::vss::functions::max_n_blocks(f1, 0),
               std::invalid_argument);
  EXPECT_THROW(cda_rail::vss::functions::max_n_blocks(f1, 1.1),
               std::invalid_argument);

  const cda_rail::vss::SeparationFunction f3 = [](size_t i, size_t n) {
    if (i >= n) {
      return 1.0;
    }
    return 1 - std::pow(2, -(static_cast<double>(i) + 1));
  };

  EXPECT_EQ(cda_rail::vss::functions::max_n_blocks(f3, 0.25), 3);

  const cda_rail::vss::SeparationFunction f4 = [](size_t i, size_t n) {
    if (i >= n) {
      return 1.0;
    }
    if (n == 1) {
      return 0.5;
    }
    if (n == 2) {
      if (i == 0) {
        return 0.35;
      }
      return 0.6;
    }
    if (n == 3) {
      if (i == 0) {
        return 0.3;
      }
      if (i == 1) {
        return 0.5;
      }
      return 0.75;
    }

    return cda_rail::vss::functions::uniform(i, n);
  };

  EXPECT_EQ(cda_rail::vss::functions::max_n_blocks(f4, 0.25), 2);
}

TEST(Helper, GreedySimulatorStateHash) {
  cda_rail::solver::astar_based::GreedySimulatorState state1;
  cda_rail::solver::astar_based::GreedySimulatorState state2;
  cda_rail::solver::astar_based::GreedySimulatorState state3;

  EXPECT_TRUE(state1 == state2);
  EXPECT_EQ(
      std::hash<cda_rail::solver::astar_based::GreedySimulatorState>()(state1),
      std::hash<cda_rail::solver::astar_based::GreedySimulatorState>()(state2));

  state1.train_edges.emplace_back();
  EXPECT_FALSE(state1 == state2);
  EXPECT_NE(
      std::hash<cda_rail::solver::astar_based::GreedySimulatorState>()(state1),
      std::hash<cda_rail::solver::astar_based::GreedySimulatorState>()(state2));

  state1.train_edges.emplace_back();
  state1.train_edges.at(1).emplace_back(1);

  EXPECT_FALSE(state1 == state2);
  EXPECT_NE(
      std::hash<cda_rail::solver::astar_based::GreedySimulatorState>()(state1),
      std::hash<cda_rail::solver::astar_based::GreedySimulatorState>()(state2));

  state2.train_edges = {{}, {1}};
  EXPECT_TRUE(state1 == state2);
  EXPECT_EQ(
      std::hash<cda_rail::solver::astar_based::GreedySimulatorState>()(state1),
      std::hash<cda_rail::solver::astar_based::GreedySimulatorState>()(state2));

  state1.stop_positions = {{1, 2}, {0.5, 2.4}, {1.4}};
  EXPECT_FALSE(state1 == state2);
  EXPECT_NE(
      std::hash<cda_rail::solver::astar_based::GreedySimulatorState>()(state1),
      std::hash<cda_rail::solver::astar_based::GreedySimulatorState>()(state2));

  state2.stop_positions = {{1, 2}, {0.5, 2.4}, {1.4}};
  EXPECT_TRUE(state1 == state2);
  EXPECT_EQ(
      std::hash<cda_rail::solver::astar_based::GreedySimulatorState>()(state1),
      std::hash<cda_rail::solver::astar_based::GreedySimulatorState>()(state2));

  state3.ttd_orders = {{1, 2, 3}};

  std::unordered_set<cda_rail::solver::astar_based::GreedySimulatorState>
      states;
  states.insert(state1);
  states.insert(state3);

  EXPECT_EQ(states.size(), 2);
  EXPECT_TRUE(states.contains(state1));
  EXPECT_TRUE(states.contains(state2));
  EXPECT_TRUE(states.contains(state3));

  states.insert(state2);
  EXPECT_EQ(states.size(), 2);
  EXPECT_TRUE(states.contains(state1));
  EXPECT_TRUE(states.contains(state2));
  EXPECT_TRUE(states.contains(state3));
}

// NOLINTEND(clang-diagnostic-unused-result)
