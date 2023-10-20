#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "VSSModel.hpp"

#include "gtest/gtest.h"
#include <cmath>
#include <iostream>

// NOLINTBEGIN(clang-diagnostic-unused-result)

TEST(Functionality, Subsets) {
  auto subsets_of_size_3 = cda_rail::subsets_of_size_k_indices(6, 3);
  // Expect 6 choose 2 number of elements
  EXPECT_EQ(subsets_of_size_3.size(), 20);
  // Expect to find all subsets of size 3
  EXPECT_TRUE(std::find(subsets_of_size_3.begin(), subsets_of_size_3.end(),
                        std::vector<size_t>({0, 1, 2})) !=
              subsets_of_size_3.end());
  EXPECT_TRUE(std::find(subsets_of_size_3.begin(), subsets_of_size_3.end(),
                        std::vector<size_t>({0, 1, 3})) !=
              subsets_of_size_3.end());
  EXPECT_TRUE(std::find(subsets_of_size_3.begin(), subsets_of_size_3.end(),
                        std::vector<size_t>({0, 1, 4})) !=
              subsets_of_size_3.end());
  EXPECT_TRUE(std::find(subsets_of_size_3.begin(), subsets_of_size_3.end(),
                        std::vector<size_t>({0, 1, 5})) !=
              subsets_of_size_3.end());
  EXPECT_TRUE(std::find(subsets_of_size_3.begin(), subsets_of_size_3.end(),
                        std::vector<size_t>({0, 2, 3})) !=
              subsets_of_size_3.end());
  EXPECT_TRUE(std::find(subsets_of_size_3.begin(), subsets_of_size_3.end(),
                        std::vector<size_t>({0, 2, 4})) !=
              subsets_of_size_3.end());
  EXPECT_TRUE(std::find(subsets_of_size_3.begin(), subsets_of_size_3.end(),
                        std::vector<size_t>({0, 2, 5})) !=
              subsets_of_size_3.end());
  EXPECT_TRUE(std::find(subsets_of_size_3.begin(), subsets_of_size_3.end(),
                        std::vector<size_t>({0, 3, 4})) !=
              subsets_of_size_3.end());
  EXPECT_TRUE(std::find(subsets_of_size_3.begin(), subsets_of_size_3.end(),
                        std::vector<size_t>({0, 3, 5})) !=
              subsets_of_size_3.end());
  EXPECT_TRUE(std::find(subsets_of_size_3.begin(), subsets_of_size_3.end(),
                        std::vector<size_t>({0, 4, 5})) !=
              subsets_of_size_3.end());
  EXPECT_TRUE(std::find(subsets_of_size_3.begin(), subsets_of_size_3.end(),
                        std::vector<size_t>({1, 2, 3})) !=
              subsets_of_size_3.end());
  EXPECT_TRUE(std::find(subsets_of_size_3.begin(), subsets_of_size_3.end(),
                        std::vector<size_t>({1, 2, 4})) !=
              subsets_of_size_3.end());
  EXPECT_TRUE(std::find(subsets_of_size_3.begin(), subsets_of_size_3.end(),
                        std::vector<size_t>({1, 2, 5})) !=
              subsets_of_size_3.end());
  EXPECT_TRUE(std::find(subsets_of_size_3.begin(), subsets_of_size_3.end(),
                        std::vector<size_t>({1, 3, 4})) !=
              subsets_of_size_3.end());
  EXPECT_TRUE(std::find(subsets_of_size_3.begin(), subsets_of_size_3.end(),
                        std::vector<size_t>({1, 3, 5})) !=
              subsets_of_size_3.end());
  EXPECT_TRUE(std::find(subsets_of_size_3.begin(), subsets_of_size_3.end(),
                        std::vector<size_t>({1, 4, 5})) !=
              subsets_of_size_3.end());
  EXPECT_TRUE(std::find(subsets_of_size_3.begin(), subsets_of_size_3.end(),
                        std::vector<size_t>({2, 3, 4})) !=
              subsets_of_size_3.end());
  EXPECT_TRUE(std::find(subsets_of_size_3.begin(), subsets_of_size_3.end(),
                        std::vector<size_t>({2, 3, 5})) !=
              subsets_of_size_3.end());
  EXPECT_TRUE(std::find(subsets_of_size_3.begin(), subsets_of_size_3.end(),
                        std::vector<size_t>({2, 4, 5})) !=
              subsets_of_size_3.end());
  EXPECT_TRUE(std::find(subsets_of_size_3.begin(), subsets_of_size_3.end(),
                        std::vector<size_t>({3, 4, 5})) !=
              subsets_of_size_3.end());

  auto subsets_of_size_2 = cda_rail::subsets_of_size_2_indices(5);
  // Expect 5 choose 2 number of elements
  EXPECT_EQ(subsets_of_size_2.size(), 10);
  // Expect all subsets to exist
  EXPECT_TRUE(std::find(subsets_of_size_2.begin(), subsets_of_size_2.end(),
                        std::pair<size_t, size_t>(0, 1)) !=
              subsets_of_size_2.end());
  EXPECT_TRUE(std::find(subsets_of_size_2.begin(), subsets_of_size_2.end(),
                        std::pair<size_t, size_t>(0, 2)) !=
              subsets_of_size_2.end());
  EXPECT_TRUE(std::find(subsets_of_size_2.begin(), subsets_of_size_2.end(),
                        std::pair<size_t, size_t>(0, 3)) !=
              subsets_of_size_2.end());
  EXPECT_TRUE(std::find(subsets_of_size_2.begin(), subsets_of_size_2.end(),
                        std::pair<size_t, size_t>(0, 4)) !=
              subsets_of_size_2.end());
  EXPECT_TRUE(std::find(subsets_of_size_2.begin(), subsets_of_size_2.end(),
                        std::pair<size_t, size_t>(1, 2)) !=
              subsets_of_size_2.end());
  EXPECT_TRUE(std::find(subsets_of_size_2.begin(), subsets_of_size_2.end(),
                        std::pair<size_t, size_t>(1, 3)) !=
              subsets_of_size_2.end());
  EXPECT_TRUE(std::find(subsets_of_size_2.begin(), subsets_of_size_2.end(),
                        std::pair<size_t, size_t>(1, 4)) !=
              subsets_of_size_2.end());
  EXPECT_TRUE(std::find(subsets_of_size_2.begin(), subsets_of_size_2.end(),
                        std::pair<size_t, size_t>(2, 3)) !=
              subsets_of_size_2.end());
  EXPECT_TRUE(std::find(subsets_of_size_2.begin(), subsets_of_size_2.end(),
                        std::pair<size_t, size_t>(2, 4)) !=
              subsets_of_size_2.end());
  EXPECT_TRUE(std::find(subsets_of_size_2.begin(), subsets_of_size_2.end(),
                        std::pair<size_t, size_t>(3, 4)) !=
              subsets_of_size_2.end());
}

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
}

// NOLINTEND(clang-diagnostic-unused-result)
