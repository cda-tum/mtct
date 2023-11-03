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

TEST(Exceptions, Content) {
  const auto e1 = cda_rail::exceptions::ModelCreationException();
  EXPECT_EQ(std::string(e1.what()), "Model creation failed.");
  const auto e2 = cda_rail::exceptions::ModelCreationException("test2");
  EXPECT_EQ(std::string(e2.what()), "test2");

  const auto e3 = cda_rail::exceptions::ExportException();
  EXPECT_EQ(std::string(e3.what()), "Export failed.");
  const auto e4 = cda_rail::exceptions::ExportException("test4");
  EXPECT_EQ(std::string(e4.what()), "test4");

  const auto e5 = cda_rail::exceptions::ConsistencyException();
  EXPECT_EQ(std::string(e5.what()), "Consistency check failed.");
  const auto e6 = cda_rail::exceptions::ConsistencyException("test6");
  EXPECT_EQ(std::string(e6.what()), "test6");

  const auto e5b = cda_rail::exceptions::InvalidInputException();
  EXPECT_EQ(std::string(e5b.what()), "Invalid input.");
  const auto e6b = cda_rail::exceptions::InvalidInputException("test6b");
  EXPECT_EQ(std::string(e6b.what()), "test6b");

  const auto e7 = cda_rail::exceptions::ImportException();
  EXPECT_EQ(std::string(e7.what()), "Import failed.");
  const auto e8 = cda_rail::exceptions::ImportException("test8");
  EXPECT_EQ(std::string(e8.what()), "Import of test8 failed.");

  const auto e9 = cda_rail::exceptions::VertexNotExistentException();
  EXPECT_EQ(std::string(e9.what()), "Some vertex specified does not exist.");
  const auto e10 = cda_rail::exceptions::VertexNotExistentException(10);
  EXPECT_EQ(std::string(e10.what()), "Vertex with ID 10 does not exist");

  const auto e11 = cda_rail::exceptions::EdgeNotExistentException();
  EXPECT_EQ(std::string(e11.what()), "Some edge specified does not exist.");
  const auto e12 = cda_rail::exceptions::EdgeNotExistentException(12);
  EXPECT_EQ(std::string(e12.what()), "Edge with ID 12 does not exist.");
  const auto e13 = cda_rail::exceptions::EdgeNotExistentException(12, 13);
  EXPECT_EQ(std::string(e13.what()),
            "Edge connecting vertices with IDs 12->13 does not exist.");
  const auto e14 = cda_rail::exceptions::EdgeNotExistentException("v12", "v14");
  EXPECT_EQ(std::string(e14.what()),
            "Edge connecting v12->v14 does not exist.");

  const auto e15 = cda_rail::exceptions::TrainNotExistentException();
  EXPECT_EQ(std::string(e15.what()), "Some train specified does not exist.");
  const auto e16 = cda_rail::exceptions::TrainNotExistentException(16);
  EXPECT_EQ(std::string(e16.what()), "Train with ID 16 does not exist.");

  const auto e17 = cda_rail::exceptions::StationNotExistentException();
  EXPECT_EQ(std::string(e17.what()), "Some station specified does not exist.");
  const auto e18 = cda_rail::exceptions::StationNotExistentException("S18");
  EXPECT_EQ(std::string(e18.what()), "Station S18 does not exist.");

  const auto e19 = cda_rail::exceptions::ScheduleNotExistentException();
  EXPECT_EQ(std::string(e19.what()), "Some schedule specified does not exist.");
  const auto e20 = cda_rail::exceptions::ScheduleNotExistentException(20);
  EXPECT_EQ(std::string(e20.what()), "Schedule with ID 20 does not exist.");
  const auto e21 = cda_rail::exceptions::ScheduleNotExistentException("S21");
  EXPECT_EQ(std::string(e21.what()), "Schedule S21 does not exist.");
}

// NOLINTEND(clang-diagnostic-unused-result)
