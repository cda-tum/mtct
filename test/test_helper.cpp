#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "EOMHelper.hpp"
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

TEST(Helper, EoMMinimalTravelTime1) {
  // Start at speed 10,
  // accelerate at rate 2 for 5 seconds until maximal speed 20 is reached,
  // keep maximal speed for 6 seconds,
  // decelerate at rate 1.2 for 5 seconds until speed 14 is reached.

  // Total distance travelled is 15*5+20*6+17*5 = 280

  EXPECT_THROW(cda_rail::min_travel_time_from_start(10, 14, 20, 2, 1.2, 10, 0),
               cda_rail::exceptions::ConsistencyException);

  // After 0 seconds the distance travelled is 0
  EXPECT_DOUBLE_EQ(
      cda_rail::min_travel_time_from_start(10, 14, 20, 2, 1.2, 280, 0), 0);

  // After 2 seconds it has reached a speed of 14, hence, travelled 12*2 = 24
  EXPECT_DOUBLE_EQ(
      cda_rail::min_travel_time_from_start(10, 14, 20, 2, 1.2, 280, 24), 2);

  // After 5 seconds it has reached a speed of 20, hence, travelled 15*5 = 75
  EXPECT_DOUBLE_EQ(
      cda_rail::min_travel_time_from_start(10, 14, 20, 2, 1.2, 280, 75), 5);

  // After 8 seconds it travelled additional 3 seconds at maximum speed, hence,
  // 75+20*3 = 135
  EXPECT_DOUBLE_EQ(
      cda_rail::min_travel_time_from_start(10, 14, 20, 2, 1.2, 280, 135), 8);

  // After 11 seconds it travelled 6 seconds at maximum speed, hence, 75+20*6 =
  // 195
  EXPECT_DOUBLE_EQ(
      cda_rail::min_travel_time_from_start(10, 14, 20, 2, 1.2, 280, 195), 11);

  // After 14 seconds it has reaced a speed of 16.4, hence, travelled 195+18.2*3
  // = 249.6
  EXPECT_DOUBLE_EQ(
      cda_rail::min_travel_time_from_start(10, 14, 20, 2, 1.2, 280, 249.6), 14);

  // Finally after 16 seconds it has reached the end, hence, travelled 280
  EXPECT_DOUBLE_EQ(
      cda_rail::min_travel_time_from_start(10, 14, 20, 2, 1.2, 280, 280), 16);
  EXPECT_DOUBLE_EQ(cda_rail::min_travel_time(10, 14, 20, 2, 1.2, 280), 16);
}

TEST(Helper, EoMMinimalTravelTime2) {
  // Train starts with speed 5,
  // accelerates at rate 1.5 for 4 seconds until speed 11 is reached,
  // immediately decelerates at rate 2 for 4 seconds until speed 3 is reached,
  // while maximal speed allowed in principle is 15.

  // Total distance travelled is 8*4+7*4 = 60

  // After 0 seconds the distance travelled is 0
  EXPECT_DOUBLE_EQ(
      cda_rail::min_travel_time_from_start(5, 3, 15, 1.5, 2, 60, 0), 0);

  // After 2 seconds it has reached a speed of 8, hence, travelled 6.5*2 = 13
  EXPECT_DOUBLE_EQ(
      cda_rail::min_travel_time_from_start(5, 3, 15, 1.5, 2, 60, 13), 2);

  // After 4 seconds it has reached a speed of 11, hence, travelled 8*4 = 32
  EXPECT_DOUBLE_EQ(
      cda_rail::min_travel_time_from_start(5, 3, 15, 1.5, 2, 60, 32), 4);

  // After 6 seconds it has reached a speed of 7, hence, travelled 32+9*2 = 50
  EXPECT_DOUBLE_EQ(
      cda_rail::min_travel_time_from_start(5, 3, 15, 1.5, 2, 60, 50), 6);

  // Finally after 8 seconds it has reached the end, hence, travelled 60
  EXPECT_DOUBLE_EQ(
      cda_rail::min_travel_time_from_start(5, 3, 15, 1.5, 2, 60, 60), 8);
  EXPECT_DOUBLE_EQ(cda_rail::min_travel_time(5, 3, 15, 1.5, 2, 60), 8);
}

TEST(Helper, EoMMinimalTravelTime3) {
  // Start at speed 10,
  // decelerates at rate 2 for 4 seconds until speed 2 is reached.
  // Theoretical maximal speed is 15.

  // Total distance travelled is 6*4 = 24

  // After 0 seconds the distance travelled is 0
  EXPECT_DOUBLE_EQ(cda_rail::min_travel_time_from_start(10, 2, 15, 1, 2, 24, 0),
                   0);

  // After 2 seconds it has reached a speed of 6, hence, travelled 8*2 = 16
  EXPECT_DOUBLE_EQ(
      cda_rail::min_travel_time_from_start(10, 2, 15, 1, 2, 24, 16), 2);

  // Finally after 4 seconds it has reached the end, hence, travelled 24
  EXPECT_DOUBLE_EQ(
      cda_rail::min_travel_time_from_start(10, 2, 15, 1, 2, 24, 24), 4);
  EXPECT_DOUBLE_EQ(cda_rail::min_travel_time(10, 2, 15, 1, 2, 24), 4);
}

TEST(Helper, EoMMinimalTravelTime4) {
  // Start at maximal speed 10,
  // stay constant for 4 seconds,
  // decelerates at rate 2 for 4 seconds until speed 2 is reached.
  // Theoretical maximal speed is 10.
  // Theoretical acceleration is 1.

  // Total distance travelled is 10*4 + 6*4 = 64

  // After 0 seconds the distance travelled is 0
  EXPECT_DOUBLE_EQ(cda_rail::min_travel_time_from_start(10, 2, 10, 1, 2, 64, 0),
                   0);

  // After 2 seconds it has travelled 2*10 = 20
  EXPECT_DOUBLE_EQ(
      cda_rail::min_travel_time_from_start(10, 2, 10, 1, 2, 64, 20), 2);

  // After 4 seconds it has travelled 4*10 = 40
  EXPECT_DOUBLE_EQ(
      cda_rail::min_travel_time_from_start(10, 2, 10, 1, 2, 64, 40), 4);

  // After 6 seconds it has reached a speed of 6, hence, travelled 40+8*2 = 56
  EXPECT_DOUBLE_EQ(
      cda_rail::min_travel_time_from_start(10, 2, 10, 1, 2, 64, 56), 6);

  // Finally after 8 seconds it has reached the end, hence, travelled 64
  EXPECT_DOUBLE_EQ(
      cda_rail::min_travel_time_from_start(10, 2, 10, 1, 2, 64, 64), 8);
  EXPECT_DOUBLE_EQ(cda_rail::min_travel_time(10, 2, 10, 1, 2, 64), 8);
}

TEST(Helper, EoMMaximalTravelTimeNoStop1) {
  // Start at speed 10,
  // decelerates to minimal speed 2 at rate 2 for 4 seconds,
  // keeps minimal speed for 6 seconds,
  // accelerates at rate 1.5 for 4 seconds until speed 8 is reached.

  // Total distance travelled is 6*4 + 2*6 + 5*4 = 56

  // After 0 seconds the distance travelled is 0
  EXPECT_DOUBLE_EQ(
      cda_rail::max_travel_time_from_start_no_stopping(10, 8, 2, 1.5, 2, 56, 0),
      0);

  // After 2 seconds it has reached a speed of 6, hence, travelled 8*2 = 16
  EXPECT_DOUBLE_EQ(cda_rail::max_travel_time_from_start_no_stopping(
                       10, 8, 2, 1.5, 2, 56, 16),
                   2);

  // After 4 seconds it has reached a speed of 2, hence, travelled 6*4 = 24
  EXPECT_DOUBLE_EQ(cda_rail::max_travel_time_from_start_no_stopping(
                       10, 8, 2, 1.5, 2, 56, 24),
                   4);

  // After 6 seconds it been constant for 2 seconds, hence, travelled 24+2*2 =
  // 28
  EXPECT_DOUBLE_EQ(cda_rail::max_travel_time_from_start_no_stopping(
                       10, 8, 2, 1.5, 2, 56, 28),
                   6);

  // After 10 seconds it has been constant for 6 seconds, hence, travelled
  // 24+2*6 = 36
  EXPECT_DOUBLE_EQ(cda_rail::max_travel_time_from_start_no_stopping(
                       10, 8, 2, 1.5, 2, 56, 36),
                   10);

  // After 12 seconds it has reached a speed of 5, hence, travelled 36+3.5*2 =
  // 43
  EXPECT_DOUBLE_EQ(cda_rail::max_travel_time_from_start_no_stopping(
                       10, 8, 2, 1.5, 2, 56, 43),
                   12);

  // Finally after 14 seconds it has reached the end, hence, travelled 56
  EXPECT_DOUBLE_EQ(cda_rail::max_travel_time_from_start_no_stopping(
                       10, 8, 2, 1.5, 2, 56, 56),
                   14);
  EXPECT_DOUBLE_EQ(cda_rail::max_travel_time_no_stopping(10, 8, 2, 1.5, 2, 56),
                   14);
}

TEST(Helper, EoMMaximalTravelTimeNoStop2) {
  // Start at speed 10,
  // decelerates to minimal speed 4 at rate 2 for 3 seconds,
  // immediately accelerates at rate 1 for 5 seconds until speed 9 is reached.
  // Theoretical minimal speed is 2.

  // Total distance travelled is 7*3 + 6.5*5 = 53.5

  // After 0 seconds the distance travelled is 0
  EXPECT_DOUBLE_EQ(
      cda_rail::max_travel_time_from_start_no_stopping(10, 9, 2, 1, 2, 53.5, 0),
      0);

  // After 2 seconds it has reached a speed of 6, hence, travelled 8*2 = 16
  EXPECT_DOUBLE_EQ(cda_rail::max_travel_time_from_start_no_stopping(
                       10, 9, 2, 1, 2, 53.5, 16),
                   2);

  // After 3 seconds it has reached a speed of 4, hence, travelled 7*3 = 21
  EXPECT_DOUBLE_EQ(cda_rail::max_travel_time_from_start_no_stopping(
                       10, 9, 2, 1, 2, 53.5, 21),
                   3);

  // After 5 seconds it has reached a speed of 6, hence, travelled 21+5*2 = 31
  EXPECT_DOUBLE_EQ(cda_rail::max_travel_time_from_start_no_stopping(
                       10, 9, 2, 1, 2, 53.5, 31),
                   5);

  // Finally after 8 seconds it has reached the end, hence, travelled 53.5
  EXPECT_DOUBLE_EQ(cda_rail::max_travel_time_from_start_no_stopping(
                       10, 9, 2, 1, 2, 53.5, 53.5),
                   8);
  EXPECT_DOUBLE_EQ(cda_rail::max_travel_time_no_stopping(10, 9, 2, 1, 2, 53.5),
                   8);
}

TEST(Helper, EoMMaximalTravelTimeNoStop3) {
  // Start at speed 10
  // Accelerates at rate 1.5 for 4 seconds until speed 16 is reached
  // Theoretical minimal speed is 2
  // Theoretical deceleration is 2

  // Total distance travelled is 13*4 = 52

  // After 0 seconds the distance travelled is 0
  EXPECT_DOUBLE_EQ(cda_rail::max_travel_time_from_start_no_stopping(
                       10, 16, 2, 1.5, 2, 52, 0),
                   0);

  // After 2 seconds it has reached a speed of 13, hence, travelled 11.5*2 = 23
  EXPECT_DOUBLE_EQ(cda_rail::max_travel_time_from_start_no_stopping(
                       10, 16, 2, 1.5, 2, 52, 23),
                   2);

  // After 4 seconds it has reached the end, hence, travelled 52
  EXPECT_DOUBLE_EQ(cda_rail::max_travel_time_from_start_no_stopping(
                       10, 16, 2, 1.5, 2, 52, 52),
                   4);
  EXPECT_DOUBLE_EQ(cda_rail::max_travel_time_no_stopping(10, 16, 2, 1.5, 2, 52),
                   4);
}

TEST(Helper, EoMMaximalTravelTimeNoStop4) {
  // Train starts at speed 2, which is also the minimal speed
  // It remains at this speed for 4 seconds
  // Then it accelerates at rate 2 for 4 seconds until speed 10 is reached
  // Theoretical deceleration is 3

  // Total distance travelled is 2*4 + 6*4 = 32

  // After 0 seconds the distance travelled is 0
  EXPECT_DOUBLE_EQ(
      cda_rail::max_travel_time_from_start_no_stopping(2, 10, 2, 2, 3, 32, 0),
      0);

  // After 2 seconds it has travelled 2*2 = 4
  EXPECT_DOUBLE_EQ(
      cda_rail::max_travel_time_from_start_no_stopping(2, 10, 2, 2, 3, 32, 4),
      2);

  // After 4 seconds it has travelled 2*4 = 8
  EXPECT_DOUBLE_EQ(
      cda_rail::max_travel_time_from_start_no_stopping(2, 10, 2, 2, 3, 32, 8),
      4);

  // After 6 seconds it has reached a speed of 6, hence, travelled 8+4*2 = 16
  EXPECT_DOUBLE_EQ(
      cda_rail::max_travel_time_from_start_no_stopping(2, 10, 2, 2, 3, 32, 16),
      6);

  // After 8 seconds it has reached the end, hence, travelled 32
  EXPECT_DOUBLE_EQ(
      cda_rail::max_travel_time_from_start_no_stopping(2, 10, 2, 2, 3, 32, 32),
      8);
  EXPECT_DOUBLE_EQ(cda_rail::max_travel_time_no_stopping(2, 10, 2, 2, 3, 32),
                   8);
}

// NOLINTEND(clang-diagnostic-unused-result)
