#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "EOMHelper.hpp"
#include "VSSModel.hpp"

#include "gtest/gtest.h"
#include <cmath>
#include <iostream>
#include <limits>

#define EXPECT_APPROX_EQ(a, b)                                                 \
  EXPECT_TRUE(std::abs((a) - (b)) < 1e-6) << (a) << " !=(approx.) " << (b)

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
  EXPECT_APPROX_EQ(
      cda_rail::min_travel_time_from_start(10, 14, 20, 2, 1.2, 280, 0), 0);

  // After 2 seconds it has reached a speed of 14, hence, travelled 12*2 = 24
  EXPECT_APPROX_EQ(
      cda_rail::min_travel_time_from_start(10, 14, 20, 2, 1.2, 280, 24), 2);

  // After 5 seconds it has reached a speed of 20, hence, travelled 15*5 = 75
  EXPECT_APPROX_EQ(
      cda_rail::min_travel_time_from_start(10, 14, 20, 2, 1.2, 280, 75), 5);

  // After 8 seconds it travelled additional 3 seconds at maximum speed, hence,
  // 75+20*3 = 135
  EXPECT_APPROX_EQ(
      cda_rail::min_travel_time_from_start(10, 14, 20, 2, 1.2, 280, 135), 8);

  // After 11 seconds it travelled 6 seconds at maximum speed, hence, 75+20*6 =
  // 195
  EXPECT_APPROX_EQ(
      cda_rail::min_travel_time_from_start(10, 14, 20, 2, 1.2, 280, 195), 11);

  // After 14 seconds it has reaced a speed of 16.4, hence, travelled 195+18.2*3
  // = 249.6
  EXPECT_APPROX_EQ(
      cda_rail::min_travel_time_from_start(10, 14, 20, 2, 1.2, 280, 249.6), 14);

  // Finally after 16 seconds it has reached the end, hence, travelled 280
  EXPECT_APPROX_EQ(
      cda_rail::min_travel_time_from_start(10, 14, 20, 2, 1.2, 280, 280), 16);
  EXPECT_APPROX_EQ(cda_rail::min_travel_time(10, 14, 20, 2, 1.2, 280), 16);
}

TEST(Helper, EoMMinimalTravelTime2) {
  // Train starts with speed 5,
  // accelerates at rate 1.5 for 4 seconds until speed 11 is reached,
  // immediately decelerates at rate 2 for 4 seconds until speed 3 is reached,
  // while maximal speed allowed in principle is 15.

  // Total distance travelled is 8*4+7*4 = 60

  // After 0 seconds the distance travelled is 0
  EXPECT_APPROX_EQ(
      cda_rail::min_travel_time_from_start(5, 3, 15, 1.5, 2, 60, 0), 0);

  // After 2 seconds it has reached a speed of 8, hence, travelled 6.5*2 = 13
  EXPECT_APPROX_EQ(
      cda_rail::min_travel_time_from_start(5, 3, 15, 1.5, 2, 60, 13), 2);

  // After 4 seconds it has reached a speed of 11, hence, travelled 8*4 = 32
  EXPECT_APPROX_EQ(
      cda_rail::min_travel_time_from_start(5, 3, 15, 1.5, 2, 60, 32), 4);

  // After 6 seconds it has reached a speed of 7, hence, travelled 32+9*2 = 50
  EXPECT_APPROX_EQ(
      cda_rail::min_travel_time_from_start(5, 3, 15, 1.5, 2, 60, 50), 6);

  // Finally after 8 seconds it has reached the end, hence, travelled 60
  EXPECT_APPROX_EQ(
      cda_rail::min_travel_time_from_start(5, 3, 15, 1.5, 2, 60, 60), 8);
  EXPECT_APPROX_EQ(cda_rail::min_travel_time(5, 3, 15, 1.5, 2, 60), 8);
}

TEST(Helper, EoMMinimalTravelTime3) {
  // Start at speed 10,
  // decelerates at rate 2 for 4 seconds until speed 2 is reached.
  // Theoretical maximal speed is 15.

  // Total distance travelled is 6*4 = 24

  // After 0 seconds the distance travelled is 0
  EXPECT_APPROX_EQ(cda_rail::min_travel_time_from_start(10, 2, 15, 1, 2, 24, 0),
                   0);

  // After 2 seconds it has reached a speed of 6, hence, travelled 8*2 = 16
  EXPECT_APPROX_EQ(
      cda_rail::min_travel_time_from_start(10, 2, 15, 1, 2, 24, 16), 2);

  // Finally after 4 seconds it has reached the end, hence, travelled 24
  EXPECT_APPROX_EQ(
      cda_rail::min_travel_time_from_start(10, 2, 15, 1, 2, 24, 24), 4);
  EXPECT_APPROX_EQ(cda_rail::min_travel_time(10, 2, 15, 1, 2, 24), 4);
}

TEST(Helper, EoMMinimalTravelTime4) {
  // Start at maximal speed 10,
  // stay constant for 4 seconds,
  // decelerates at rate 2 for 4 seconds until speed 2 is reached.
  // Theoretical maximal speed is 10.
  // Theoretical acceleration is 1.

  // Total distance travelled is 10*4 + 6*4 = 64

  // After 0 seconds the distance travelled is 0
  EXPECT_APPROX_EQ(cda_rail::min_travel_time_from_start(10, 2, 10, 1, 2, 64, 0),
                   0);

  // After 2 seconds it has travelled 2*10 = 20
  EXPECT_APPROX_EQ(
      cda_rail::min_travel_time_from_start(10, 2, 10, 1, 2, 64, 20), 2);

  // After 4 seconds it has travelled 4*10 = 40
  EXPECT_APPROX_EQ(
      cda_rail::min_travel_time_from_start(10, 2, 10, 1, 2, 64, 40), 4);

  // After 6 seconds it has reached a speed of 6, hence, travelled 40+8*2 = 56
  EXPECT_APPROX_EQ(
      cda_rail::min_travel_time_from_start(10, 2, 10, 1, 2, 64, 56), 6);

  // Finally after 8 seconds it has reached the end, hence, travelled 64
  EXPECT_APPROX_EQ(
      cda_rail::min_travel_time_from_start(10, 2, 10, 1, 2, 64, 64), 8);
  EXPECT_APPROX_EQ(cda_rail::min_travel_time(10, 2, 10, 1, 2, 64), 8);
}

TEST(Helper, EoMMaximalTravelTimeNoStop1) {
  // Start at speed 10,
  // decelerates to minimal speed 2 at rate 2 for 4 seconds,
  // keeps minimal speed for 6 seconds,
  // accelerates at rate 1.5 for 4 seconds until speed 8 is reached.

  // Total distance travelled is 6*4 + 2*6 + 5*4 = 56

  // After 0 seconds the distance travelled is 0
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_from_start_no_stopping(10, 8, 2, 1.5, 2, 56, 0),
      0);
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_from_start(10, 8, 2, 1.5, 2, 56, 0, false), 0);

  // After 2 seconds it has reached a speed of 6, hence, travelled 8*2 = 16
  EXPECT_APPROX_EQ(cda_rail::max_travel_time_from_start_no_stopping(
                       10, 8, 2, 1.5, 2, 56, 16),
                   2);
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_from_start(10, 8, 2, 1.5, 2, 56, 16, false), 2);

  // After 4 seconds it has reached a speed of 2, hence, travelled 6*4 = 24
  EXPECT_APPROX_EQ(cda_rail::max_travel_time_from_start_no_stopping(
                       10, 8, 2, 1.5, 2, 56, 24),
                   4);
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_from_start(10, 8, 2, 1.5, 2, 56, 24, false), 4);

  // After 6 seconds it been constant for 2 seconds, hence, travelled 24+2*2 =
  // 28
  EXPECT_APPROX_EQ(cda_rail::max_travel_time_from_start_no_stopping(
                       10, 8, 2, 1.5, 2, 56, 28),
                   6);
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_from_start(10, 8, 2, 1.5, 2, 56, 28, false), 6);

  // After 10 seconds it has been constant for 6 seconds, hence, travelled
  // 24+2*6 = 36
  EXPECT_APPROX_EQ(cda_rail::max_travel_time_from_start_no_stopping(
                       10, 8, 2, 1.5, 2, 56, 36),
                   10);
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_from_start(10, 8, 2, 1.5, 2, 56, 36, false),
      10);

  // After 12 seconds it has reached a speed of 5, hence, travelled 36+3.5*2 =
  // 43
  EXPECT_APPROX_EQ(cda_rail::max_travel_time_from_start_no_stopping(
                       10, 8, 2, 1.5, 2, 56, 43),
                   12);
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_from_start(10, 8, 2, 1.5, 2, 56, 43, false),
      12);

  // Finally after 14 seconds it has reached the end, hence, travelled 56
  EXPECT_APPROX_EQ(cda_rail::max_travel_time_from_start_no_stopping(
                       10, 8, 2, 1.5, 2, 56, 56),
                   14);
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_from_start(10, 8, 2, 1.5, 2, 56, 56, false),
      14);
  EXPECT_APPROX_EQ(cda_rail::max_travel_time_no_stopping(10, 8, 2, 1.5, 2, 56),
                   14);
  EXPECT_APPROX_EQ(cda_rail::max_travel_time(10, 8, 2, 1.5, 2, 56, false), 14);
}

TEST(Helper, EoMMaximalTravelTimeNoStop2) {
  // Start at speed 10,
  // decelerates to minimal speed 4 at rate 2 for 3 seconds,
  // immediately accelerates at rate 1 for 5 seconds until speed 9 is reached.
  // Theoretical minimal speed is 2.

  // Total distance travelled is 7*3 + 6.5*5 = 53.5

  // After 0 seconds the distance travelled is 0
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_from_start_no_stopping(10, 9, 2, 1, 2, 53.5, 0),
      0);
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_from_start(10, 9, 2, 1, 2, 53.5, 0, false), 0);

  // After 2 seconds it has reached a speed of 6, hence, travelled 8*2 = 16
  EXPECT_APPROX_EQ(cda_rail::max_travel_time_from_start_no_stopping(
                       10, 9, 2, 1, 2, 53.5, 16),
                   2);
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_from_start(10, 9, 2, 1, 2, 53.5, 16, false), 2);

  // After 3 seconds it has reached a speed of 4, hence, travelled 7*3 = 21
  EXPECT_APPROX_EQ(cda_rail::max_travel_time_from_start_no_stopping(
                       10, 9, 2, 1, 2, 53.5, 21),
                   3);
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_from_start(10, 9, 2, 1, 2, 53.5, 21, false), 3);

  // After 5 seconds it has reached a speed of 6, hence, travelled 21+5*2 = 31
  EXPECT_APPROX_EQ(cda_rail::max_travel_time_from_start_no_stopping(
                       10, 9, 2, 1, 2, 53.5, 31),
                   5);
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_from_start(10, 9, 2, 1, 2, 53.5, 31, false), 5);

  // Finally after 8 seconds it has reached the end, hence, travelled 53.5
  EXPECT_APPROX_EQ(cda_rail::max_travel_time_from_start_no_stopping(
                       10, 9, 2, 1, 2, 53.5, 53.5),
                   8);
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_from_start(10, 9, 2, 1, 2, 53.5, 53.5, false),
      8);
  EXPECT_APPROX_EQ(cda_rail::max_travel_time_no_stopping(10, 9, 2, 1, 2, 53.5),
                   8);
  EXPECT_APPROX_EQ(cda_rail::max_travel_time(10, 9, 2, 1, 2, 53.5, false), 8);
}

TEST(Helper, EoMMaximalTravelTimeNoStop3) {
  // Start at speed 10
  // Accelerates at rate 1.5 for 4 seconds until speed 16 is reached
  // Theoretical minimal speed is 2
  // Theoretical deceleration is 2

  // Total distance travelled is 13*4 = 52

  // After 0 seconds the distance travelled is 0
  EXPECT_APPROX_EQ(cda_rail::max_travel_time_from_start_no_stopping(
                       10, 16, 2, 1.5, 2, 52, 0),
                   0);
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_from_start(10, 16, 2, 1.5, 2, 52, 0, false), 0);

  // After 2 seconds it has reached a speed of 13, hence, travelled 11.5*2 = 23
  EXPECT_APPROX_EQ(cda_rail::max_travel_time_from_start_no_stopping(
                       10, 16, 2, 1.5, 2, 52, 23),
                   2);
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_from_start(10, 16, 2, 1.5, 2, 52, 23, false),
      2);

  // After 4 seconds it has reached the end, hence, travelled 52
  EXPECT_APPROX_EQ(cda_rail::max_travel_time_from_start_no_stopping(
                       10, 16, 2, 1.5, 2, 52, 52),
                   4);
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_from_start(10, 16, 2, 1.5, 2, 52, 52, false),
      4);
  EXPECT_APPROX_EQ(cda_rail::max_travel_time_no_stopping(10, 16, 2, 1.5, 2, 52),
                   4);
  EXPECT_APPROX_EQ(cda_rail::max_travel_time(10, 16, 2, 1.5, 2, 52, false), 4);
}

TEST(Helper, EoMMaximalTravelTimeNoStop4) {
  // Train starts at speed 2, which is also the minimal speed
  // It remains at this speed for 4 seconds
  // Then it accelerates at rate 2 for 4 seconds until speed 10 is reached
  // Theoretical deceleration is 3

  // Total distance travelled is 2*4 + 6*4 = 32

  // After 0 seconds the distance travelled is 0
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_from_start_no_stopping(2, 10, 2, 2, 3, 32, 0),
      0);
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_from_start(2, 10, 2, 2, 3, 32, 0, false), 0);

  // After 2 seconds it has travelled 2*2 = 4
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_from_start_no_stopping(2, 10, 2, 2, 3, 32, 4),
      2);
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_from_start(2, 10, 2, 2, 3, 32, 4, false), 2);

  // After 4 seconds it has travelled 2*4 = 8
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_from_start_no_stopping(2, 10, 2, 2, 3, 32, 8),
      4);
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_from_start(2, 10, 2, 2, 3, 32, 8, false), 4);

  // After 6 seconds it has reached a speed of 6, hence, travelled 8+4*2 = 16
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_from_start_no_stopping(2, 10, 2, 2, 3, 32, 16),
      6);
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_from_start(2, 10, 2, 2, 3, 32, 16, false), 6);

  // After 8 seconds it has reached the end, hence, travelled 32
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_from_start_no_stopping(2, 10, 2, 2, 3, 32, 32),
      8);
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_from_start(2, 10, 2, 2, 3, 32, 32, false), 8);
  EXPECT_APPROX_EQ(cda_rail::max_travel_time_no_stopping(2, 10, 2, 2, 3, 32),
                   8);
  EXPECT_APPROX_EQ(cda_rail::max_travel_time(2, 10, 2, 2, 3, 32, false), 8);
}

TEST(Helper, EoMMaximalTravelTimeNoStop5) {
  // Train starts at speed 0.
  // Hence, it accelerates to minimal speed 2 at rate 1 for 2 second.
  // Then, it stays at this speed for 4 seconds.
  // Finally, it decelerates at rate 2 for 1 second until it stops.

  // Total distance travelled is 1*2 + 2*4 + 1*1 = 11

  // After 0 seconds the distance travelled is 0
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_from_start_no_stopping(0, 0, 2, 1, 2, 11, 0),
      0);
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_from_start(0, 0, 2, 1, 2, 11, 0, false), 0);

  // After 1 seconds it has reached a speed of 1, hence, travelled 0.5*1 = 0.5
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_from_start_no_stopping(0, 0, 2, 1, 2, 11, 0.5),
      1);
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_from_start(0, 0, 2, 1, 2, 11, 0.5, false), 1);

  // After 2 seconds it has reached a speed of 2, hence, travelled 1*2 = 2
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_from_start_no_stopping(0, 0, 2, 1, 2, 11, 2),
      2);
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_from_start(0, 0, 2, 1, 2, 11, 2, false), 2);

  // After 4 seconds it has travelled 2 + 2*2 = 6
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_from_start_no_stopping(0, 0, 2, 1, 2, 11, 6),
      4);
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_from_start(0, 0, 2, 1, 2, 11, 6, false), 4);

  // After 6 seconds it has travelled 2 + 2*4 = 10
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_from_start_no_stopping(0, 0, 2, 1, 2, 11, 10),
      6);
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_from_start(0, 0, 2, 1, 2, 11, 10, false), 6);

  // After 6.5 seconds it has reached speed 1, hence travelled 10 + 1.5*0.5
  // = 10.75
  EXPECT_APPROX_EQ(cda_rail::max_travel_time_from_start_no_stopping(
                       0, 0, 2, 1, 2, 11, 10.75),
                   6.5);
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_from_start(0, 0, 2, 1, 2, 11, 10.75, false),
      6.5);

  // Finally after 7 seconds it has reached the end, hence, travelled 11
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_from_start_no_stopping(0, 0, 2, 1, 2, 11, 11),
      7);
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_from_start(0, 0, 2, 1, 2, 11, 11, false), 7);
  EXPECT_APPROX_EQ(cda_rail::max_travel_time_no_stopping(0, 0, 2, 1, 2, 11), 7);
  EXPECT_APPROX_EQ(cda_rail::max_travel_time(0, 0, 2, 1, 2, 11, false), 7);
}

TEST(Helper, EoMMaximalTravelTimeNoStop6) {
  // Train starts at speed 0
  // Accelerates to speed 4 at rate 2 for 2 seconds
  // Minimal speed is 5
  // Yet it decelerates at rate 1 for 2 seconds until speed 2 is reached

  // Total distance travelled is 2*2 + 3*2 = 10

  // After 0 seconds the distance travelled is 0
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_from_start_no_stopping(0, 2, 5, 2, 1, 10, 0),
      0);
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_from_start(0, 2, 5, 2, 1, 10, 0, false), 0);

  // After 1 seconds it has reached a speed of 2, hence, travelled 1*1 = 1
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_from_start_no_stopping(0, 2, 5, 2, 1, 10, 1),
      1);
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_from_start(0, 2, 5, 2, 1, 10, 1, false), 1);

  // After 2 seconds it has reached a speed of 4, hence, travelled 2*2 = 4
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_from_start_no_stopping(0, 2, 5, 2, 1, 10, 4),
      2);
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_from_start(0, 2, 5, 2, 1, 10, 4, false), 2);

  // After 3 seconds it has reached a speed of 3, hence, travelled 4+3.5*1 = 7.5
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_from_start_no_stopping(0, 2, 5, 2, 1, 10, 7.5),
      3);
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_from_start(0, 2, 5, 2, 1, 10, 7.5, false), 3);

  // Finally after 4 seconds it has reached the end, hence, travelled 10
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_from_start_no_stopping(0, 2, 5, 2, 1, 10, 10),
      4);
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_from_start(0, 2, 5, 2, 1, 10, 10, false), 4);
  EXPECT_APPROX_EQ(cda_rail::max_travel_time_no_stopping(0, 2, 5, 2, 1, 10), 4);
  EXPECT_APPROX_EQ(cda_rail::max_travel_time(0, 2, 5, 2, 1, 10, false), 4);
}

TEST(Helper, EoMMaximalTravelTimeNoStop7) {
  // Train starts at speed 11
  // It decelerates to minimal speed 1 at rate 1 for 10 seconds
  // traveling a distance of 6*10 = 60
  // It remains constant for 2 seconds traveling 2*1 = 2
  // It then stops in 1 second traveling 0.5*1 = 0.5
  // Total distance travelled is 60 + 2 + 0.5 = 62.5

  // After 0 seconds the distance travelled is 0
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_from_start_no_stopping(11, 0, 1, 1, 1, 62.5, 0),
      0);

  // After 10 seconds the distance travelled is 60
  EXPECT_APPROX_EQ(cda_rail::max_travel_time_from_start_no_stopping(
                       11, 0, 1, 1, 1, 62.5, 60),
                   10);

  // After 12 seconds the distance travelled is 62
  EXPECT_APPROX_EQ(cda_rail::max_travel_time_from_start_no_stopping(
                       11, 0, 1, 1, 1, 62.5, 62),
                   12);

  // Finally after 13 seconds it has reached the end, hence, travelled 62.5
  EXPECT_APPROX_EQ(cda_rail::max_travel_time_from_start_no_stopping(
                       11, 0, 1, 1, 1, 62.5, 62.5),
                   13);
  EXPECT_APPROX_EQ(cda_rail::max_travel_time_no_stopping(11, 0, 1, 1, 1, 62.5),
                   13);
}

TEST(Helper, EoMMaximalTravelTimeNoStop8) {
  // Train starts at speed 8
  // It decelerates to minimal speed 0.3 at rate 2 for 7.7/2= 3.85 seconds
  // Traveling 8.3/2 * 3.85 = 15.9775
  // It remains constant for 1 second traveling 0.3
  // It then stops in 0.15 seconds traveling 0.15*0.15 = 0.0225
  // Total distance travelled is 15.9775 + 0.3 + 0.0225 = 16.3
  // Total time is 3.85 + 1 + 0.15 = 5
  EXPECT_APPROX_EQ(cda_rail::max_travel_time_no_stopping(8, 0, 0.3, 1, 2, 16.3),
                   5);
}

TEST(Helper, EoMMinimalTravelTimeToEnd) {
  // Start at speed 10,
  // accelerate at rate 2 for 5 seconds until maximal speed 20 is reached,
  // keep maximal speed for 6 seconds,
  // decelerate at rate 1.2 for 5 seconds until speed 14 is reached.

  // Total distance travelled is 15*5+20*6+17*5 = 280

  EXPECT_THROW(cda_rail::min_travel_time_to_end(10, 14, 20, 2, 1.2, 10, 0),
               cda_rail::exceptions::ConsistencyException);

  // After 0 seconds the distance travelled is 0, 16 seconds left
  EXPECT_APPROX_EQ(cda_rail::min_travel_time_to_end(10, 14, 20, 2, 1.2, 280, 0),
                   16);

  // After 2 seconds it has reached a speed of 14, hence, travelled 12*2 = 24,
  // 14 seconds left
  EXPECT_APPROX_EQ(
      cda_rail::min_travel_time_to_end(10, 14, 20, 2, 1.2, 280, 24), 14);

  // After 5 seconds it has reached a speed of 20, hence, travelled 15*5 = 75,
  // 11 seconds left
  EXPECT_APPROX_EQ(
      cda_rail::min_travel_time_to_end(10, 14, 20, 2, 1.2, 280, 75), 11);

  // After 8 seconds it travelled additional 3 seconds at maximum speed, hence,
  // 75+20*3 = 135, 8 seconds left
  EXPECT_APPROX_EQ(
      cda_rail::min_travel_time_to_end(10, 14, 20, 2, 1.2, 280, 135), 8);

  // After 11 seconds it travelled 6 seconds at maximum speed, hence, 75+20*6 =
  // 195, 5 seconds left
  EXPECT_APPROX_EQ(
      cda_rail::min_travel_time_to_end(10, 14, 20, 2, 1.2, 280, 195), 5);

  // After 14 seconds it has reaced a speed of 16.4, hence, travelled 195+18.2*3
  // = 249.6, 2 seconds left
  EXPECT_APPROX_EQ(
      cda_rail::min_travel_time_to_end(10, 14, 20, 2, 1.2, 280, 249.6), 2);

  // Finally after 16 seconds it has reached the end, hence, travelled 280, 0
  // seconds left
  EXPECT_APPROX_EQ(
      cda_rail::min_travel_time_to_end(10, 14, 20, 2, 1.2, 280, 280), 0);
}

TEST(Helper, EoMMaximalTravelTimeToEndNoStopping) {
  // Start at speed 10,
  // decelerates to minimal speed 2 at rate 2 for 4 seconds,
  // keeps minimal speed for 6 seconds,
  // accelerates at rate 1.5 for 4 seconds until speed 8 is reached.

  // Total distance travelled is 6*4 + 2*6 + 5*4 = 56

  // After 0 seconds the distance travelled is 0, 14 seconds left
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_to_end_no_stopping(10, 8, 2, 1.5, 2, 56, 0),
      14);
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_to_end(10, 8, 2, 1.5, 2, 56, 0, false), 14);

  // After 2 seconds it has reached a speed of 6, hence, travelled 8*2 = 16, 12
  // seconds left
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_to_end_no_stopping(10, 8, 2, 1.5, 2, 56, 16),
      12);
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_to_end(10, 8, 2, 1.5, 2, 56, 16, false), 12);

  // After 4 seconds it has reached a speed of 2, hence, travelled 6*4 = 24, 10
  // seconds left
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_to_end_no_stopping(10, 8, 2, 1.5, 2, 56, 24),
      10);
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_to_end(10, 8, 2, 1.5, 2, 56, 24, false), 10);

  // After 6 seconds it been constant for 2 seconds, hence, travelled 24+2*2 =
  // 28, 8 seconds left
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_to_end_no_stopping(10, 8, 2, 1.5, 2, 56, 28),
      8);
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_to_end(10, 8, 2, 1.5, 2, 56, 28, false), 8);

  // After 10 seconds it has been constant for 6 seconds, hence, travelled
  // 24+2*6 = 36, 4 seconds left
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_to_end_no_stopping(10, 8, 2, 1.5, 2, 56, 36),
      4);
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_to_end(10, 8, 2, 1.5, 2, 56, 36, false), 4);

  // After 12 seconds it has reached a speed of 5, hence, travelled 36+3.5*2 =
  // 43, 2 seconds left
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_to_end_no_stopping(10, 8, 2, 1.5, 2, 56, 43),
      2);
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_to_end(10, 8, 2, 1.5, 2, 56, 43, false), 2);

  // Finally after 14 seconds it has reached the end, hence, travelled 56, 0
  // seconds left
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_to_end_no_stopping(10, 8, 2, 1.5, 2, 56, 56),
      0);
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_to_end(10, 8, 2, 1.5, 2, 56, 56, false), 0);
}

TEST(Helper, EoMMaximalTravelTimeStopping) {
  // Start at speed 10,
  // decelerates at rate 2 for 5 seconds until full stop
  // accelerates at rate 1 for 5 seconds until speed 5 is reached
  // Deceleration distance is 5*5 = 25
  // Acceleration distance is 2.5*5 = 12.5

  // Total distance travelled is at least 25+12.5 = 37.5, e.g., 40

  // After 0 seconds the distance travelled is 0
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_from_start_stopping_allowed(10, 5, 1, 2, 40, 0),
      0);
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_from_start(10, 5, 1, 1, 2, 40, 0, true), 0);

  // After 2 seconds it has reached a speed of 6, hence, travelled 8*2 = 16
  EXPECT_APPROX_EQ(cda_rail::max_travel_time_from_start_stopping_allowed(
                       10, 5, 1, 2, 40, 16),
                   2);
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_from_start(10, 5, 1, 1, 2, 40, 16, true), 2);

  // After 25m it came to a full stop
  EXPECT_DOUBLE_EQ(cda_rail::max_travel_time_from_start_stopping_allowed(
                       10, 5, 1, 2, 40, 25),
                   std::numeric_limits<double>::infinity());
  EXPECT_DOUBLE_EQ(
      cda_rail::max_travel_time_from_start(10, 5, 1, 1, 2, 40, 25, true),
      std::numeric_limits<double>::infinity());

  // Going backwards at 40-12.5 = 27.5 train can still stop
  EXPECT_DOUBLE_EQ(
      cda_rail::max_travel_time_to_end_stopping_allowed(10, 5, 1, 2, 40, 27.5),
      std::numeric_limits<double>::infinity());
  EXPECT_DOUBLE_EQ(
      cda_rail::max_travel_time_to_end(10, 5, 1, 1, 2, 40, 27.5, true),
      std::numeric_limits<double>::infinity());

  // 2 seconds before the end, the train has speed 3. It will travel 4*2 = 8m,
  // hence is at 40-8 = 32
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_to_end_stopping_allowed(10, 5, 1, 2, 40, 32),
      2);
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_to_end(10, 5, 1, 1, 2, 40, 32, true), 2);

  // At 40m the train is already at the end
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_to_end_stopping_allowed(10, 5, 1, 2, 40, 40),
      0);
  EXPECT_APPROX_EQ(
      cda_rail::max_travel_time_to_end(10, 5, 1, 1, 2, 40, 40, true), 0);
}

TEST(Helper, EoMMinimalTimePushMA) {
  // Start at speed 10 with a = 2 and d = 1
  // Current braking distance is 10*10/2 = 50

  // First, if speed remains constant, the ma is pushed forward with speed 10
  // After 2 seconds it moved 10*2 = 20
  EXPECT_APPROX_EQ(cda_rail::min_time_to_push_ma_forward(10, 0, 1, 20), 2);

  // After 2 seconds speed 14 is reached, hence travelled 12*2 = 24
  // New braking distance is 14*14/2 = 98
  // MA is 24+98 = 122 before initial point
  // Hence, braking overlap is 122 - 50 = 72
  EXPECT_APPROX_EQ(cda_rail::min_time_to_push_ma_forward(10, 2, 1, 72), 2);

  // After 10 seconds speed 30 is reached, hence travelled 20*10 = 200
  // New braking distance is 30*30/2 = 450
  // MA is 200+450 = 650 before initial point
  // Hence, braking overlap is 650 - 50 = 600
  EXPECT_APPROX_EQ(cda_rail::min_time_to_push_ma_forward(10, 2, 1, 600), 10);
}

TEST(Helper, EoMMinimalTimeMA) {
  // Train starts at v_1 = 16, a = 3, d = 1
  // It accelerates for 2 seconds to reach speed 22, which is also maximal
  // After 2 seconds it has travelled 19*2 = 38
  // It remains constant for 2 seconds and travels 22*2 = 44
  // It decelerates 2 seconds to final speed 20
  // Deceleration distance is 21*2 = 42
  // Total distance travelled is 38+44+42 = 124 within 6 seconds
  // Breaking distance at end is 20*20/2 = 200
  // Breaking distance at start is 16*16/2 = 128, i.e., 4 after end

  EXPECT_APPROX_EQ(cda_rail::min_travel_time(16, 20, 22, 3, 1, 124), 6);

  // If start is MA point, then obd is 200-4 = 196
  EXPECT_APPROX_EQ(
      cda_rail::min_time_from_front_to_ma_point(16, 20, 22, 3, 1, 124, 196), 0);
  EXPECT_APPROX_EQ(cda_rail::min_time_profile_from_rear_to_ma_point(
                       16, 20, 22, 3, 1, 124, 196),
                   6);

  // After 1 second it has reached speed 19, hence travelled 17.5*1 = 17.5
  // Its braking distance is 19*19/2 = 180.5
  // Hence, MA is at 17.5+180.5 = 198, i.e., 198-124 = 74 after end
  // Then obd is 200 - 74 = 126
  // To end this is 6-1 = 5
  EXPECT_APPROX_EQ(
      cda_rail::min_time_from_front_to_ma_point(16, 20, 22, 3, 1, 124, 126), 1);
  EXPECT_APPROX_EQ(cda_rail::min_time_profile_from_rear_to_ma_point(
                       16, 20, 22, 3, 1, 124, 126),
                   5);

  // After 2 seconds it has travelled 38
  // Braking distance is 22*22/2 = 242
  // Hence, MA is at 38+242 = 280, i.e., 280-124 = 156 after end
  // Then obd is 200 - 156 = 44
  // To end this is 6-2 = 4
  EXPECT_APPROX_EQ(
      cda_rail::min_time_from_front_to_ma_point(16, 20, 22, 3, 1, 124, 44), 2);
  EXPECT_APPROX_EQ(cda_rail::min_time_profile_from_rear_to_ma_point(
                       16, 20, 22, 3, 1, 124, 44),
                   4);

  // After 3 seconds it has travelled additional 22, i.e., 38+22 = 60
  // Braking distance is 22*22/2 = 242
  // Hence, MA is at 60+242 = 302, i.e., 302-124 = 178 after end
  // Then obd is 200 - 178 = 22
  // To end this is 6-3 = 3
  EXPECT_APPROX_EQ(
      cda_rail::min_time_from_front_to_ma_point(16, 20, 22, 3, 1, 124, 22), 3);
  EXPECT_APPROX_EQ(cda_rail::min_time_profile_from_rear_to_ma_point(
                       16, 20, 22, 3, 1, 124, 22),
                   3);

  // If obd is 0, then result to back is 0
  EXPECT_APPROX_EQ(
      cda_rail::min_time_from_front_to_ma_point(16, 20, 22, 3, 1, 124, 0), 6);
  EXPECT_APPROX_EQ(cda_rail::min_time_profile_from_rear_to_ma_point(
                       16, 20, 22, 3, 1, 124, 0),
                   0);

  // Other case without constant part
  // Train starts with v1 = 20, a = 4, d = 2
  // It accelerates for 2 seconds to reach speed 28
  // After 2 seconds it has travelled 24*2 = 48
  // The theoretical maximal speed is 30
  // However, it immediately decelerates for 1 second to reach speed 26
  // For this, the distance is 27*1=27
  // Total distance travelled is 48+27 = 75 within 3 seconds
  // Braking distance at begin is 20*20/4 = 100, i.e., 25 after end
  // Braking distance at end is 26*26/4 = 169

  // If start is MA point, then obd is 169-25 = 144
  EXPECT_APPROX_EQ(
      cda_rail::min_time_from_front_to_ma_point(20, 26, 30, 4, 2, 75, 144), 0);
  EXPECT_APPROX_EQ(cda_rail::min_time_profile_from_rear_to_ma_point(
                       20, 26, 30, 4, 2, 75, 144),
                   3);

  // After 1 second it has reached speed 24, hence travelled 22*1 = 22
  // Its braking distance is 24*24/4 = 144
  // Hence, MA is at 22+144 = 166, i.e., 166-75 = 91 after end
  // Then obd is 169 - 91 = 78
  // To end this is 3-1 = 2
  EXPECT_APPROX_EQ(
      cda_rail::min_time_from_front_to_ma_point(20, 26, 30, 4, 2, 75, 78), 1);
  EXPECT_APPROX_EQ(cda_rail::min_time_profile_from_rear_to_ma_point(
                       20, 26, 30, 4, 2, 75, 78),
                   2);

  // If obd is 0, then result to back is 0
  EXPECT_APPROX_EQ(
      cda_rail::min_time_from_front_to_ma_point(20, 26, 30, 4, 2, 75, 0), 3);
  EXPECT_APPROX_EQ(
      cda_rail::min_time_profile_from_rear_to_ma_point(20, 26, 30, 4, 2, 75, 0),
      0);
}

TEST(Helper, EoMMaximalTimeNoStopping1) {
  // Train starts at speed 20
  // It decelerates at rate 1 for 2 second until speed 18 is reached, which is
  // minimal Traveling 19*2 = 38 It remains constant for 1 second travelling 18
  // Finally it accelerates at rate 2 for 2 second reaching 22
  // Acceleration distance is 20*2 = 40
  // Total distance travelled is 38+18+40 = 96
  // Braking distance at begin is 20*20/2 = 200, i.e., 104 after end
  // Braking distance at end is 22*22/2 = 242

  // After 2 seconds the distance travelled is 38
  // Its braking distance is 18*18/2 = 162
  // Hence, MA is at 38+162 = 200, i.e., 200-96 = 104 after end
  // Then obd is 242 - 104 = 138
  // To end this is 5-2 = 3
  // Same already holds at 0 seconds!
  EXPECT_APPROX_EQ(cda_rail::max_time_from_front_to_ma_point_no_stopping(
                       20, 22, 18, 2, 1, 96, 138),
                   0);
  EXPECT_APPROX_EQ(cda_rail::max_time_from_front_to_ma_point(20, 22, 18, 2, 1,
                                                             96, 138, false),
                   0);
  EXPECT_APPROX_EQ(cda_rail::max_time_profile_from_rear_to_ma_point(
                       20, 22, 18, 2, 1, 96, 138),
                   5);

  // After 3 seconds it has travelled additional 18, i.e., 38+18 = 56
  // Braking distance is 18*18/2 = 162
  // Hence, MA is at 56+162 = 218, i.e., 218-96 = 122 after end
  // Then obd is 242 - 122 = 120
  // To end this is 5-3 = 2
  EXPECT_APPROX_EQ(cda_rail::max_time_from_front_to_ma_point_no_stopping(
                       20, 22, 18, 2, 1, 96, 120),
                   3);
  EXPECT_APPROX_EQ(cda_rail::max_time_from_front_to_ma_point(20, 22, 18, 2, 1,
                                                             96, 120, false),
                   3);
  EXPECT_APPROX_EQ(cda_rail::max_time_profile_from_rear_to_ma_point(
                       20, 22, 18, 2, 1, 96, 120),
                   2);

  // After 4 seconds it has travelled additional 19, i.e., 56+19 = 75
  // Braking distance is 20*20/2 = 200
  // Hence, MA is at 75+200 = 275, i.e., 275-96 = 179 after end
  // Then obd is 242 - 179 = 63
  // To end this is 5-4 = 1
  EXPECT_APPROX_EQ(cda_rail::max_time_from_front_to_ma_point_no_stopping(
                       20, 22, 18, 2, 1, 96, 63),
                   4);
  EXPECT_APPROX_EQ(cda_rail::max_time_from_front_to_ma_point(20, 22, 18, 2, 1,
                                                             96, 63, false),
                   4);
  EXPECT_APPROX_EQ(cda_rail::max_time_profile_from_rear_to_ma_point(
                       20, 22, 18, 2, 1, 96, 63),
                   1);

  // If obd is 0, then it is the end point at 5 seconds
  EXPECT_APPROX_EQ(cda_rail::max_time_from_front_to_ma_point_no_stopping(
                       20, 22, 18, 2, 1, 96, 0),
                   5);
  EXPECT_APPROX_EQ(
      cda_rail::max_time_from_front_to_ma_point(20, 22, 18, 2, 1, 96, 0, false),
      5);
  EXPECT_APPROX_EQ(
      cda_rail::max_time_profile_from_rear_to_ma_point(20, 22, 18, 2, 1, 96, 0),
      0);
}

TEST(Helper, EoMMaximalTimeNoStopping2) {
  // Train starts at speed 20, however minimal speed is 22
  // It accelerates at rate 2 for 1 second to reach speed 22
  // Traveling 21*1 = 21
  // It remains constant for 1 second travelling 22
  // Finally, it decelerates at rate 1 for 2 seconds until speed 20 is reached
  // Deceleration distance is 21*2 = 42
  // Total distance travelled is 21+22+42 = 85
  // within 4 seconds
  // Braking distance at begin is 20*20/2 = 200, i.e., 115 after end
  // Braking distance at end is also 20*20/2 = 200

  // After 0 seconds the distance travelled is 0
  // The braking distance is 20*20/2 = 200
  // Hence, MA is at 200, i.e., 200-85 = 115 after end
  // Then obd is 200 - 115 = 85
  // To end this is 4-0 = 4
  EXPECT_APPROX_EQ(cda_rail::max_time_from_front_to_ma_point_no_stopping(
                       20, 20, 22, 2, 1, 85, 85),
                   0);
  EXPECT_APPROX_EQ(cda_rail::max_time_from_front_to_ma_point(20, 20, 22, 2, 1,
                                                             85, 85, false),
                   0);
  EXPECT_APPROX_EQ(cda_rail::max_time_profile_from_rear_to_ma_point(
                       20, 20, 22, 2, 1, 85, 85),
                   4);

  // After 1 second it has travelled 21
  // Braking distance is 22*22/2 = 242
  // Hence, MA is at 21+242 = 263, i.e., 263-85 = 178 after end
  // Then obd is 200 - 178 = 22
  // To end this is 4-1 = 3
  EXPECT_APPROX_EQ(cda_rail::max_time_from_front_to_ma_point_no_stopping(
                       20, 20, 22, 2, 1, 85, 22),
                   1);
  EXPECT_APPROX_EQ(cda_rail::max_time_from_front_to_ma_point(20, 20, 22, 2, 1,
                                                             85, 22, false),
                   1);
  EXPECT_APPROX_EQ(cda_rail::max_time_profile_from_rear_to_ma_point(
                       20, 20, 22, 2, 1, 85, 22),
                   3);

  // After 2 seconds it has travelled 43
  // Braking distance is 22*22/2 = 242
  // Hence, MA is at 43+242 = 285, i.e., 285-85 = 200 after end
  // Then obd is 200 - 200 = 0
  // To end this is 4-2 = 2
  EXPECT_APPROX_EQ(cda_rail::max_time_from_front_to_ma_point_no_stopping(
                       20, 20, 22, 2, 1, 85, 0),
                   2);
  EXPECT_APPROX_EQ(
      cda_rail::max_time_from_front_to_ma_point(20, 20, 22, 2, 1, 85, 0, false),
      2);
  EXPECT_APPROX_EQ(
      cda_rail::max_time_profile_from_rear_to_ma_point(20, 20, 22, 2, 1, 85, 0),
      2);
}

TEST(Helper, EoMMaximalTimeStopping) {
  // Train starts at speed 10
  // It decelerates at rate 1 for 10 seconds until speed 0 is reached
  // Total distance travelled is 5*10 = 50
  // It then accelerates at rate 4 for 5 seconds until speed 20 is reached
  // Acceleration distance is 10*5 = 50
  // Total distance travelled is 50+50 = 100
  // Braking distance at end is 20*20/2 = 200

  // 2 seconds before the end, the train has speed 12
  // It still travels 16*2 = 32
  // Its braking distance 12*12/2 = 72
  // Hence its MA is 72-32 = 40 after the end
  // Then obd is 200 - 40 = 160
  EXPECT_DOUBLE_EQ(cda_rail::max_time_from_front_to_ma_point_stopping_allowed(
                       10, 20, 4, 1, 100, 160),
                   std::numeric_limits<double>::infinity());
  EXPECT_DOUBLE_EQ(cda_rail::max_time_from_front_to_ma_point(10, 20, 0, 4, 1,
                                                             100, 160, true),
                   std::numeric_limits<double>::infinity());
  EXPECT_APPROX_EQ(cda_rail::max_time_profile_from_rear_to_ma_point(
                       10, 20, 0, 4, 1, 100, 160),
                   2);

  // If obd is 0, then from rear is 0
  EXPECT_DOUBLE_EQ(cda_rail::max_time_from_front_to_ma_point_stopping_allowed(
                       10, 20, 4, 1, 100, 0),
                   std::numeric_limits<double>::infinity());
  EXPECT_DOUBLE_EQ(
      cda_rail::max_time_from_front_to_ma_point(10, 20, 0, 4, 1, 100, 0, true),
      std::numeric_limits<double>::infinity());
  EXPECT_APPROX_EQ(
      cda_rail::max_time_profile_from_rear_to_ma_point(10, 20, 0, 4, 1, 100, 0),
      0);

  // Other scenario
  // Train starts at speed 10
  // It decelerates at rate 1 for 4 seconds until speed 6 is reached
  // Distance travelled is 8*4 = 32
  // It then accelerates at rate 2 for 2 seconds until speed 10 is reached again
  // Acceleration distance is 8*2 = 16
  // Total distance travelled is 32+16 = 48
  // Braking distance at end is 10*10/2 = 50

  // Braking distance at start is 10*10/2 = 50, i.e., 2 after end
  // Hence, MA is 2 after the end
  // Then obd is 50 - 2 = 48
  // From end this is 6 - 0 = 6
  EXPECT_APPROX_EQ(cda_rail::max_time_from_front_to_ma_point_stopping_allowed(
                       10, 10, 2, 1, 48, 48),
                   0);
  EXPECT_APPROX_EQ(
      cda_rail::max_time_from_front_to_ma_point(10, 10, 0, 2, 1, 48, 48, true),
      0);
  EXPECT_APPROX_EQ(
      cda_rail::max_time_profile_from_rear_to_ma_point(10, 10, 0, 2, 1, 48, 48),
      6);

  // After 5 seconds it has travelled 32+7=39
  // Braking distance is 8*8/2 = 32
  // Hence, MA is 39+32 = 71, i.e., 71-48 = 23 after end
  // Then obd is 50 - 23 = 27
  // From end this is 6 - 5 = 1
  EXPECT_APPROX_EQ(cda_rail::max_time_from_front_to_ma_point_stopping_allowed(
                       10, 10, 2, 1, 48, 27),
                   5);
  EXPECT_APPROX_EQ(
      cda_rail::max_time_from_front_to_ma_point(10, 10, 0, 2, 1, 48, 27, true),
      5);
  EXPECT_APPROX_EQ(
      cda_rail::max_time_profile_from_rear_to_ma_point(10, 10, 0, 2, 1, 48, 27),
      1);

  // After 6 seconds obd is 0
  EXPECT_APPROX_EQ(cda_rail::max_time_from_front_to_ma_point_stopping_allowed(
                       10, 10, 2, 1, 48, 0),
                   6);
  EXPECT_APPROX_EQ(
      cda_rail::max_time_from_front_to_ma_point(10, 10, 0, 2, 1, 48, 0, true),
      6);
  EXPECT_APPROX_EQ(
      cda_rail::max_time_profile_from_rear_to_ma_point(10, 10, 0, 2, 1, 48, 0),
      0);
}

TEST(Helper, EoMMaxTimeToMAFromRear) {
  EXPECT_APPROX_EQ(cda_rail::max_time_from_rear_to_ma_point(
                       20, 22, 18, 25, 2, 1, 96, 138,
                       cda_rail::MATimingStrategy::ExtremeProfiles),
                   5);
  EXPECT_APPROX_EQ(cda_rail::min_time_from_rear_to_ma_point(
                       16, 20, 15, 22, 3, 1, 124, 196,
                       cda_rail::MATimingStrategy::ExtremeProfiles),
                   6);
}

TEST(Helper, EoMMinTimeMoveMABackwards) {
  // Train with acceleration 2 and deceleration 1
  // Start with speed 10 -> BD = 10*10/2 = 50
  // Accelerate for 2 seconds to reach speed 14
  // Travelled 12*2 = 24
  // New BD = 14*14/2 = 98
  // MA moved by 98 + 24 - 50 = 72

  EXPECT_DOUBLE_EQ(cda_rail::min_time_to_push_ma_backward(14, 2, 1, 72), 2);

  // After 3s it reached speed 10 + 3*2 = 16
  // Travelled 13*3 = 39
  // New BD = 16*16/2 = 128
  // MA moved by 128 + 39 - 50 = 117
  EXPECT_DOUBLE_EQ(cda_rail::min_time_to_push_ma_backward(16, 2, 1, 117), 3);

  // Time to fully move 50m forward
  // v^2 - 10^2 = 2 * 2 * 50
  // Hence, v = 10 * sqrt(3)
  // Time t such that 10 + 2*t = 10 * sqrt(3)
  // Hence, t = 5 * (sqrt(3) - 1)
  // obd = v^2 / 2 = 300/ 2 = 150

  EXPECT_APPROX_EQ(
      cda_rail::min_time_to_push_ma_backward(10 * std::sqrt(3), 2, 1, 150),
      5 * (std::sqrt(3) - 1));
  EXPECT_APPROX_EQ(
      cda_rail::min_time_to_push_ma_fully_backward(10 * std::sqrt(3), 2, 1),
      5 * (std::sqrt(3) - 1));
}

TEST(Helper, EoMMaximalLineSpeed) {
  // Train starts with speed 10
  // Accelerates for 2 seconds at rate 2 to reach speed 14
  // Distance travelled is 12*2 = 24
  // Then decelerates for 4 seconds at rate 3 to reach speed 2
  // Distance travelled is 8*4 = 32
  // Total distance travelled is 24+32 = 56

  EXPECT_APPROX_EQ(cda_rail::maximal_line_speed(10, 2, 20, 2, 3, 56), 14);
  EXPECT_APPROX_EQ(cda_rail::maximal_line_speed(10, 2, 14, 2, 3, 70), 14);
  EXPECT_APPROX_EQ(cda_rail::maximal_line_speed(10, 2, 10, 2, 3, 70), 10);

  // Train starts with speed 10
  // Decelerates at rate 1 for 2 seconds to rech speed 8
  // Distance travelled is 9*2 = 18

  EXPECT_APPROX_EQ(cda_rail::maximal_line_speed(10, 8, 20, 2, 1, 18), 10);
}

TEST(HELPER, EoMMinimalLineSpeed) {
  // Train starts with speed 10
  // Decelerates at rate 2 for 2 seconds to reach speed 6
  // Distance travelled is 8*2 = 16
  // Then accelerates for 4 seconds at rate 3 to reach speed 18
  // Distance travelled is 12*4 = 48
  // Total distance travelled is 16+48 = 64

  EXPECT_APPROX_EQ(cda_rail::minimal_line_speed(10, 18, 1, 3, 2, 64), 6);
  EXPECT_APPROX_EQ(cda_rail::minimal_line_speed(10, 18, 6, 3, 2, 150), 6);

  // Train starts with speed 0
  // Accelerates at rate 2 for 2 seconds to reach speed 4
  // Distance travelled is 2*2 = 4
  // Then decelerates for 4 seconds at rate 1 to reach speed 0
  // Distance travelled is 2*4 = 8
  // Total distance travelled is 4+8 = 12

  EXPECT_APPROX_EQ(cda_rail::minimal_line_speed(0, 0, 5, 2, 1, 12), 4);
  EXPECT_APPROX_EQ(cda_rail::minimal_line_speed(0, 0, 4, 2, 1, 20), 4);
}

TEST(Helper, EoMTravelTimePerLineSpeed) {
  // Train starts with speed 10
  // Accelerates for 2 seconds at rate 2 to reach speed 14
  // Distance travelled is 12*2 = 24
  // Then travels at speed 14 for 4 seconds
  // Distance travelled is 14*4 = 56
  // Then accelerates for 4 seconds at rate 2 to reach speed 22
  // Distance travelled is 18*4 = 72
  // Total distance travelled is 24+56+72 = 152
  // Total distance without constant speed is 24+72 = 96

  EXPECT_APPROX_EQ(cda_rail::time_on_edge(10, 22, 14, 2, 1, 152), 10);
  EXPECT_APPROX_EQ(cda_rail::time_on_edge(10, 22, 14, 2, 1, 96), 6);

  // Train starts with speed 10
  // Accelerates for 2 seconds at rate 2 to reach speed 14
  // Distance travelled is 12*2 = 24
  // Then travels at speed 14 for 4 seconds
  // Distance travelled is 14*4 = 56
  // Then decelerates for 6 seconds at rate 1 to reach speed 8
  // Distance travelled is 11*6 = 66
  // Total distance travelled is 24+56+66 = 146

  EXPECT_APPROX_EQ(cda_rail::time_on_edge(10, 8, 14, 2, 1, 146), 12);

  // Train starts with speed 10
  // Decelerates for 2 seconds at rate 1 to reach speed 8
  // Distance travelled is 9*2 = 18
  // Then travels at speed 8 for 4 seconds
  // Distance travelled is 8*4 = 32
  // Then accelerates for 6 seconds at rate 2 to reach speed 8+12 = 20
  // Distance travelled is 14*6 = 84
  // Total distance travelled is 18+32+84 = 134

  EXPECT_APPROX_EQ(cda_rail::time_on_edge(10, 20, 8, 2, 1, 134), 12);

  // Train starts with speed 10
  // Decelerates for 2 seconds at rate 1 to reach speed 8
  // Distance travelled is 9*2 = 18
  // Then travels at speed 8 for 4 seconds
  // Distance travelled is 8*4 = 32
  // Then decelerates another 2 seconds at rate 1 to reach speed 6
  // Distance travelled is 7*2 = 14
  // Total distance travelled is 18+32+14 = 64
  // Total distance without constant speed is 18+14 = 32

  EXPECT_APPROX_EQ(cda_rail::time_on_edge(10, 6, 8, 2, 1, 64), 8);
  EXPECT_APPROX_EQ(cda_rail::time_on_edge(10, 6, 8, 2, 1, 32), 4);
}

TEST(Helper, EoMGetLineSpeed) {
  // Train starts with speed 10
  // Accelerates for 2 seconds at rate 2 to reach speed 14
  // Distance travelled is 12*2 = 24
  // Then travels at speed 14 for 4 seconds
  // Distance travelled is 14*4 = 56
  // Then accelerates for 4 seconds at rate 2 to reach speed 22
  // Distance travelled is 18*4 = 72
  // Total distance travelled is 24+56+72 = 152
  // Total time travelled is 2+4+4 = 10

  const auto line_speed =
      cda_rail::get_line_speed(10, 22, 1, 25, 2, 1, 152, 10);
  EXPECT_TRUE(std::abs(line_speed - 14) <= 0.27 ||
              std::abs(cda_rail::time_on_edge(10, 22, line_speed, 2, 1, 152) -
                       10) <= 1);

  // Train starts with speed 10
  // Accelerates for 2 seconds at rate 2 to reach speed 14
  // Distance travelled is 12*2 = 24
  // Then travels at speed 14 for 4 seconds
  // Distance travelled is 14*4 = 56
  // Then decelerates at rate 1 for 4 seconds to reach speed 10
  // Distance travelled is 12*4 = 48
  // Total distance travelled is 24+56+48 = 128
  // Total time travelled is 2+4+4 = 10

  const auto line_speed2 =
      cda_rail::get_line_speed(10, 10, 1, 25, 2, 1, 128, 10);
  EXPECT_TRUE(std::abs(line_speed2 - 14) <= 0.27 ||
              std::abs(cda_rail::time_on_edge(10, 10, line_speed2, 2, 1, 128) -
                       10) <= 1);

  // Train starts with speed 10
  // Then decelerates at rate 2 for 2 seconds to reach speed 6
  // Distance travelled is 8*2 = 16
  // Then travels at speed 6 for 4 seconds
  // Distance travelled is 6*4 = 24
  // Then accelerates at rate 3 for 4 seconds to reach speed 18
  // Distance travelled is 12*4 = 48
  // Total distance travelled is 16+24+48 = 88
  // Total time travelled is 2+4+4 = 10

  const auto line_speed3 =
      cda_rail::get_line_speed(10, 18, 1, 25, 3, 2, 88, 10);
  EXPECT_TRUE(std::abs(line_speed3 - 6) <= 0.27 ||
              std::abs(cda_rail::time_on_edge(10, 18, line_speed3, 2, 1, 88) -
                       10) <= 1);

  // Train starts with speed 0
  // Accelerates at rate 0.5 for 1 second to reach speed 0.5
  // Distance travelled is 0.25
  // Then decelerates at rate 0.5 for 1 second to reach speed 0
  // Distance travelled is 0.25
  // Total distance travelled is 0.5
  // Total time travelled is 1+1 = 2

  const auto line_speed4 =
      cda_rail::get_line_speed(0, 0, 1, 20, 0.5, 0.5, 0.5, 2);
  EXPECT_TRUE(
      std::abs(line_speed4 - 0.5) <= 0.27 ||
      std::abs(cda_rail::time_on_edge(0, 0, line_speed4, 0.5, 0.5, 0.5) - 2) <=
          1);

  // Train starts with speed 10
  // Then decelerates at rate 1 for 10 seconds to stop
  // Distance travelled is 5*10 = 50
  // Then accelerates at rate 2 for 5 seconds to reach speed 20
  // Distance travelled is 10*5 = 50
  // Total distance travelled is 100 in at least 15 seconds

  const auto line_speed5 =
      cda_rail::get_line_speed(10, 20, 1, 25, 2, 1, 100, 20);
  EXPECT_APPROX_EQ(line_speed5, 0);

  // If train ends after 50
  const auto line_speed6 = cda_rail::get_line_speed(10, 0, 1, 25, 2, 1, 50, 10);
  EXPECT_APPROX_EQ(line_speed6, 10);
}

TEST(Helper, EoMPosOnEdgeAtTime) {
  // Train starts with speed 10
  // Acceleration Rate 2, Deceleration Rate 1

  // Accelerates for 1 second to reach speed 12
  // Distance travelled is 11 within 1 second

  // Then continues accelerating for 1 second to reach speed 14
  // Distance travelled is 13
  // Total distance until here is 24 within 2 seconds

  // Then remains constant at line speed 14 for 2 seconds
  // Distance travelled is 28
  // Total distance until here is 52 within 4 seconds

  // Remains at line speed for another second
  // Distance travelled is 14
  // Total distance until here is 66 within 5 seconds

  // Then accelerates for another second to reach speed 16
  // Distance travelled is 15
  // Total distance until here is 81 within 6 seconds

  // Finally accelerates another 2 seconds to reach speed 20
  // Distance travelled is 18*2 = 36
  // Total distance until here is 117 within 8 seconds

  EXPECT_APPROX_EQ(cda_rail::time_on_edge(10, 20, 14, 2, 1, 117), 8);

  EXPECT_APPROX_EQ(cda_rail::pos_on_edge_at_time(10, 20, 14, 2, 1, 117, 0), 0);
  EXPECT_APPROX_EQ(cda_rail::pos_on_edge_at_time(10, 20, 14, 2, 1, 117, 1), 11);
  EXPECT_APPROX_EQ(cda_rail::pos_on_edge_at_time(10, 20, 14, 2, 1, 117, 2), 24);
  EXPECT_APPROX_EQ(cda_rail::pos_on_edge_at_time(10, 20, 14, 2, 1, 117, 4), 52);
  EXPECT_APPROX_EQ(cda_rail::pos_on_edge_at_time(10, 20, 14, 2, 1, 117, 5), 66);
  EXPECT_APPROX_EQ(cda_rail::pos_on_edge_at_time(10, 20, 14, 2, 1, 117, 6), 81);
  EXPECT_APPROX_EQ(cda_rail::pos_on_edge_at_time(10, 20, 14, 2, 1, 117, 8),
                   117);

  EXPECT_APPROX_EQ(cda_rail::pos_on_edge_at_time(10, 14, 12, 2, 1, 24, 0), 0);
  EXPECT_APPROX_EQ(cda_rail::pos_on_edge_at_time(10, 14, 12, 2, 1, 24, 1), 11);
  EXPECT_APPROX_EQ(cda_rail::pos_on_edge_at_time(10, 14, 12, 2, 1, 24, 2), 24);

  // Train starts with speed 10

  // Decelerates for 2 seconds at rate 1 to reach speed 8
  // Distance travelled is 9*2 = 18
  // Total 18 within 2 seconds

  // Continues decelerating for another 2 seconds to reach speed 6
  // Distance travelled is 7*2 = 14
  // Total 32 within 4 seconds

  // Remains constant for 1 second
  // Distance travelled is 6
  // Total 38 within 5 seconds

  // Remains constant for another 2 seconds
  // Distance travelled is 12
  // Total 50 within 7 seconds

  // Option a: Decelerates for 2 seconds to reach speed 4
  // Distance travelled is 5*2 = 10
  // Total 60 within 9 seconds

  // Decelerates another 4 seconds to reach speed 0
  // Distance travelled is 2*4 = 8
  // Total 68 within 13 seconds

  // Option b: Accelerates at rate 2 for 1 second to reach speed 8
  // Distance travelled is 7
  // Total 57 within 8 seconds

  // Accelerates another 2 seconds to reach speed 12
  // Distance travelled is 10*2 = 20
  // Total 77 within 10 seconds

  EXPECT_APPROX_EQ(cda_rail::time_on_edge(10, 0, 6, 2, 1, 68), 13);
  EXPECT_APPROX_EQ(cda_rail::time_on_edge(10, 12, 6, 2, 1, 77), 10);

  EXPECT_APPROX_EQ(cda_rail::pos_on_edge_at_time(10, 0, 6, 2, 1, 68, 0), 0);
  EXPECT_APPROX_EQ(cda_rail::pos_on_edge_at_time(10, 0, 6, 2, 1, 68, 2), 18);
  EXPECT_APPROX_EQ(cda_rail::pos_on_edge_at_time(10, 0, 6, 2, 1, 68, 4), 32);
  EXPECT_APPROX_EQ(cda_rail::pos_on_edge_at_time(10, 0, 6, 2, 1, 68, 5), 38);
  EXPECT_APPROX_EQ(cda_rail::pos_on_edge_at_time(10, 0, 6, 2, 1, 68, 7), 50);
  EXPECT_APPROX_EQ(cda_rail::pos_on_edge_at_time(10, 0, 6, 2, 1, 68, 9), 60);
  EXPECT_APPROX_EQ(cda_rail::pos_on_edge_at_time(10, 0, 6, 2, 1, 68, 13), 68);

  EXPECT_APPROX_EQ(cda_rail::pos_on_edge_at_time(10, 12, 6, 2, 1, 77, 0), 0);
  EXPECT_APPROX_EQ(cda_rail::pos_on_edge_at_time(10, 12, 6, 2, 1, 77, 2), 18);
  EXPECT_APPROX_EQ(cda_rail::pos_on_edge_at_time(10, 12, 6, 2, 1, 77, 4), 32);
  EXPECT_APPROX_EQ(cda_rail::pos_on_edge_at_time(10, 12, 6, 2, 1, 77, 5), 38);
  EXPECT_APPROX_EQ(cda_rail::pos_on_edge_at_time(10, 12, 6, 2, 1, 77, 7), 50);
  EXPECT_APPROX_EQ(cda_rail::pos_on_edge_at_time(10, 12, 6, 2, 1, 77, 8), 57);
  EXPECT_APPROX_EQ(cda_rail::pos_on_edge_at_time(10, 12, 6, 2, 1, 77, 10), 77);

  EXPECT_APPROX_EQ(cda_rail::pos_on_edge_at_time(10, 6, 8, 2, 1, 32, 0), 0);
  EXPECT_APPROX_EQ(cda_rail::pos_on_edge_at_time(10, 6, 8, 2, 1, 32, 2), 18);
  EXPECT_APPROX_EQ(cda_rail::pos_on_edge_at_time(10, 6, 8, 2, 1, 32, 4), 32);
}

// NOLINTEND(clang-diagnostic-unused-result)
