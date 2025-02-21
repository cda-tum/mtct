#include "MultiArray.hpp"

#include "gtest/gtest.h"
#include <cstddef>
#include <iostream>
#include <stdexcept>
#include <vector>

using std::size_t;

TEST(Functionality, MultiArray) {
  cda_rail::MultiArray<size_t> a1(1, 2, 3);

  // Set elements
  for (size_t i = 0; i < 1; ++i) {
    for (size_t j = 0; j < 2; ++j) {
      for (size_t k = 0; k < 3; ++k) {
        a1(i, j, k) = 6 * i + 3 * j + k;
      }
    }
  }

  // Check elements
  for (size_t i = 0; i < 1; ++i) {
    for (size_t j = 0; j < 2; ++j) {
      for (size_t k = 0; k < 3; ++k) {
        EXPECT_EQ(a1(i, j, k), (6 * i) + (3 * j + k));
      }
    }
  }

  EXPECT_EQ(a1.size(), 6);
  EXPECT_EQ(a1.dimensions(), 3);
  EXPECT_EQ(a1.get_shape(), std::vector<size_t>({1, 2, 3}));

  // Calling with wrong number of arguments should throw std::invalid_argument
  EXPECT_THROW(a1(0), std::invalid_argument);
  EXPECT_THROW(a1(0, 0), std::invalid_argument);
  EXPECT_THROW(a1(0, 0, 0, 0), std::invalid_argument);

  // Calling with index too large should throw std::out_of_range
  EXPECT_THROW(a1(1, 0, 0), std::out_of_range);
  EXPECT_THROW(a1(0, 2, 0), std::out_of_range);
  EXPECT_THROW(a1(0, 0, 3), std::out_of_range);
}
