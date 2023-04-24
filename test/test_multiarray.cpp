#include "MultiArray.hpp"
#include <iostream>

#include "gtest/gtest.h"

TEST(Functionality, MultiArray) {
    cda_rail::MultiArray<int> a1 (1,2,3);

    // Set elements
    for (size_t i = 0; i < 1; ++i) {
        for (size_t j = 0; j < 2; ++j) {
            for (size_t k = 0; k < 3; ++k) {
                a1(i,j,k) = 6*i + 3*j + k;
            }
        }
    }

    // Check elements
    for (size_t i = 0; i < 1; ++i) {
        for (size_t j = 0; j < 2; ++j) {
            for (size_t k = 0; k < 3; ++k) {
                EXPECT_TRUE(a1(i,j,k) == 6*i + 3*j + k);
            }
        }
    }

    // Calling with wrong number of arguments should throw std::invalid_argument
    EXPECT_THROW(a1(0), std::invalid_argument);
    EXPECT_THROW(a1(0,0), std::invalid_argument);
    EXPECT_THROW(a1(0,0,0,0), std::invalid_argument);

    // Calling with index too large should throw std::out_of_range
    EXPECT_THROW(a1(1,0,0), std::out_of_range);
    EXPECT_THROW(a1(0,2,0), std::out_of_range);
    EXPECT_THROW(a1(0,0,3), std::out_of_range);
}