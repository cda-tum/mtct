#include "RailwayNetwork.hpp"
#include "gtest/gtest.h"

TEST(Functionaliy, SimpleTest) {
    RailwayNetwork g;
    EXPECT_TRUE(g.edges.size() == 0); // dummy test
}