#include "RailwayNetwork.hpp"
#include "gtest/gtest.h"

#include "gurobi_c++.h"
TEST(Functionaliy, SimpleTest) {
    RailwayNetwork g;
    GRBLinExpr linexpr;

    EXPECT_TRUE(g.edges.size() == 0); // dummy test
}