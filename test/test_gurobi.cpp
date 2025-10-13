#include "gurobi_c++.h"
#include "gurobi_c.h"

#include "gtest/gtest.h"
#include <exception>
#include <iostream>

TEST(Gurobi, GurobiInstallation) {
  try {
    GRBEnv env = GRBEnv(true);
    env.start();
    GRBModel model = GRBModel(env);

    // max 2*y
    // -x + y <= 1
    // 3x + 2y <= 12
    // 2x + 3y <= 12
    // x, y >= 0 integer

    const GRBVar x = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_INTEGER, "x");
    const GRBVar y = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_INTEGER, "y");
    model.setObjective(2 * y, GRB_MAXIMIZE);
    model.addConstr(-x + y <= 1, "c0");
    model.addConstr(3 * x + 2 * y <= 12, "c1");
    model.addConstr(2 * x + 3 * y <= 12, "c2");

    model.optimize();

    std::cout << "x: " << x.get(GRB_DoubleAttr_X) << '\n';
    std::cout << "y: " << y.get(GRB_DoubleAttr_X) << '\n';
    std::cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << '\n';

    EXPECT_EQ(model.get(GRB_IntAttr_Status), GRB_OPTIMAL);
    EXPECT_EQ(model.get(GRB_DoubleAttr_ObjVal), 4);
  } catch (GRBException& e) {
    std::cout << "Error code = " << e.getErrorCode() << '\n';
    std::cout << e.getMessage() << '\n';
    throw e;
  } catch (const std::exception& e) {
    std::cout << "Exception: " << e.what() << '\n';
    throw e;
  }
}
