#pragma once

#include "MultiArray.hpp"
#include "gurobi_c++.h"
#include "solver/GeneralSolver.hpp"

#include <optional>
#include <plog/Log.h>
#include <string>
#include <type_traits>
#include <unordered_map>

namespace cda_rail::solver::mip_based {

// NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay)

class MessageCallback : public GRBCallback {
public:
  explicit MessageCallback() = default;

protected:
  void callback() override {
    if (where == GRB_CB_MESSAGE) {
      std::string msg = getStringInfo(GRB_CB_MSG_STRING);
      if (!msg.empty() && msg.back() == '\n') {
        msg.pop_back(); // Remove the last character (newline)
      }
      PLOGI << msg;
    }
  }
};

// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay)

template <typename T, typename S>
class GeneralMIPSolver : public GeneralSolver<T, S> {
  static_assert(
      std::is_base_of_v<cda_rail::instances::GeneralProblemInstance, T>,
      "T must be a child of GeneralProblemInstance");
  static_assert(
      std::is_base_of_v<cda_rail::instances::SolGeneralProblemInstance<T>, S>,
      "S must be a child of SolGeneralProblemInstance<T>");

protected:
  // Gurobi variables
  std::optional<GRBEnv>                               env;
  std::optional<GRBModel>                             model;
  std::unordered_map<std::string, MultiArray<GRBVar>> vars;
  GRBLinExpr                                          objective_expr;

  GeneralMIPSolver() = default;
  explicit GeneralMIPSolver(T instance)
      : GeneralSolver<T, S>(std::move(instance)){};
};
} // namespace cda_rail::solver::mip_based
