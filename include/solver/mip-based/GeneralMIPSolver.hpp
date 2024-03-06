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

  void solve_init_general_mip(int time_limit, bool debug_input) {
    this->solve_init_general(time_limit, debug_input);

    PLOGD << "Create Gurobi environment and model";
    this->env.emplace(true);
    this->env->start();
    this->model.emplace(env.value());

    static MessageCallback message_callback = MessageCallback();
    this->model->setCallback(&message_callback);
    this->model->set(GRB_IntParam_LogToConsole, 0);
  };

  GeneralMIPSolver() = default;
  explicit GeneralMIPSolver(const T& instance)
      : GeneralSolver<T, S>(instance){};
  explicit GeneralMIPSolver(const std::filesystem::path& p)
      : GeneralSolver<T, S>(p){};
  explicit GeneralMIPSolver(const std::string& path)
      : GeneralSolver<T, S>(path){};
  explicit GeneralMIPSolver(const char* path) : GeneralSolver<T, S>(path){};
};
} // namespace cda_rail::solver::mip_based
