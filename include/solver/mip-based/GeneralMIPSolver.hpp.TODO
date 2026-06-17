#pragma once

#include "Definitions.hpp"
#include "MultiArray.hpp"
#include "gurobi_c++.h"
#include "gurobi_c.h"
#include "probleminstances/GeneralProblemInstance.hpp"
#include "solver/GeneralSolver.hpp"

#include <filesystem>
#include <optional>
#include <plog/Log.h>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>

namespace cda_rail::solver::mip_based {

struct SolutionSettings {
  bool         postprocess   = false;
  ExportOption export_option = ExportOption::NoExport;
  std::string  name          = "model";
  std::string  path;
};

struct SolutionSettingsMovingBlock {
  ExportOption export_option = ExportOption::NoExport;
  std::string  name          = "model";
  std::string  path;
};

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
  std::vector<GRBTempConstr> lazy_constraints;

  // Gurobi variables
  std::optional<GRBEnv>                               env;
  std::optional<GRBModel>                             model;
  std::unordered_map<std::string, MultiArray<GRBVar>> vars;
  GRBLinExpr                                          objective_expr;

  virtual void cleanup() {
    objective_expr = 0;
    lazy_constraints.clear();
    model->reset(1);
    vars.clear();
    model.reset();
    env.reset();
  };

  void solve_init_general_mip(int time_limit, bool debug_input,
                              bool overwrite_severity) {
    static auto message_callback = MessageCallback();
    this->solve_init_general_mip(time_limit, debug_input, overwrite_severity,
                                 &message_callback);
  };

  void solve_init_general_mip(int time_limit, bool debug_input,
                              bool overwrite_severity, GRBCallback* cb) {
    this->solve_init_general(time_limit, debug_input, overwrite_severity);

    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    PLOGD << "Create Gurobi environment and model";
    this->env.emplace(true);
    this->env->start();
    this->model.emplace(env.value());

    this->model->setCallback(cb);
    this->model->set(GRB_IntParam_LogToConsole, 0);
  };

  GeneralMIPSolver() = default;
  explicit GeneralMIPSolver(const T& instance)
      : GeneralSolver<T, S>(instance) {};
  explicit GeneralMIPSolver(const std::filesystem::path& p)
      : GeneralSolver<T, S>(p) {};
  explicit GeneralMIPSolver(const std::string& path)
      : GeneralSolver<T, S>(path) {};
  explicit GeneralMIPSolver(const char* path) : GeneralSolver<T, S>(path) {};
};
} // namespace cda_rail::solver::mip_based
