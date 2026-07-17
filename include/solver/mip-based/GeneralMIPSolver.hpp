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
      std::is_base_of_v<cda_rail::instances::SolGeneralProblemInstance, S>,
      "S must be a child of SolGeneralProblemInstance<T>");

protected:
  std::vector<GRBTempConstr> m_lazy_constraints;

  // Gurobi variables
  std::optional<GRBEnv>                               m_env;
  std::optional<GRBModel>                             m_model;
  std::unordered_map<std::string, MultiArray<GRBVar>> m_vars;
  GRBLinExpr                                          m_objective_expr;

  virtual void cleanup() {
    m_objective_expr = 0;
    m_lazy_constraints.clear();
    m_model->reset(1);
    m_vars.clear();
    m_model.reset();
    m_env.reset();
  };

  void solve_init_general_mip(bool debug_input, bool overwrite_severity) {
    static auto message_callback = MessageCallback();
    this->solve_init_general_mip(debug_input, overwrite_severity,
                                 &message_callback);
  };

  void solve_init_general_mip(bool debug_input, bool overwrite_severity,
                              GRBCallback* cb) {
    this->solve_init_general(debug_input, overwrite_severity);

    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    PLOGD << "Create Gurobi environment and model";
    this->m_env.emplace(true);
    this->m_env->start();
    this->m_model.emplace(m_env.value());

    this->m_model->setCallback(cb);
    this->m_model->set(GRB_IntParam_LogToConsole, 0);
  };

  GeneralMIPSolver() = default;
  explicit GeneralMIPSolver(const T& instance)
      : GeneralSolver<T, S>(instance) {};
  template <typename... Args>
  explicit GeneralMIPSolver(Args&&... args)
    requires(
        !GeneralSolver<T, S>::template IsSingleInstanceArgument<Args...>::value)
      : GeneralSolver<T, S>(std::forward<Args>(args)...) {}
};
} // namespace cda_rail::solver::mip_based
