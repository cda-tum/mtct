#pragma once

#include "MultiArray.hpp"
#include "gurobi_c++.h"
#include "gurobi_c.h"
#include "probleminstances/GeneralProblemInstance.hpp"
#include "solver/GeneralSolver.hpp"

#include <optional>
#include <plog/Log.h>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>

namespace cda_rail::solver::mip_based {

/**
 * @brief Export settings shared by all MIP based solvers.
 *
 * The solution itself is exported using the general settings, i.e., to the
 * standard path
 * working_directory/solutions/solution_subdirectory/instance_subdirectory/
 * instance_name[-parameter_identifier]. In addition, the MIP model itself can
 * be written to the very same directory using model_name as file name.
 */
struct SolutionSettingsMIP : GeneralSolutionSettings {
  bool        export_lp_model = false;
  std::string model_name      = "model";
};

/**
 * @brief Export settings of the moving block MIP solver.
 *
 * The moving block MIP solver does not have any export settings beyond the
 * ones shared by all MIP based solvers.
 */
using SolutionSettingsMovingBlock = SolutionSettingsMIP;

/**
 * @brief Export settings of the VSS generation MIP solver.
 *
 * In addition to the settings shared by all MIP based solvers, the extracted
 * solution can be postprocessed in order to remove potentially unused VSS.
 */
struct SolutionSettingsVSSGen : SolutionSettingsMIP {
  bool postprocess = false;
};

// NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay)

/**
 * @brief Routes the output of Gurobi into the logging of this tool.
 *
 * Without this callback Gurobi writes to the console directly, which would
 * bypass the log file and the severity the solver was started with.
 */
class MessageCallback : public GRBCallback {
public:
  explicit MessageCallback() = default;

protected:
  /** @brief Logs one message of Gurobi, without its trailing newline. */
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

/**
 * @brief Base of every solver that builds a MIP and hands it to Gurobi.
 *
 * It owns the Gurobi environment, the model, and its variables, and takes care
 * of everything that is the same for every model, namely its creation and its
 * cleanup.
 *
 * @tparam T The problem instance type, a child of GeneralProblemInstance.
 * @tparam S The solution type belonging to it, a child of
 *         SolGeneralProblemInstance.
 */
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

  /** @brief Releases the model and the environment after solving. */
  virtual void cleanup() {
    m_objective_expr = 0;
    m_lazy_constraints.clear();
    m_model->reset(1);
    m_vars.clear();
    m_model.reset();
    m_env.reset();
  };

  /** @brief Initializes the model with the default message callback. */
  void solve_init_general_mip(bool debug_input, bool overwrite_severity) {
    static auto message_callback = MessageCallback();
    this->solve_init_general_mip(debug_input, overwrite_severity,
                                 &message_callback);
  };

  /**
   * @brief Initializes the logging, the Gurobi environment, and the model.
   *
   * @param debug_input If true, enables debug-level logging.
   * @param overwrite_severity If true, overwrites the logging severity level.
   * @param cb The callback of the model, which a solver using lazy constraints
   *        provides itself.
   */
  void solve_init_general_mip(bool debug_input, bool overwrite_severity,
                              GRBCallback* cb) {
    this->solve_init_general(debug_input, overwrite_severity);

    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    PLOGD << "Create Gurobi environment and model";
    this->m_env.emplace(true);
    // this->m_env->set(GRB_StringParam_LogFile, "test_log.log");
    this->m_env->start();
    this->m_model.emplace(m_env.value());

    this->m_model->setCallback(cb);
    this->m_model->set(GRB_IntParam_LogToConsole, 0);
  };

  /** @brief Constructs a solver with a default-initialized problem instance. */
  GeneralMIPSolver() = default;
  /** @brief Constructs a solver from a problem instance. */
  explicit GeneralMIPSolver(const T& instance)
      : GeneralSolver<T, S>(instance) {};
  /** @brief Constructs the problem instance from the given arguments. */
  template <typename... Args>
  explicit GeneralMIPSolver(Args&&... args)
    requires(
        !GeneralSolver<T, S>::template IsSingleInstanceArgument<Args...>::value)
      : GeneralSolver<T, S>(std::forward<Args>(args)...) {}
};
} // namespace cda_rail::solver::mip_based
