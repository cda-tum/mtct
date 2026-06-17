#pragma once

#include "GeneralHelper.hpp"
#include "probleminstances/GeneralProblemInstance.hpp"

#include <chrono>
#include <cstdint>
#include <optional>
#include <plog/Logger.h>
#include <plog/Severity.h>
#include <string>
#include <type_traits>
#include <utility>

namespace cda_rail::solver {
enum class GeneralExportOption : std::uint8_t {
  NoExport                   = 0,
  ExportSolution             = 1,
  ExportSolutionWithInstance = 2
};

struct GeneralSolutionSettings {
  GeneralExportOption        export_option{GeneralExportOption::NoExport};
  std::string                working_directory{};
  std::string                solution_subdirectory{"unnamed-experiment"};
  std::optional<std::string> parameter_identifier{};
};

template <typename T, typename S> class GeneralSolver {
  static_assert(
      std::is_base_of_v<cda_rail::instances::GeneralProblemInstance, T>,
      "T must be a child of GeneralProblemInstance");
  static_assert(
      std::is_base_of_v<cda_rail::instances::SolGeneralProblemInstance, S>,
      "S must be a child of SolGeneralProblemInstance<T>");

protected:
  T                                                   m_instance;
  decltype(std::chrono::high_resolution_clock::now()) m_start;
  decltype(std::chrono::high_resolution_clock::now()) m_model_created;
  decltype(std::chrono::high_resolution_clock::now()) m_model_solved;
  int64_t                                             m_create_time = 0;
  int64_t                                             m_solve_time  = 0;

  template <typename... Args>
  struct IsSingleInstanceArgument : std::false_type {};
  template <typename Arg>
  struct IsSingleInstanceArgument<Arg> : std::is_same<T, std::decay_t<Arg>> {};

  /**
   * @brief Initializes logging and conditionally records solver start time.
   *
   * Initializes the plog logging framework according to the settings provided.
   * Records the current high-resolution clock time as the solver's start
   * timestamp if either debug logging is enabled or a positive time limit is
   * specified.
   *
   * @param time_limit If positive, triggers start time recording.
   * @param debug_input If true, enables debug-level logging.
   * @param overwrite_severity If true, overwrites the logging severity level.
   */
  void solve_init_general(int time_limit, bool debug_input,
                          bool overwrite_severity) {
    cda_rail::initialize_plog(debug_input, overwrite_severity);

    if (plog::get()->checkSeverity(plog::debug) || time_limit > 0) {
      m_start = std::chrono::high_resolution_clock::now();
    }
  }

  /**
   * @brief Constructs a solver with a default-initialized problem instance.
   */
  GeneralSolver() = default;
  /**
   * @brief Constructs a solver from a problem instance.
   *
   * @param instance The problem instance to store.
   */
  explicit GeneralSolver(const T& instance) : m_instance(instance) {}
  /**
   * @brief Constructs a GeneralSolver with arguments for the problem instance
   * constructor.
   *
   * @param args Arguments passed to construct the problem instance.
   */
  template <typename... Args>
  explicit GeneralSolver(Args&&... args)
    requires(!IsSingleInstanceArgument<Args...>::value)
      : m_instance(std::forward<Args>(args)...) {}

public:
  /**
   * @brief Accesses the problem instance stored in this solver.
   * @return const T& Const reference to the stored problem instance.
   */
  [[nodiscard]] const T& get_instance() const { return m_instance; }
  /**
   * @brief Provides mutable access to the problem instance.
   * @return T& Reference to the problem instance.
   */
  [[nodiscard]] T& editable_instance() { return m_instance; }

  /**
   * @brief Solves the problem instance using default configuration.
   *
   * @return S The solution.
   */
  [[nodiscard]] S solve() { return solve(-1, false); };
  /**
   * @brief Solves the problem instance.
   *
   * @param time_limit Maximum time allowed for solving. Zero or negative values
   * disable the limit.
   * @param debug_input Whether to enable debug input mode.
   * @return S The computed solution.
   */
  [[nodiscard]] S solve(int time_limit, bool debug_input) {
    return solve(time_limit, debug_input, true);
  }
  [[nodiscard]] virtual S solve(int time_limit, bool debug_input,
                                bool overwrite_severity) = 0;

  virtual ~GeneralSolver() = default;
};
} // namespace cda_rail::solver
