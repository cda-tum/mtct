#pragma once

#include "Definitions.hpp"
#include "probleminstances/GeneralProblemInstance.hpp"

#include <chrono>
#include <cstdint>
#include <filesystem>
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

  void solve_init_general(int time_limit, bool debug_input,
                          bool overwrite_severity) {
    cda_rail::initialize_plog(debug_input, overwrite_severity);

    if (plog::get()->checkSeverity(plog::debug) || time_limit > 0) {
      m_start = std::chrono::high_resolution_clock::now();
    }
  }

  GeneralSolver() = default;
  explicit GeneralSolver(const T& instance) : m_instance(instance) {}
  template <typename... Args>
  explicit GeneralSolver(Args&&... args)
      : m_instance(std::forward<Args>(args)...) {}

public:
  [[nodiscard]] const T& get_instance() const { return m_instance; }
  [[nodiscard]] T&       editable_instance() { return m_instance; }

  [[nodiscard]] S solve() { return solve(-1, false); };
  [[nodiscard]] S solve(int time_limit, bool debug_input) {
    return solve(time_limit, debug_input, true);
  }
  [[nodiscard]] virtual S solve(int time_limit, bool debug_input,
                                bool overwrite_severity) = 0;

  virtual ~GeneralSolver() = default;
};
} // namespace cda_rail::solver
