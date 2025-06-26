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

namespace cda_rail::solver {
template <typename T, typename S> class GeneralSolver {
  static_assert(
      std::is_base_of_v<cda_rail::instances::GeneralProblemInstance, T>,
      "T must be a child of GeneralProblemInstance");
  static_assert(
      std::is_base_of_v<cda_rail::instances::SolGeneralProblemInstance<T>, S>,
      "S must be a child of SolGeneralProblemInstance<T>");

protected:
  T                                                   instance;
  decltype(std::chrono::high_resolution_clock::now()) start;
  decltype(std::chrono::high_resolution_clock::now()) model_created;
  decltype(std::chrono::high_resolution_clock::now()) model_solved;
  int64_t                                             create_time = 0;
  int64_t                                             solve_time  = 0;

  void solve_init_general(int time_limit, bool debug_input,
                          bool overwrite_severity) {
    cda_rail::initialize_plog(debug_input, overwrite_severity);

    if (plog::get()->checkSeverity(plog::debug) || time_limit > 0) {
      start = std::chrono::high_resolution_clock::now();
    }
  }

  GeneralSolver() = default;
  explicit GeneralSolver(const T& instance) : instance(instance) {};
  explicit GeneralSolver(const std::filesystem::path& p) : instance(p) {};
  explicit GeneralSolver(const std::string& path) : instance(path) {};
  explicit GeneralSolver(const char* path) : instance(path) {};

public:
  [[nodiscard]] const T& get_instance() const { return instance; }
  [[nodiscard]] T&       editable_instance() { return instance; }

  [[nodiscard]] S solve() { return solve(-1, false); };
  [[nodiscard]] S solve(int time_limit, bool debug_input) {
    return solve(time_limit, debug_input, true);
  };
  [[nodiscard]] virtual S solve(int time_limit, bool debug_input,
                                bool overwrite_severity) = 0;

  virtual ~GeneralSolver() = default;
};
} // namespace cda_rail::solver
