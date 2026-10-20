#pragma once

#include "datastructure/RailwayNetwork.hpp"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"
#include "solver/GeneralSolver.hpp"

#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <utility>

namespace cda_rail::cli {

/**
 * @brief The state a `rail_cli` session operates on.
 *
 * A session holds at most one instance and, independently of that, at most one
 * standalone network. The two are deliberately kept apart because an instance
 * only references its network by name on disk: saving an instance and saving a
 * network are two different acts, see the `current_network` documentation.
 *
 * Nothing is written to disk unless one of the `save_*` functions is called.
 * Every mutating command marks the corresponding object as modified, which
 * `status` reports and which the commands that discard state warn about.
 *
 * In addition, the session keeps the solution of the last `solve`, so that it
 * can be evaluated afterwards.
 */
class Session {
  std::filesystem::path m_working_directory;

  std::optional<instances::GeneralPerformanceOptimizationInstance> m_instance;
  bool m_instance_modified{false};

  // Used only while no instance is loaded; otherwise the instance's own
  // network is the current one.
  std::optional<Network> m_standalone_network;
  bool                   m_network_modified{false};

  // The solution of the last 'solve', see set_solution. It is discarded
  // whenever another instance is loaded or created.
  std::unique_ptr<instances::SolGeneralPerformanceOptimizationInstance>
                                  m_solution;
  std::optional<double>           m_solution_time_step;
  solver::GeneralSolutionSettings m_solution_export_settings;

public:
  explicit Session(std::filesystem::path working_directory)
      : m_working_directory(std::move(working_directory)) {}

  // ---------------------------------------------------------------- general

  [[nodiscard]] const std::filesystem::path& working_directory() const {
    return m_working_directory;
  }
  void set_working_directory(std::filesystem::path working_directory) {
    m_working_directory = std::move(working_directory);
  }

  /** @brief Prompt reflecting what the session currently edits. */
  [[nodiscard]] std::string prompt() const;

  /** @brief Whether anything would be lost by discarding the session. */
  [[nodiscard]] bool has_unsaved_changes() const {
    return m_instance_modified || m_network_modified;
  }

  // --------------------------------------------------------------- instance

  [[nodiscard]] bool has_instance() const { return m_instance.has_value(); }
  [[nodiscard]] bool instance_modified() const { return m_instance_modified; }
  void               mark_instance_modified() { m_instance_modified = true; }

  /**
   * @brief The loaded instance.
   * @throws cda_rail::exceptions::InvalidInputException If none is loaded.
   */
  [[nodiscard]] instances::GeneralPerformanceOptimizationInstance& instance();
  [[nodiscard]] const instances::GeneralPerformanceOptimizationInstance&
  const_instance() const;

  /**
   * @brief Creates a new empty instance on top of the given network.
   *
   * If @p network_name names the standalone network of this session, that
   * in-memory network is adopted, so that a network built in this session can
   * be used without saving it first. Otherwise the network is read from disk.
   */
  void new_instance(const std::string& name, const std::string& subdirectory,
                    const std::string& network_name);
  void load_instance(const std::string& name, const std::string& subdirectory);
  /** @brief Rereads the instance from disk, discarding unsaved changes. */
  void reload_instance();
  void close_instance();
  /**
   * @brief Writes the instance and returns the directory it was written to.
   *
   * The network is only written if @p with_network is set; an instance save
   * otherwise leaves the network on disk untouched, even if it was edited.
   */
  std::filesystem::path save_instance(bool with_network);

  // ---------------------------------------------------------------- network

  /**
   * @brief Whether a network is currently being edited.
   *
   * This is the case if an instance is loaded (then it is the instance's
   * network) or if a standalone network is loaded.
   */
  [[nodiscard]] bool has_network() const {
    return m_instance.has_value() || m_standalone_network.has_value();
  }
  [[nodiscard]] bool has_standalone_network() const {
    return m_standalone_network.has_value();
  }
  [[nodiscard]] bool network_modified() const { return m_network_modified; }
  void               mark_network_modified() { m_network_modified = true; }

  /**
   * @brief The network the `network ...` commands act on.
   *
   * This is the loaded instance's network if there is one and the standalone
   * network otherwise. Editing an instance's network through this reference is
   * intended: the instance holds its network by value, so an instance's
   * network can be edited and written out on its own.
   *
   * @throws cda_rail::exceptions::InvalidInputException If neither exists.
   */
  [[nodiscard]] Network&       current_network();
  [[nodiscard]] const Network& const_current_network() const;

  /** @throws If an instance is loaded, whose network would be replaced. */
  void new_network(const std::string& name);
  void load_network(const std::string& name);
  /** @brief Rereads the network from disk, discarding unsaved changes. */
  void reload_network();
  void close_network();
  /**
   * @brief Writes the current network and returns the directory written to.
   * @param new_name If given, the network is renamed before it is written.
   */
  std::filesystem::path
  save_network(const std::optional<std::string>& new_name);

  // --------------------------------------------------------------- solution

  [[nodiscard]] bool has_solution() const { return m_solution != nullptr; }

  /**
   * @brief The solution of the last `solve`.
   * @throws cda_rail::exceptions::InvalidInputException If there is none.
   */
  [[nodiscard]] const instances::SolGeneralPerformanceOptimizationInstance&
  solution() const;

  /**
   * @brief Keeps @p solution as the solution of the session.
   *
   * @param time_step The time step of the solver, if it has one, which
   *        evaluate_solution uses by default.
   * @param export_settings Where the solver exported @p solution to, which is
   *        where evaluate_solution writes it back to.
   */
  void set_solution(
      std::unique_ptr<instances::SolGeneralPerformanceOptimizationInstance>
                                      solution,
      std::optional<double>           time_step,
      solver::GeneralSolutionSettings export_settings);

  /**
   * @brief Evaluates the solution, see
   *        instances::SolGeneralPerformanceOptimizationInstance::evaluate, and
   *        writes it back with its evaluation if the solver exported it.
   *
   * @param time_step Time step of the evaluation. If unset, the time step of
   *        the solver is used.
   * @return The directory written to, or `std::nullopt` if the solution was not
   *         exported.
   * @throws cda_rail::exceptions::InvalidInputException If there is no
   *         solution, or if no time step is given and the solver has none.
   */
  std::optional<std::filesystem::path>
  evaluate_solution(std::optional<double> time_step);
};

} // namespace cda_rail::cli
