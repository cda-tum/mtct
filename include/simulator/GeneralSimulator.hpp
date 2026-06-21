#pragma once

#include "Definitions.hpp"
#include "datastructure/RailwayNetwork.hpp"
#include "probleminstances/GeneralProblemInstance.hpp"

#include <cstddef>
#include <map>
#include <memory>
#include <optional>
#include <unordered_set>
#include <vector>

namespace cda_rail::simulator {

struct PosVel {
  double pos;
  double vel;
};

struct SimulatorResults {
  bool success; // true if simulation was successful, false if it was infeasible
  std::vector<double> exit_times; // exit times of each train (or route end time
                                  // if route was only partially simulated)
  std::vector<std::vector<double>>
      stop_times; // For every train, a vector of times at which scheduled stops
                  // were reached
  std::vector<double> vertex_headways; // for every vertex, earliest time at
                                       // which next train can enter
  std::vector<std::map<double, PosVel>>
      train_trajectories; // For every train, a map of time to position and
                          // velocity at that time
};

struct SimulatorState {
  std::vector<cda_rail::index_vector> train_edges;
  std::vector<cda_rail::index_vector> ttd_orders;
  std::vector<cda_rail::index_vector> vertex_orders;
  std::vector<std::vector<double>>    stop_positions;

  bool operator==(const SimulatorState& other) const;

  bool operator>(const SimulatorState& other) const;
};

class GeneralSimulator {
  /**
   * This abstract class defines the interface for a general simulator to
   * simulate train trajectories given all routing, ordering, and stopping
   * information. It can, e.g., be used within search algorithms like A* for
   * objective evaluation. This abstract class defines all necessary functions
   * to be implemented and provides some general helper functions.
   */

private:
  std::shared_ptr<
      const cda_rail::instances::GeneralProblemInstanceWithScheduleAndRoutes>
                                   m_instance;
  std::vector<cda_rail::index_set> m_ttd_sections;

  std::vector<cda_rail::index_vector> m_train_edges;
  std::vector<cda_rail::index_vector> m_ttd_orders;
  std::vector<cda_rail::index_vector> m_vertex_orders;
  std::vector<std::vector<double>>    m_stop_positions;

  // ----------------------
  // Constructor
  // ----------------------
protected:
  GeneralSimulator(
      std::shared_ptr<const cda_rail::instances::
                          GeneralProblemInstanceWithScheduleAndRoutes>
                                       instance,
      std::vector<cda_rail::index_set> ttd_sections);
  GeneralSimulator(
      std::shared_ptr<const cda_rail::instances::
                          GeneralProblemInstanceWithScheduleAndRoutes>
                                          instance,
      std::vector<cda_rail::index_set>    ttd_sections,
      std::vector<cda_rail::index_vector> train_edges,
      std::vector<cda_rail::index_vector> ttd_orders,
      std::vector<cda_rail::index_vector> vertex_orders,
      std::vector<std::vector<double>>    stop_positions);

public:
  GeneralSimulator(
      instances::GeneralProblemInstanceWithScheduleAndRoutes& instance,
      std::vector<cda_rail::index_set>                        ttd_sections);
  GeneralSimulator(
      instances::GeneralProblemInstanceWithScheduleAndRoutes& instance,
      std::vector<cda_rail::index_set>                        ttd_sections,
      std::vector<cda_rail::index_vector>                     train_edges,
      std::vector<cda_rail::index_vector>                     ttd_orders,
      std::vector<cda_rail::index_vector>                     vertex_orders,
      std::vector<std::vector<double>>                        stop_positions);

  /**
   * @brief Virtual destructor.
   *
   * Ensures proper cleanup when derived class instances are deleted through
   * base class pointers.
   */
  virtual ~GeneralSimulator() = default;

  // Rule of 5 (due to virtual deconstructor)
  GeneralSimulator(GeneralSimulator const&)            = default;
  GeneralSimulator(GeneralSimulator&&)                 = default;
  GeneralSimulator& operator=(GeneralSimulator const&) = default;
  GeneralSimulator& operator=(GeneralSimulator&&)      = default;

  // ---------------------
  // SIMPLE GETTER
  /**
   * @brief Provides access to the problem instance for this simulator.
   * @return Const pointer to the problem instance.
   */
  [[nodiscard]] virtual instances::
      GeneralProblemInstanceWithScheduleAndRoutes const*
      get_instance() const {
    return m_instance.get();
  }

  /**
   * @brief Retrieves the TTD sections considered by the simulator.
   *
   * @return const std::vector<cda_rail::index_set>& Vector of TTD section
   * identifier sets.
   */
  [[nodiscard]] const std::vector<cda_rail::index_set>&
  get_ttd_sections() const {
    return m_ttd_sections;
  }

  /**
   * @brief Accesses the edge sequences for all trains.
   *
   * @return The per-train edge sequences.
   */
  [[nodiscard]] const std::vector<cda_rail::index_vector>&
  get_train_edges() const {
    return m_train_edges;
  }
  [[nodiscard]] const cda_rail::index_vector&
  get_train_edges_of_tr(size_t train_id) const;

  /**
   * @brief Retrieves the ordering vectors for all TTD sections.
   * @return const std::vector<index_vector>& The per-TTD ordering vectors.
   */
  [[nodiscard]] const std::vector<cda_rail::index_vector>&
  get_ttd_orders() const {
    return m_ttd_orders;
  }
  [[nodiscard]] const cda_rail::index_vector&
  get_ttd_orders_of_ttd(size_t ttd_index) const;
  [[nodiscard]] std::optional<size_t>
  get_ttd(Network::EdgeInput const& edge) const;

  /**
   * @brief Accesses the train ordering for each vertex.
   *
   * @return const std::vector<cda_rail::index_vector>& The ordering vectors per
   * vertex.
   */
  [[nodiscard]] const std::vector<cda_rail::index_vector>&
  get_vertex_orders() const {
    return m_vertex_orders;
  }
  [[nodiscard]] const cda_rail::index_vector&
  get_vertex_orders_of_vertex(Network::VertexInput const& vertex) const;

  /**
   * @brief Accesses all trains' scheduled stop positions.
   *
   * @return Const reference to a vector where each element is a vector of stop
   * positions for the corresponding train.
   */
  [[nodiscard]] const std::vector<std::vector<double>>&
  get_stop_positions() const {
    return m_stop_positions;
  }
  [[nodiscard]] const std::vector<double>&
  get_stop_positions_of_tr(size_t train_id) const;

  // -------------------
  // HELPFUL VALUES
  // -------------------
  [[nodiscard]] double train_edge_length(size_t tr) const;
  [[nodiscard]] double get_edge_position(size_t train_id, size_t edge_id) const;
  [[nodiscard]] size_t get_edge_at_position(size_t train_id,
                                            double position) const;

  [[nodiscard]] bool
  is_route_end_valid_stop_pos(size_t                        tr,
                              const cda_rail::index_vector& edges) const;
  /**
   * @brief Checks whether the train's current last edge is a valid stop
   * position.
   *
   * @param tr Train ID.
   * @return `true` if the position is valid for stopping, `false` otherwise.
   */
  [[nodiscard]] bool is_current_pos_valid_stop_position(size_t tr) const {
    /**
     * This function checks if the current last edge can be used as a stop for a
     * specific train. A stop is possible, if the train length back from the
     * target vertex is fully within the next station.
     *
     * @param tr: The id of the train to check.
     * @return: A boolean indicating whether the current position is a valid
     * stop position.
     */
    return is_route_end_valid_stop_pos(tr, get_train_edges_of_tr(tr));
  }

  // -------------------
  // EDITING FUNCTIONS
  // -------------------

  void set_train_edges(std::vector<cda_rail::index_vector> tr_edges);
  void set_train_edges_of_tr(size_t train_id, cda_rail::index_vector edges);
  void append_train_edge_to_tr(size_t train_id, size_t edge);

  void set_ttd_orders(std::vector<cda_rail::index_vector> orders);
  void set_ttd_orders_of_ttd(size_t ttd_index, cda_rail::index_vector orders);

  void set_vertex_orders(std::vector<cda_rail::index_vector> orders);
  void set_vertex_orders_of_vertex(cda_rail::Network::VertexInput const& vertex,
                                   cda_rail::index_vector orders);

  void set_stop_positions(std::vector<std::vector<double>> positions);
  void set_stop_positions_of_tr(size_t train_id, std::vector<double> positions);
  void append_stop_position_to_tr(size_t train_id, double position);
  void append_stop_edge_to_tr(size_t train_id, size_t edge);
  void append_current_stop_position_of_tr(size_t train_id);

  // ------------------
  // STATE HELPER
  // ------------------

  [[nodiscard]] bool is_final_state() const;

  // ------------------
  // SIMULATION
  // ------------------

  // virtual function
  [[nodiscard]] virtual SimulatorResults
  simulate(bool late_entry_possible, bool limit_speed_by_leaving_edges,
           bool save_trajectories,
           bool disappear_at_partial_route_end) const = 0;

  /**
   * @brief Simulates train movements given the current routing, ordering, and
   * stopping configuration. For partial routes, it is assumed that trains
   * instantaneously stop at their route end. If
   * disappear_at_partial_route_end=true, they also disappear.
   *
   * @return SimulatorResults containing the simulation outcome, including train
   * exit times, stop timestamps, vertex headways, and—when
   * enabled—train trajectories over time.
   */
  [[nodiscard]] SimulatorResults
  simulate(bool late_entry_possible          = false,
           bool limit_speed_by_leaving_edges = true,
           bool save_trajectories =
               false) const { // default values for virtual function
    return simulate(late_entry_possible, limit_speed_by_leaving_edges,
                    save_trajectories, false);
  };

protected:
  // --------------------
  // PROTECTED HELPER
  // --------------------

  [[nodiscard]] double tr_braking_distance(size_t tr, double v) const;

  [[nodiscard]] std::vector<double> edge_milestones(size_t tr) const;

  [[nodiscard]] bool is_on_route(size_t tr, size_t edge_id) const;

  [[nodiscard]] std::vector<std::unordered_set<size_t>> tr_on_edges() const;

private:
  // -----------------------
  // EXCEPTION HELPER
  // -----------------------
  void check_ttd_sections(
      std::vector<cda_rail::index_set> const& ttd_sections) const;

  void check_train_edges_for_tr(size_t                        tr,
                                cda_rail::index_vector const& edges) const;
  void check_train_edges(
      std::vector<cda_rail::index_vector> const& train_edges) const;

  void check_if_all_tr_valid(cda_rail::index_vector const& train_ids) const;
  static void check_if_all_tr_unique(cda_rail::index_vector const& train_ids);
  void
  check_ttd_orders(std::vector<cda_rail::index_vector> const& ttd_orders) const;
  void check_vertex_orders(
      std::vector<cda_rail::index_vector> const& vertex_orders) const;

  void check_stop_positions_for_tr(size_t                     tr,
                                   std::vector<double> const& positions) const;
  void check_stop_positions(
      std::vector<std::vector<double>> const& stop_positions) const;
};
} // namespace cda_rail::simulator
