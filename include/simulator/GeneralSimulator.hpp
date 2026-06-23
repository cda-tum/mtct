#pragma once

#include "Definitions.hpp"
#include "datastructure/RailwayNetwork.hpp"
#include "probleminstances/GeneralProblemInstance.hpp"

#include <cstddef>
#include <functional>
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

  /**
   * @brief Compares two simulator states for equality.
   *
   * @param other State to compare against.
   * @return `true` if all state components match, otherwise `false`.
   */
  bool operator==(const SimulatorState& other) const;

  /**
   * @brief Provides a strict ordering between simulator states.
   *
   * @param other State to compare against.
   * @return `true` if this state is ordered after @p other.
   */
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
  /**
   * @brief Constructs a simulator from shared instance data.
   *
   * @param instance Shared problem instance.
   * @param ttd_sections TTD sections used by the simulator.
   */
  GeneralSimulator(
      std::shared_ptr<const cda_rail::instances::
                          GeneralProblemInstanceWithScheduleAndRoutes>
                                       instance,
      std::vector<cda_rail::index_set> ttd_sections);
  /**
   * @brief Constructs a simulator from shared instance data and explicit state.
   *
   * @param instance Shared problem instance.
   * @param ttd_sections TTD sections used by the simulator.
   * @param train_edges Per-train edge sequences.
   * @param ttd_orders Per-TTD train orders.
   * @param vertex_orders Per-vertex train orders.
   * @param stop_positions Per-train stop positions.
   */
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
  /**
   * @brief Constructs a simulator from a mutable instance reference.
   *
   * @param instance Problem instance.
   * @param ttd_sections TTD sections used by the simulator.
   */
  GeneralSimulator(
      instances::GeneralProblemInstanceWithScheduleAndRoutes& instance,
      std::vector<cda_rail::index_set>                        ttd_sections);
  /**
   * @brief Constructs a simulator from explicit state vectors.
   *
   * @param instance Problem instance.
   * @param ttd_sections TTD sections used by the simulator.
   * @param train_edges Per-train edge sequences.
   * @param ttd_orders Per-TTD train orders.
   * @param vertex_orders Per-vertex train orders.
   * @param stop_positions Per-train stop positions.
   */
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
  /** @brief Copy constructor. */
  GeneralSimulator(GeneralSimulator const&) = default;
  /** @brief Move constructor. */
  GeneralSimulator(GeneralSimulator&&) = default;
  /** @brief Copy assignment operator. */
  GeneralSimulator& operator=(GeneralSimulator const&) = default;
  /** @brief Move assignment operator. */
  GeneralSimulator& operator=(GeneralSimulator&&) = default;

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
  /**
   * @brief Returns the current edge sequence for one train.
   *
   * @param train_id Train index.
   * @return Edge sequence for the specified train.
   */
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
  /**
   * @brief Returns the train order for one TTD section.
   *
   * @param ttd_index TTD section index.
   * @return Train order assigned to the specified TTD.
   */
  [[nodiscard]] const cda_rail::index_vector&
  get_ttd_orders_of_ttd(size_t ttd_index) const;
  /**
   * @brief Returns the TTD section containing an edge, if any.
   *
   * @param edge Edge descriptor.
   * @return TTD section index, or `std::nullopt` if the edge is not in any
   *         configured TTD section.
   */
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
  /**
   * @brief Returns the train order stored for a vertex.
   *
   * @param vertex Vertex descriptor.
   * @return Train order assigned to the specified vertex.
   */
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
  /**
   * @brief Returns the stop positions of one train.
   *
   * @param train_id Train index.
   * @return Stop-position vector for the specified train.
   */
  [[nodiscard]] const std::vector<double>&
  get_stop_positions_of_tr(size_t train_id) const;

  // -------------------
  // HELPFUL VALUES
  // -------------------
  /**
   * @brief Returns the length of the current edge sequence of a train.
   *
   * @param tr Train index.
   * @return Route length of the current train edge sequence.
   */
  [[nodiscard]] double train_edge_length(size_t tr) const;
  /**
   * @brief Returns the route position at which an edge starts.
   *
   * @param train_id Train index.
   * @param edge_id Edge index on the route.
   * @return Position of the edge start in metres.
   */
  [[nodiscard]] double get_edge_position(size_t train_id, size_t edge_id) const;
  /**
   * @brief Returns the edge that contains a route position.
   *
   * @param train_id Train index.
   * @param position Position on the train route.
   * @return Edge index containing @p position.
   */
  [[nodiscard]] size_t get_edge_at_position(size_t train_id,
                                            double position) const;

  /**
   * @brief Checks whether a route end can serve as the next stop position.
   *
   * @param tr Train index.
   * @param edges Current route edges.
   * @return `true` if the route end is a valid stop position.
   */
  [[nodiscard]] bool
  is_route_end_valid_stop_pos(size_t                        tr,
                              const cda_rail::index_vector& edges) const {
    return get_instance()->is_route_end_valid_stop_pos(
        tr, edges, get_stop_positions_of_tr(tr).size());
  }
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

  /** @brief Replaces the full simulator state. */
  void set_simulator_state(SimulatorState state);

  /** @brief Replaces all train edge sequences. */
  void set_train_edges(std::vector<cda_rail::index_vector> tr_edges);
  /** @brief Replaces the edge sequence of one train. */
  void set_train_edges_of_tr(size_t train_id, cda_rail::index_vector edges);
  /** @brief Appends one edge to a train edge sequence. */
  void append_train_edge_to_tr(size_t train_id, size_t edge);

  /** @brief Replaces all TTD train orders. */
  void set_ttd_orders(std::vector<cda_rail::index_vector> orders);
  /** @brief Replaces the order stored for one TTD section. */
  void set_ttd_orders_of_ttd(size_t ttd_index, cda_rail::index_vector orders);

  /** @brief Replaces all vertex train orders. */
  void set_vertex_orders(std::vector<cda_rail::index_vector> orders);
  /** @brief Replaces the train order stored for one vertex. */
  void set_vertex_orders_of_vertex(cda_rail::Network::VertexInput const& vertex,
                                   cda_rail::index_vector orders);

  /** @brief Replaces all stop-position vectors. */
  void set_stop_positions(std::vector<std::vector<double>> positions);
  /** @brief Replaces the stop positions of one train. */
  void set_stop_positions_of_tr(size_t train_id, std::vector<double> positions);
  /** @brief Appends one stop position to a train. */
  void append_stop_position_to_tr(size_t train_id, double position);
  /** @brief Appends a stop edge reference to a train. */
  void append_stop_edge_to_tr(size_t train_id, size_t edge);
  /** @brief Appends the current route-end position as a stop for a train. */
  void append_current_stop_position_of_tr(size_t train_id);

  // ------------------
  // STATE HELPER
  // ------------------

  /**
   * @brief Checks whether the current simulator state is terminal.
   *
   * @return `true` if no further state extension is needed.
   */
  [[nodiscard]] bool is_final_state() const;

  // ------------------
  // SIMULATION
  // ------------------

  // virtual function
  /**
   * @brief Simulates the current state with explicit configuration flags.
   *
   * @param late_entry_possible Whether trains may enter later than scheduled.
   * @param limit_speed_by_leaving_edges Whether leaving edges constrain speed.
   * @param save_trajectories Whether to retain detailed trajectories.
   * @param disappear_at_partial_route_end Whether trains disappear at partial
   *        route ends.
   * @return Simulation results.
   */
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

  /**
   * @brief Computes braking distance for a train at a given speed.
   *
   * @param tr Train index.
   * @param v Speed.
   * @return Braking distance in metres.
   */
  [[nodiscard]] double tr_braking_distance(size_t tr, double v) const;

  /**
   * @brief Returns cumulative edge milestones for a train route.
   *
   * @param tr Train index.
   * @return Edge milestone positions in metres.
   */
  [[nodiscard]] std::vector<double> edge_milestones(size_t tr) const;

  /**
   * @brief Checks whether an edge is on a train's current route.
   *
   * @param tr Train index.
   * @param edge_id Edge index.
   * @return `true` if the edge is on the route.
   */
  [[nodiscard]] bool is_on_route(size_t tr, size_t edge_id) const;

  /**
   * @brief Returns, for each edge, the trains currently using it.
   *
   * @return Edge-indexed sets of train indices.
   */
  [[nodiscard]] std::vector<std::unordered_set<size_t>> tr_on_edges() const;

private:
  /**
   * @brief Computes the trains whose routes contain a path.
   *
   * @param edges Path edges to test.
   * @param tr_edges Per-train edge sequences.
   * @return Set of train indices using the path.
   */
  [[nodiscard]] static cda_rail::index_set
  trains_on_path_helper(cda_rail::index_vector const&              edges,
                        std::vector<cda_rail::index_vector> const& tr_edges);

public:
  /**
   * @brief Returns the trains whose current routes contain a path.
   *
   * @param edges Path edges to test.
   * @param also_reverse_path Whether to also test the reverse path.
   * @return Set of train indices using the path.
   */
  [[nodiscard]] cda_rail::index_set
  trains_on_path(cda_rail::index_vector const& edges,
                 bool also_reverse_path = false) const {
    return trains_on_path(edges, get_train_edges(), also_reverse_path);
  }
  /**
   * @brief Returns the trains from provided routes that contain a path.
   *
   * @param edges Path edges to test.
   * @param tr_edges Per-train edge sequences.
   * @param also_reverse_path Whether to also test the reverse path.
   * @return Set of train indices using the path.
   */
  [[nodiscard]] cda_rail::index_set
  trains_on_path(cda_rail::index_vector const&              edges,
                 std::vector<cda_rail::index_vector> const& tr_edges,
                 bool also_reverse_path = false) const {
    return trains_on_path(edges, tr_edges, get_instance()->get_const_network(),
                          also_reverse_path);
  }
  /**
   * @brief Returns the trains from provided routes that contain a path.
   *
   * @param edges Path edges to test.
   * @param tr_edges Per-train edge sequences.
   * @param network Network used for reverse-path lookup.
   * @param also_reverse_path Whether to also test the reverse path.
   * @return Set of train indices using the path.
   */
  [[nodiscard]] static cda_rail::index_set
  trains_on_path(cda_rail::index_vector const&              edges,
                 std::vector<cda_rail::index_vector> const& tr_edges,
                 Network const& network, bool also_reverse_path = false);

private:
  // -----------------------
  // EXCEPTION HELPER
  // -----------------------
  /** @brief Validates the configured TTD sections. */
  void check_ttd_sections(
      std::vector<cda_rail::index_set> const& ttd_sections) const;

  /** @brief Validates one train edge sequence. */
  void check_train_edges_for_tr(size_t                        tr,
                                cda_rail::index_vector const& edges) const;
  /** @brief Validates all train edge sequences. */
  void check_train_edges(
      std::vector<cda_rail::index_vector> const& train_edges) const;

  /** @brief Validates that all listed trains exist. */
  void check_if_all_tr_valid(cda_rail::index_vector const& train_ids) const;
  /** @brief Validates that a train list contains no duplicates. */
  static void check_if_all_tr_unique(cda_rail::index_vector const& train_ids);
  /** @brief Validates all TTD order vectors. */
  void
  check_ttd_orders(std::vector<cda_rail::index_vector> const& ttd_orders) const;
  /** @brief Validates all vertex order vectors. */
  void check_vertex_orders(
      std::vector<cda_rail::index_vector> const& vertex_orders) const;

  /** @brief Validates one train stop-position vector. */
  void check_stop_positions_for_tr(size_t                     tr,
                                   std::vector<double> const& positions) const;
  /** @brief Validates all stop-position vectors. */
  void check_stop_positions(
      std::vector<std::vector<double>> const& stop_positions) const;
};
} // namespace cda_rail::simulator

template <> struct std::hash<cda_rail::simulator::SimulatorState> {
  /**
   * @brief Computes a hash value for a simulator state.
   *
   * @param state State to hash.
   * @return Hash value.
   */
  size_t
  operator()(const cda_rail::simulator::SimulatorState& state) const noexcept;
}; // namespace std
