#pragma once

#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "EOMHelper.hpp"
#include "probleminstances/GeneralProblemInstance.hpp"

#include <algorithm>
#include <cstddef>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <type_traits>
#include <unordered_set>
#include <utility>
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
  std::vector<double>
      braking_times; // time (in seconds) at which each train had to brake due
                     // to approaching their route end
  std::vector<double>
      braking_distances; // distance before route end at which each train had to
                         // brake due to approaching their route end
  std::vector<double> vertex_headways; // for every vertex, earliest time at
                                       // which next train can enter
  std::vector<std::map<double, PosVel>>
      train_trajectories; // For every train, a map of time to position and
                          // velocity at that time
};

template <typename T> class GeneralSimulator {
  /**
   * This abstract class defines the interface for a general simulator to
   * simulate train trajectories given all routing, ordering, and stopping
   * information. It can, e.g., be used within search algorithms like A* for
   * objective evaluation. This abstract class defines all necessary functions
   * to be implemented and provides some general helper functions.
   */

  // T must be a child of GeneralProblemInstanceWithScheduleAndRoutes<S> for any
  // valid S
  template <typename S>
  static std::true_type
  is_gp_instance([[maybe_unused]] const instances::
                     GeneralProblemInstanceWithScheduleAndRoutes<S>* ptr) {
    return {};
  }
  static std::false_type is_gp_instance([[maybe_unused]] const void* ptr) {
    return {};
  }
  static_assert(
      decltype(is_gp_instance(std::declval<T*>()))::value,
      "T must be a child of GeneralProblemInstanceWithScheduleAndRoutes<S> for "
      "any valid S");

protected:
  std::shared_ptr<const T>            instance;
  std::vector<cda_rail::index_vector> ttd_sections;

  std::vector<cda_rail::index_vector> train_edges;
  std::vector<cda_rail::index_vector> ttd_orders;
  std::vector<cda_rail::index_vector> vertex_orders;
  std::vector<std::vector<double>>    stop_positions;

  // helper functions
  [[nodiscard]] double tr_braking_distance(size_t tr, double v) const {
    /**
     * Calculates the braking distance for a train with id `tr` at velocity `v`.
     *
     * @param tr: The id of the train for which the braking distance is
     * calculated.
     * @param v: The velocity at which the braking distance is calculated.
     *
     * @return: The braking distance for the train at the given velocity.
     */
    if (!instance->get_timetable().get_train_list().has_train(tr)) {
      throw cda_rail::exceptions::TrainNotExistentException(tr);
    }
    if (v < -EPS) {
      throw cda_rail::exceptions::InvalidInputException(
          "Velocity must be non-negative.");
    }
    if (v < 0) {
      return 0.0; // No braking distance if the train is not moving
    }
    return cda_rail::braking_distance(
        v, instance->get_train_list().get_train(tr).deceleration);
  };

  [[nodiscard]] std::vector<double> edge_milestones(size_t tr) const {
    /**
     * This function returns the individual milestones, i.e., the distance on
     * the individual route to each edges starting point. The last value is the
     * distance to the exit node.
     *
     * @param tr: The id of the train for which the milestones are calculated.
     * @return: A vector of doubles with the milestones for each edge of the
     * train.
     */
    if (!instance->get_timetable().get_train_list().has_train(tr)) {
      throw cda_rail::exceptions::TrainNotExistentException(tr);
    }
    const auto& edges = train_edges.at(tr);
    if (edges.empty()) {
      return {}; // No edges, no milestones
    }
    std::vector<double> milestones;
    milestones.reserve(edges.size() + 1);
    milestones.emplace_back(0.0); // First milestone is always 0
    for (const auto& edge_id : edges) {
      const auto& edge = instance->const_n().get_edge(edge_id);
      milestones.emplace_back(milestones.back() + edge.length);
    }
    return milestones;
  };

  [[nodiscard]] bool is_on_route(size_t tr, size_t edge_id) const {
    if (!instance->get_timetable().get_train_list().has_train(tr)) {
      throw cda_rail::exceptions::TrainNotExistentException(tr);
    }
    if (!instance->const_n().has_edge(edge_id)) {
      throw cda_rail::exceptions::EdgeNotExistentException(edge_id);
    }
    const auto& tr_edges = train_edges.at(tr);
    return std::ranges::contains(tr_edges, edge_id);
  };

  [[nodiscard]] std::vector<std::unordered_set<size_t>> tr_on_edges() const {
    /**
     * This function returns a vector of unordered sets, where each set
     * contains the indices of trains that are routed on a specific edge.
     */

    std::vector<std::unordered_set<size_t>> trains_on_edges(
        instance->const_n().number_of_edges());
    for (size_t tr = 0; tr < instance->get_timetable().get_train_list().size();
         ++tr) {
      const auto& edges = train_edges.at(tr);
      for (const auto& edge_id : edges) {
        trains_on_edges.at(edge_id).insert(tr);
      }
    }
    return trains_on_edges;
  };

  [[nodiscard]] std::optional<size_t> get_ttd(size_t edge_id) const {
    if (!instance->const_n().has_edge(edge_id)) {
      throw cda_rail::exceptions::EdgeNotExistentException(edge_id);
    }
    for (size_t ttd_index = 0; ttd_index < ttd_sections.size(); ++ttd_index) {
      const auto& ttd_section = ttd_sections[ttd_index];
      if (std::ranges::contains(ttd_section, edge_id)) {
        return ttd_index; // Found the TTD section containing the edge
      }
    }
    return {}; // Edge is not part of any TTD section
  };

public:
  explicit GeneralSimulator(T&                                  instance,
                            std::vector<cda_rail::index_vector> ttd_sections)
      : instance(std::make_shared<const T>(instance)),
        ttd_sections(std::move(ttd_sections)) {
    train_edges.resize(instance.get_timetable().get_train_list().size());
    ttd_orders.resize(this->ttd_sections.size());
    vertex_orders.resize(instance.const_n().number_of_vertices());
    stop_positions.resize(instance.get_timetable().get_train_list().size());
  };
  explicit GeneralSimulator(T&                                  instance,
                            std::vector<cda_rail::index_vector> ttd_sections,
                            std::vector<cda_rail::index_vector> train_edges,
                            std::vector<cda_rail::index_vector> ttd_orders,
                            std::vector<cda_rail::index_vector> vertex_orders,
                            std::vector<std::vector<double>>    stop_positions)
      : instance(std::make_shared<const T>(instance)),
        ttd_sections(std::move(ttd_sections)),
        train_edges(std::move(train_edges)), ttd_orders(std::move(ttd_orders)),
        vertex_orders(std::move(vertex_orders)),
        stop_positions(std::move(stop_positions)) {
    if (this->train_edges.size() !=
            this->instance->get_timetable().get_train_list().size() ||
        this->ttd_orders.size() != this->ttd_sections.size() ||
        this->vertex_orders.size() !=
            this->instance->const_n().number_of_vertices() ||
        this->stop_positions.size() !=
            this->instance->get_timetable().get_train_list().size()) {
      throw cda_rail::exceptions::InvalidInputException(
          "Simulator state vector sizes do not match the referenced "
          "instance.");
    }
  };

  [[nodiscard]] std::shared_ptr<const T> get_instance() const {
    return instance;
  }

  [[nodiscard]] double train_edge_length(size_t tr) const {
    if (train_edges.size() <= tr) {
      throw cda_rail::exceptions::TrainNotExistentException(tr);
    }
    return instance->const_n().length_of_path(train_edges.at(tr));
  };

  void set_train_edges(std::vector<cda_rail::index_vector> tr_edges) {
    if (tr_edges.size() != instance->get_timetable().get_train_list().size()) {
      throw cda_rail::exceptions::InvalidInputException(
          "Size of train_edges does not match number of trains in instance.");
    }
    train_edges = std::move(tr_edges);
  };
  void set_train_edges_of_tr(size_t train_id, cda_rail::index_vector edges) {
    if (!instance->get_timetable().get_train_list().has_train(train_id)) {
      throw cda_rail::exceptions::TrainNotExistentException(train_id);
    }
    train_edges.at(train_id) = std::move(edges);
  };
  void append_train_edge_to_tr(size_t train_id, size_t edge) {
    if (!instance->get_timetable().get_train_list().has_train(train_id)) {
      throw cda_rail::exceptions::TrainNotExistentException(train_id);
    }
    train_edges.at(train_id).push_back(edge);
  };
  [[nodiscard]] const std::vector<cda_rail::index_vector>&
  get_train_edges() const {
    return train_edges;
  };
  [[nodiscard]] const cda_rail::index_vector&
  get_train_edges_of_tr(size_t train_id) const {
    if (train_id >= train_edges.size()) {
      throw cda_rail::exceptions::TrainNotExistentException(train_id);
    }
    return train_edges.at(train_id);
  };

  void set_ttd_orders(std::vector<cda_rail::index_vector> orders) {
    if (orders.size() != ttd_sections.size()) {
      throw cda_rail::exceptions::InvalidInputException(
          "Size of ttd_orders does not match number of ttd sections in "
          "instance.");
    }
    ttd_orders = std::move(orders);
  };
  void set_ttd_orders_of_ttd(size_t ttd_index, cda_rail::index_vector orders) {
    if (ttd_index >= ttd_orders.size()) {
      throw cda_rail::exceptions::InvalidInputException(
          "TTD index out of bounds.");
    }
    ttd_orders.at(ttd_index) = std::move(orders);
  };
  [[nodiscard]] const std::vector<cda_rail::index_vector>&
  get_ttd_orders() const {
    return ttd_orders;
  };
  [[nodiscard]] const cda_rail::index_vector&
  get_ttd_orders_of_ttd(size_t ttd_index) const {
    if (ttd_index >= ttd_orders.size()) {
      throw cda_rail::exceptions::InvalidInputException(
          "TTD index out of bounds.");
    }
    return ttd_orders.at(ttd_index);
  };
  [[nodiscard]] const std::vector<cda_rail::index_vector>&
  get_ttd_sections() const {
    return ttd_sections;
  };

  void set_vertex_orders(std::vector<cda_rail::index_vector> orders) {
    if (orders.size() != instance->const_n().number_of_vertices()) {
      throw cda_rail::exceptions::InvalidInputException(
          "Size of vertex_orders does not match number of vertices in "
          "instance.");
    }
    vertex_orders = std::move(orders);
  };
  void set_vertex_orders_of_vertex(size_t                 vertex_id,
                                   cda_rail::index_vector orders) {
    if (vertex_id >= vertex_orders.size()) {
      throw cda_rail::exceptions::InvalidInputException(
          "Vertex index out of bounds.");
    }
    vertex_orders.at(vertex_id) = std::move(orders);
  };
  [[nodiscard]] const std::vector<cda_rail::index_vector>&
  get_vertex_orders() const {
    return vertex_orders;
  };
  [[nodiscard]] const cda_rail::index_vector&
  get_vertex_orders_of_vertex(size_t vertex_id) const {
    if (vertex_id >= vertex_orders.size()) {
      throw cda_rail::exceptions::InvalidInputException(
          "Vertex index out of bounds.");
    }
    return vertex_orders.at(vertex_id);
  };
  void set_stop_positions(std::vector<std::vector<double>> positions) {
    if (positions.size() != instance->get_timetable().get_train_list().size()) {
      throw cda_rail::exceptions::InvalidInputException(
          "Size of stop_positions does not match number of trains in "
          "instance.");
    }
    stop_positions = std::move(positions);
  };
  void set_stop_positions_of_tr(size_t              train_id,
                                std::vector<double> positions) {
    if (stop_positions.size() <= train_id) {
      throw cda_rail::exceptions::TrainNotExistentException(train_id);
    }
    if (positions.size() >
        instance->get_timetable().get_schedule(train_id).get_stops().size()) {
      throw cda_rail::exceptions::InvalidInputException(
          "Too many stop positions for train " + std::to_string(train_id) +
          ". Train has only " +
          std::to_string(instance->get_timetable()
                             .get_schedule(train_id)
                             .get_stops()
                             .size()) +
          " scheduled stops.");
    }
    stop_positions.at(train_id) = std::move(positions);
  };
  void append_stop_position_to_tr(size_t train_id, double position) {
    cda_rail::exceptions::throw_if_negative(position, "Stop position");
    if (train_id >= stop_positions.size()) {
      throw cda_rail::exceptions::TrainNotExistentException(train_id);
    }
    if (stop_positions.at(train_id).size() >=
        instance->get_timetable().get_schedule(train_id).get_stops().size()) {
      throw cda_rail::exceptions::ConsistencyException(
          "All scheduled stops for train " + std::to_string(train_id) +
          " are already set.");
    }
    if (!stop_positions.at(train_id).empty() &&
        position < stop_positions.at(train_id).back()) {
      throw cda_rail::exceptions::ConsistencyException(
          "Stop positions must be non-decreasing for train " +
          std::to_string(train_id) + ". Last position is " +
          std::to_string(stop_positions.at(train_id).back()) +
          ", new position is " + std::to_string(position) + ".");
    }
    stop_positions.at(train_id).push_back(position);
  };
  void append_stop_edge_to_tr(size_t train_id, size_t edge) {
    const auto& stop_positions_of_tr = get_stop_positions_of_tr(train_id);
    const auto& tr_stops =
        instance->get_timetable().get_schedule(train_id).get_stops();
    if (stop_positions_of_tr.size() >= tr_stops.size()) {
      throw cda_rail::exceptions::ConsistencyException(
          "All scheduled stops for train " + std::to_string(train_id) +
          " are already set.");
    }
    const auto& next_stop =
        tr_stops.at(stop_positions_of_tr.size()).get_station_name();
    const auto& next_stop_edges =
        instance->get_station_list().get_station(next_stop).tracks;
    if (!std::ranges::contains(next_stop_edges, edge)) {
      throw cda_rail::exceptions::ConsistencyException(
          "Edge " + std::to_string(edge) +
          " is not a valid stop edge for "
          "train " +
          std::to_string(train_id) + ". Next stop is " + next_stop + ".");
    }
    append_stop_position_to_tr(train_id, get_edge_position(train_id, edge));
  };
  void append_current_stop_position_of_tr(size_t train_id) {
    const auto& tr_edges = get_train_edges_of_tr(train_id);
    if (tr_edges.empty()) {
      throw cda_rail::exceptions::ConsistencyException(
          "Train " + std::to_string(train_id) +
          " has no edges in its route. Cannot append current stop position.");
    }
    append_stop_edge_to_tr(train_id, tr_edges.back());
  };
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
  };
  [[nodiscard]] bool
  is_route_end_valid_stop_pos(size_t                        tr,
                              const cda_rail::index_vector& edges) const {
    if (!instance->get_timetable().get_train_list().has_train(tr)) {
      throw cda_rail::exceptions::TrainNotExistentException(tr);
    }
    const auto& tr_length =
        instance->get_timetable().get_train_list().get_train(tr).length;
    const auto& tr_schedule =
        instance->get_timetable().get_schedule(tr).get_stops();
    if (stop_positions.at(tr).size() >= tr_schedule.size()) {
      // All stops have been set, hence, no further stop is possible
      return false;
    }
    const auto& next_station_name =
        tr_schedule.at(stop_positions.at(tr).size()).get_station_name();
    const auto& next_station_tracks =
        instance->get_station_list().get_station(next_station_name).tracks;
    double len = 0;
    for (auto it = edges.rbegin(); (len < tr_length) && (it != edges.rend());
         ++it) {
      if (!std::ranges::contains(next_station_tracks, *it)) {
        // Track does not belong to the next station
        return false;
      }
      len += instance->const_n().get_edge(*it).length;
    }

    return len >= tr_length;
  };
  [[nodiscard]] const std::vector<std::vector<double>>&
  get_stop_positions() const {
    return stop_positions;
  };
  [[nodiscard]] const std::vector<double>&
  get_stop_positions_of_tr(size_t train_id) const {
    if (train_id >= stop_positions.size()) {
      throw cda_rail::exceptions::TrainNotExistentException(train_id);
    }
    return stop_positions.at(train_id);
  };
  [[nodiscard]] double get_edge_position(size_t train_id,
                                         size_t edge_id) const {
    if (!instance->const_n().has_edge(edge_id)) {
      throw cda_rail::exceptions::EdgeNotExistentException(edge_id);
    }
    const auto& tr_edges = get_train_edges_of_tr(train_id);
    double      pos      = 0.0;
    for (const auto& edge : tr_edges) {
      pos += instance->const_n().get_edge(edge).length;
      if (edge == edge_id) {
        return pos;
      }
    }
    throw cda_rail::exceptions::ConsistencyException(
        "Edge " + std::to_string(edge_id) + " not found in train " +
        std::to_string(train_id) + "'s route.");
  };

  virtual ~GeneralSimulator() = default;

  // State helper (not needed for simulation itself)
  [[nodiscard]] bool is_final_state() const {
    for (size_t tr = 0; tr < instance->get_timetable().get_train_list().size();
         ++tr) {
      if (train_edges.at(tr).empty()) {
        return false;
      }
      if (instance->get_schedule(tr).get_stops().size() !=
          stop_positions.at(tr).size()) {
        return false; // Not all stops have been set for the train
      }
      if (instance->const_n().get_edge(train_edges.at(tr).back()).target !=
          instance->get_schedule(tr).get_exit()) {
        return false; // Train has not reached the end of its route
      }
    }
    return true; // All trains have reached the end of their route and all stops
    // have been set
  };

  [[nodiscard]] bool check_consistency() const {
    if (!instance->check_consistency(false)) {
      return false;
    }

    // All vectors are of correct size
    if (train_edges.size() !=
        instance->get_timetable().get_train_list().size()) {
      return false;
    }
    if (ttd_orders.size() != ttd_sections.size()) {
      return false;
    }
    if (vertex_orders.size() != instance->const_n().number_of_vertices()) {
      return false;
    }
    if (stop_positions.size() !=
        instance->get_timetable().get_train_list().size()) {
      return false;
    }

    // All edges are valid
    for (const auto& edges : train_edges) {
      for (const auto& edge : edges) {
        if (!instance->const_n().has_edge(edge)) {
          return false;
        }
      }
    }

    // All edges are valid successors of each other
    for (const auto& edges : train_edges) {
      const auto& s = edges.size();
      for (size_t i = 0; s > 1 && i < s - 1; ++i) {
        if (!instance->const_n().is_valid_successor(edges[i], edges[i + 1])) {
          return false;
        }
      }
    }

    // If not empty, the first edge of each train must be an entry edge
    for (size_t train_id = 0; train_id < train_edges.size(); ++train_id) {
      if (!train_edges[train_id].empty()) {
        const auto& first_edge_id = train_edges[train_id].front();
        const auto& first_edge    = instance->const_n().get_edge(first_edge_id);
        if (first_edge.source !=
            instance->get_timetable().get_schedule(train_id).get_entry()) {
          return false;
        }
      }
    }

    // ttd_orders only contains valid train indices
    for (const auto& orders : ttd_orders) {
      std::unordered_set<size_t> seen;
      for (const auto& train_id : orders) {
        if (!instance->get_timetable().get_train_list().has_train(train_id) ||
            !seen.insert(train_id).second) {
          return false;
        }
      }
    }

    // vertex_orders only contains valid train indices
    for (const auto& orders : vertex_orders) {
      std::unordered_set<size_t> seen;
      for (const auto& train_id : orders) {
        if (!instance->get_timetable().get_train_list().has_train(train_id) ||
            !seen.insert(train_id).second) {
          return false;
        }
      }
    }

    // vertex orders only contain trains that are entering or leaving the
    // network through the respective vertex
    for (size_t vertex_id = 0; vertex_id < vertex_orders.size(); ++vertex_id) {
      for (const auto& train_id : vertex_orders[vertex_id]) {
        if (vertex_id !=
                instance->get_timetable().get_schedule(train_id).get_entry() &&
            vertex_id !=
                instance->get_timetable().get_schedule(train_id).get_exit()) {
          return false;
        }
      }
    }

    // Every train can have at most as many stop positions as scheduled stops
    for (size_t train_id = 0;
         train_id < instance->get_timetable().get_train_list().size();
         ++train_id) {
      const auto train_schedule_size =
          instance->get_timetable().get_schedule(train_id).get_stops().size();
      if (stop_positions.at(train_id).size() > train_schedule_size) {
        return false; // Too many stop positions for the train
      }
      // stop_positions.at(train_id) must be non-negative and sorted
      if (std::ranges::any_of(stop_positions.at(train_id),
                              [](double pos) { return pos < 0; })) {
        return false; // Negative stop position found
      }
      if (!std::ranges::is_sorted(stop_positions.at(train_id))) {
        return false; // Stop positions are not sorted
      }
      if (!train_edges.at(train_id).empty()) {
        const auto route_len = train_edge_length(train_id);
        if (std::ranges::any_of(
                stop_positions.at(train_id),
                [route_len](double pos) { return pos > route_len + EPS; })) {
          return false; // Stop lies beyond the routed path
        }
      }
    }

    return true;
  };

  // virtual function
  [[nodiscard]] virtual SimulatorResults
  simulate(bool late_entry_possible, bool late_exit_possible,
           bool late_stop_possible, bool limit_speed_by_leaving_edges,
           bool save_trajectories) const = 0;

  [[nodiscard]] SimulatorResults
  simulate(bool late_entry_possible = false, bool late_exit_possible = false,
           bool late_stop_possible = false,
           bool limit_speed_by_leaving_edges =
               true) const { // default values for virtual function
    return simulate(late_entry_possible, late_exit_possible, late_stop_possible,
                    limit_speed_by_leaving_edges, false);
  };
};

} // namespace cda_rail::simulator
