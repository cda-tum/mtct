#pragma once

#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "probleminstances/GeneralProblemInstance.hpp"

#include <algorithm>
#include <cstddef>
#include <map>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace cda_rail::simulator {

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

  std::vector<std::map<int, std::pair<double, double>>>
      train_trajectories; // time -> {pos, vel}

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
        stop_positions(std::move(stop_positions)) {};

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
    return is_route_end_valid_stop_pos(tr, train_edges.at(tr));
  };
  [[nodiscard]] bool
  is_route_end_valid_stop_pos(size_t tr, cda_rail::index_vector edges) const {
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
  [[nodiscard]] std::vector<std::map<int, std::pair<double, double>>>
  get_last_trajectories() const {
    return train_trajectories;
  };
};

} // namespace cda_rail::simulator
