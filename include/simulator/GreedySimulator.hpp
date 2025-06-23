#pragma once

#include "CustomExceptions.hpp"
#include "datastructure/Train.hpp"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"

// NOLINTNEXTLINE(misc-include-cleaner)
#include "gtest/gtest_prod.h"
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <unordered_set>
#include <utility>
#include <vector>

// If TEST_FRIENDS has value true, the corresponding test is friended to test
// complex private functions
// This is not good practice, however after consideration, it was decided that
// - it is not reasonable to make the functions public
// - they have a complexity that should be tested
// - by only testing the overall solution, there is too much code tested at once
#ifndef TEST_FRIENDS
#define TEST_FRIENDS false
#endif
#if TEST_FRIENDS
class GreedySimulator;
class GreedySimulator_BasicPrivateFunctions_Test;
class GreedySimulator_EdgePositions_Test;
class GreedySimulator_TrainsOnEdges_Test;
class GreedySimulator_IsOkToEnter_Test;
class GreedySimulator_AbsoluteDistanceMA_Test;
class GreedySimulator_FutureSpeedRestrictionConstraints_Test;
class GreedySimulator_EoMDisplacement_Test;
class GreedySimulator_NextStopMA_Test;
class GreedySimulator_TimeToExitObjective_Test;
class GreedySimulator_ExitHeadwaySpeedConstraint_Test;
#endif

namespace cda_rail::simulator {

class GreedySimulator {
  std::shared_ptr<cda_rail::instances::GeneralPerformanceOptimizationInstance>
                                   instance;
  std::vector<std::vector<size_t>> ttd_sections;

  std::vector<std::vector<size_t>> train_edges;
  std::vector<std::vector<size_t>> ttd_orders;
  std::vector<std::vector<size_t>> vertex_orders;
  std::vector<std::vector<double>> stop_positions;

private:
#if TEST_FRIENDS
  FRIEND_TEST(::GreedySimulator, BasicPrivateFunctions);
  FRIEND_TEST(::GreedySimulator, EdgePositions);
  FRIEND_TEST(::GreedySimulator, TrainsOnEdges);
  FRIEND_TEST(::GreedySimulator, IsOkToEnter);
  FRIEND_TEST(::GreedySimulator, AbsoluteDistanceMA);
  FRIEND_TEST(::GreedySimulator, FutureSpeedRestrictionConstraints);
  FRIEND_TEST(::GreedySimulator, EoMDisplacement);
  FRIEND_TEST(::GreedySimulator, NextStopMA);
  FRIEND_TEST(::GreedySimulator, TimeToExitObjective);
  FRIEND_TEST(::GreedySimulator, ExitHeadwaySpeedConstraint);
#endif

  enum class TTDOccupationType : std::uint8_t {
    OnlyOccupied,
    OnlyBehind,
    OccupiedOrBehind
  };

  [[nodiscard]] double braking_distance(size_t tr, double v) const;
  [[nodiscard]] std::pair<bool, std::unordered_set<size_t>>
  get_entering_trains(int t, const std::unordered_set<size_t>& tr_present,
                      const std::unordered_set<size_t>& tr_left,
                      bool late_entry_possible) const;
  [[nodiscard]] std::vector<double> edge_milestones(size_t tr) const;
  [[nodiscard]] std::tuple<bool, std::pair<bool, bool>,
                           std::pair<double, double>>
  get_position_on_route_edge(size_t tr, const std::pair<double, double>& pos,
                             size_t              edge_number,
                             std::vector<double> milestones = {}) const;
  [[nodiscard]] std::tuple<bool, std::pair<bool, bool>,
                           std::pair<double, double>>
  get_position_on_edge(size_t tr, const std::pair<double, double>& pos,
                       size_t              edge_id,
                       std::vector<double> milestones = {}) const {
    if (!instance->get_timetable().get_train_list().has_train(tr)) {
      throw cda_rail::exceptions::TrainNotExistentException(tr);
    }
    if (!instance->const_n().has_edge(edge_id)) {
      throw cda_rail::exceptions::EdgeNotExistentException(edge_id);
    }
    const auto& tr_edges = train_edges.at(tr);
    const auto  edge_number =
        std::find(tr_edges.begin(), tr_edges.end(), edge_id);
    if (edge_number == tr_edges.end()) {
      throw cda_rail::exceptions::ConsistencyException(
          "Train " + std::to_string(tr) + " does not have edge " +
          std::to_string(edge_id) + " in its route.");
    }
    const auto edge_index = std::distance(tr_edges.begin(), edge_number);
    return get_position_on_route_edge(tr, pos, edge_index,
                                      std::move(milestones));
  };
  [[nodiscard]] bool is_on_route(size_t tr, size_t edge_id) const {
    if (!instance->get_timetable().get_train_list().has_train(tr)) {
      throw cda_rail::exceptions::TrainNotExistentException(tr);
    }
    if (!instance->const_n().has_edge(edge_id)) {
      throw cda_rail::exceptions::EdgeNotExistentException(edge_id);
    }
    const auto& tr_edges = train_edges.at(tr);
    return std::find(tr_edges.begin(), tr_edges.end(), edge_id) !=
           tr_edges.end();
  };
  [[nodiscard]] bool is_on_ttd(size_t tr, size_t ttd,
                               const std::pair<double, double>& pos,
                               TTDOccupationType occupation_type =
                                   TTDOccupationType::OnlyOccupied) const;
  [[nodiscard]] bool
  is_on_or_behind_ttd(size_t tr, size_t ttd,
                      const std::pair<double, double>& pos) const {
    return is_on_ttd(tr, ttd, pos, TTDOccupationType::OccupiedOrBehind);
  };
  [[nodiscard]] bool is_behind_ttd(size_t tr, size_t ttd,
                                   const std::pair<double, double>& pos) const {
    return is_on_ttd(tr, ttd, pos, TTDOccupationType::OnlyBehind);
  };
  [[nodiscard]] std::vector<std::unordered_set<size_t>> tr_on_edges() const;
  [[nodiscard]] std::optional<size_t> get_ttd(size_t edge_id) const;
  [[nodiscard]] bool                  is_ok_to_enter(
                       size_t tr, const std::vector<std::pair<double, double>>& train_positions,
                       const std::unordered_set<size_t>&              trains_in_network,
                       const std::vector<std::unordered_set<size_t>>& tr_on_edges) const;
  [[nodiscard]] double max_displacement(const Train& train, double v_0,
                                        int dt) const;
  [[nodiscard]] double get_absolute_distance_ma(
      size_t tr, double max_displacement,
      const std::vector<std::pair<double, double>>&  train_positions,
      const std::unordered_set<size_t>&              trains_in_network,
      const std::vector<std::unordered_set<size_t>>& tr_on_edges) const;
  [[nodiscard]] std::pair<double, double>
  get_future_max_speed_constraints(size_t tr, const Train& train, double pos,
                                   double v_0, double max_displacement, int dt,
                                   bool also_limit_by_leaving_edges) const;
  [[nodiscard]] static std::pair<double, double>
  speed_restriction_helper(double ma, double max_v, double pos,
                           double vertex_pos, double v_0, double v_m, double d,
                           int dt);
  [[nodiscard]] static double
                       get_next_stop_ma(double max_displacement, double pos,
                                        std::optional<double> next_stop_pos);
  [[nodiscard]] double get_max_speed_exit_headway(size_t tr, const Train& train,
                                                  double pos, double v_0, int h,
                                                  int dt) const;
  [[nodiscard]] static std::pair<bool, double>
  time_to_exit_objective(double v_0, double v_1, double v_e, double s, double a,
                         double d, int dt);

  [[nodiscard]] std::pair<double, double> get_ma_and_vmax(
      size_t tr, int t, double v_0, double h, int dt,
      const std::vector<std::pair<double, double>>&  train_positions,
      const std::unordered_set<size_t>&              trains_in_network,
      const std::unordered_set<size_t>&              trains_left,
      const std::vector<std::unordered_set<size_t>>& tr_on_edges) const;

public:
  explicit GreedySimulator(
      cda_rail::instances::GeneralPerformanceOptimizationInstance& instance,
      std::vector<std::vector<size_t>>                             ttd_sections)
      : instance(std::make_shared<
                 cda_rail::instances::GeneralPerformanceOptimizationInstance>(
            instance)),
        ttd_sections(std::move(ttd_sections)) {
    train_edges.resize(instance.get_timetable().get_train_list().size());
    ttd_orders.resize(this->ttd_sections.size());
    vertex_orders.resize(instance.const_n().number_of_vertices());
    stop_positions.resize(instance.get_timetable().get_train_list().size());
  };
  explicit GreedySimulator(
      cda_rail::instances::GeneralPerformanceOptimizationInstance& instance,
      std::vector<std::vector<size_t>>                             ttd_sections,
      std::vector<std::vector<size_t>>                             train_edges,
      std::vector<std::vector<size_t>>                             ttd_orders,
      std::vector<std::vector<size_t>> vertex_orders,
      std::vector<std::vector<double>> stop_positions)
      : instance(std::make_shared<
                 cda_rail::instances::GeneralPerformanceOptimizationInstance>(
            instance)),
        ttd_sections(std::move(ttd_sections)),
        train_edges(std::move(train_edges)), ttd_orders(std::move(ttd_orders)),
        vertex_orders(std::move(vertex_orders)),
        stop_positions(std::move(stop_positions)) {};
  GreedySimulator() = delete;
  [[nodiscard]] bool check_consistency() const;

  void set_train_edges(std::vector<std::vector<size_t>> tr_edges) {
    if (tr_edges.size() != instance->get_timetable().get_train_list().size()) {
      throw cda_rail::exceptions::InvalidInputException(
          "Size of train_edges does not match number of trains in instance.");
    }
    train_edges = std::move(tr_edges);
  };
  void set_train_edges_of_tr(size_t train_id, std::vector<size_t> edges) {
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
  [[nodiscard]] const std::vector<std::vector<size_t>>&
  get_train_edges() const {
    return train_edges;
  };
  [[nodiscard]] const std::vector<size_t>&
  get_train_edges_of_tr(size_t train_id) const {
    if (train_id >= train_edges.size()) {
      throw cda_rail::exceptions::TrainNotExistentException(train_id);
    }
    return train_edges.at(train_id);
  };

  void set_ttd_orders(std::vector<std::vector<size_t>> orders) {
    if (orders.size() != ttd_sections.size()) {
      throw cda_rail::exceptions::InvalidInputException(
          "Size of ttd_orders does not match number of ttd sections in "
          "instance.");
    }
    ttd_orders = std::move(orders);
  };
  void set_ttd_orders_of_ttd(size_t ttd_index, std::vector<size_t> orders) {
    if (ttd_index >= ttd_orders.size()) {
      throw cda_rail::exceptions::InvalidInputException(
          "TTD index out of bounds.");
    }
    ttd_orders.at(ttd_index) = std::move(orders);
  };
  [[nodiscard]] const std::vector<std::vector<size_t>>& get_ttd_orders() const {
    return ttd_orders;
  };
  [[nodiscard]] const std::vector<size_t>&
  get_ttd_orders_of_ttd(size_t ttd_index) const {
    if (ttd_index >= ttd_orders.size()) {
      throw cda_rail::exceptions::InvalidInputException(
          "TTD index out of bounds.");
    }
    return ttd_orders.at(ttd_index);
  };

  void set_vertex_orders(std::vector<std::vector<size_t>> orders) {
    if (orders.size() != instance->const_n().number_of_vertices()) {
      throw cda_rail::exceptions::InvalidInputException(
          "Size of vertex_orders does not match number of vertices in "
          "instance.");
    }
    vertex_orders = std::move(orders);
  };
  void set_vertex_orders_of_vertex(size_t              vertex_id,
                                   std::vector<size_t> orders) {
    if (vertex_id >= vertex_orders.size()) {
      throw cda_rail::exceptions::InvalidInputException(
          "Vertex index out of bounds.");
    }
    vertex_orders.at(vertex_id) = std::move(orders);
  };
  [[nodiscard]] const std::vector<std::vector<size_t>>&
  get_vertex_orders() const {
    return vertex_orders;
  };
  [[nodiscard]] const std::vector<size_t>&
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
    if (position < 0) {
      throw cda_rail::exceptions::InvalidInputException(
          "Stop position must be non-negative.");
    }
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
    if (std::find(next_stop_edges.begin(), next_stop_edges.end(), edge) ==
        next_stop_edges.end()) {
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
  }
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

  [[nodiscard]] std::pair<bool, std::vector<int>>
  simulate(int dt = 6, bool late_entry_possible = false,
           bool late_exit_possible = false,
           bool late_stop_possible = false) const;
};

} // namespace cda_rail::simulator
