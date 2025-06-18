#pragma once

#include "CustomExceptions.hpp"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"

// NOLINTNEXTLINE(misc-include-cleaner)
#include "gtest/gtest_prod.h"
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <memory>
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
#endif

namespace cda_rail::simulator {

class GreedySimulator {
  std::shared_ptr<cda_rail::instances::GeneralPerformanceOptimizationInstance>
                                   instance;
  std::vector<std::vector<size_t>> ttd_sections;

  std::vector<std::vector<size_t>> train_edges;
  std::vector<std::vector<size_t>> ttd_orders;
  std::vector<std::vector<size_t>> entry_orders;

private:
#if TEST_FRIENDS
  FRIEND_TEST(::GreedySimulator, BasicPrivateFunctions);
  FRIEND_TEST(::GreedySimulator, EdgePositions);
  FRIEND_TEST(::GreedySimulator, TrainsOnEdges);
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
  [[nodiscard]] std::tuple<bool, std::pair<double, double>,
                           std::pair<double, double>>
  get_position_on_route_edge(size_t tr, double pos, size_t edge_number,
                             std::vector<double> milestones = {}) const;
  [[nodiscard]] std::tuple<bool, std::pair<double, double>,
                           std::pair<double, double>>
  get_position_on_edge(size_t tr, double pos, size_t edge_id,
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
  [[nodiscard]] bool is_on_ttd(size_t tr, size_t ttd, double pos,
                               TTDOccupationType occupation_type =
                                   TTDOccupationType::OnlyOccupied) const;
  [[nodiscard]] bool is_on_or_behind_ttd(size_t tr, size_t ttd,
                                         double pos) const {
    return is_on_ttd(tr, ttd, pos, TTDOccupationType::OccupiedOrBehind);
  };
  [[nodiscard]] bool is_behind_ttd(size_t tr, size_t ttd, double pos) const {
    return is_on_ttd(tr, ttd, pos, TTDOccupationType::OnlyBehind);
  };
  [[nodiscard]] std::vector<std::unordered_set<size_t>> tr_on_edges() const;
  [[nodiscard]] std::optional<size_t> get_ttd(size_t edge_id) const;

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
    entry_orders.resize(instance.const_n().number_of_vertices());
  };
  explicit GreedySimulator(
      cda_rail::instances::GeneralPerformanceOptimizationInstance& instance,
      std::vector<std::vector<size_t>>                             ttd_sections,
      std::vector<std::vector<size_t>>                             train_edges,
      std::vector<std::vector<size_t>>                             ttd_orders,
      std::vector<std::vector<size_t>>                             entry_orders)
      : instance(std::make_shared<
                 cda_rail::instances::GeneralPerformanceOptimizationInstance>(
            instance)),
        ttd_sections(std::move(ttd_sections)),
        train_edges(std::move(train_edges)), ttd_orders(std::move(ttd_orders)),
        entry_orders(std::move(entry_orders)) {};
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

  void set_entry_orders(std::vector<std::vector<size_t>> orders) {
    if (orders.size() != instance->const_n().number_of_vertices()) {
      throw cda_rail::exceptions::InvalidInputException(
          "Size of entry_orders does not match number of vertices in "
          "instance.");
    }
    entry_orders = std::move(orders);
  };
  void set_entry_orders_of_vertex(size_t              vertex_id,
                                  std::vector<size_t> orders) {
    if (vertex_id >= entry_orders.size()) {
      throw cda_rail::exceptions::InvalidInputException(
          "Vertex index out of bounds.");
    }
    entry_orders.at(vertex_id) = std::move(orders);
  };
  [[nodiscard]] const std::vector<std::vector<size_t>>&
  get_entry_orders() const {
    return entry_orders;
  };
  [[nodiscard]] const std::vector<size_t>&
  get_entry_orders_of_vertex(size_t vertex_id) const {
    if (vertex_id >= entry_orders.size()) {
      throw cda_rail::exceptions::InvalidInputException(
          "Vertex index out of bounds.");
    }
    return entry_orders.at(vertex_id);
  };

  [[nodiscard]] std::pair<bool, std::vector<int>>
  simulate(int dt = 6, bool late_entry_possible = false,
           bool late_exit_possible = false,
           bool late_stop_possible = false) const;
};

} // namespace cda_rail::simulator
