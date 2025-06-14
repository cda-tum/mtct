#pragma once

#include "CustomExceptions.hpp"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"

// NOLINTNEXTLINE(misc-include-cleaner)
#include "gtest/gtest_prod.h"
#include <cstddef>
#include <memory>
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
#endif

  double braking_distance(size_t tr, double v);

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
