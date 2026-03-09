#pragma once

#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "datastructure/Train.hpp"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"
#include "simulator/GeneralSimulator.hpp"

// NOLINTNEXTLINE(misc-include-cleaner)
#include "gtest/gtest_prod.h"
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iterator>
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
class GreedySimulator_MAandMaxV_Test;
class GreedySimulator_MAtoV_Test;
class GreedySimulator_MoveTrain_Test;
class GreedySimulator_UpdateRearPositions_Test;
class GreedySimulator_ScheduleFeasibility_Test;
class GreedySimulator_ReverseEdgeMA_Test;
class GreedySimulator_ExitVertexOrder_Test;
class GreedySimulator_FutureSpeedRestrictionConstraintsAfterLeaving_Test;
#endif

namespace cda_rail::simulator {

#define GREEDY_SIMULATOR_MAX_TIME_FACTOR 10

class GreedySimulator
    : public GeneralSimulator<
          cda_rail::instances::GeneralPerformanceOptimizationInstance> {
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
  FRIEND_TEST(::GreedySimulator, MAandMaxV);
  FRIEND_TEST(::GreedySimulator, MAtoV);
  FRIEND_TEST(::GreedySimulator, MoveTrain);
  FRIEND_TEST(::GreedySimulator, UpdateRearPositions);
  FRIEND_TEST(::GreedySimulator, ScheduleFeasibility);
  FRIEND_TEST(::GreedySimulator, ReverseEdgeMA);
  FRIEND_TEST(::GreedySimulator, ExitVertexOrder);
  FRIEND_TEST(::GreedySimulator, FutureSpeedRestrictionConstraintsAfterLeaving);
#endif

  enum class TTDOccupationType : std::uint8_t {
    OnlyOccupied,
    OnlyBehind,
    OccupiedOrBehind
  };
  enum class DestinationType : std::uint8_t { None, Network, Station, Edge };

  // private simulator helper functions
  [[nodiscard]] std::pair<bool, std::unordered_set<size_t>>
  get_entering_trains(int t, const std::unordered_set<size_t>& tr_present,
                      const std::unordered_set<size_t>& tr_left,
                      const std::unordered_set<size_t>& tr_finished_simulating,
                      bool late_entry_possible) const;

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
    const auto& tr_edges    = train_edges.at(tr);
    const auto  edge_number = std::ranges::find(tr_edges, edge_id);
    if (edge_number == tr_edges.end()) {
      throw cda_rail::exceptions::ConsistencyException(
          "Train " + std::to_string(tr) + " does not have edge " +
          std::to_string(edge_id) + " in its route.");
    }
    const auto edge_index = std::distance(tr_edges.begin(), edge_number);
    return get_position_on_route_edge(tr, pos, edge_index,
                                      std::move(milestones));
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

  [[nodiscard]] bool is_ok_to_enter(
      size_t tr, const std::vector<std::pair<double, double>>& train_positions,
      const std::vector<double>&                     train_velocities,
      const std::unordered_set<size_t>&              trains_in_network,
      const std::vector<std::unordered_set<size_t>>& tr_on_edges) const;

  [[nodiscard]] static double max_displacement(const Train& train, double v_0,
                                               int dt);

  [[nodiscard]] double get_absolute_distance_ma(
      size_t tr, double max_displacement,
      const std::vector<std::pair<double, double>>&  train_positions,
      const std::vector<double>&                     train_velocities,
      const std::unordered_set<size_t>&              trains_in_network,
      const std::unordered_set<size_t>&              trains_left,
      const std::vector<std::unordered_set<size_t>>& tr_on_edges) const;

  [[nodiscard]] std::pair<double, double>
  get_future_max_speed_constraints(size_t tr, const Train& train, double pos,
                                   double v_0, double max_displacement, int dt,
                                   bool also_limit_by_leaving_edges) const;

  [[nodiscard]] double
  get_exit_vertex_order_ma(size_t tr, const Train& train, double pos,
                           double                            max_displacement,
                           const std::unordered_set<size_t>& trains_in_network,
                           const std::unordered_set<size_t>& trains_left) const;

  [[nodiscard]] static std::pair<double, double>
  speed_restriction_helper(double ma, double max_v, double pos,
                           double vertex_pos, double v_0, double v_m, double d,
                           int dt);

  [[nodiscard]] static double
  get_next_stop_ma(double max_displacement, double pos, double next_stop_pos);

  [[nodiscard]] double get_max_speed_exit_headway(size_t tr, const Train& train,
                                                  double pos, double v_0, int h,
                                                  int dt) const;

  [[nodiscard]] static std::pair<bool, double>
  time_to_exit_objective(double v_0, double v_1, double v_e, double s, double a,
                         double d, int dt);

  [[nodiscard]] std::pair<double, double>
  get_ma_and_maxv(size_t tr, const std::vector<double>& train_velocities,
                  std::optional<size_t> next_stop, int h, int dt,
                  const std::vector<std::pair<double, double>>& train_positions,
                  const std::unordered_set<size_t>& trains_in_network,
                  const std::unordered_set<size_t>& trains_left,
                  const std::vector<std::unordered_set<size_t>>& tr_on_edges,
                  bool also_limit_speed_by_leaving_edges) const;

  [[nodiscard]] static double get_v1_from_ma(double v_0, double ma, double d,
                                             int dt);

  [[nodiscard]] static bool
  move_train(size_t tr, double v_0, double v_1, double ma, int dt,
             std::vector<std::pair<double, double>>& train_positions);

  void update_rear_positions(
      std::vector<std::pair<double, double>>& train_positions) const;

  [[nodiscard]] bool is_feasible_to_schedule(
      int t, const std::vector<std::optional<size_t>>& next_stop_id,
      const std::vector<std::pair<double, double>>& train_positions,
      const std::unordered_set<size_t>&             trains_in_network,
      const std::unordered_set<size_t>&             trains_left,
      const std::unordered_set<size_t>&             trains_finished_simulating,
      bool late_entry_possible, bool late_exit_possible,
      bool late_stop_possible) const;

  [[nodiscard]] DestinationType
  tr_reached_end(size_t                                        tr,
                 const std::vector<std::pair<double, double>>& train_pos) const;

public:
  // Constructors
  explicit GreedySimulator(
      cda_rail::instances::GeneralPerformanceOptimizationInstance& instance,
      std::vector<cda_rail::index_vector>                          ttd_sections)
      : GeneralSimulator<instances::GeneralPerformanceOptimizationInstance>(
            instance, std::move(ttd_sections)) {};
  explicit GreedySimulator(
      cda_rail::instances::GeneralPerformanceOptimizationInstance& instance,
      std::vector<cda_rail::index_vector>                          ttd_sections,
      std::vector<cda_rail::index_vector>                          train_edges,
      std::vector<cda_rail::index_vector>                          ttd_orders,
      std::vector<cda_rail::index_vector> vertex_orders,
      std::vector<std::vector<double>>    stop_positions)
      : GeneralSimulator<instances::GeneralPerformanceOptimizationInstance>(
            instance, std::move(ttd_sections), std::move(train_edges),
            std::move(ttd_orders), std::move(vertex_orders),
            std::move(stop_positions)) {};
  GreedySimulator()           = delete;
  ~GreedySimulator() override = default;

  using GeneralSimulator::simulate;
  [[nodiscard]] SimulatorResults
  simulate(int dt = 6, bool late_entry_possible = false,
           bool late_exit_possible = false, bool late_stop_possible = false,
           bool limit_speed_by_leaving_edges = true,
           bool save_trajectories            = false) const;

  [[nodiscard]] SimulatorResults
  simulate(bool late_entry_possible, bool late_exit_possible,
           bool late_stop_possible, bool limit_speed_by_leaving_edges,
           bool save_trajectories) const override {
    return simulate(6, late_entry_possible, late_exit_possible,
                    late_stop_possible, limit_speed_by_leaving_edges,
                    save_trajectories);
  }
};

} // namespace cda_rail::simulator
