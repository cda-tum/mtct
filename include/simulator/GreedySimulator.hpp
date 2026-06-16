#pragma once

#include "Definitions.hpp"
#include "datastructure/Train.hpp"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"
#include "simulator/GeneralSimulator.hpp"

// NOLINTNEXTLINE(misc-include-cleaner)
#include "gtest/gtest_prod.h"
#include <cstddef>
#include <cstdint>
#include <optional>
#include <unordered_set>
#include <utility>
#include <vector>

enum : std::uint16_t { CycleLimit = 1000 };

// If TEST_FRIENDS has value true, the corresponding test is friended to test
// complex private functions.
// This is not good practice. However, after consideration, it was decided that
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
class GreedySimulator_FutureMaxSpeedWithMiddleEdge_Test;
class GreedySimulator_FutureMaxSpeedWithBlockedVertices_Test;
class GreedySimulator_EoMDisplacement_Test;
class GreedySimulator_NextStopMA_Test;
class GreedySimulator_MAandMaxV_Test;
class GreedySimulator_MAtoV_Test;
class GreedySimulator_MoveTrain_Test;
class GreedySimulator_UpdateRearPositions_Test;
class GreedySimulator_ReverseEdgeMA_Test;
class GreedySimulator_ExitVertexOrder_Test;
class GreedySimulator_FutureSpeedRestrictionConstraintsAfterLeaving_Test;
#endif

namespace cda_rail::simulator {

class GreedySimulator : public GeneralSimulator {
public:
  // reuse default values
  using GeneralSimulator::simulate;

private:
#if TEST_FRIENDS
  FRIEND_TEST(::GreedySimulator, BasicPrivateFunctions);
  FRIEND_TEST(::GreedySimulator, EdgePositions);
  FRIEND_TEST(::GreedySimulator, TrainsOnEdges);
  FRIEND_TEST(::GreedySimulator, IsOkToEnter);
  FRIEND_TEST(::GreedySimulator, AbsoluteDistanceMA);
  FRIEND_TEST(::GreedySimulator, FutureSpeedRestrictionConstraints);
  FRIEND_TEST(::GreedySimulator, FutureMaxSpeedWithMiddleEdge);
  FRIEND_TEST(::GreedySimulator, FutureMaxSpeedWithBlockedVertices);
  FRIEND_TEST(::GreedySimulator, EoMDisplacement);
  FRIEND_TEST(::GreedySimulator, NextStopMA);
  FRIEND_TEST(::GreedySimulator, MAandMaxV);
  FRIEND_TEST(::GreedySimulator, MAtoV);
  FRIEND_TEST(::GreedySimulator, MoveTrain);
  FRIEND_TEST(::GreedySimulator, UpdateRearPositions);
  FRIEND_TEST(::GreedySimulator, ReverseEdgeMA);
  FRIEND_TEST(::GreedySimulator, ExitVertexOrder);
  FRIEND_TEST(::GreedySimulator, FutureSpeedRestrictionConstraintsAfterLeaving);
#endif

public:
  // ----------------
  // CONSTRUCTOR
  // ----------------
  explicit GreedySimulator(
      cda_rail::instances::GeneralPerformanceOptimizationInstance const&
                                       instance,
      std::vector<cda_rail::index_set> ttd_sections);
  explicit GreedySimulator(
      cda_rail::instances::GeneralPerformanceOptimizationInstance const&
                                          instance,
      std::vector<cda_rail::index_set>    ttd_sections,
      std::vector<cda_rail::index_vector> train_edges,
      std::vector<cda_rail::index_vector> ttd_orders,
      std::vector<cda_rail::index_vector> vertex_orders,
      std::vector<std::vector<double>>    stop_positions);

  // Rule of 5
  GreedySimulator(GreedySimulator const&)            = default;
  GreedySimulator(GreedySimulator&&)                 = default;
  GreedySimulator& operator=(GreedySimulator const&) = default;
  GreedySimulator& operator=(GreedySimulator&&)      = default;
  ~GreedySimulator() override                        = default;

  /**
   * @brief Retrieves the typed performance optimization instance.
   *
   * @return Const pointer to the `GeneralPerformanceOptimizationInstance` for this simulator.
   */
  [[nodiscard]] instances::GeneralPerformanceOptimizationInstance const*
  get_instance() const override {
    return dynamic_cast<
        instances::GeneralPerformanceOptimizationInstance const*>(
        GeneralSimulator::get_instance());
  }

  // ------------------
  // SIMULATION
  // ------------------
  [[nodiscard]] SimulatorResults
  simulate(double dt, bool late_entry_possible = false,
           bool limit_speed_by_leaving_edges = true,
           bool save_trajectories            = false) const;

  /**
   * @brief Runs the simulation using a fixed time step of 6.0 seconds.
   *
   * @return SimulatorResults containing the simulation outcome.
   */
  [[nodiscard]] SimulatorResults
  simulate(bool late_entry_possible, bool limit_speed_by_leaving_edges,
           bool save_trajectories) const override {
    return simulate(6.0, late_entry_possible, limit_speed_by_leaving_edges,
                    save_trajectories);
  }

private:
  // ---------------------------
  // PRIVATE HELPER FUNCTIONS
  // ---------------------------

  struct TrainPosition {
    double rear;
    double front;
  };

  // Positioning Helper

  struct OnEdgeIndicator {
    bool tr_on_edge;
    bool rear_on_edge;
    bool front_on_edge;
  };
  struct PosOnEdgeReturn {
    OnEdgeIndicator tr_on_edge_indicator;
    TrainPosition   tr_position_on_edge;
  };
  [[nodiscard]] PosOnEdgeReturn
  get_position_on_route_edge(size_t tr, const TrainPosition& pos,
                             size_t              edge_number,
                             std::vector<double> milestones = {}) const;
  [[nodiscard]] PosOnEdgeReturn
  get_position_on_edge(size_t tr, const TrainPosition& pos, size_t edge_id,
                       std::vector<double> milestones = {}) const;

  enum class TTDOccupationType : std::uint8_t {
    OnlyOccupied,
    OnlyBehind,
    OccupiedOrBehind
  };
  [[nodiscard]] bool is_on_ttd(size_t tr, size_t ttd, const TrainPosition& pos,
                               TTDOccupationType occupation_type =
                                   TTDOccupationType::OnlyOccupied) const;

  /**
   * @brief Determines if a train is at or behind a time-table/track-section location.
   * @return `true` if the train is at or behind the TTD, `false` otherwise.
   */
  [[nodiscard]] bool is_on_or_behind_ttd(size_t tr, size_t ttd,
                                         const TrainPosition& pos) const {
    return is_on_ttd(tr, ttd, pos, TTDOccupationType::OccupiedOrBehind);
  }

  /**
   * @brief Determines if a train is behind a given time-table/track-section.
   *
   * @param tr Index of the train.
   * @param ttd Index of the time-table/track-section.
   * @param pos Current position of the train.
   * @return `true` if the train is behind the TTD, `false` otherwise.
   */
  [[nodiscard]] bool is_behind_ttd(size_t tr, size_t ttd,
                                   const TrainPosition& pos) const {
    return is_on_ttd(tr, ttd, pos, TTDOccupationType::OnlyBehind);
  }

  // Entering Helper

  [[nodiscard]] bool is_ok_to_enter(
      size_t tr, const std::vector<TrainPosition>& train_positions,
      const std::vector<double>&                     train_velocities,
      const std::unordered_set<size_t>&              trains_in_network,
      const std::vector<std::unordered_set<size_t>>& tr_on_edges) const;

  [[nodiscard]] std::pair<bool, std::unordered_set<size_t>>
  get_entering_trains(double t, const std::unordered_set<size_t>& tr_present,
                      const std::unordered_set<size_t>& tr_left,
                      const std::unordered_set<size_t>& tr_finished_simulating,
                      bool late_entry_possible, double buffer_time) const;

  // Moving Authority Helper

  [[nodiscard]] static double max_displacement(const Train& train, double v_0,
                                               double dt);

  [[nodiscard]] double get_absolute_distance_ma(
      size_t tr, double max_displacement,
      const std::vector<TrainPosition>&              train_positions,
      const std::vector<double>&                     train_velocities,
      const std::unordered_set<size_t>&              trains_in_network,
      const std::unordered_set<size_t>&              trains_left,
      const std::vector<std::unordered_set<size_t>>& tr_on_edges) const;

  [[nodiscard]] static PosVel speed_restriction_helper(double ma, double max_v,
                                                       double pos,
                                                       double vertex_pos,
                                                       double v_0, double v_m,
                                                       double d, double dt);

  struct MaAndMaxVResult {
    double ma;
    double ma_without_route_end;
    double max_v;
    double max_v_without_route_end;
  };
  [[nodiscard]] MaAndMaxVResult
  get_future_max_speed_constraints(size_t tr, const Train& train, double pos,
                                   double v_0, double max_displacement,
                                   double                     dt,
                                   cda_rail::index_set const& blocked_vertices,
                                   bool also_limit_by_leaving_edges) const;

  [[nodiscard]] static double
  get_next_stop_ma(double max_displacement, double pos, double next_stop_pos);

  [[nodiscard]] double
  get_exit_vertex_order_ma(size_t tr, double pos, double max_displacement,
                           const std::unordered_set<size_t>& trains_in_network,
                           const std::unordered_set<size_t>& trains_left) const;

  [[nodiscard]] MaAndMaxVResult
  get_ma_and_maxv(size_t tr, const std::vector<double>& train_velocities,
                  std::optional<size_t> next_stop, double dt,
                  cda_rail::index_set const&        blocked_vertices,
                  const std::vector<TrainPosition>& train_positions,
                  const std::unordered_set<size_t>& trains_in_network,
                  const std::unordered_set<size_t>& trains_left,
                  const std::vector<std::unordered_set<size_t>>& tr_on_edges,
                  bool also_limit_speed_by_leaving_edges) const;

  // MA to Speed Helper

  [[nodiscard]] static double get_v1_from_ma(double v_0, double ma, double d,
                                             double dt);

  // Train Movement Helper

  [[nodiscard]] static bool
  move_train(size_t tr, double v_0, double v_1, double ma, double dt,
             std::vector<TrainPosition>& train_positions);

  void update_rear_positions(std::vector<TrainPosition>& train_positions) const;

  // Final State Helper

  enum class DestinationType : std::uint8_t { None, Network, Station, Edge };
  [[nodiscard]] DestinationType
  tr_reached_end(size_t tr, const std::vector<TrainPosition>& train_pos) const;
};

} // namespace cda_rail::simulator
