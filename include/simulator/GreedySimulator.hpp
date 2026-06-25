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

enum : std::uint16_t { CycleLimit = 1000 };

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
  /**
   * @brief Constructs a greedy simulator from an instance and TTD sections.
   *
   * @param instance Performance optimization instance.
   * @param ttd_sections TTD sections used by the simulator.
   */
  explicit GreedySimulator(
      cda_rail::instances::GeneralPerformanceOptimizationInstance const&
                                       instance,
      std::vector<cda_rail::index_set> ttd_sections);
  /**
   * @brief Constructs a greedy simulator with explicit state vectors.
   *
   * @param instance Performance optimization instance.
   * @param ttd_sections TTD sections used by the simulator.
   * @param train_edges Per-train edge sequences.
   * @param ttd_orders Per-TTD train orders.
   * @param vertex_orders Per-vertex train orders.
   * @param stop_positions Per-train stop positions.
   */
  explicit GreedySimulator(
      cda_rail::instances::GeneralPerformanceOptimizationInstance const&
                                          instance,
      std::vector<cda_rail::index_set>    ttd_sections,
      std::vector<cda_rail::index_vector> train_edges,
      std::vector<cda_rail::index_vector> ttd_orders,
      std::vector<cda_rail::index_vector> vertex_orders,
      std::vector<std::vector<double>>    stop_positions);

  // Rule of 5
  /** @brief Copy constructor. */
  GreedySimulator(GreedySimulator const&) = default;
  /** @brief Move constructor. */
  GreedySimulator(GreedySimulator&&) = default;
  /** @brief Copy assignment operator. */
  GreedySimulator& operator=(GreedySimulator const&) = default;
  /** @brief Move assignment operator. */
  GreedySimulator& operator=(GreedySimulator&&) = default;
  /** @brief Virtual destructor. */
  ~GreedySimulator() override = default;

  /**
   * @brief Retrieves the typed performance optimization instance.
   *
   * @return Const pointer to the `GeneralPerformanceOptimizationInstance` for
   * this simulator.
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
  /**
   * @brief Runs the greedy simulation with an explicit time step.
   *
   * @param dt Simulation time step in seconds.
   * @param late_entry_possible Whether trains may enter later than scheduled.
   * @param limit_speed_by_leaving_edges Whether leaving edges constrain speed.
   * @param save_trajectories Whether to retain detailed trajectories.
   * @param block_vertices_after_disappearing Whether trains block their last
   * vertex after disappearing on a partial route.
   * @return Simulation results.
   */
  [[nodiscard]] SimulatorResults
  simulate(double dt, bool late_entry_possible = false,
           bool limit_speed_by_leaving_edges      = true,
           bool save_trajectories                 = false,
           bool block_vertices_after_disappearing = true) const;

  /**
   * @brief Runs the simulation using a fixed time step of 6.0 seconds.
   *
   * @param late_entry_possible Whether trains may enter later than scheduled.
   * @param limit_speed_by_leaving_edges Whether leaving edges constrain speed.
   * @param save_trajectories Whether to retain detailed trajectories.
   * @param block_vertices_after_disappearing Whether trains block their last
   *        vertex after disappearing on a partial route.
   * @return SimulatorResults containing the simulation outcome.
   */
  [[nodiscard]] SimulatorResults
  simulate(bool late_entry_possible, bool limit_speed_by_leaving_edges,
           bool save_trajectories,
           bool block_vertices_after_disappearing) const override {
    return simulate(6.0, late_entry_possible, limit_speed_by_leaving_edges,
                    save_trajectories, block_vertices_after_disappearing);
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
  /**
   * @brief Computes a train's rear/front positions relative to one route edge.
   *
   * @param tr Train index.
   * @param pos Absolute train position on the route.
   * @param edge_number Index of the route edge within the train route.
   * @param milestones Optional cached route-edge milestone positions.
   * @return Edge-occupancy flags together with rear/front coordinates on the
   *         route edge.
   */
  [[nodiscard]] PosOnEdgeReturn
  get_position_on_route_edge(size_t tr, const TrainPosition& pos,
                             size_t              edge_number,
                             std::vector<double> milestones = {}) const;
  /**
   * @brief Computes a train's rear/front positions relative to a network edge.
   *
   * @param tr Train index.
   * @param pos Absolute train position on the route.
   * @param edge_id Network edge index.
   * @param milestones Optional cached route-edge milestone positions.
   * @return Edge-occupancy flags together with rear/front coordinates on the
   *         specified edge.
   */
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
   * @brief Determines if a train is at or behind a time-table/track-section
   * location.
   * @param tr Index of the train.
   * @param ttd Index of the time-table/track-section.
   * @param pos Current position of the train.
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

  /**
   * @brief Checks whether a train may enter the network at the current state.
   *
   * @param tr Train index.
   * @param train_positions Current per-train positions.
   * @param train_velocities Current per-train velocities.
   * @param trains_in_network Trains currently present in the network.
   * @param tr_on_edges Edge-indexed sets of trains whose routes use the edge.
   * @return `true` if the train can safely enter, `false` otherwise.
   */
  [[nodiscard]] bool is_ok_to_enter(
      size_t tr, const std::vector<TrainPosition>& train_positions,
      const std::vector<double>&                     train_velocities,
      const std::unordered_set<size_t>&              trains_in_network,
      const std::vector<std::unordered_set<size_t>>& tr_on_edges) const;

  /**
   * @brief Determines which trains are due and able to enter at time @p t.
   *
   * @param t Current simulation time.
   * @param tr_present Trains currently in the network.
   * @param tr_left Trains that already left the network.
   * @param tr_finished_simulating Trains whose simulation is already complete.
   * @param late_entry_possible Whether later-than-scheduled entry is allowed.
   * @param buffer_time Time-step-based entry buffer.
   * @return Pair of feasibility flag and set of trains selected to enter.
   */
  [[nodiscard]] std::pair<bool, std::unordered_set<size_t>>
  get_entering_trains(double t, const std::unordered_set<size_t>& tr_present,
                      const std::unordered_set<size_t>& tr_left,
                      const std::unordered_set<size_t>& tr_finished_simulating,
                      bool late_entry_possible, double buffer_time) const;

  // Moving Authority Helper

  /**
   * @brief Computes the maximal distance a train front can advance in one step.
   *
   * @param train Train object containing dynamic limits.
   * @param v_0 Current train speed.
   * @param dt Simulation time step.
   * @return Maximal forward displacement in the time step.
   */
  [[nodiscard]] static double max_displacement(const Train& train, double v_0,
                                               double dt);

  /**
   * @brief Computes a moving-authority limit induced by other trains.
   *
   * @param tr Train index.
   * @param max_displacement Current upper bound on forward displacement.
   * @param train_positions Current per-train positions.
   * @param train_velocities Current per-train velocities.
   * @param trains_in_network Trains currently present in the network.
   * @param trains_left Trains that already left the network.
   * @param tr_on_edges Edge-indexed sets of trains whose routes use the edge.
   * @param also_check_reverse_edges Whether opposite-direction conflicts are
   *        considered.
   * @return Distance-based moving-authority limit for the train.
   */
  [[nodiscard]] double get_absolute_distance_ma(
      size_t tr, double max_displacement,
      const std::vector<TrainPosition>&              train_positions,
      const std::vector<double>&                     train_velocities,
      const std::unordered_set<size_t>&              trains_in_network,
      const std::unordered_set<size_t>&              trains_left,
      const std::vector<std::unordered_set<size_t>>& tr_on_edges,
      bool also_check_reverse_edges) const;

  /**
   * @brief Applies an upcoming edge-speed restriction to MA and speed bounds.
   *
   * @param ma Current moving-authority limit.
   * @param max_v Current speed limit.
   * @param pos Current train-front position.
   * @param vertex_pos Route position of the next relevant vertex.
   * @param v_0 Current train speed.
   * @param v_m Speed limit to apply beyond the vertex.
   * @param d Train deceleration.
   * @param dt Simulation time step.
   * @return Updated moving-authority and speed limits.
   */
  [[nodiscard]] static PosVel speed_restriction_helper(double ma, double max_v,
                                                       double pos,
                                                       double vertex_pos,
                                                       double v_0, double v_m,
                                                       double d, double dt);

  struct MaAndMaxVResult {
    double ma;
    double max_v;
  };
  /**
   * @brief Computes future speed and MA constraints along the current route.
   *
   * @param tr Train index.
   * @param train Train object containing dynamic limits.
   * @param pos Current train-front position.
   * @param v_0 Current train speed.
   * @param max_displacement Current upper bound on forward displacement.
   * @param current_time Current simulation time.
   * @param dt Simulation time step.
   * @param blocked_vertices Vertices currently blocked by headway.
   * @param also_limit_by_leaving_edges Whether already-left edges still impose
   *        speed constraints.
   * @return Combined moving-authority and speed limits.
   */
  [[nodiscard]] MaAndMaxVResult
  get_future_max_speed_constraints(size_t tr, const Train& train, double pos,
                                   double v_0, double max_displacement,
                                   double current_time, double dt,
                                   cda_rail::index_set const& blocked_vertices,
                                   bool also_limit_by_leaving_edges) const;

  /**
   * @brief Converts a next-stop position into a moving-authority distance.
   *
   * @param max_displacement Maximal forward displacement in the time step.
   * @param pos Current train-front position.
   * @param next_stop_pos Route position of the next stop.
   * @return Moving-authority distance toward the next stop.
   */
  [[nodiscard]] static double
  get_next_stop_ma(double max_displacement, double pos, double next_stop_pos);

  /**
   * @brief Limits moving authority by the exit-vertex order, if applicable.
   *
   * @param tr Train index.
   * @param pos Current train-front position.
   * @param max_displacement Current upper bound on forward displacement.
   * @param trains_in_network Trains currently present in the network.
   * @param trains_left Trains that already left the network.
   * @return Moving-authority limit respecting exit ordering.
   */
  [[nodiscard]] double
  get_exit_vertex_order_ma(size_t tr, double pos, double max_displacement,
                           const std::unordered_set<size_t>& trains_in_network,
                           const std::unordered_set<size_t>& trains_left) const;
  /**
   * @brief Checks whether another train still blocks this train's exit vertex.
   *
   * @param tr Train index.
   * @param trains_in_network Trains currently present in the network.
   * @param trains_left Trains that already left the network.
   * @return `true` if the exit vertex is still blocked by ordering, `false`
   *         otherwise.
   */
  [[nodiscard]] bool
  is_exit_vertex_blocked(size_t                            tr,
                         const std::unordered_set<size_t>& trains_in_network,
                         const std::unordered_set<size_t>& trains_left) const;

  /**
   * @brief Combines stop, ordering, occupancy, and speed constraints.
   *
   * @param tr Train index.
   * @param train_velocities Current per-train velocities.
   * @param next_stop Optional index of the next scheduled stop.
   * @param current_time Current simulation time.
   * @param dt Simulation time step.
   * @param blocked_vertices Vertices currently blocked by headway.
   * @param train_positions Current per-train positions.
   * @param trains_in_network Trains currently present in the network.
   * @param trains_left Trains that already left the network.
   * @param tr_on_edges Edge-indexed sets of trains whose routes use the edge.
   * @param also_limit_speed_by_leaving_edges Whether already-left edges still
   *        impose speed constraints.
   * @param also_check_reverse_edges Whether opposite-direction conflicts are
   *        considered.
   * @return Combined moving-authority and speed bounds.
   */
  [[nodiscard]] MaAndMaxVResult
  get_ma_and_maxv(size_t tr, const std::vector<double>& train_velocities,
                  std::optional<size_t> next_stop, double current_time,
                  double dt, cda_rail::index_set const& blocked_vertices,
                  const std::vector<TrainPosition>& train_positions,
                  const std::unordered_set<size_t>& trains_in_network,
                  const std::unordered_set<size_t>& trains_left,
                  const std::vector<std::unordered_set<size_t>>& tr_on_edges,
                  bool also_limit_speed_by_leaving_edges,
                  bool also_check_reverse_edges) const;

  // MA to Speed Helper

  /**
   * @brief Converts a moving-authority limit into a feasible next-step speed.
   *
   * @param v_0 Current train speed.
   * @param ma Moving-authority distance.
   * @param d Train deceleration.
   * @param dt Simulation time step.
   * @return Feasible speed at the end of the time step.
   */
  [[nodiscard]] static double get_v1_from_ma(double v_0, double ma, double d,
                                             double dt);

  // Train Movement Helper

  /**
   * @brief Advances a train front position for one time step.
   *
   * @param tr Train index.
   * @param v_0 Initial speed.
   * @param v_1 End-of-step speed.
   * @param ma Moving-authority distance.
   * @param dt Simulation time step.
   * @param train_positions Train positions updated in place.
   * @return `true` if the train moved forward, `false` otherwise.
   */
  [[nodiscard]] static bool
  move_train(size_t tr, double v_0, double v_1, double ma, double dt,
             std::vector<TrainPosition>& train_positions);

  /**
   * @brief Updates rear positions after front positions were advanced.
   *
   * @param train_positions Train positions updated in place.
   */
  void update_rear_positions(std::vector<TrainPosition>& train_positions) const;

  // Final State Helper

  enum class DestinationType : std::uint8_t { None, Network, Station, Edge };
  /**
   * @brief Classifies what destination a train has reached, if any.
   *
   * @param tr Train index.
   * @param train_pos Current train positions.
   * @param has_stop_left Whether the train still has an unsatisfied stop.
   * @return Destination classification for the train.
   */
  [[nodiscard]] DestinationType
  tr_reached_end(size_t tr, const std::vector<TrainPosition>& train_pos,
                 bool has_stop_left) const;

  // Distance helper
  struct VertexRoutePos {
    bool   is_on_route;
    double pos;
  };
  /**
   * @brief Returns the route position of a vertex for a given train route.
   *
   * @param tr Train index.
   * @param vertex Vertex index.
   * @return Flag indicating whether the vertex lies on the route together with
   *         its route position if it does.
   */
  [[nodiscard]] VertexRoutePos get_vertex_pos(size_t tr, size_t vertex) const;
};

} // namespace cda_rail::simulator
