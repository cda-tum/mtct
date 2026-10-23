#pragma once

#include "Definitions.hpp"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"
#include "simulator/GeneralSimulator.hpp"

// NOLINTNEXTLINE(misc-include-cleaner)
#include "gtest/gtest_prod.h"
#include <cstddef>
#include <vector>

// If TEST_FRIENDS has value true, the corresponding test is friended to test
// complex private functions, see GreedySimulator.hpp.
#ifndef TEST_FRIENDS
#define TEST_FRIENDS false
#endif
#if TEST_FRIENDS
class EventSimulator_Movement_Test;
class EventSimulator_FastestPhases_Test;
class EventSimulator_SpeedLimits_Test;
class EventSimulator_ExitMovement_Test;
#endif

namespace cda_rail::simulator {

/**
 * @brief Simulates the trains of a moving block instance event by event.
 *
 * The trains drive by the same rules as in the GreedySimulator. Instead of
 * advancing all trains in time steps of a fixed length, however, a train plans
 * its fastest movement up to the end of its movement authority and is only
 * simulated again once something relevant to it happens, e.g., once it has
 * reached a stop or once the train it waits for has passed. Trains that wait
 * for another train are woken up by that train.
 *
 * Hence, the trajectories are exact, apart from one approximation: if a
 * train's movement authority ends at the rear of the train ahead, it plans as
 * if that rear stayed where it is. It is simulated again once it starts to
 * brake, but at the earliest after `dt` seconds, see simulate.
 *
 * The simulator is based on Sebastian Ludwig's bachelor's thesis
 * "Event-Based Simulation of Modern Railway Networks", Technical University of
 * Munich, 2026.
 */
class EventSimulator : public GeneralSimulator {
public:
  // reuse default values
  using GeneralSimulator::simulate;

private:
#if TEST_FRIENDS
  FRIEND_TEST(::EventSimulator, Movement);
  FRIEND_TEST(::EventSimulator, FastestPhases);
  FRIEND_TEST(::EventSimulator, SpeedLimits);
  FRIEND_TEST(::EventSimulator, ExitMovement);
#endif

public:
  // ----------------
  // CONSTRUCTOR
  // ----------------
  /**
   * @brief Constructs an event-based simulator from an instance and TTD
   * sections.
   *
   * @param instance Performance optimization instance.
   * @param ttd_sections TTD sections used by the simulator.
   */
  explicit EventSimulator(
      cda_rail::instances::GeneralPerformanceOptimizationInstance const&
                                       instance,
      std::vector<cda_rail::index_set> ttd_sections);
  /**
   * @brief Constructs an event-based simulator with explicit state vectors.
   *
   * @param instance Performance optimization instance.
   * @param ttd_sections TTD sections used by the simulator.
   * @param train_edges Per-train edge sequences.
   * @param ttd_orders Per-TTD train orders.
   * @param vertex_orders Per-vertex train orders.
   * @param stop_positions Per-train stop positions.
   */
  explicit EventSimulator(
      cda_rail::instances::GeneralPerformanceOptimizationInstance const&
                                          instance,
      std::vector<cda_rail::index_set>    ttd_sections,
      std::vector<cda_rail::index_vector> train_edges,
      std::vector<cda_rail::index_vector> ttd_orders,
      std::vector<cda_rail::index_vector> vertex_orders,
      std::vector<std::vector<double>>    stop_positions);

  // Rule of 5
  /** @brief Copy constructor. */
  EventSimulator(EventSimulator const&) = default;
  /** @brief Move constructor. */
  EventSimulator(EventSimulator&&) = default;
  /** @brief Copy assignment operator. */
  EventSimulator& operator=(EventSimulator const&) = default;
  /** @brief Move assignment operator. */
  EventSimulator& operator=(EventSimulator&&) = default;
  /** @brief Virtual destructor. */
  ~EventSimulator() override = default;

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
   * @brief Runs the event-based simulation.
   *
   * @param dt Minimal time in seconds after which a train whose movement
   * authority ends at the rear of another train is simulated again.
   * @param late_entry_possible Whether trains may enter later than scheduled.
   * @param limit_speed_by_leaving_edges Whether leaving edges constrain speed.
   * @param save_trajectories Whether to retain detailed trajectories.
   * @param block_vertices_after_disappearing Whether trains block their last
   * vertex after disappearing on a partial route.
   * @return Simulation results.
   */
  [[nodiscard]] SimulatorResults
  simulate(double dt, bool late_entry_possible,
           bool limit_speed_by_leaving_edges, bool save_trajectories,
           bool block_vertices_after_disappearing) const override;

private:
  // ---------------------------
  // MOVEMENT
  // ---------------------------

  /** @brief A time span in which a train's speed changes linearly. */
  struct Phase {
    double duration;
    double v_start;
    double v_end;

    /** @brief The distance covered within the phase. */
    [[nodiscard]] double distance() const {
      return (v_start + v_end) * duration / 2;
    }
    /** @brief The constant acceleration within the phase. */
    [[nodiscard]] double acceleration() const {
      return duration > 0 ? (v_end - v_start) / duration : 0.0;
    }
  };

  /**
   * @brief The planned movement of a train's front along its route.
   *
   * It starts at position `pos` with speed `v` at time `t` and consists of
   * consecutive phases, after which the train stays where it is.
   */
  struct Movement {
    double             t;
    double             pos;
    double             v;
    std::vector<Phase> phases;

    /** @brief The time at which the last phase ends. */
    [[nodiscard]] double end_time() const;
    /** @brief The position at which the last phase ends. */
    [[nodiscard]] double end_pos() const;
    /** @brief The speed at which the last phase ends. */
    [[nodiscard]] double end_speed() const {
      return phases.empty() ? v : phases.back().v_end;
    }
    /** @brief The position and speed at @p time, which is not before `t`. */
    [[nodiscard]] PosVel at(double time) const;
    /** @brief The first time the front is at @p position, INF if never. */
    [[nodiscard]] double time_at(double position) const;
    /**
     * @brief The first time the braking point, at which the train would come
     * to a stop braking fully, is at @p position, INF if never.
     *
     * @param position Position along the route.
     * @param d The train's deceleration.
     */
    [[nodiscard]] double braking_point_time(double position, double d) const;
    /** @brief The time at which the train starts its final braking. */
    [[nodiscard]] double final_braking_start() const;
    /** @brief The part of the movement until the front is at @p position. */
    [[nodiscard]] Movement until(double position) const;
  };

  /** @brief A part of a route with speed limit `v_max`, ending at `end`. */
  struct SpeedLimit {
    double end;
    double v_max;
  };

  /**
   * @brief The fastest way to get from @p pos to @p target.
   *
   * The speed is the minimum of the speed limits, of accelerating fully, and of
   * braking fully for every lower speed limit ahead and for the target, which
   * is reached with a speed of at most @p v_target. In terms of the squared
   * speed, all three change linearly along the route. A train that starts too
   * fast is slowed down instantaneously, as in the greedy simulator.
   *
   * @param pos Start position.
   * @param v_0 Start speed.
   * @param target Target position.
   * @param v_target Maximal speed at the target, INF if there is none.
   * @param limits Speed limits covering the way from @p pos to @p target.
   * @param a Acceleration.
   * @param d Deceleration.
   * @return The phases of the movement, empty if @p target is not ahead.
   */
  [[nodiscard]] static std::vector<Phase>
  fastest_phases(double pos, double v_0, double target, double v_target,
                 std::vector<SpeedLimit> const& limits, double a, double d);

  /**
   * @brief The speed limits of a train's front between two positions.
   *
   * An edge limits the speed while the train's front is on it or, if
   * @p limit_speed_by_leaving_edges, any part of the train. Beyond the train's
   * last edge, its route continues at that edge's speed limit.
   */
  [[nodiscard]] std::vector<SpeedLimit>
  speed_limits(size_t tr, double from, double to,
               bool limit_speed_by_leaving_edges) const;

  /**
   * @brief The fastest movement of a train to the position at which its rear
   * passes its exit vertex that does not get there before its exit time.
   *
   * As in the greedy simulator, a train that is too early brakes fully as late
   * as possible. If it would then become slower than V_MIN outside the
   * network, it instead brakes towards its exit vertex and accelerates again as
   * soon as it can leave at its exit time, or, if even that is too early, it
   * waits with its front at its exit vertex.
   *
   * @param tr Train index.
   * @param t Current time.
   * @param front The train's front position and speed at @p t.
   * @param limit_speed_by_leaving_edges Whether leaving edges constrain speed.
   * @return The movement up to the train's exit.
   */
  [[nodiscard]] Movement exit_movement(size_t tr, double t, PosVel front,
                                       bool limit_speed_by_leaving_edges) const;

  /** @brief The state of a single simulation run. */
  class Run;
};

} // namespace cda_rail::simulator
