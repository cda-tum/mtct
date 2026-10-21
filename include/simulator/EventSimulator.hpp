#pragma once

#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "datastructure/Train.hpp"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"
#include "simulator/GeneralSimulator.hpp"

// NOLINTNEXTLINE(misc-include-cleaner)
#include "../../extern/cli11/include/CLI/App.hpp"
#include "EOMHelper.hpp"

#include "gtest/gtest_prod.h"
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <optional>
#include <queue>
#include <span>
#include <string>
#include <tuple>
#include <unordered_set>
#include <utility>
#include <vector>

#ifndef TEST_FRIENDS
#define TEST_FRIENDS false
#endif
#if TEST_FRIENDS
class EventSimulator_CalculateOptimalTrainMovements_Test;
class EventSimulator_MoveTrain_Test;
#endif

namespace cda_rail::simulator {

class EventSimulator : public GeneralSimulator {
public:
  // reuse default values
  using GeneralSimulator::simulate;

  // ----------------
  // CONSTRUCTOR
  // ----------------
  explicit EventSimulator(
      cda_rail::instances::GeneralPerformanceOptimizationInstance const&
                                       instance,
      std::vector<cda_rail::index_set> ttd_sections);
  explicit EventSimulator(
      cda_rail::instances::GeneralPerformanceOptimizationInstance const&
                                          instance,
      std::vector<cda_rail::index_set>    ttd_sections,
      std::vector<cda_rail::index_vector> train_edges,
      std::vector<cda_rail::index_vector> ttd_orders,
      std::vector<cda_rail::index_vector> vertex_orders,
      std::vector<std::vector<double>>    stop_positions);

  // Rule of 5
  EventSimulator(EventSimulator const&)            = default;
  EventSimulator(EventSimulator&&)                 = default;
  EventSimulator& operator=(EventSimulator const&) = default;
  EventSimulator& operator=(EventSimulator&&)      = default;
  ~EventSimulator() override                       = default;

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
   * @brief Runs the simulation.
   *
   * @return SimulatorResults containing the simulation outcome.
   */
  [[nodiscard]] SimulatorResults
  simulate(bool late_entry_possible, bool limit_speed_by_leaving_edges,
           bool save_trajectories,
           bool disappear_at_partial_route_end) const override;

private:
#if TEST_FRIENDS
  FRIEND_TEST(::EventSimulator, CalculateOptimalTrainMovements);
  FRIEND_TEST(::EventSimulator, MoveTrain);
#endif

  static constexpr double ITERATIVE_MOVEMENT_TIME_WIDTH = 6.0;

  /**
   * Describes linear train movement with initial speed u, final speed v,
   * acceleration a, distance traveled s over time t.
   */
  struct TrainMovement {
    TrainMovement(double u, double v, double a, double s, double t) {
      this->u = u;
      this->v = v;
      this->a = a;
      this->s = s;
      this->t = t;
    }

    TrainMovement(double u, double v, double a, double s)
        : TrainMovement(u, v, a, s, time_taken(u, v, s)) {}

    TrainMovement(double u, double v, double a)
        : TrainMovement(u, v, a, distance_travelled(u, v, a)) {}

    double u;
    double v;
    double a;
    double s;
    double t;
  };

  /**
   * Describes the stop at the end of some free track. The end can either be a
   * station stop, any temporary end (moving train, vertex order/TTD
   * restriction) or the end of the train's line
   */
  enum class FreeTrackStopType { StationStop, TemporaryEnd, LineEnd };

  /**
   * Describes a dependency which may appear at the end of a stretch of free
   * track, the depended on train may be followable, or may just need to pass a
   * certain point.
   */
  enum class FreeTrackDependencyType { Follow, Pass };

  /**
   * Describes (part of) an edge, storing only length and max_speed
   */
  struct EdgeSegment {
    double length;
    double max_speed;
  };

  /**
   * Holds a train's front and read positions
   */
  struct TrainPosition {
    double rear;
    double front;
  };

  struct TrainTimestep {
    size_t tr;
    double timestep;
  };

  /**
   * Describes a train dependence. The dependent_tr requires the leading train
   * to be at least required_distance along it's route to be able to continue.
   */
  struct TrainDependency {
    size_t leading_tr;
    size_t dependent_tr;
    double required_cleared_location;
  };

  struct MaxSpeedChangePoint {
    double position;
    double new_max_speed;
  };

  struct Compare {
    bool operator()(const TrainTimestep& a, const TrainTimestep& b) const {
      return a.timestep > b.timestep;
    }
  };

  using train_timestep_queue =
      std::priority_queue<TrainTimestep, std::vector<TrainTimestep>, Compare>;

  using train_dependencies = std::vector<std::vector<TrainDependency>>;

  /**
   * Check whether a train can be entered into the network based on vertex and
   * TTD orders. If it cannot be added due to depending on other trains to pass
   * a position first, the first dependency found will be returned, else an
   * empty optional is returned.
   * @param train_positions All train positions
   * @param trains_finished_simulating
   * @param trains_in_network
   * @param trains_on_edges
   * @param train_id ID of train to enter
   * @remark This only returns an arbitrary dependency rather than the
   * first/last/all of them.
   * @return empty optional if the train can be entered into the network, else a
   * dependency of the train entering the network.
   */
  [[nodiscard]] std::optional<TrainDependency>
  can_enter_train_or_get_dependency(
      const std::vector<TrainPosition>&              train_positions,
      const std::unordered_set<size_t>&              trains_finished_simulating,
      const std::unordered_set<size_t>&              trains_in_network,
      const std::vector<std::unordered_set<size_t>>& trains_on_edges,
      size_t                                         train_id) const;

  [[nodiscard]] static std::optional<double> get_time_when_dependency_cleared(
      const TrainDependency&            dependency,
      const std::vector<TrainMovement>& leading_movements,
      const TrainPosition& leading_position, double leading_time_to_timestep);

  /**
   * Enter a train into the network, assumes it is not already present
   * @param trains_in_network Reference to the trains currently in the network
   * @param train_positions Reference to the positions of the trains
   * @param train_velocities Reference to the velocities of the trains
   * @param train_id The ID of the train to enter
   */
  void enter_train(std::unordered_set<size_t>& trains_in_network,
                   std::vector<TrainPosition>& train_positions,
                   std::vector<double>&        train_velocities,
                   size_t                      train_id) const;

  bool is_edge_ttd_free_for_tr(
      const std::vector<TrainPosition>& train_positions,
      const std::unordered_set<size_t>& trains_finished_simulating,
      size_t edge_id, size_t train_id, TrainDependency& out_dependency) const;

  bool is_vertex_free_for_tr(
      const std::vector<TrainPosition>& train_positions,
      const std::unordered_set<size_t>& trains_finished_simulating,
      size_t vertex_id, size_t train_id, TrainDependency& out_dependency) const;

  double edge_distance_free_for_tr(
      const std::vector<TrainPosition>&              train_positions,
      const std::unordered_set<size_t>&              trains_in_network,
      const std::vector<std::unordered_set<size_t>>& trains_on_edges,
      size_t edge_id, size_t train_id, double distance_into_edge,
      TrainDependency& out_dependency) const;

  double
  distance_into_current_edge(const std::vector<TrainPosition>& train_positions,
                             size_t                            train_id) const;

  /**
   * Get the position in the current_train's path where the train no longer has
   * a free path, either due to another train on a shared edge or a train which
   * is yet to cross a vertex first.
   * @param train_positions Train positions
   * @param trains_in_network Trains in network, only these will be considered
   * for collisions on edges
   * @param trains_finished_simulating
   * @param next_stop_indices
   * @param trains_on_edges
   * @param current_train ID of current train to find the free track end for
   * @returns Tuple of vector with free edge segments, the track end type, an
   * optional pair of dependency, and a free track dependency type. The optional
   * has a value if the track ends due to waiting for another train to first
   * traverse a vertex or TTD. The free track dependency type indicates whether
   * the train which ended the free track can be followed or just needs to pass
   * a certain point.
   */
  [[nodiscard]] std::tuple<
      std::vector<EdgeSegment>, FreeTrackStopType,
      std::optional<std::pair<TrainDependency, FreeTrackDependencyType>>>
  find_free_track_ahead(
      const std::vector<TrainPosition>&              train_positions,
      const std::unordered_set<size_t>&              trains_in_network,
      const std::unordered_set<size_t>&              trains_finished_simulating,
      const std::vector<int>&                        next_stop_indices,
      const std::vector<std::unordered_set<size_t>>& trains_on_edges,
      size_t                                         current_train) const;

  /**
   *
   * @param free_track Vector of EdgeSegments describing the free track ahead.
   * This parameter may be altered by the function
   * @param initial_speed initial speed at which train is traveling
   * @param train The train to calculate movements for
   * @return Series of train movements to traverse the free track as fast as
   * possible
   */
  [[nodiscard]] static std::vector<TrainMovement>
  calculate_optimal_train_movements(std::vector<EdgeSegment>& free_track,
                                    double initial_speed, const Train& train);

  static void
  push_back_movement_if_not_zero(std::vector<TrainMovement>& movements,
                                 const TrainMovement&        m);

  /**
   * For a given series of EdgeSegments, returns the point of, and the speed
   * restriction of the first point with a reduced speed limit which needs to be
   * considered by a train. Also returns the peak speed squared achieved when
   * accelerating for as long as possible and then braking until the point of
   * the new restriction
   * @param free_track vector of edge segments representing the track available
   * to the train
   * @param init_segment_index The index of the segment to start searching for
   * new speed restrictions from
   * @param train The train traversing the edges
   * @param current_speed The train's current speed
   * @return An optional which only has a value if there is another speed
   * restriction ahead. The optional holds a pair with a MaxSpeedChangePoint,
   * holding position and new speed limit of the restricting point, and a double
   * representing the peak speed which can be reached while traveling to the
   * point, squared.
   */
  static std::optional<std::pair<MaxSpeedChangePoint, double>>
  find_first_speed_restriction(const std::vector<EdgeSegment>& free_track,
                               int init_segment_index, const Train& train,
                               double current_speed);

  /**
   * For each segment in segments, sets the max_speed to the minimum of the
   * current value and the max_speed parameter.
   * @param segments
   * @param speed_limit
   */
  static void limit_segment_speeds(std::vector<EdgeSegment>& segments,
                                   double                    speed_limit);

  [[nodiscard]] double get_shared_track_ahead_distance(TrainPosition& tr1_pos,
                                                       size_t         tr1,
                                                       size_t tr2) const;

  /**
   * Given the upcoming movements of a train which is following another train
   * and may therefore need to move iteratively, calculate when the next
   * timestep should be placed for the following train.
   * @param movements
   * @remark This assumes that the final movement in the sequence is braking to
   * a standstill
   * @return
   */
  static double iterative_movement_time_to_timestep(
      const std::vector<TrainMovement>& movements);

  [[nodiscard]] std::vector<TrainMovement> follow_train(size_t tr_follower,
                                                        size_t tr_leader);

  /**
   * Calculates train movement which traverses a span of tracks as quickly as
   * possible and appends these to the movements vector passed by reference.
   * @param movements Vector of train movements which the newly generated
   * movements will be appended to
   * @param free_track Span of free tracks which should be traversed as fast as
   * possible
   * @param initial_speed Initial train speed
   * @param train Train object which is traversing the track
   */
  static void
  append_max_permitted_speed_movements(std::vector<TrainMovement>&  movements,
                                       std::span<const EdgeSegment> free_track,
                                       double       initial_speed,
                                       const Train& train);

  /**
   * Add a TrainMovement to the vector only if the distance covered or time
   * passed by the movement is not zero
   * @remark The movement may be zero if an algorithm assumes that a train must
   * accelerate to a certain velocity as part of optimal movement, but the train
   * is already traveling at that velocity. While a movement with distance and
   * time of zero should not break anything else, it is nicer for debugging
   * purposes to never add these movements to begin with.
   * @param movements Reference to vector of TrainMovements
   * @param m Movement to be appended to the back of movements if it does not
   * represent a distance of zero
   */
  static void append_movement_if_not_zero(std::vector<TrainMovement>& movements,
                                          const TrainMovement&        m);

  /**
   *
   * @param movements Train movements to sum time for
   * @return The total time needed to complete all the train movements
   */
  static double
  get_movements_total_time(const std::span<const TrainMovement>& movements);

  static double
  get_edge_segments_total_distance(std::span<const EdgeSegment> segments);

  static void move_train(TrainPosition& train_position, double& train_velocity,
                         std::vector<TrainMovement>& train_movements, double t);

  /**
   * Calculates the distance traveled in a certain time and changes the
   * train_movement to represent the remaining movement after the time only.
   * @param movement Train movement to partially apply, will be updated
   * in-place to only hold the 'non-consumed' part.
   * @param t The time to apply to the movement
   * @return Distance traveled in time t.
   */
  static double distance_traveled_consume_movement(TrainMovement& movement,
                                                   double         t);

  /**
   * Get the position of a vertex on the path of the given train
   * @param train_id Train for which to get the position
   * @param vertex_id Vertex for which to get the position
   * @throws ConsistencyException if vertex is not on trian's path
   * @return Position relative to train's path where the vertex is located
   */
  double get_vertex_position(size_t train_id, size_t vertex_id) const;

  /**
   * Checks whether a train's exit vertex (from the schedule) is on the train's
   * current path.
   * @param train_id Train ID
   * @return true if the exit vertex appears at the end of the train's path.
   */
  bool path_includes_exit_vertex(size_t train_id) const;

  /**
   * Get a set of all the edges a train has already passed.
   * @param train_position
   * @param train_id
   * @return
   */
  index_set get_passed_edges_of_train(const TrainPosition& train_position,
                                      size_t               train_id) const;

  bool train_has_passed_vertex(const TrainPosition& train_position,
                               size_t train_id, size_t vertex_id) const;

  /**
   * Get the position relative to a train's path where it is on the given TTD
   * for the last moment
   * @param train_position Train position of train
   * @param train_id Train ID
   * @param ttd_id TTD ID
   * @throws InvalidInputException if train never passes through this TTD
   * @return Position on train's path of the TTD end
   */
  double train_ttd_end_position(const TrainPosition& train_position,
                                size_t train_id, size_t ttd_id) const;

  bool train_has_passed_ttd(const TrainPosition& train_position,
                            size_t train_id, size_t ttd_id) const;

  static double squared(double x);
};
} // namespace cda_rail::simulator
