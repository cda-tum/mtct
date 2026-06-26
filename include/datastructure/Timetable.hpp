#pragma once
#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "datastructure/RailwayNetwork.hpp"
#include "datastructure/Station.hpp"
#include "datastructure/Train.hpp"
#include "nlohmann/json_fwd.hpp"

#include <algorithm>
#include <cstddef>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace cda_rail {

class ScheduledStop {
  /**
   * @brief Scheduling information for a stop within a timetable.
   *
   * Stores the earliest service start time, the minimum service duration,
   * and the associated station.
   *
   * @invariant m_service_time >= 0 (non-negative).
   * @invariant m_service_duration >= 0 (non-negative).
   * @invariant m_station is always non-null after construction.
   */

private:
  // member variables have no default -> user-defined constructors
  double                   m_service_time;     // earliest start of service
  double                   m_service_duration; // minimal service duration
  std::shared_ptr<Station> m_station;          // pointer to a station

  /**
   * @brief Validates that a station pointer is not null.
   *
   * @param ptr Pointer to validate.
   * @throws cda_rail::exceptions::InvalidInputException If `ptr` is null.
   */
  static void check_ptr_validity(std::shared_ptr<Station const> const& ptr) {
    if (!ptr) {
      throw cda_rail::exceptions::InvalidInputException(
          "Station pointer cannot be null");
    }
  }

public:
  /*
   * CONSTRUCTOR
   */

  // user-defined constructor
  /**
   * @brief Constructs a scheduled stop for a concrete station.
   *
   * @pre serviceTime >= 0 (non-negative).
   * @pre serviceDuration >= 0 (non-negative).
   * @pre station must not be null.
   * @param serviceTime Earliest service start time.
   * @param serviceDuration Minimum required service duration.
   * @param station Station at which the stop is scheduled.
   * @throws cda_rail::exceptions::InvalidInputException If preconditions are
   * violated.
   */
  ScheduledStop(double const serviceTime, double const serviceDuration,
                std::shared_ptr<Station> station)
      : m_service_time(serviceTime), m_service_duration(serviceDuration),
        m_station(std::move(station)) {
    cda_rail::exceptions::throw_if_negative(m_service_time, "Service time");
    cda_rail::exceptions::throw_if_negative(m_service_duration,
                                            "Service duration");
    check_ptr_validity(m_station);
  }

  // Rule of 0 (default constructor overwriting does not affect copy/move
  // constructors and destructor)

  /*
   * GETTER
   */
  /**
   * @brief Retrieves the earliest service start time.
   *
   * @return The service start time, guaranteed to be non-negative.
   */
  [[nodiscard]] double get_service_time() const { return m_service_time; }

  /**
   * @brief Computes the earliest departure time from this stop.
   *
   * @return The earliest departure time.
   */
  [[nodiscard]] double get_earliest_departure() const {
    return m_service_time + m_service_duration;
  }

  /**
   * @brief Accesses the minimum service duration.
   *
   * @return Minimum service duration (guaranteed >= 0).
   */
  [[nodiscard]] double get_service_duration() const {
    return m_service_duration;
  }

  /**
   * @brief Accesses the station associated with this stop.
   *
   * @return Constant reference to the associated station (guaranteed non-null).
   */
  [[nodiscard]] Station const& get_station() const { return *m_station; }

  /*
   * SETTER
   */
  /**
   * @brief Sets the earliest service start time.
   *
   * @pre new_service_time >= 0 (non-negative).
   * @param new_service_time New earliest service start time.
   * @throws cda_rail::exceptions::InvalidInputException If `new_service_time`
   * is negative.
   */
  void set_service_time(double new_service_time) {
    cda_rail::exceptions::throw_if_negative(new_service_time, "Service time");
    m_service_time = new_service_time;
  }

  /**
   * @brief Sets the minimum service duration.
   *
   * @param new_service_duration New minimum service duration.
   * @throws cda_rail::exceptions::InvalidInputException If
   * the duration is negative.
   */
  void set_service_duration(double new_service_duration) {
    cda_rail::exceptions::throw_if_negative(new_service_duration,
                                            "Service duration");
    m_service_duration = new_service_duration;
  }

  /**
   * @brief Sets the station for this scheduled stop.
   *
   * @pre new_station must not be null.
   * @param new_station New station pointer.
   * @throws cda_rail::exceptions::InvalidInputException If `new_station` is
   * null.
   */
  void set_station(std::shared_ptr<Station> new_station) {
    check_ptr_validity(new_station);
    m_station = std::move(new_station);
  }
};

class Schedule {
  /**
   * @brief Specific schedule object with fixed entry/exit constraints.
   *
   * Stores timing, velocity, path endpoints, and an ordered list of scheduled
   * stops for one train.
   *
   * @invariant m_entry_time >= 0.
   * @invariant m_exit_time >= m_entry_time.
   * @invariant m_initial_velocity >= 0.
   * @invariant m_exit_velocity >= 0.
   * @invariant m_stops is ordered by service time
   */

private:
  double m_entry_time{}; // (earliest) entry time: >= 0
  double m_exit_time{};  // (desired) exit time: >= entry_time

  double m_initial_velocity{}; // initial velocity: >= 0
  double m_exit_velocity{};    // (desired) exit velocity: >= 0

  size_t m_entry_vertex{}; // id of entry vertex
  size_t m_exit_vertex{};  // id of exit vertex

  std::vector<ScheduledStop> m_stops{}; // list of stops, may be empty

  /**
   * @brief Validates a stop list for schedule consistency.
   *
   * Checks that stops are sorted in non-decreasing order by service time and
   * that station names are pairwise distinct.
   *
   * @param stops Stop list to validate.
   * @throws cda_rail::exceptions::InvalidInputException If stops are not
   * ordered by service time or if a station appears more than once.
   */
  static void check_stops_validity(std::vector<ScheduledStop> const& stops);

  /**
   * @brief Validates a stop list and returns a structured result.
   *
   * @param stops Stop list to validate.
   * @return Pair consisting of a success flag and an optional exception object
   *         describing the first detected issue.
   */
  static std::pair<bool, std::optional<cda_rail::exceptions::CustomException>>
  check_stops_validity_helper(std::vector<ScheduledStop> const& stops);

  /**
   * @brief Constructs a Schedule with uninitialized members.
   *
   * Private constructor that creates a Schedule in an uninitialized state.
   * Intended for use by `Timetable` to initialize vectors prior to proper
   * assignment of entry/exit constraints and stops.
   */
  Schedule() = default;
  friend class Timetable;

public:
  ScheduledStop& editable_scheduled_stop(size_t idx) { return m_stops.at(idx); }

  // user-defined-constructor
  /**
   * @brief Constructs a schedule with fixed entry/exit constraints.
   *
   * @pre entryTime >= 0.
   * @pre exitTime >= entryTime.
   * @pre initialVelocity >= 0.
   * @pre exitVelocity >= 0.
   * @pre stops are ordered by service time and cover pairwise distinct stations
   * @param entry_time Earliest allowed entry time.
   * @param initial_velocity Initial velocity at entry.
   * @param entry_vertex Entry vertex id.
   * @param exit_time Desired exit time.
   * @param exit_velocity Desired velocity at exit.
   * @param exit_vertex Exit vertex id.
   * @param stops Ordered scheduled stops for this train.
   * @throws cda_rail::exceptions::InvalidInputException If any of the
   * preconditions is violated.
   */
  Schedule(double entry_time, double initial_velocity, size_t entry_vertex,
           double exit_time, double exit_velocity, size_t exit_vertex,
           std::vector<ScheduledStop> stops = {});

  // Rule of 0: defaults suffice

  /*
   * GETTER
   */

  /**
   * @brief Provides the entry time.
   *
   * @return The entry time.
   */
  [[nodiscard]] double get_entry_time() const { return m_entry_time; }

  /**
   * @brief Gets the scheduled exit time.
   *
   * @return The exit time constraint.
   */
  [[nodiscard]] double get_exit_time() const { return m_exit_time; }

  /**
   * @brief Obtains the initial velocity.
   *
   * @return The initial velocity of the train at entry.
   */
  [[nodiscard]] double get_initial_velocity() const {
    return m_initial_velocity;
  }

  /**
   * @brief Obtains the exit velocity.
   *
   * @return The exit velocity.
   */
  [[nodiscard]] double get_exit_velocity() const { return m_exit_velocity; }

  /**
   * @brief Determines the network vertex where the train enters.
   *
   * @return The entry vertex ID.
   */
  [[nodiscard]] size_t get_entry_vertex() const { return m_entry_vertex; }

  /**
   * @brief Gets the exit vertex of the schedule.
   *
   * @return The exit vertex ID.
   */
  [[nodiscard]] size_t get_exit_vertex() const { return m_exit_vertex; }

  /**
   * @brief Retrieves the scheduled stops for this train.
   *
   * @return Const reference to the vector of scheduled stops, ordered by
   * service time.
   */
  [[nodiscard]] std::vector<ScheduledStop> const& get_stops() const {
    return m_stops;
  }

  /**
   * @brief Returns the index of a scheduled stop by station name.
   *
   * @param station_name Name of the stop station.
   * @return Index of the matching stop in `get_stops()`.
   * @throws cda_rail::exceptions::StationNotExistentException If the station
   *         does not appear in the schedule.
   */
  [[nodiscard]] size_t get_station_index(std::string const& station_name) const;

  /*
   * SETTER
   */

  /**
   * @brief Sets the entry time.
   *
   * @param newEntryTime New entry time.
   * @throws cda_rail::exceptions::InvalidInputException If `newEntryTime` is
   * negative.
   */
  void set_entry_time(double const newEntryTime) {
    cda_rail::exceptions::throw_if_negative(newEntryTime, "Entry time");
    m_entry_time = newEntryTime;
  }

  /**
   * @brief Sets the exit time.
   *
   * @pre newExitTime >= m_entry_time.
   * @param newExitTime New exit time.
   * @throws cda_rail::exceptions::InvalidInputException If `newExitTime` is
   * smaller than the current entry time.
   */
  void set_exit_time(double const newExitTime) {
    cda_rail::exceptions::throw_if_less_than(newExitTime, m_entry_time,
                                             "Exit time");
    m_exit_time = newExitTime;
  }

  /**
   * @brief Updates the initial velocity.
   *
   * @param newInitialVelocity The new initial velocity.
   * @throws cda_rail::exceptions::InvalidInputException If the value is
   * negative.
   */
  void set_initial_velocity(double const newInitialVelocity) {
    cda_rail::exceptions::throw_if_negative(newInitialVelocity,
                                            "Initial velocity");
    m_initial_velocity = newInitialVelocity;
  }

  /**
   * @brief Sets the exit velocity.
   *
   * @param newExitVelocity New exit velocity.
   * @throws cda_rail::exceptions::InvalidInputException If the exit velocity
   * is negative.
   */
  void set_exit_velocity(double const newExitVelocity) {
    cda_rail::exceptions::throw_if_negative(newExitVelocity, "Exit velocity");
    m_exit_velocity = newExitVelocity;
  }

  /**
   * @brief Sets the entry vertex.
   *
   * @param newEntryVertex New entry vertex id.
   */
  void set_entry_vertex(size_t const newEntryVertex) {
    m_entry_vertex = newEntryVertex;
  }

  /**
   * @brief Sets the entry vertex id after validating it against a network.
   *
   * @param newEntryVertex New entry vertex id.
   * @param network Network used for vertex existence validation.
   * @throws cda_rail::exceptions::VertexNotExistentException If
   * `newEntryVertex` does not exist in `network`.
   */
  void set_entry_vertex(size_t const newEntryVertex, Network const& network) {
    if (!network.has_vertex(newEntryVertex)) {
      throw exceptions::VertexNotExistentException(newEntryVertex);
    }
    set_entry_vertex(newEntryVertex);
  }

  /**
   * @brief Sets the exit vertex id.
   *
   * @param newExitVertex New exit vertex id.
   */
  void set_exit_vertex(size_t const newExitVertex) {
    m_exit_vertex = newExitVertex;
  }

  /**
   * @brief Sets the exit vertex id after validating it against a network.
   *
   * @param newExitVertex New exit vertex id.
   * @param network Network used for vertex existence validation.
   * @throws cda_rail::exceptions::VertexNotExistentException If
   * `newExitVertex` does not exist in `network`.
   */
  void set_exit_vertex(size_t const newExitVertex, Network const& network) {
    if (!network.has_vertex(newExitVertex)) {
      throw exceptions::VertexNotExistentException(newExitVertex);
    }
    set_exit_vertex(newExitVertex);
  }

  /**
   * @brief Replaces the stop list for this schedule.
   *
   * @param new_stops New stop list to replace the current one.
   * @throws cda_rail::exceptions::InvalidInputException If the stop list
   * is not sorted by service time or contains duplicate stations.
   */
  void set_stops(std::vector<ScheduledStop> new_stops) {
    check_stops_validity(new_stops);
    m_stops = std::move(new_stops);
  }

  /**
   * @brief Inserts one stop while preserving stop-list invariants.
   *
   * Inserts the stop such that the list remains ordered by service time.
   * If other stops have the same service time, the new stop is inserted after
   * them.
   *
   * @pre The station of `new_stop` does not already exist in this schedule.
   * @param new_stop Stop to insert.
   * @throws cda_rail::exceptions::InvalidInputException If the station already
   * appears in the current stop list.
   */
  void insert_stop(ScheduledStop new_stop);

  /**
   * @brief Removes a stop by station name.
   *
   * @param station_name Name of the station whose stop should be removed.
   * @param throw_exception_if_not_existent If `true`, throw when no stop with
   * `station_name` exists; if `false`, perform no action in that case.
   * @throws cda_rail::exceptions::InvalidInputException If no matching stop
   * exists and `throw_exception_if_not_existent` is `true`.
   */
  void remove_stop(std::string const& station_name,
                   bool               throw_exception_if_not_existent = true);

  /**
   * @brief Sorts the scheduled stops by service time.
   */

  void sort_stops_by_service_time() {
    std::ranges::sort(m_stops, {}, &ScheduledStop::get_service_time);
  }
};

class Timetable {
  /**
   * Timetable class
   */
private:
  StationList           m_station_list{};
  TrainList             m_train_list{};
  std::vector<Schedule> m_schedules{};

  /**
   * @brief Sets the train list and reinitializes all schedules.
   *
   * Copies the provided train list and creates a default-initialized schedule
   * for each train. The schedules vector is resized to match the number of
   * trains in the list.
   *
   * @param tl The train list to set.
   */

  void set_train_list(const TrainList& tl) {
    this->m_train_list = tl;
    this->m_schedules  = std::vector<Schedule>(
        tl.size(), Schedule()); // this requires friendship
  }

  /**
   * @brief Parses one train schedule entry from JSON input.
   *
   * @param schedule_data JSON object containing schedule information.
   * @param i Train index whose schedule is populated.
   */
  void parse_schedule_data(const nlohmann::json& schedule_data, size_t i);

  /**
   * @brief Appends one schedule entry to an export JSON object.
   *
   * @param j JSON object updated in place.
   * @param i Train index whose schedule is exported.
   * @param network Network used to resolve vertex names.
   */
  void add_json_data(nlohmann::json& j, size_t i, const Network& network) const;

  /**
   * @brief Sorts the stops within each train schedule by service time.
   */
  void sort_stops_by_service_time() {
    for (auto& schedule : m_schedules) {
      schedule.sort_stops_by_service_time();
    }
  }

  std::pair<bool, std::optional<cda_rail::exceptions::CustomException>>
  check_consistency_helper() const;

public:
  Schedule& editable_schedule(size_t train_index) {
    return m_schedules.at(train_index);
  }

  /**
   * @brief Creates an empty timetable.
   */
  Timetable() = default;
  /**
   * @brief Constructs a timetable from explicit station, train, and schedule
   *        data.
   *
   * @param station_list Station list owned by the timetable.
   * @param train_list Train list owned by the timetable.
   * @param schedules Schedule vector indexed by train.
   */
  Timetable(StationList station_list, TrainList train_list,
            const std::vector<Schedule>& schedules);
  /**
   * @brief Loads a timetable from a filesystem path.
   *
   * @param p Path to the timetable data.
   * @param network Network used for validation and vertex resolution.
   */
  Timetable(const std::filesystem::path& p, const Network& network);
  /**
   * @brief Constructs a timetable from a file path specified as a string.
   *
   * @param path Path to the timetable data.
   */
  Timetable(const std::string& path, const Network& network)
      : Timetable(std::filesystem::path(path), network) {};
  /**
   * @brief Constructs a timetable from a C-string path.
   *
   * @param path C-string path to the timetable file.
   * @param network Network used for vertex validation.
   */
  Timetable(char const* const path, Network const& network)
      : Timetable(std::filesystem::path(path), network) {};

  // Rule of 0 suffices

  /**
   * @brief Provides a read-only iterator to the first schedule.
   * @return Const iterator to the beginning of the schedule list.
   */
  [[nodiscard]] constexpr auto begin() const { return m_schedules.cbegin(); };
  /**
   * @brief Provides the end iterator for iterating over schedules.
   * @return A const iterator to the position past the last schedule.
   */
  [[nodiscard]] constexpr auto end() const { return m_schedules.cend(); };
  /** @brief Read-only iterator to the first schedule (explicit const). */
  [[nodiscard]] constexpr auto cbegin() const { return m_schedules.cbegin(); };
  /**
   * @brief Returns a const iterator to one past the last schedule.
   *
   * @return A const iterator to the position following the last schedule.
   */
  [[nodiscard]] constexpr auto cend() const { return m_schedules.cend(); };
  /**
   * @brief Const reverse iterator to the last schedule.
   * @return auto Const reverse iterator to the beginning of the reversed
   * schedule sequence.
   */
  [[nodiscard]] constexpr auto crbegin() const {
    return m_schedules.crbegin();
  };
  /** @brief Read-only reverse iterator before the first schedule. */
  [[nodiscard]] constexpr auto crend() const { return m_schedules.crend(); };

  // Export / Import
  /**
   * @brief Exports the timetable to the given filesystem path.
   *
   * @param p Output path.
   * @param network Network context used during export.
   */
  void export_timetable(const std::filesystem::path& p,
                        const Network&               network) const;
  /**
   * @brief Writes the timetable to a file.
   *
   * @param path File path where the timetable should be written.
   * @param network Network context used for the export.
   */
  void export_timetable(const std::string& path, const Network& network) const {
    export_timetable(std::filesystem::path(path), network);
  };
  /**
   * @brief Exports the timetable to a file.
   */
  void export_timetable(char const* const path, Network const& network) const {
    export_timetable(std::filesystem::path(path), network);
  };

  /**
   * @brief Imports a timetable from a file.
   *
   * @param path Path to the timetable file.
   * @param network Network used for validation and context.
   * @return Timetable The imported timetable.
   */
  [[nodiscard]] static Timetable import_timetable(const std::string& path,
                                                  const Network&     network) {
    return {path, network};
  };
  /**
   * @brief Imports a timetable from a file.
   *
   * @param p Path to the timetable file.
   * @param network Network context for validating timetable data.
   * @return A Timetable loaded from the specified file.
   */
  [[nodiscard]] static Timetable
  import_timetable(const std::filesystem::path& p, const Network& network) {
    return {p, network};
  };
  /**
   * @brief Loads a timetable from a file.
   *
   * @param path File path as a C-string.
   * @param network Network used for validation.
   * @return Timetable loaded from the file.
   */
  [[nodiscard]] static Timetable import_timetable(char const* const path,
                                                  const Network&    network) {
    return {path, network};
  };

  /**
   * @brief Provides access to the station list.
   *
   * @return const StationList& Const reference to the station list.
   */

  [[nodiscard]] StationList const& get_station_list() const {
    return m_station_list;
  };
  /**
   * @brief Provides access to the train list.
   *
   * @return const reference to the TrainList managed by this timetable.
   */
  [[nodiscard]] TrainList const& get_train_list() const {
    return m_train_list;
  };

  /**
   * @brief Retrieves the schedule for a given train.
   *
   * @param trainIndex The index of the train.
   * @return const Schedule& The schedule for the specified train.
   * @throws exceptions::TrainNotExistentException if the train does not exist.
   */
  [[nodiscard]] Schedule const& get_schedule(size_t const trainIndex) const {
    if (!m_train_list.has_train(trainIndex)) {
      throw exceptions::TrainNotExistentException(trainIndex);
    }
    return m_schedules.at(trainIndex);
  };
  /**
   * @brief Retrieves the schedule for a train by name.
   *
   * @param train_name Name of the train.
   * @return The `Schedule` for the specified train.
   * @throws exceptions::TrainNotExistentException if the train does not exist.
   */
  [[nodiscard]] Schedule const&
  get_schedule(std::string const& train_name) const {
    return get_schedule(m_train_list.get_train_index(train_name));
  };

  /**
   * @brief Returns the latest scheduled exit time across all trains.
   *
   * @return Maximum exit time in the timetable, or `0` if no trains exist.
   */
  [[nodiscard]] double latest_exit_time() const;

  /**
   * @brief Provides mutable access to a train by index.
   * @return A mutable reference to the train at the given index.
   */
  [[nodiscard]] Train& editable_train(size_t const index) {
    return m_train_list.editable_train(index);
  };
  /**
   * @brief Obtains a mutable reference to a train by name.
   *
   * @param name The name of the train.
   * @return Train& A mutable reference to the Train with the given name.
   */
  [[nodiscard]] Train& editable_train(std::string const& name) {
    return m_train_list.editable_train(name);
  };

  /**
   * @brief Retrieves available track segments for a train at a station.
   *
   * @param tr Train index.
   * @param station_name Name of the station.
   * @param network The network containing track and vertex information.
   * @param edges_to_consider Network edges to filter available tracks.
   * @return Vector of pairs, each pairing a track identifier with index vectors
   *         indicating valid positioning ranges on that track for the specified
   *         train.
   */
  [[nodiscard]] std::vector<
      std::pair<size_t, std::vector<cda_rail::index_vector>>>
  get_stop_tracks(size_t const tr, std::string const& station_name,
                  Network const&             network,
                  cda_rail::index_set const& edges_to_consider) const {
    return m_station_list.get_stop_tracks(
        station_name, m_train_list.get_train(tr).get_length(), network,
        edges_to_consider);
  };

  /**
   * @brief Adds a new station to the timetable with no scheduled stops.
   *
   * @param station_name Name of the station to add.
   */

  void add_empty_station(const std::string& station_name) {
    m_station_list.add_empty_station(station_name);
  };

  /**
   * @brief Inserts a scheduled stop for a train identified by index.
   *
   * @param train_index Index of the train.
   * @param station_name Name of the station.
   * @param service_time Earliest service start time.
   * @param service_duration Minimum service duration.
   */
  void insert_stop(size_t train_index, std::string const& station_name,
                   double service_time, double service_duration);
  /**
   * @brief Inserts a scheduled stop for a train identified by name.
   *
   * @param train_name Name of the train.
   * @param station_name Name of the station.
   * @param serviceTime Earliest service start time.
   * @param serviceDuration Minimum service duration.
   *
   * @throws cda_rail::exceptions::InvalidInputException if the station already
   * exists in the train's schedule.
   */
  void insert_stop(std::string const& train_name,
                   std::string const& station_name, double const serviceTime,
                   double const serviceDuration) {
    insert_stop(m_train_list.get_train_index(train_name), station_name,
                serviceTime, serviceDuration);
  }

  /**
   * @brief Removes a stop from a train's schedule by train index.
   *
   * @param train_index Index of the train.
   * @param station_name Name of the station to remove.
   * @param throw_exception_if_not_existent If `true`, throws when no matching
   *        stop exists.
   */
  void remove_stop(size_t train_index, std::string const& station_name,
                   bool throw_exception_if_not_existent = true);
  /**
   * @brief Removes a stop from a train's schedule by station name.
   *
   * @param train_name Name of the train.
   * @param station_name Name of the station to remove.
   * @param throw_exception_if_not_existent If `true`, throws an exception if
   * the station is not found in the schedule.
   *
   * @throws cda_rail::exceptions::InvalidInputException If
   * `throw_exception_if_not_existent` is `true` and the station is not in the
   * train's schedule.
   */
  void remove_stop(std::string const& train_name,
                   std::string const& station_name,
                   bool               throw_exception_if_not_existent = true) {
    remove_stop(m_train_list.get_train_index(train_name), station_name,
                throw_exception_if_not_existent);
  }

  /**
   * @brief Adds a new track to the specified station.
   *
   * @param station_name Name of the station.
   * @param new_edge Edge input describing the new track.
   * @param network Network context for validation and resolution.
   */
  void add_track_to_station(std::string const&        station_name,
                            Network::EdgeInput const& new_edge,
                            Network const&            network) {
    m_station_list.add_track_to_station(station_name, new_edge, network);
  };

private:
  /**
   * @brief Adds a train and its schedule using resolved entry/exit vertices.
   *
   * @param train_name The name of the train.
   * @param length The length of the train in m.
   * @param max_speed The maximum speed of the train in m/s.
   * @param acceleration The acceleration of the train in m/s^2.
   * @param deceleration The deceleration of the train in m/s^2.
   * @param tim Whether train integrity monitoring is available.
   * @param entry_time Earliest time at which the train may enter the network.
   * @param initial_velocity Initial velocity at the entry vertex in m/s.
   * @param entry_vertex Index of the entry vertex in the network.
   * @param exit_time Desired exit time from the network.
   * @param exit_velocity Desired velocity at the exit vertex in m/s.
   * @param exit_vertex Index of the exit vertex in the network.
   *
   * @return The index of the train in the train list.
   * @throws cda_rail::exceptions::ConsistencyException If a train with the
   * same name already exists.
   */
  [[nodiscard]] size_t add_train_private_helper(
      std::string const& train_name, double length, double max_speed,
      double acceleration, double deceleration, bool tim, double entry_time,
      double initial_velocity, size_t entry_vertex, double exit_time,
      double exit_velocity, size_t exit_vertex);

public:
  /**
   * @brief Adds a train to the timetable with specified entry/exit constraints.
   *
   * @param train_name The name identifier for the train.
   * @param length The train's length.
   * @param max_speed The train's maximum speed.
   * @param acceleration The train's acceleration rate.
   * @param deceleration The train's deceleration rate.
   * @param tim Whether the train operates in time-independent movement mode.
   * @param entry_time The service start time at the entry vertex.
   * @param initial_velocity The velocity at entry.
   * @param entry_vertex The entry vertex specification.
   * @param exit_time The service end time at the exit vertex.
   * @param exit_velocity The velocity at exit.
   * @param exit_vertex The exit vertex specification.
   * @return size_t The index of the newly added train.
   */
  [[nodiscard]] size_t add_train(std::string const& train_name, double length,
                                 double max_speed, double acceleration,
                                 double deceleration, bool tim,
                                 double entry_time, double initial_velocity,
                                 Network::VertexInput const& entry_vertex,
                                 double exit_time, double exit_velocity,
                                 Network::VertexInput const& exit_vertex,
                                 Network const&              network) {
    return add_train_private_helper(
        train_name, length, max_speed, acceleration, deceleration, tim,
        entry_time, initial_velocity, entry_vertex.resolve(&network), exit_time,
        exit_velocity, exit_vertex.resolve(&network));
  };
  /**
   * @brief Adds a new train to the timetable.
   *
   * @param train_name Name of the train.
   * @param length Train length.
   * @param max_speed Maximum speed.
   * @param acceleration Acceleration rate.
   * @param deceleration Deceleration rate.
   * @param entry_time Scheduled entry time.
   * @param initial_velocity Velocity at entry.
   * @param entry_vertex Entry vertex; resolved against the network.
   * @param exit_time Scheduled exit time.
   * @param exit_velocity Velocity at exit.
   * @param exit_vertex Exit vertex; resolved against the network.
   * @param network Network to resolve vertex inputs against.
   * @return Index of the newly added train.
   */
  [[nodiscard]] size_t
  add_train(std::string const& train_name, double length, double max_speed,
            double acceleration, double deceleration, double entry_time,
            double initial_velocity, Network::VertexInput const& entry_vertex,
            double exit_time, double exit_velocity,
            Network::VertexInput const& exit_vertex, Network const& network) {
    return add_train(train_name, length, max_speed, acceleration, deceleration,
                     true, entry_time, initial_velocity, entry_vertex,
                     exit_time, exit_velocity, exit_vertex, network);
  };

  // No stop removal for now

  // Further helpers

  /**
   * @brief Checks whether timetable data is consistent with the network.
   *
   * Verifies that entry and exit vertices exist and are terminal vertices,
   * station tracks refer to valid network edges, scheduled stops are ordered by
   * service time and use distinct stations, and each stop's service interval is
   * contained in the train's entry/exit time window.
   *
   * @param network The network to which the timetable belongs.
   *
   * @return True if the timetable is consistent with the network, false
   * otherwise.
   */
  [[nodiscard]] bool check_consistency(Network const& network) const;

  /**
   * @brief Updates the timetable after network discretization.
   *
   * @param new_edges Pairs of vertex IDs and their associated edge indices
   * after discretization.
   */
  void update_after_discretization(
      std::vector<std::pair<size_t, cda_rail::index_set>> const& new_edges) {
    m_station_list.update_after_discretization(new_edges);
  }
  /**
   * @brief Updates the timetable after discretization.
   *
   * @param new_edges Vector of pairs mapping edge identifiers to their
   * discretized indices.
   */
  void update_after_discretization(
      std::vector<std::pair<size_t, cda_rail::index_vector>> const& new_edges) {
    m_station_list.update_after_discretization(new_edges);
  }

  /**
   * @brief Returns the discrete time-index interval relevant for one train.
   *
   * @param train_index Index of the train.
   * @param dt Time discretization step.
   * @param tn_inclusive Whether the upper bound includes the final time index.
   * @return Pair `(first_index, last_index)`.
   */
  [[nodiscard]] std::pair<size_t, size_t>
  time_index_interval(size_t train_index, double dt,
                      bool tn_inclusive = true) const;
  /**
   * @brief Computes the time index interval for a train, identified by name.
   *
   * @param train_name The name of the train.
   * @param dt The time discretization step.
   * @param tnInclusive Whether to include the final time index. Defaults to
   * `true`.
   *
   * @return A pair `(start_index, end_index)` of discretized time indices for
   * the train.
   */
  [[nodiscard]] std::pair<size_t, size_t>
  time_index_interval(std::string const& train_name, double const dt,
                      bool const tnInclusive = true) const {
    return time_index_interval(m_train_list.get_train_index(train_name), dt,
                               tnInclusive);
  };
};
} // namespace cda_rail
