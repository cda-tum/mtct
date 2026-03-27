#pragma once
#include "datastructure/RailwayNetwork.hpp"
#include "datastructure/Station.hpp"
#include "datastructure/Train.hpp"

#include <cstddef>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
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
  double                         m_service_time; // earliest start of service
  double                         m_service_duration; // minimal service duration
  std::shared_ptr<Station const> m_station;          // pointer to a station

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
                std::shared_ptr<Station const> station)
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
   * @brief Returns the earliest service start time.
   *
   * @return Earliest service start time (guaranteed >= 0).
   */
  [[nodiscard]] double get_service_time() const { return m_service_time; }

  /**
   * @brief Returns the earliest possible departure time from this stop.
   *
   * @return Earliest departure time, computed as service time plus service
   * duration (guaranteed >= 0).
   */
  [[nodiscard]] double get_earliest_departure() const {
    return m_service_time + m_service_duration;
  }

  /**
   * @brief Returns the minimum required service duration.
   *
   * @return Minimum service duration (guaranteed >= 0).
   */
  [[nodiscard]] double get_service_duration() const {
    return m_service_duration;
  }

  /**
   * @brief Returns the station referenced by this stop.
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
   * @pre new_service_duration >= 0 (non-negative).
   * @param new_service_duration New minimum service duration.
   * @throws cda_rail::exceptions::InvalidInputException If
   * `new_service_duration` is negative.
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
  void set_station(std::shared_ptr<Station const> new_station) {
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
  double m_entry_time; // (earliest) entry time: >= 0
  double m_exit_time;  // (desired) exit time: >= entry_time

  double m_initial_velocity; // initial velocity: >= 0
  double m_exit_velocity;    // (desired) exit velocity: >= 0

  size_t m_entry_vertex; // id of entry vertex
  size_t m_exit_vertex;  // id of exit vertex

  std::vector<ScheduledStop> m_stops{}; // list of stops, may be empty

  static void check_stops_validity(std::vector<ScheduledStop> const& stops);

public:
  // user-defined-constructor
  /**
   * @brief Constructs a schedule with fixed entry/exit constraints.
   *
   * @pre entryTime >= 0.
   * @pre exitTime >= entryTime.
   * @pre initialVelocity >= 0.
   * @pre exitVelocity >= 0.
   * @pre stops are ordered by service time and cover pairwise distinct stations
   * @param entryTime Earliest allowed entry time.
   * @param initialVelocity Initial velocity at entry.
   * @param entryVertex Entry vertex id.
   * @param exitTime Desired exit time.
   * @param exitVelocity Desired velocity at exit.
   * @param exitVertex Exit vertex id.
   * @param stops Ordered scheduled stops for this train.
   * @throws cda_rail::exceptions::InvalidInputException If any of the
   * preconditions is violated.
   */
  Schedule(double const entryTime, double const initialVelocity,
           size_t const entryVertex, double const exitTime,
           double const exitVelocity, size_t const exitVertex,
           std::vector<ScheduledStop> stops = {})
      : m_entry_time(entryTime), m_exit_time(exitTime),
        m_initial_velocity(initialVelocity), m_exit_velocity(exitVelocity),
        m_entry_vertex(entryVertex), m_exit_vertex(exitVertex),
        m_stops(std::move(stops)) {
    cda_rail::exceptions::throw_if_negative(m_entry_time, "Entry time");
    cda_rail::exceptions::throw_if_negative(
        m_exit_time - m_entry_time,
        "(to ensure exit_time >= entry_time) exit_time - entry_time");
    cda_rail::exceptions::throw_if_negative(m_initial_velocity,
                                            "Initial velocity");
    cda_rail::exceptions::throw_if_negative(m_exit_velocity, "Exit velocity");
    check_stops_validity(m_stops);
  }

  // Rule of 0: defaults suffice

  /*
   * GETTER
   */

  /**
   * @brief Returns the entry time.
   *
   * @return Entry time.
   */
  [[nodiscard]] double get_entry_time() const { return m_entry_time; }

  /**
   * @brief Returns the exit time.
   *
   * @return Exit time.
   */
  [[nodiscard]] double get_exit_time() const { return m_exit_time; }

  /**
   * @brief Returns the initial velocity.
   *
   * @return Initial velocity.
   */
  [[nodiscard]] double get_initial_velocity() const {
    return m_initial_velocity;
  }

  /**
   * @brief Returns the desired exit velocity.
   *
   * @return Exit velocity.
   */
  [[nodiscard]] double get_exit_velocity() const { return m_exit_velocity; }

  /**
   * @brief Returns the entry vertex id.
   *
   * @return Entry vertex id.
   */
  [[nodiscard]] size_t get_entry_vertex() const { return m_entry_vertex; }

  /**
   * @brief Returns the exit vertex id.
   *
   * @return Exit vertex id.
   */
  [[nodiscard]] size_t get_exit_vertex() const { return m_exit_vertex; }

  /**
   * @brief Returns all scheduled stops.
   *
   * @return Constant reference to the list of scheduled stops.
   */
  [[nodiscard]] std::vector<ScheduledStop> const& get_stops() const {
    return m_stops;
  }

  /*
   * SETTER
   */

  /**
   * @brief Sets the entry time.
   *
   * @pre newEntryTime >= 0.
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
    cda_rail::exceptions::throw_if_negative(
        newExitTime - m_entry_time,
        "(to ensure exit_time >= entry_time) newExitTime - entry_time");
    m_exit_time = newExitTime;
  }

  /**
   * @brief Sets the initial velocity.
   *
   * @pre newInitialVelocity >= 0.
   * @param newInitialVelocity New initial velocity.
   * @throws cda_rail::exceptions::InvalidInputException If
   * `newInitialVelocity` is negative.
   */
  void set_initial_velocity(double const newInitialVelocity) {
    cda_rail::exceptions::throw_if_negative(newInitialVelocity,
                                            "Initial velocity");
    m_initial_velocity = newInitialVelocity;
  }

  /**
   * @brief Sets the exit velocity.
   *
   * @pre newExitVelocity >= 0.
   * @param newExitVelocity New exit velocity.
   * @throws cda_rail::exceptions::InvalidInputException If `newExitVelocity`
   * is negative.
   */
  void set_exit_velocity(double const newExitVelocity) {
    cda_rail::exceptions::throw_if_negative(newExitVelocity, "Exit velocity");
    m_exit_velocity = newExitVelocity;
  }

  /**
   * @brief Sets the entry vertex id.
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

  void set_stops(std::vector<ScheduledStop> new_stops) {
    check_stops_validity(new_stops);
    m_stops = std::move(new_stops);
  }
  void insert_stop(ScheduledStop new_stop);
  void remove_stop(std::string const& station_name,
                   bool               throw_exception_if_not_existent = true);
};

class Timetable : public GeneralTimetable<Schedule> {
  /**
   * Timetable class
   */
public:
  // Constructors
  Timetable() = default;
  Timetable(const std::filesystem::path& p, const Network& network)
      : GeneralTimetable(p, network) {};
  Timetable(const std::string& path, const Network& network)
      : Timetable(std::filesystem::path(path), network) {};
  Timetable(const char* path, const Network& network)
      : Timetable(std::filesystem::path(path), network) {};
  Timetable(const StationList& station_list, const TrainList& train_list,
            const std::vector<Schedule>& schedules)
      : GeneralTimetable(station_list, train_list, schedules) {};
  virtual ~Timetable() = default;

  [[nodiscard]] std::pair<size_t, size_t>
  time_index_interval(size_t train_index, int dt,
                      bool tn_inclusive = true) const;
  [[nodiscard]] std::pair<size_t, size_t>
  time_index_interval(const std::string& train_name, int dt,
                      bool tn_inclusive = true) const {
    return time_index_interval(train_list.get_train_index(train_name), dt,
                               tn_inclusive);
  };

  [[nodiscard]] GeneralTimetable<GeneralSchedule<GeneralScheduledStop>>
  parse_to_general_timetable() const {
    std::vector<GeneralSchedule<GeneralScheduledStop>> general_schedules;
    general_schedules.reserve(this->get_train_list().size());
    for (const auto& schedule : schedules) {
      general_schedules.push_back(schedule.parse_to_general_schedule());
    }
    return {station_list, train_list, general_schedules};
  };

  template <typename S, typename = std::enable_if_t<
                            std::is_base_of_v<GeneralScheduledStop, S>>>
  [[nodiscard]] static Timetable cast_from_general_timetable(
      const GeneralTimetable<GeneralSchedule<S>>& general_timetable_obj,
      bool                                        throw_error = true) {
    std::vector<Schedule> schedules;
    const size_t          number_of_element =
        general_timetable_obj.get_train_list().size();
    schedules.reserve(number_of_element);
    for (size_t tr = 0; tr < number_of_element; ++tr) {
      schedules.push_back(Schedule::cast_from_general_schedule(
          general_timetable_obj.get_schedule(tr), throw_error));
    }
    return {general_timetable_obj.get_station_list(),
            general_timetable_obj.get_train_list(), schedules};
  }

  [[nodiscard]] static Timetable import_timetable(const std::string& path,
                                                  const Network&     network) {
    return {path, network};
  };
  [[nodiscard]] static Timetable
  import_timetable(const std::filesystem::path& p, const Network& network) {
    return {p, network};
  };
  [[nodiscard]] static Timetable import_timetable(const char*    path,
                                                  const Network& network) {
    return {path, network};
  };
};
} // namespace cda_rail
