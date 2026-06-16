#include "datastructure/Timetable.hpp"

#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "GeneralHelper.hpp"
#include "datastructure/RailwayNetwork.hpp"
#include "datastructure/Train.hpp"
#include "nlohmann/json.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <ranges>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

// using directives from header

/**
 * @brief Initializes a schedule with entry and exit times, velocities, vertices, and stops.
 *
 * Validates that entry time is non-negative, exit time is at least the entry 
 * time, velocities are non-negative, and stops are ordered by service time 
 * with unique station names.
 *
 * @param entryVertex Index of the starting vertex in the network.
 * @param exitVertex Index of the ending vertex in the network.
 * @param stops List of scheduled stops; moved into the schedule.
 *
 * @throws InvalidInputException if entry time is negative, exit time is less 
 *         than entry time, any velocity is negative, or stops are not ordered 
 *         by service time or contain duplicate station names.
 * @throws ConsistencyException if stop validation unexpectedly fails without 
 *         providing an exception object.
 */

cda_rail::Schedule::Schedule(double const entryTime,
                             double const initialVelocity,
                             size_t const entryVertex, double const exitTime,
                             double const exitVelocity, size_t const exitVertex,
                             std::vector<ScheduledStop> stops)
    : m_entry_time(entryTime), m_exit_time(exitTime),
      m_initial_velocity(initialVelocity), m_exit_velocity(exitVelocity),
      m_entry_vertex(entryVertex), m_exit_vertex(exitVertex),
      m_stops(std::move(stops)) {
  cda_rail::exceptions::throw_if_negative(m_entry_time, "Entry time");
  cda_rail::exceptions::throw_if_less_than(m_exit_time, m_entry_time,
                                           "Exit time");
  cda_rail::exceptions::throw_if_negative(m_initial_velocity,
                                          "Initial velocity");
  cda_rail::exceptions::throw_if_negative(m_exit_velocity, "Exit velocity");
  check_stops_validity(m_stops);
}

/**
 * @brief Validates that stops are ordered by service time and have unique station names.
 *
 * @return A pair where the first element is `true` if validation passes, `false` otherwise. When `false`, the second element contains an InvalidInputException describing the failure; when `true`, the second element is an empty optional.
 */
std::pair<bool, std::optional<cda_rail::exceptions::CustomException>>
cda_rail::Schedule::check_stops_validity_helper(
    std::vector<ScheduledStop> const& stops) {
  // 1. Ordered by service time
  if (!std::ranges::is_sorted(stops, {}, &ScheduledStop::get_service_time)) {
    return {false, cda_rail::exceptions::InvalidInputException(
                       "Scheduled stops must be ordered by service time")};
  }

  // 2. All station names are unique
  std::unordered_set<std::string> station_names;
  for (auto const& stop : stops) {
    auto const& stop_name = stop.get_station().name;
    if (station_names.contains(stop_name)) {
      return {false, cda_rail::exceptions::InvalidInputException(
                         stop_name +
                         " appears multiple times in the scheduled stops.")};
    }
    station_names.insert(stop_name);
  }

  return {true, {}};
}

/**
 * @brief Validates scheduled stops for proper ordering and uniqueness.
 *
 * Ensures stops are sorted in non-decreasing order by service time and
 * that each station appears exactly once.
 *
 * @param stops The stops to validate.
 *
 * @throws InvalidInputException If stops are not properly ordered by
 *         service time or if any station name is duplicated.
 * @throws ConsistencyException If validation fails without an error object
 *         (indicates an unexpected internal inconsistency).
 */
void cda_rail::Schedule::check_stops_validity(
    std::vector<ScheduledStop> const& stops) {
  if (auto const [result, exception] = check_stops_validity_helper(stops);
      !result) {
    throw exception.value_or(cda_rail::exceptions::ConsistencyException(
        "Consistency check returned false, but no error object was returned."));
  }
}

/**
 * @brief Inserts a scheduled stop into the schedule in sorted order by service time.
 *
 * If multiple stops have the same service time, the new stop is appended after existing ones.
 *
 * @param new_stop The stop to insert.
 * @throws cda_rail::exceptions::InvalidInputException if a stop with the same station name already exists in the schedule.
 */
void cda_rail::Schedule::insert_stop(ScheduledStop new_stop) {
  auto const& stop_name = new_stop.get_station().name;
  auto const& stop_time = new_stop.get_service_time();
  if (std::ranges::contains(m_stops, stop_name, [](auto const& stop) {
        return stop.get_station().name;
      })) {
    throw cda_rail::exceptions::InvalidInputException(
        stop_name + " already appears in the scheduled stops.");
  }

  // Insert in stops while maintaining order by service time
  // If multiple stops with the same service time exist, append
  auto const insert_pos =
      std::upper_bound(m_stops.begin(), m_stops.end(), stop_time,
                       [](double time, ScheduledStop const& stop) {
                         return time < stop.get_service_time();
                       });
  m_stops.insert(insert_pos, std::move(new_stop));
}

/**
 * @brief Removes a scheduled stop for the given station.
 *
 * @param station_name Name of the station whose stop should be removed.
 * @param throwExceptionIfNotExistent If `true`, throws when the stop does not exist;
 *                                    if `false`, does nothing if the stop is not found.
 *
 * @throws InvalidInputException If @p throwExceptionIfNotExistent is `true` and
 *                               the station name does not appear in scheduled stops.
 */
void cda_rail::Schedule::remove_stop(std::string const& station_name,
                                     bool const throwExceptionIfNotExistent) {
  auto const stop_it =
      std::ranges::find(m_stops, station_name, [](auto const& stop) {
        return stop.get_station().name;
      });
  if (stop_it == m_stops.end()) {
    if (throwExceptionIfNotExistent) {
      throw cda_rail::exceptions::InvalidInputException(
          station_name + " does not appear in the scheduled stops.");
    }
    return; // No stop to remove, but no exception thrown
  }
  m_stops.erase(stop_it);
}

/*
 * TIMETABLE
 */

/**
 * @brief Initializes a timetable by importing trains, stations, and schedules from a directory.
 *
 * @param p The directory containing the timetable files.
 * @param network The network associated with the timetable.
 *
 * @throws ImportException if the path does not exist or is not a directory.
 * @throws ScheduleNotExistentException if a train lacks schedule data.
 */

cda_rail::Timetable::Timetable(const std::filesystem::path& p,
                               const Network&               network) {
  /**
   * This method constructs the object and imports a timetable from a
   * directory. In particular the following files are read:
   * - trains.json according to the function defined in
   * cda_rail::TrainList::import_trains
   * - stations.json according to the function defined in
   * cda_rail::StationList::import_stations
   * - schedules.json of the format described in the respective
   * export_timetable
   *
   * @param p The path to the directory where the files should be read from.
   * @param network The network to which the timetable belongs.
   */

  if (!std::filesystem::exists(p)) {
    throw exceptions::ImportException("Path does not exist.");
  }
  if (!std::filesystem::is_directory(p)) {
    throw exceptions::ImportException("Path is not a directory.");
  }

  this->set_train_list(TrainList::import_trains(p));
  this->m_station_list = StationList::import_stations(p, network);

  std::ifstream f(p / "schedules.json");
  json          data = json::parse(f);

  for (size_t i = 0; i < this->m_train_list.size(); i++) {
    const auto& tr = this->m_train_list.get_train(i);
    if (!data.contains(tr.get_name())) {
      throw exceptions::ScheduleNotExistentException(tr.get_name());
    }

    const auto& schedule_data = data.at(tr.get_name());

    this->m_schedules.at(i).set_initial_velocity(
        schedule_data.at("v_0").get<double>());
    this->m_schedules.at(i).set_entry_vertex(
        network.get_vertex_index(schedule_data.at("entry").get<std::string>()));
    this->m_schedules.at(i).set_exit_velocity(
        schedule_data.at("v_n").get<double>());
    this->m_schedules.at(i).set_exit_vertex(
        network.get_vertex_index(schedule_data.at("exit").get<std::string>()));

    parse_schedule_data(schedule_data, i);
  }

  this->sort_stops_by_service_time();
};

/**
 * @brief Constructs a timetable with consistency validation.
 *
 * @param station_list Station list to move into the timetable.
 * @param train_list Train list to move into the timetable.
 * @param schedules Schedules to copy into the timetable.
 *
 * @throw ConsistencyException if train and schedule counts do not match, or if scheduled stops reference invalid stations.
 * @throw InvalidInputException if any schedule has improperly ordered stops or duplicate stations.
 */
cda_rail::Timetable::Timetable(StationList station_list, TrainList train_list,
                               const std::vector<Schedule>& schedules)
    : m_station_list(std::move(station_list)),
      m_train_list(std::move(train_list)), m_schedules(schedules) {
  auto const [consistency_result, consistency_details] =
      check_consistency_helper();
  if (!consistency_result) {
    throw consistency_details.value_or(
        cda_rail::exceptions::ConsistencyException(
            "Consistency check returned false, but no error object was "
            "returned."));
  }
}

/**
 * @brief Populates a schedule's entry/exit times and stops from JSON data.
 *
 * Reads schedule times and stop entries from the provided JSON object and applies
 * them to the schedule at index `i`.
 *
 * @param schedule_data JSON object with entry/exit times and stop definitions.
 * @param i             Schedule index.
 */
void cda_rail::Timetable::parse_schedule_data(json const&  schedule_data,
                                              size_t const i) {
  this->m_schedules.at(i).set_entry_time(schedule_data.at("t_0").get<double>());
  this->m_schedules.at(i).set_exit_time(schedule_data.at("t_n").get<double>());
  for (const auto& stop_data : schedule_data.at("stops")) {
    this->insert_stop(i, stop_data.at("station").get<std::string>(),
                      stop_data.at("begin").get<double>(),
                      stop_data.at("duration").get<double>());
  }
}

/**
 * @brief Adds schedule data for a specified train to a JSON object.
 *
 * Creates a JSON entry keyed by the train's name, containing entry and exit
 * times, velocities, station vertex names from the network, and all scheduled
 * stops.
 *
 * @param j JSON object to populate.
 * @param i Index of the train.
 * @param network Network used to resolve vertex names.
 */
void cda_rail::Timetable::add_json_data(json& j, const size_t i,
                                        const Network& network) const {
  const auto& schedule = m_schedules.at(i);
  json        stops;

  for (const auto& stop : schedule.get_stops()) {
    stops.push_back({{"begin", stop.get_service_time()},
                     {"duration", stop.get_service_duration()},
                     {"station", stop.get_station().name}});
  }
  // NOLINTNEXTLINE(*-pro-bounds-avoid-unchecked-container-access)
  j[m_train_list.get_train(i).get_name()] = {
      {"t_0", schedule.get_entry_time()},
      {"v_0", schedule.get_initial_velocity()},
      {"entry", network.get_vertex(schedule.get_entry_vertex()).name},
      {"t_n", schedule.get_exit_time()},
      {"v_n", schedule.get_exit_velocity()},
      {"exit", network.get_vertex(schedule.get_exit_vertex()).name},
      {"stops", stops}};
}

/**
 * @brief Exports the timetable to a directory with trains, stations, and schedules data.
 *
 * Creates three JSON files in the specified directory:
 * - `trains.json`: Train definitions.
 * - `stations.json`: Station definitions and network references.
 * - `schedules.json`: Schedule data keyed by train name, containing entry/exit times and velocities, entry/exit station names, and an array of stops with service time, duration, and station name.
 *
 * @param p Directory path where the files will be created. Created if it does not exist.
 * @param network The network used to resolve vertex names for entry and exit stations.
 *
 * @throws ExportException if the directory cannot be created.
 */

void cda_rail::Timetable::export_timetable(const std::filesystem::path& p,
                                           const Network& network) const {
  /**
   * This method exports the general timetable to a directory. In particular
   * the following files are created:
   * - trains.json according to the function defined in
   * cda_rail::TrainList::export_trains
   * - stations.json according to the function defined in
   * cda_rail::StationList::export_stations
   * - schedules.json of the following format:
   *  {"tr1": {"t_0": t_0, "v_0": v_0, "entry": v_name, "t_n": t_n, "v_n":
   * v_n, "exit": v_name, "stops": [{"begin": t_b, "duration": dt,
   * "station": s_name},
   * ...]}, ...}
   *
   *  @param p The path to the directory where the files should be created.
   */

  if (!is_directory_and_create(p)) {
    throw exceptions::ExportException("Could not create directory " +
                                      p.string());
  }

  m_train_list.export_trains(p);
  m_station_list.export_stations(p, network);

  json j = json::object();
  for (size_t i = 0; i < m_schedules.size(); ++i) {
    add_json_data(j, i, network);
  }

  std::ofstream file(p / "schedules.json");
  file << j << '\n';
}

/**
 * @brief Finds the latest exit time among all schedules.
 *
 * @return double The maximum exit time across all schedules, or NAN if no schedules exist.
 */

double cda_rail::Timetable::latest_exit_time() const {
  if (m_schedules.empty()) {
    return NAN;
  }
  return std::ranges::max(m_schedules |
                          std::views::transform(&Schedule::get_exit_time));
}

/**
 * @brief Computes the discretized time index interval for a train schedule.
 *
 * @param dt Time step length for discretization.
 * @param tnInclusive If true, the exit time index is inclusive; if false, exclusive.
 * @return A pair where the first element is the time index at the train's entry and the second is the time index at the train's exit.
 */
</code>
std::pair<size_t, size_t> cda_rail::Timetable::time_index_interval(
    size_t const trainIndex, double const dt, bool const tnInclusive) const {
  /**
   * This method returns the time interval of a train schedule as indices given
   * a time step length dt.
   *
   * @param train_index The index of the train in the train list.
   * @param dt The time step length.
   * @return A pair of integers (t_0, t_n) where t_0 is the time index at which
   * the train enters the network and t_n is the time index at which the train
   * leaves the network.
   */

  if (!m_train_list.has_train(trainIndex)) {
    throw exceptions::TrainNotExistentException(trainIndex);
  }
  cda_rail::exceptions::throw_if_non_positive(dt, "Time step length");

  const auto& schedule = m_schedules.at(trainIndex);
  const auto& t_0      = schedule.get_entry_time();
  const auto& t_n      = schedule.get_exit_time();

  cda_rail::exceptions::throw_if_negative(t_0, "Entry time");
  cda_rail::exceptions::throw_if_negative(t_n, "Exit time");

  auto const t_0_index = static_cast<size_t>(std::floor(t_0 / dt));
  // if t_n is divisible by dt (approx)
  if (std::abs(std::fmod(t_n, dt)) < EPS) {
    // if tn_inclusive, we want to include the time step at t_n, which is t_n /
    // dt if tn_inclusive is false, we want to exclude the time step at t_n,
    // which is (t_n / dt) - 1
    return {t_0_index,
            static_cast<size_t>(tnInclusive ? (t_n / dt) : (t_n / dt) - 1)};
  }

  size_t const t_n_index =
      static_cast<size_t>(std::round(t_n / dt)) + (tnInclusive ? 1 : 0);

  return {t_0_index, t_n_index};
}

/**
 * @brief Adds a new train and its initial schedule to the timetable.
 *
 * @param tim Train integrity monitoring flag.
 *
 * @return size_t Index of the newly added train.
 *
 * @throws ConsistencyException if a train with the given name already exists.
 * @throws ConsistencyException if the schedule count does not match the train list size after adding.
 */

size_t cda_rail::Timetable::add_train_private_helper(
    std::string const& train_name, double const length, double const maxSpeed,
    double const acceleration, double const deceleration, bool const tim,
    double const entryTime, double const initialVelocity,
    size_t const entryVertex, double const exitTime, double const exitVelocity,
    size_t const exitVertex) {
  if (m_train_list.has_train(train_name)) {
    throw exceptions::ConsistencyException("Train " + train_name +
                                           " already exists.");
  }
  auto const index = m_train_list.add_train(train_name, length, maxSpeed,
                                            acceleration, deceleration, tim);
  m_schedules.emplace_back(entryTime, initialVelocity, entryVertex, exitTime,
                           exitVelocity, exitVertex);
  if (m_schedules.size() != m_train_list.size()) {
    throw exceptions::ConsistencyException(
        "Schedule size (" + std::to_string(m_schedules.size()) +
        ") does not match train list size (" +
        std::to_string(m_train_list.size()) + ") after adding a train.");
  }
  return index;
}

/**
 * @brief Inserts a stop into a train's schedule.
 *
 * @param trainIndex Index of the train.
 * @param station_name Name of the station where the train stops.
 * @param serviceTime Time at which the service occurs.
 * @param serviceDuration Duration of the service at this stop.
 *
 * @throws TrainNotExistentException if the train index does not exist.
 * @throws StationNotExistentException if the station name does not exist.
 */
void cda_rail::Timetable::insert_stop(size_t const       trainIndex,
                                      std::string const& station_name,
                                      double const       serviceTime,
                                      double const       serviceDuration) {
  if (!m_train_list.has_train(trainIndex)) {
    throw exceptions::TrainNotExistentException(trainIndex);
  }
  if (!m_station_list.has_station(station_name)) {
    throw exceptions::StationNotExistentException(station_name);
  }

  m_schedules.at(trainIndex)
      .insert_stop({serviceTime, serviceDuration,
                    m_station_list.get_station_ptr(station_name)});
}

/**
 * @brief Removes a scheduled stop from a train.
 *
 * @param train_index Index of the train in the timetable.
 * @param station_name Name of the station to remove from the schedule.
 * @param throw_exception_if_not_existent If true, throws an exception when the stop is not found; otherwise no action is taken.
 *
 * @throws TrainNotExistentException if the train does not exist.
 * @throws StationNotExistentException if the station does not exist.
 * @throws InvalidInputException if `throw_exception_if_not_existent` is true and the stop is not found in the schedule.
 */
void cda_rail::Timetable::remove_stop(size_t             train_index,
                                      std::string const& station_name,
                                      bool throw_exception_if_not_existent) {
  if (!m_train_list.has_train(train_index)) {
    throw exceptions::TrainNotExistentException(train_index);
  }
  if (!m_station_list.has_station(station_name)) {
    throw exceptions::StationNotExistentException(station_name);
  }

  m_schedules.at(train_index)
      .remove_stop(station_name, throw_exception_if_not_existent);
}

/**
 * @brief Validates internal consistency of the timetable.
 *
 * Checks that the schedule list size matches the train list size, each schedule's stops are properly ordered and unique by station, and all scheduled stops reference stations that exist and match those in the station list (both by value and by identity).
 *
 * @return `{true, {}}` if all consistency checks pass; `{false, <exception>}` with a `ConsistencyException` describing the first detected inconsistency otherwise.
 */

std::pair<bool, std::optional<cda_rail::exceptions::CustomException>>
cda_rail::Timetable::check_consistency_helper() const {
  // size of train list and schedule is consistent
  if (m_train_list.size() != m_schedules.size()) {
    return {false, cda_rail::exceptions::ConsistencyException(
                       "Schedule size (" + std::to_string(m_schedules.size()) +
                       ") does not match train list size (" +
                       std::to_string(m_train_list.size()) + ").")};
  }

  // for every schedule the stop list is consistent
  for (auto const& schedule : m_schedules) {
    if (auto [result_tmp, exception_obj] =
            cda_rail::Schedule::check_stops_validity_helper(
                schedule.get_stops());
        !result_tmp) {
      return {false, std::move(exception_obj)};
    }
  }

  for (auto const& schedule : m_schedules) {
    for (auto const& stop : schedule.get_stops()) {
      auto const& station = stop.get_station();
      if (!m_station_list.has_station(station.name)) {
        return {false, cda_rail::exceptions::ConsistencyException(
                           "Station " + station.name +
                           " does not exist in station list.")};
      }
      if (station != m_station_list.get_station(station.name)) {
        return {false, cda_rail::exceptions::ConsistencyException(
                           "Scheduled stop station object " + station.name +
                           " differs from the station specified in station "
                           "list (e.g. different tracks).")};
      }
      if (std::addressof(station) !=
          std::addressof(m_station_list.get_station(station.name))) {
        return {false, cda_rail::exceptions::ConsistencyException(
                           "Scheduled stop station " + station.name +
                           " points to an equivalent object.")};
      }
    }
  }

  return {true, {}};
}

/**
 * @brief Validates the timetable's consistency with the provided network.
 *
 * Ensures internal timetable validity and alignment with the network structure:
 * all station tracks must exist as network edges, entry and exit vertices must
 * be valid terminals (existing vertices with exactly one neighbor), and all
 * scheduled stops must fall within their schedule's timeframe.
 *
 * @param network The network to validate against.
 * @return bool `true` if all consistency checks pass, `false` otherwise.
 */
bool cda_rail::Timetable::check_consistency(Network const& network) const {
  if (auto const [result, exception] = check_consistency_helper(); !result) {
    return false;
  }

  for (const auto& station : m_station_list | std::views::values) {
    if (!std::ranges::all_of(station->tracks, [&network](auto track) {
          return network.has_edge(track);
        })) {
      return false;
    }
  }

  // Helper: Checks if a terminal vertex exists and has exactly one neighbor
  auto is_valid_terminal = [&network](auto vertex) {
    return network.has_vertex(vertex) && network.neighbors(vertex).size() == 1;
  };

  // Helper: Validates a single schedule
  auto is_schedule_consistent = [&is_valid_terminal](Schedule const& schedule) {
    if (!is_valid_terminal(schedule.get_entry_vertex()) ||
        !is_valid_terminal(schedule.get_exit_vertex())) {
      return false;
    }

    // Check that all stops fall within the schedule's timeframe
    const auto& stops = schedule.get_stops();
    return std::ranges::all_of(stops, [schedule](const auto& stop) {
      return stop.get_service_time() >= schedule.get_entry_time() &&
             stop.get_earliest_departure() <= schedule.get_exit_time();
    });
  };

  return std::ranges::all_of(m_schedules, is_schedule_consistent);
}
