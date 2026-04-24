#include "datastructure/Timetable.hpp"

#include "CustomExceptions.hpp"
#include "nlohmann/json.hpp"
#include "nlohmann/json_fwd.hpp"

#include <cstddef>
#include <string>
#include <unordered_set>
#include <utility>

// using directives from header

/*
 * SCHEDULE
 */

cda_rail::Schedule::Schedule(double const entry_time,
                             double const initial_velocity,
                             size_t const entry_vertex, double const exit_time,
                             double const               exit_velocity,
                             size_t const               exit_vertex,
                             std::vector<ScheduledStop> stops)
    : m_entry_time(entry_time), m_exit_time(exit_time),
      m_initial_velocity(initial_velocity), m_exit_velocity(exit_velocity),
      m_entry_vertex(entry_vertex), m_exit_vertex(exit_vertex),
      m_stops(std::move(stops)) {
  cda_rail::exceptions::throw_if_negative(m_entry_time, "Entry time");
  cda_rail::exceptions::throw_if_less_than(m_exit_time, m_entry_time,
                                           "Exit time");
  cda_rail::exceptions::throw_if_negative(m_initial_velocity,
                                          "Initial velocity");
  cda_rail::exceptions::throw_if_negative(m_exit_velocity, "Exit velocity");
  check_stops_validity(m_stops);
}

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

void cda_rail::Schedule::check_stops_validity(
    std::vector<ScheduledStop> const& stops) {
  if (auto const [result, exception] = check_stops_validity_helper(stops);
      !result) {
    throw exception.value_or(cda_rail::exceptions::ConsistencyException(
        "Consistency check returned false, but no error object was returned."));
  }
}

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
  auto const insert_pos = std::ranges::upper_bound(
      m_stops, stop_time, {}, &ScheduledStop::get_service_time);
  m_stops.insert(insert_pos, std::move(new_stop));
}

void cda_rail::Schedule::remove_stop(
    std::string const& station_name,
    bool const         throw_exception_if_not_existent) {
  auto const stop_it =
      std::ranges::find(m_stops, station_name, [](auto const& stop) {
        return stop.get_station().name;
      });
  if (stop_it == m_stops.end()) {
    if (throw_exception_if_not_existent) {
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

// Constructors

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
        static_cast<double>(schedule_data["v_0"]));
    this->m_schedules.at(i).set_entry_vertex(
        network.get_vertex_index(schedule_data["entry"]));
    this->m_schedules.at(i).set_exit_velocity(
        static_cast<double>(schedule_data["v_n"]));
    this->m_schedules.at(i).set_exit_vertex(
        network.get_vertex_index(schedule_data["exit"]));

    parse_schedule_data(schedule_data, i);
  }

  this->sort_stops_by_service_time();
};

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

// Private helpers
void cda_rail::Timetable::parse_schedule_data(json const& schedule_data,
                                              int const   i) {
  this->m_schedules.at(i).set_entry_time(
      static_cast<double>(schedule_data["t_0"]));
  this->m_schedules.at(i).set_exit_time(
      static_cast<double>(schedule_data["t_n"]));
  for (const auto& stop_data : schedule_data["stops"]) {
    this->insert_stop(i, stop_data["station"].get<std::string>(),
                      stop_data["begin"].get<double>(),
                      stop_data["duration"].get<double>());
  }
}

void cda_rail::Timetable::add_json_data(json& j, const size_t i,
                                        const Network& network) const {
  const auto& schedule = m_schedules.at(i);
  json        stops;

  for (const auto& stop : schedule.get_stops()) {
    stops.push_back({{"begin", stop.get_service_time()},
                     {"duration", stop.get_service_duration()},
                     {"station", stop.get_station().name}});
  }
  j.at(m_train_list.get_train(i).get_name()) = {
      {"t_0", schedule.get_entry_time()},
      {"v_0", schedule.get_initial_velocity()},
      {"entry", network.get_vertex(schedule.get_entry_vertex()).name},
      {"t_n", schedule.get_exit_time()},
      {"v_n", schedule.get_exit_velocity()},
      {"exit", network.get_vertex(schedule.get_exit_vertex()).name},
      {"stops", stops}};
}

// EXPORT

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
   * v_n, "exit": v_name, "stops": [{"begin": t_b, "end": t_e, "station":
   * s_name},
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

  json j;
  for (size_t i = 0; i < m_schedules.size(); ++i) {
    add_json_data(j, i, network);
  }

  std::ofstream file(p / "schedules.json");
  file << j << '\n';
}

// GETTER

double cda_rail::Timetable::latest_exit_time() const {
  if (m_schedules.empty()) {
    return NAN;
  }
  return std::ranges::max(m_schedules |
                          std::views::transform(&Schedule::get_exit_time));
}

std::pair<size_t, size_t> cda_rail::Timetable::time_index_interval(
    size_t const train_index, double const dt, bool const tn_inclusive) const {
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

  if (!m_train_list.has_train(train_index)) {
    throw exceptions::TrainNotExistentException(train_index);
  }
  cda_rail::exceptions::throw_if_non_positive(dt, "Time step length");

  const auto& schedule = m_schedules.at(train_index);
  const auto& t_0      = schedule.get_entry_time();
  const auto& t_n      = schedule.get_exit_time();

  cda_rail::exceptions::throw_if_negative(t_0, "Entry time");
  cda_rail::exceptions::throw_if_negative(t_n, "Exit time");

  if (t_0 < 0 || t_n < 0) {
    throw exceptions::ConsistencyException("Time cannot be negative.");
  }

  size_t const t_0_index = static_cast<size_t>(std::floor(t_0 / dt));
  // if t_n is divisible by dt (approx)
  if (std::abs(std::fmod(t_n, dt)) < EPS) {
    // if tn_inclusive, we want to include the time step at t_n, which is t_n /
    // dt if tn_inclusive is false, we want to exclude the time step at t_n,
    // which is (t_n / dt) - 1
    return {t_0_index,
            static_cast<size_t>(tn_inclusive ? (t_n / dt) : (t_n / dt) - 1)};
  }

  size_t const t_n_index = static_cast<size_t>(std::round(t_n / dt)) +
                           (tn_inclusive ? 1 : 0) +
                           (std::abs(std::fmod(t_n, dt)) < EPS ? -1 : 0);

  return {t_0_index, t_n_index};
}

// EDITING

size_t cda_rail::Timetable::add_train(
    std::string const& train_name, double const length, double const max_speed,
    double const acceleration, double const deceleration, bool const tim,
    double const entry_time, double const initial_velocity,
    size_t const entry_vertex, double const exit_time,
    double const exit_velocity, size_t const exit_vertex,
    Network const& network) {
  if (!network.has_vertex(entry_vertex)) {
    throw exceptions::VertexNotExistentException(entry_vertex);
  }
  if (!network.has_vertex(exit_vertex)) {
    throw exceptions::VertexNotExistentException(exit_vertex);
  }
  if (m_train_list.has_train(train_name)) {
    throw exceptions::ConsistencyException("Train " + train_name +
                                           " already exists.");
  }
  auto const index = m_train_list.add_train(train_name, length, max_speed,
                                            acceleration, deceleration, tim);
  m_schedules.emplace_back(entry_time, initial_velocity, entry_vertex,
                           exit_time, exit_velocity, exit_vertex);
  if (m_schedules.size() != m_train_list.size()) {
    throw exceptions::ConsistencyException(
        "Schedule size (" + std::to_string(m_schedules.size()) +
        ") does not match train list size (" +
        std::to_string(m_train_list.size()) + ") after adding a train.");
  }
  return index;
}

void cda_rail::Timetable::insert_stop(size_t const       train_index,
                                      std::string const& station_name,
                                      double const       service_time,
                                      double const       service_duration) {
  if (!m_train_list.has_train(train_index)) {
    throw exceptions::TrainNotExistentException(train_index);
  }
  if (!m_station_list.has_station(station_name)) {
    throw exceptions::StationNotExistentException(station_name);
  }

  m_schedules.at(train_index)
      .insert_stop({service_time, service_duration,
                    m_station_list.get_station_ptr(station_name)});
}

// HELPER

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

bool cda_rail::Timetable::check_consistency(Network const& network) {
  if (auto const [result, exception] = check_consistency_helper(); !result) {
    return false;
  }

  // Helper: Checks if a terminal vertex exists and has exactly one neighbor
  auto is_valid_terminal = [&network](auto vertex) {
    return network.has_vertex(vertex) && network.neighbors(vertex).size() == 1;
  };

  // Helper: Validates that all tracks in a given station exist in the network
  auto are_station_tracks_valid = [&network](ScheduledStop const& stop) {
    return std::ranges::all_of(
        stop.get_station().tracks,
        [&network](auto track) { return network.has_edge(track); });
  };

  // Helper: Validates a single schedule (combines the 1st and 3rd loops)
  auto is_schedule_consistent = [&is_valid_terminal, &are_station_tracks_valid](
                                    Schedule const& schedule) {
    if (!is_valid_terminal(schedule.get_entry_vertex()) ||
        !is_valid_terminal(schedule.get_exit_vertex())) {
      return false;
    }

    if (!std::ranges::all_of(schedule.get_stops(), are_station_tracks_valid)) {
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
