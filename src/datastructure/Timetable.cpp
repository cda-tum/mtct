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

void cda_rail::Schedule::check_stops_validity(
    std::vector<ScheduledStop> const& stops) {
  // 1. Ordered by service time
  if (!std::ranges::is_sorted(stops, {}, &ScheduledStop::get_service_time)) {
    throw cda_rail::exceptions::InvalidInputException(
        "Scheduled stops must be ordered by service time");
  }

  // 2. All station names are unique
  std::unordered_set<std::string> station_names;
  for (auto const& stop : stops) {
    auto const& stop_name = stop.get_station().name;
    station_names.contains(stop_name)
        ? throw cda_rail::exceptions::InvalidInputException(
              stop_name + " appears multiple times in the scheduled stops.")
        : station_names.insert(stop_name);
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
  this->station_list = StationList::import_stations(p, network);

  std::ifstream f(p / "schedules.json");
  json          data = json::parse(f);

  for (size_t i = 0; i < this->train_list.size(); i++) {
    const auto& tr = this->train_list.get_train(i);
    if (!data.contains(tr.get_name())) {
      throw exceptions::ScheduleNotExistentException(tr.get_name());
    }

    const auto& schedule_data = data.at(tr.get_name());

    this->schedules.at(i).set_initial_velocity(
        static_cast<double>(schedule_data["v_0"]));
    this->schedules.at(i).set_entry_vertex(
        network.get_vertex_index(schedule_data["entry"]));
    this->schedules.at(i).set_exit_velocity(
        static_cast<double>(schedule_data["v_n"]));
    this->schedules.at(i).set_exit_vertex(
        network.get_vertex_index(schedule_data["exit"]));

    parse_schedule_data(schedule_data, i);
  }

  this->sort_stops_by_service_time();
};

cda_rail::Timetable::Timetable(StationList station_list, TrainList train_list,
                               const std::vector<Schedule>& schedules)
    : station_list(std::move(station_list)), train_list(std::move(train_list)),
      schedules(schedules) {
  // TODO: check conformity
}

// Private helpers
void cda_rail::Timetable::parse_schedule_data(json const& schedule_data,
                                              int const   i) {
  this->schedules.at(i).set_entry_time(
      static_cast<double>(schedule_data["t_0"]));
  this->schedules.at(i).set_exit_time(
      static_cast<double>(schedule_data["t_n"]));
  for (const auto& stop_data : schedule_data["stops"]) {
    this->add_stop(i, stop_data["station"].get<std::string>(), false,
                   stop_data["begin"].get<double>(),
                   stop_data["duration"].get<double>());
  }
}

void cda_rail::Timetable::add_json_data(json& j, const size_t i,
                                        const Network& network) const {
  const auto& schedule = schedules.at(i);
  json        stops;

  for (const auto& stop : schedule.get_stops()) {
    stops.push_back({{"begin", stop.get_service_time()},
                     {"duration", stop.get_service_duration()},
                     {"station", stop.get_station().name}});
  }
  j.at(train_list.get_train(i).get_name()) = {
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

  train_list.export_trains(p);
  station_list.export_stations(p, network);

  json j;
  for (size_t i = 0; i < schedules.size(); ++i) {
    add_json_data(j, i, network);
  }

  std::ofstream file(p / "schedules.json");
  file << j << '\n';
}

// GETTER

double cda_rail::Timetable::latest_exit_time() const {
  if (schedules.empty()) {
    return NAN;
  }
  return std::ranges::max(schedules |
                          std::views::transform(&Schedule::get_exit_time));
}

std::pair<size_t, size_t>
cda_rail::Timetable::time_index_interval(size_t train_index, int dt,
                                         bool tn_inclusive) const {
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

  if (!train_list.has_train(train_index)) {
    throw exceptions::TrainNotExistentException(train_index);
  }

  const auto& schedule = schedules.at(train_index);
  const auto& t_0      = schedule.get_t_0();
  const auto& t_n      = schedule.get_t_n();

  if (t_0 < 0 || t_n < 0) {
    throw exceptions::ConsistencyException("Time cannot be negative.");
  }

  const auto t_0_index = t_0 / dt;
  const auto t_n_index =
      (t_n % dt == 0 ? (t_n / dt) - 1 : t_n / dt) + (tn_inclusive ? 1 : 0);

  return {static_cast<size_t>(t_0_index), static_cast<size_t>(t_n_index)};
}

// EDITING

void cda_rail::Timetable::insert_stop(size_t const       train_index,
                                      std::string const& station_name,
                                      double const       service_time,
                                      double const       service_duration) {
  if (!train_list.has_train(train_index)) {
    throw exceptions::TrainNotExistentException(train_index);
  }
  if (!station_list.has_station(station_name)) {
    throw exceptions::StationNotExistentException(station_name);
  }

  schedules.at(train_index)
      .insert_stop({service_time, service_duration,
                    station_list.get_station_ptr(station_name)});
}
