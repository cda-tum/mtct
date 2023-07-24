#include "datastructure/Timetable.hpp"

#include "Definitions.hpp"
#include "datastructure/Station.hpp"
#include "nlohmann/json.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <string>

using json = nlohmann::json;

const cda_rail::Schedule&
cda_rail::Timetable::get_schedule(size_t index) const {
  /**
   * This method returns the schedule of a train with given index.
   *
   * @param index The index of the train in the train list.
   *
   * @return The schedule of the train.
   */
  if (!train_list.has_train(index)) {
    throw std::invalid_argument("Train does not exist.");
  }
  return schedules.at(index);
}

size_t cda_rail::Timetable::add_train(const std::string& name, int length,
                                      double max_speed, double acceleration,
                                      double deceleration, int t_0, double v_0,
                                      size_t entry, int t_n, double v_n,
                                      size_t exit, const Network& network) {
  /**
   * This method adds a train to the timetable. The train is specified by its
   * parameters.
   *
   * @param name The name of the train.
   * @param length The length of the train in m.
   * @param max_speed The maximum speed of the train in m/s.
   * @param acceleration The acceleration of the train in m/s^2.
   * @param deceleration The deceleration of the train in m/s^2.
   * @param t_0 The time at which the train enters the network in s.
   * @param v_0 The speed at which the train enters the network in m/s.
   * @param entry The index of the entry vertex in the network.
   * @param t_n The time at which the train leaves the network in s.
   * @param v_n The speed at which the train leaves the network in m/s.
   * @param exit The index of the exit vertex in the network.
   * @param network The network to which the timetable belongs.
   *
   * @return The index of the train in the train list.
   */
  if (!network.has_vertex(entry)) {
    throw std::invalid_argument("Entry vertex does not exist.");
  }
  if (!network.has_vertex(exit)) {
    throw std::invalid_argument("Exit vertex does not exist.");
  }
  if (train_list.has_train(name)) {
    throw std::invalid_argument("Train already exists.");
  }
  auto const index =
      train_list.add_train(name, length, max_speed, acceleration, deceleration);
  schedules.emplace_back(t_0, v_0, entry, t_n, v_n, exit);
  return index;
}

void cda_rail::Timetable::add_stop(size_t             train_index,
                                   const std::string& station_name, int begin,
                                   int end, bool sort) {
  /**
   * This method adds a stop to a train schedule. The stop is specified by its
   * parameters.
   *
   * @param train_index The index of the train in the train list.
   * @param station_name The name of the station.
   * @param begin The time at which the train stops at the station in s.
   * @param end The time at which the train leaves the station in s.
   * @param sort If true, the stops are sorted after insertion.
   */
  if (!train_list.has_train(train_index)) {
    throw std::invalid_argument("Train does not exist.");
  }
  if (!station_list.has_station(station_name)) {
    throw std::invalid_argument("Station does not exist.");
  }
  if (begin < 0 || end < 0) {
    throw std::invalid_argument("Time cannot be negative.");
  }
  if (begin >= end) {
    throw std::invalid_argument("End time has to be after the start time.");
  }

  auto& stops_reference = schedules.at(train_index).stops;
  for (const auto& stop : stops_reference) {
    if (stop.station == station_name) {
      throw std::invalid_argument("Train already stops at station.");
    }

    if (begin <= stop.end && end >= stop.begin) {
      throw std::invalid_argument("Train has another stop at this time.");
    }
  }

  stops_reference.emplace_back(begin, end, station_name);
  if (sort) {
    std::sort(stops_reference.begin(), stops_reference.end());
  }
}

void cda_rail::Timetable::sort_stops() {
  /**
   * This methods sorts all stops of all trains according to the operator < of
   * ScheduledStop.
   */

  for (auto& schedule : schedules) {
    std::sort(schedule.stops.begin(), schedule.stops.end());
  }
}

void cda_rail::Timetable::export_timetable(const std::filesystem::path& p,
                                           const Network& network) const {
  /**
   * This method exports the timetable to a directory. In particular the
   * following files are created:
   * - trains.json according to the function defined in
   * cda_rail::TrainList::export_trains
   * - stations.json according to the function defined in
   * cda_rail::StationList::export_stations
   * - schedules.json of the following format:
   *  {"tr1": {"t_0": t_0, "v_0": v_0, "entry": v_name, "t_n": t_n, "v_n": v_n,
   * "exit": v_name, "stops": [{"begin": t_b, "end": t_e, "station": s_name},
   * ...]}, ...}
   *
   *  @param p The path to the directory where the files should be created.
   */

  if (!is_directory_and_create(p)) {
    throw std::runtime_error("Could not create directory " + p.string());
  }

  train_list.export_trains(p);
  station_list.export_stations(p, network);

  json j;
  for (size_t i = 0; i < schedules.size(); ++i) {
    const auto& schedule = schedules.at(i);
    json        stops;

    for (const auto& stop : schedule.stops) {
      stops.push_back(
          {{"begin", stop.begin},
           {"end", stop.end},
           {"station", station_list.get_station(stop.station).name}});
    }
    j[train_list.get_train(i).name] = {
        {"t_0", schedule.t_0},
        {"v_0", schedule.v_0},
        {"entry", network.get_vertex(schedule.entry).name},
        {"t_n", schedule.t_n},
        {"v_n", schedule.v_n},
        {"exit", network.get_vertex(schedule.exit).name},
        {"stops", stops}};
  }

  std::ofstream file(p / "schedules.json");
  file << j << std::endl;
}

void cda_rail::Timetable::set_train_list(const TrainList& tl) {
  /**
   * This method sets the train list of the timetable.
   *
   * @param tl The train list to set.
   */
  train_list = tl;
  schedules  = std::vector<Schedule>(tl.size());
}

bool cda_rail::Timetable::check_consistency(const Network& network) const {
  /**
   * This method checks if the timetable is consistent with the network, i.e.,
   * if the following holds:
   * - All vertices used as entry and exit points are valid vertices of the
   * network
   * - Entry and exit vertices have exactly one neighboring vertex
   * - All edges of stations are valid edges of the network
   * - All scheduled stops are comparable by < or >, hence not overlapping
   * - All scheduled stops lie within t_0 and t_n
   *
   * @param network The network to which the timetable belongs.
   *
   * @return True if the timetable is consistent with the network, false
   * otherwise.
   */

  for (const auto& schedule : schedules) {
    if (!network.has_vertex(schedule.entry) ||
        !network.has_vertex(schedule.exit)) {
      return false;
    }
    if (network.neighbors(schedule.entry).size() != 1 ||
        network.neighbors(schedule.exit).size() != 1) {
      return false;
    }
  }

  for (const auto& station_name : station_list.get_station_names()) {
    const auto& station = station_list.get_station(station_name);
    for (auto track : station.tracks) {
      if (!network.has_edge(track)) {
        return false;
      }
    }
  }

  for (const auto& schedule : schedules) {
    for (const auto& stop : schedule.stops) {
      if (stop.begin < schedule.t_0 || stop.end > schedule.t_n ||
          stop.end < stop.begin) {
        return false;
      }
    }
  }

  for (const auto& schedule : schedules) {
    for (size_t i = 0; i < schedule.stops.size(); ++i) {
      for (size_t j = i + 1; j < schedule.stops.size(); ++j) {
        if (!(schedule.stops.at(i) < schedule.stops.at(j)) &&
            !(schedule.stops.at(j) < schedule.stops.at(i))) {
          return false;
        }
      }
    }
  }

  return true;
}

cda_rail::Timetable::Timetable(const std::filesystem::path& p,
                               const Network&               network) {
  /**
   * This method constructs the object and imports a timetable from a directory.
   * In particular the following files are read:
   * - trains.json according to the function defined in
   * cda_rail::TrainList::import_trains
   * - stations.json according to the function defined in
   * cda_rail::StationList::import_stations
   * - schedules.json of the format described in
   * cda_rail::Timetable::export_timetable
   *
   * @param p The path to the directory where the files should be read from.
   * @param network The network to which the timetable belongs.
   */

  if (!std::filesystem::exists(p)) {
    throw std::invalid_argument("Path does not exist.");
  }
  if (!std::filesystem::is_directory(p)) {
    throw std::invalid_argument("Path is not a directory.");
  }

  this->set_train_list(TrainList::import_trains(p));
  this->station_list = StationList::import_stations(p, network);

  std::ifstream f(p / "schedules.json");
  json          data = json::parse(f);

  for (size_t i = 0; i < this->train_list.size(); i++) {
    const auto& tr = this->train_list.get_train(i);
    if (!data.contains(tr.name)) {
      throw std::invalid_argument("Schedule for train " + tr.name +
                                  " not found.");
    }
    auto& schedule_data       = data[tr.name];
    this->schedules.at(i).t_0 = schedule_data["t_0"];
    this->schedules.at(i).v_0 = schedule_data["v_0"];
    this->schedules.at(i).entry =
        network.get_vertex_index(schedule_data["entry"]);
    this->schedules.at(i).t_n = schedule_data["t_n"];
    this->schedules.at(i).v_n = schedule_data["v_n"];
    this->schedules.at(i).exit =
        network.get_vertex_index(schedule_data["exit"]);

    for (const auto& stop_data : schedule_data["stops"]) {
      this->add_stop(i, stop_data["station"].get<std::string>(),
                     stop_data["begin"].get<int>(), stop_data["end"].get<int>(),
                     false);
    }
  }

  this->sort_stops();
}

int cda_rail::Timetable::max_t() const {
  /**
   * This method returns the maximum time of all trains, i.e., the time at which
   * the last train leaves the network.
   *
   * @return The maximum time of all trains.
   */
  int ret = 0;
  for (const auto& schedule : schedules) {
    if (schedule.t_n > ret) {
      ret = schedule.t_n;
    }
  }
  return ret;
}

std::pair<int, int>
cda_rail::Timetable::time_interval(size_t train_index) const {
  /**
   * This method returns the time interval of a train schedule, i.e., the time
   * at which it enters the network and the time at which it leaves the network.
   *
   * @param train_index The index of the train in the train list.
   * @return A pair of integers (t_0, t_n) where t_0 is the time at which the
   * train enters the network and t_n is the time at which the train leaves the
   * network.
   */

  if (!train_list.has_train(train_index)) {
    throw std::invalid_argument("Train does not exist.");
  }

  const auto& schedule = schedules.at(train_index);
  return {schedule.t_0, schedule.t_n};
}
