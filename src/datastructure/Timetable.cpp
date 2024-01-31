#include "datastructure/Timetable.hpp"

#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "datastructure/Station.hpp"
#include "nlohmann/json.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <string>

using json = nlohmann::json;

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
    if (!network.has_vertex(schedule.get_entry()) ||
        !network.has_vertex(schedule.get_exit())) {
      return false;
    }
    if (network.neighbors(schedule.get_entry()).size() != 1 ||
        network.neighbors(schedule.get_exit()).size() != 1) {
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
    for (const auto& stop : schedule.get_stops()) {
      if (stop.arrival() < schedule.get_t_0() ||
          stop.departure() > schedule.get_t_n() ||
          stop.departure() < stop.arrival()) {
        return false;
      }
    }
  }

  for (const auto& schedule : schedules) {
    for (size_t i = 0; i < schedule.get_stops().size(); ++i) {
      for (size_t j = i + 1; j < schedule.get_stops().size(); ++j) {
        if (schedule.get_stops().at(i).conflicts(schedule.get_stops().at(j))) {
          return false;
        }
      }
    }
  }

  return true;
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
    if (schedule.get_t_n() > ret) {
      ret = schedule.get_t_n();
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
    throw exceptions::TrainNotExistentException(train_index);
  }

  const auto& schedule = schedules.at(train_index);
  return {schedule.get_t_0(), schedule.get_t_n()};
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
      (t_n % dt == 0 ? t_n / dt - 1 : t_n / dt) + (tn_inclusive ? 1 : 0);

  return {static_cast<size_t>(t_0_index), static_cast<size_t>(t_n_index)};
}
