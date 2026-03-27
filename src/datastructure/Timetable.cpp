#include "datastructure/Timetable.hpp"

#include "CustomExceptions.hpp"

#include <cstddef>
#include <string>
#include <unordered_set>
#include <utility>

using std::size_t;

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
