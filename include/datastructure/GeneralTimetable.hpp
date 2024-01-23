#pragma once

#include "CustomExceptions.hpp"

#include <algorithm>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace cda_rail {
class GeneralScheduledStop {
  /**
   * A (general) scheduled stop.
   */
protected:
  std::pair<int, int> begin;
  std::pair<int, int> end;
  int                 min_stopping_time;
  std::string         station;

public:
  bool operator<(const GeneralScheduledStop& other) const {
    return (!conflicts(other) && begin.second < other.begin.first &&
            end.first < other.begin.second);
  }
  bool operator>(const GeneralScheduledStop& other) const {
    return (!conflicts(other) && other.begin.second < begin.first &&
            other.end.first < begin.second);
  }
  bool operator==(const GeneralScheduledStop& other) const {
    return (begin == other.begin && end == other.end);
  }
  bool operator<=(const GeneralScheduledStop& other) const {
    return *this < other || *this == other;
  }
  bool operator>=(const GeneralScheduledStop& other) const {
    return *this > other || *this == other;
  }
  bool operator!=(const GeneralScheduledStop& other) const {
    return !(*this == other);
  }

  [[nodiscard]] bool conflicts(const GeneralScheduledStop& other) const {
    // Same station name is also a conflict
    if (station == other.station) {
      return true;
    }

    // If there is a time where both have to stop, there is a conflict
    const auto& interval1 = get_forced_stopping_interval();
    const auto& interval2 = other.get_forced_stopping_interval();
    if (interval1.first > interval1.second ||
        interval2.first > interval2.second) {
      return false;
    }
    return interval1.first <= interval2.second &&
           interval2.first <= interval1.second;
  }

  [[nodiscard]] std::pair<int, int> get_forced_stopping_interval() const {
    std::pair<int, int> interval = {begin.second, end.first};
    if (begin.first + min_stopping_time > interval.second) {
      interval.second = begin.first + min_stopping_time;
    }
    if (end.second - min_stopping_time < interval.first) {
      interval.first = end.second - min_stopping_time;
    }
    return interval;
  }

  [[nodiscard]] int get_min_stopping_time() const { return min_stopping_time; }
  [[nodiscard]] const std::string& get_station_name() const { return station; }

  // Constructor
  GeneralScheduledStop(std::pair<int, int> begin, std::pair<int, int> end,
                       int min_stopping_time, std::string station)
      : begin(std::move(begin)), end(std::move(end)),
        min_stopping_time(min_stopping_time), station(std::move(station)) {}
};

template <typename T = GeneralScheduledStop> class GeneralSchedule {
  /**
   * General schedule object
   * @param t_0 start time of schedule in seconds
   * @param v_0 initial velocity in m/s
   * @param entry entry vertex index of the schedule
   * @param t_n end time of schedule in seconds
   * @param v_n target end velocity in m/s
   * @param exit exit vertex index of the schedule
   * @param stops vector of scheduled stops
   */
  static_assert(std::is_base_of_v<GeneralScheduledStop, T>,
                "T must be derived from GeneralScheduledStop");

protected:
  std::pair<int, int> t_0;
  double              v_0;
  size_t              entry;
  std::pair<int, int> t_n;
  double              v_n;
  size_t              exit;
  std::vector<T>      stops = {};

public:
  [[nodiscard]] const std::pair<int, int>& get_t_0() const { return t_0; }
  [[nodiscard]] double                     get_v_0() const { return v_0; }
  [[nodiscard]] size_t                     get_entry() const { return entry; }
  [[nodiscard]] const std::pair<int, int>& get_t_n() const { return t_n; }
  [[nodiscard]] double                     get_v_n() const { return v_n; }
  [[nodiscard]] size_t                     get_exit() const { return exit; }
  [[nodiscard]] const std::vector<T>&      get_stops() const { return stops; }

  void set_t_0(std::pair<int, int> t_0) { this->t_0 = std::move(t_0); }
  void set_v_0(double v_0) { this->v_0 = v_0; }
  void set_entry(size_t entry) { this->entry = entry; }
  void set_t_n(std::pair<int, int> t_n) { this->t_n = std::move(t_n); }
  void set_v_n(double v_n) { this->v_n = v_n; }
  void set_exit(size_t exit) { this->exit = exit; }
  void set_stops(std::vector<T> stops) { this->stops = std::move(stops); }

  template <typename... Args> void add_stop(bool sort, Args... args) {
    const T new_stop(args...);
    for (const auto& stop : stops) {
      if (stop.conflicts(new_stop)) {
        throw exceptions::ConsistencyException(
            "Stop conflicts with existing stop");
      }
    }

    stops.push_back(new_stop);

    if (sort) {
      sort_stops();
    }
  }

  void sort_stops() { std::sort(stops.begin(), stops.end()); }

  // Constructor
  GeneralSchedule()
      : t_0({-1, -1}), v_0(-1), entry(-1), t_n({-1, -1}), v_n(-1), exit(-1) {}
  GeneralSchedule(std::pair<int, int> t_0, double v_0, size_t entry,
                  std::pair<int, int> t_n, double v_n, size_t exit,
                  std::vector<T> stops = {})
      : t_0(std::move(t_0)), v_0(v_0), entry(entry), t_n(std::move(t_n)),
        v_n(v_n), exit(exit), stops(std::move(stops)) {}
};

} // namespace cda_rail
