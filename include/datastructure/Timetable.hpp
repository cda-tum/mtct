#pragma once
#include "datastructure/GeneralTimetable.hpp"
#include "datastructure/RailwayNetwork.hpp"
#include "datastructure/Station.hpp"
#include "datastructure/Train.hpp"

#include <cstddef>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace cda_rail {
class ScheduledStop : public GeneralScheduledStop {
  /**
   * A scheduled stop with fixed times
   */

public:
  [[nodiscard]] static int time_type() {
    // return the type of the desired time type
    return int();
  }

  [[nodiscard]] int arrival() const {
    return GeneralScheduledStop::get_begin_range().first;
  }
  [[nodiscard]] int departure() const {
    return GeneralScheduledStop::get_end_range().first;
  }

  // Constructor
  ScheduledStop(int begin, int end, std::string station)
      : GeneralScheduledStop({begin, begin}, {end, end}, end - begin,
                             std::move(station)) {}
};

class Schedule : public GeneralSchedule<ScheduledStop> {
  /**
   * Specific Schedule object
   */

public:
  [[nodiscard]] int get_t_0() const { return get_t_0_range().first; }
  [[nodiscard]] int get_t_n() const { return get_t_n_range().first; }

  void set_t_0(int t_0) { set_t_0_range({t_0, t_0}); }
  void set_t_n(int t_n) { set_t_n_range({t_n, t_n}); }

  [[nodiscard]] GeneralSchedule<GeneralScheduledStop>
  parse_to_general_schedule() const {
    const auto&                       stops = this->get_stops();
    std::vector<GeneralScheduledStop> general_stops;
    general_stops.reserve(stops.size());
    for (const auto& stop : stops) {
      general_stops.push_back(stop);
    }
    return {get_t_0_range(), get_v_0(),  get_entry(),  get_t_n_range(),
            get_v_n(),       get_exit(), general_stops};
  }

  template <typename S, typename = std::enable_if_t<
                            std::is_base_of_v<GeneralScheduledStop, S>>>
  [[nodiscard]] static Schedule
  cast_from_general_schedule(const GeneralSchedule<S>& general_schedule_obj,
                             bool                      throw_error = true) {
    const std::pair<int, int>& t_0_range = general_schedule_obj.get_t_0_range();
    const std::pair<int, int>& t_n_range = general_schedule_obj.get_t_n_range();
    if (throw_error && t_0_range.first != t_0_range.second) {
      throw std::invalid_argument("Schedule must have fixed initial time");
    }
    if (throw_error && t_n_range.first != t_n_range.second) {
      throw std::invalid_argument("Schedule must have fixed final time");
    }

    const std::vector<S>&      stops = general_schedule_obj.get_stops();
    std::vector<ScheduledStop> scheduled_stops;
    scheduled_stops.reserve(stops.size());
    for (const auto& stop : stops) {
      const std::pair<int, int>& b_range = stop.get_begin_range();
      const std::pair<int, int>& e_range = stop.get_end_range();
      const std::string&         s_name  = stop.get_station_name();
      if (throw_error && b_range.first != b_range.second) {
        throw std::invalid_argument("Scheduled stop must have fixed arrival");
      }
      if (throw_error && e_range.first != e_range.second) {
        throw std::invalid_argument("Scheduled stop must have fixed departure");
      }
      scheduled_stops.emplace_back(b_range.first, e_range.first, s_name);
    }

    return {t_0_range.first,
            general_schedule_obj.get_v_0(),
            general_schedule_obj.get_entry(),
            t_n_range.first,
            general_schedule_obj.get_v_n(),
            general_schedule_obj.get_exit(),
            scheduled_stops};
  }

  // Constructor
  // NOLINTNEXTLINE(readability-redundant-member-init)
  Schedule() : GeneralSchedule() {}
  Schedule(int t_0, double v_0, size_t entry, int t_n, double v_n, size_t exit,
           std::vector<ScheduledStop> stops = {})
      : GeneralSchedule({t_0, t_0}, v_0, entry, {t_n, t_n}, v_n, exit,
                        std::move(stops)) {}
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
