#pragma once
#include "datastructure/RailwayNetwork.hpp"
#include "datastructure/Station.hpp"
#include "datastructure/Train.hpp"

#include <filesystem>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace cda_rail {
struct ScheduledStop {
  /**
   * A scheduled stop.
   */
  int         begin;
  int         end;
  std::string station;

  bool operator<(const ScheduledStop& other) const {
    return (end < other.begin);
  }
  bool operator>(const ScheduledStop& other) const {
    return (begin > other.end);
  }
  bool operator==(const ScheduledStop& other) const {
    return (begin == other.begin && end == other.end);
  }
  bool operator<=(const ScheduledStop& other) const {
    return *this < other || *this == other;
  }
  bool operator>=(const ScheduledStop& other) const {
    return *this > other || *this == other;
  }
  bool operator!=(const ScheduledStop& other) const {
    return !(*this == other);
  }

  // Constructor
  ScheduledStop(int begin, int end, std::string station)
      : begin(begin), end(end), station(std::move(station)) {}
};

struct Schedule {
  /**
   * Schedule object
   * @param t_0 start time of schedule in seconds
   * @param v_0 initial velocity in m/s
   * @param entry entry vertex index of the schedule
   * @param t_n end time of schedule in seconds
   * @param v_n target end velocity in m/s
   * @param exit exit vertex index of the schedule
   * @param stops vector of scheduled stops
   *
   * For stops in stations he has to occupy the station for the entire interval.
   */
  int                        t_0;
  double                     v_0;
  size_t                     entry;
  int                        t_n;
  double                     v_n;
  size_t                     exit;
  std::vector<ScheduledStop> stops = {};

  // Constructor
  Schedule() : t_0(-1), v_0(-1), entry(-1), t_n(-1), v_n(-1), exit(-1) {}
  Schedule(int t_0, double v_0, size_t entry, int t_n, double v_n, size_t exit,
           std::vector<ScheduledStop> stops = {})
      : t_0(t_0), v_0(v_0), entry(entry), t_n(t_n), v_n(v_n), exit(exit),
        stops(std::move(stops)) {}
};

class Timetable {
  /**
   * Timetable class
   */
private:
  StationList           station_list;
  TrainList             train_list;
  std::vector<Schedule> schedules;

  void set_train_list(const TrainList& tl);

public:
  // Constructors
  Timetable() = default;
  Timetable(const std::filesystem::path& p, const Network& network);
  Timetable(const std::string& path, const Network& network)
      : Timetable(std::filesystem::path(path), network){};
  Timetable(const char* path, const Network& network)
      : Timetable(std::filesystem::path(path), network){};

  // Rule of 5
  Timetable(const Timetable& other)                = default;
  Timetable(Timetable&& other) noexcept            = default;
  Timetable& operator=(const Timetable& other)     = default;
  Timetable& operator=(Timetable&& other) noexcept = default;
  ~Timetable()                                     = default;

  size_t add_train(const std::string& name, int length, double max_speed,
                   double acceleration, double deceleration, int t_0,
                   double v_0, size_t entry, int t_n, double v_n, size_t exit,
                   const Network& network) {
    return add_train(name, length, max_speed, acceleration, deceleration, true,
                     t_0, v_0, entry, t_n, v_n, exit, network);
  };
  size_t add_train(const std::string& name, int length, double max_speed,
                   double acceleration, double deceleration, int t_0,
                   double v_0, const std::string& entry, int t_n, double v_n,
                   const std::string& exit, const Network& network) {
    return add_train(name, length, max_speed, acceleration, deceleration, true,
                     t_0, v_0, entry, t_n, v_n, exit, network);
  };
  size_t add_train(const std::string& name, int length, double max_speed,
                   double acceleration, double deceleration, bool tim, int t_0,
                   double v_0, size_t entry, int t_n, double v_n, size_t exit,
                   const Network& network);
  size_t add_train(const std::string& name, int length, double max_speed,
                   double acceleration, double deceleration, bool tim, int t_0,
                   double v_0, const std::string& entry, int t_n, double v_n,
                   const std::string& exit, const Network& network) {
    return add_train(name, length, max_speed, acceleration, deceleration, tim,
                     t_0, v_0, network.get_vertex_index(entry), t_n, v_n,
                     network.get_vertex_index(exit), network);
  };

  Train& editable_tr(size_t index) { return train_list.editable_tr(index); };
  Train& editable_tr(const std::string& name) {
    return train_list.editable_tr(name);
  };

  void add_station(const std::string& name) { station_list.add_station(name); };

  void add_track_to_station(const std::string& name, size_t track,
                            const Network& network) {
    station_list.add_track_to_station(name, track, network);
  };
  void add_track_to_station(const std::string& name, size_t source,
                            size_t target, const Network& network) {
    station_list.add_track_to_station(name, source, target, network);
  };
  void add_track_to_station(const std::string& name, const std::string& source,
                            const std::string& target, const Network& network) {
    station_list.add_track_to_station(name, source, target, network);
  };

  void add_stop(size_t train_index, const std::string& station_name, int begin,
                int end, bool sort = true);
  void add_stop(const std::string& train_name, const std::string& station_name,
                int begin, int end, bool sort = true) {
    add_stop(train_list.get_train_index(train_name), station_name, begin, end,
             sort);
  };

  [[nodiscard]] const StationList& get_station_list() const {
    return station_list;
  };
  [[nodiscard]] const TrainList& get_train_list() const { return train_list; };
  [[nodiscard]] const Schedule&  get_schedule(size_t index) const;
  [[nodiscard]] const Schedule&
  get_schedule(const std::string& train_name) const {
    return get_schedule(train_list.get_train_index(train_name));
  };

  [[nodiscard]] int                 max_t() const;
  [[nodiscard]] std::pair<int, int> time_interval(size_t train_index) const;
  [[nodiscard]] std::pair<int, int>
  time_interval(const std::string& train_name) const {
    return time_interval(train_list.get_train_index(train_name));
  };
  [[nodiscard]] std::pair<size_t, size_t>
  time_index_interval(size_t train_index, int dt,
                      bool tn_inclusive = true) const;
  [[nodiscard]] std::pair<size_t, size_t>
  time_index_interval(const std::string& train_name, int dt,
                      bool tn_inclusive = true) const {
    return time_index_interval(train_list.get_train_index(train_name), dt,
                               tn_inclusive);
  };

  void sort_stops();

  [[nodiscard]] bool check_consistency(const Network& network) const;

  void export_timetable(const std::string& path, const Network& network) const {
    export_timetable(std::filesystem::path(path), network);
  };
  void export_timetable(const char* path, const Network& network) const {
    export_timetable(std::filesystem::path(path), network);
  };
  void export_timetable(const std::filesystem::path& p,
                        const Network&               network) const;
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

  void update_after_discretization(
      const std::vector<std::pair<size_t, std::vector<size_t>>>& new_edges) {
    station_list.update_after_discretization(new_edges);
  };
};
} // namespace cda_rail
