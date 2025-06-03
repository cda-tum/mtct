#pragma once

#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "RailwayNetwork.hpp"
#include "Station.hpp"
#include "Train.hpp"
#include "nlohmann/json.hpp"
#include "nlohmann/json_fwd.hpp"

#include <algorithm>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

using json = nlohmann::json;

namespace cda_rail {
template <typename, typename = void> struct HasGetStops : std::false_type {};

template <typename T>
struct HasGetStops<T, std::void_t<decltype(std::declval<T>().get_stops())>>
    : std::true_type {};

class GeneralScheduledStop {
  /**
   * A (general) scheduled stop.
   */
  std::pair<int, int> begin;
  std::pair<int, int> end;
  int                 min_stopping_time;
  std::string         station;

public:
  [[nodiscard]] static std::pair<int, int> time_type() {
    // return the type of the desired time type
    return {};
  }

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

  [[nodiscard]] bool is_forced_to_stop(int time) const {
    const auto& [forced_lb, forced_ub] = get_forced_stopping_interval();
    return forced_lb <= time && time <= forced_ub;
  }

  [[nodiscard]] std::pair<int, int> get_forced_stopping_interval() const {
    std::pair<int, int> interval = {begin.second, end.first};
    interval.second =
        std::max(begin.first + min_stopping_time, interval.second);
    interval.first = std::min(end.second - min_stopping_time, interval.first);
    return interval;
  }

  [[nodiscard]] const std::pair<int, int>& get_begin_range() const {
    return begin;
  }
  [[nodiscard]] const std::pair<int, int>& get_end_range() const { return end; }
  [[nodiscard]] int get_min_stopping_time() const { return min_stopping_time; }
  [[nodiscard]] const std::string& get_station_name() const { return station; }

  // Constructor
  // NOLINTBEGIN(modernize-pass-by-value)
  // cpp-linter cycles here because std::move were trivial for std::pair and
  // should not be used in that context
  GeneralScheduledStop(std::pair<int, int> begin, std::pair<int, int> end,
                       int min_stopping_time, std::string station)
      : begin(begin), end(end), min_stopping_time(min_stopping_time),
        station(std::move(station)) {
    // NOLINTEND(modernize-pass-by-value)
    if (this->begin.second < this->begin.first) {
      throw exceptions::InvalidInputException(
          "Interval begin has negative length");
    }
    if (this->end.second < this->end.first) {
      throw exceptions::InvalidInputException(
          "Interval end has negative length");
    }
    if (this->min_stopping_time <= 0) {
      throw exceptions::InvalidInputException(
          "Minimum stopping time is non-positive");
    }
    if (this->begin.first < 0) {
      throw exceptions::InvalidInputException(
          "Interval begin has negative start time");
    }
    if (this->end.first < 0) {
      throw exceptions::InvalidInputException(
          "Interval end has negative start time");
    }
    if (this->end.second < this->begin.first) {
      throw exceptions::InvalidInputException(
          "Interval end starts before interval begin");
    }
    if (this->end.second - this->begin.first < this->min_stopping_time) {
      throw exceptions::InvalidInputException(
          "Maximal Interval is shorter than minimum stopping time");
    }
  }
};

class BaseGeneralSchedule {
  /**
   * This class is only used for static_assert in GeneralTimetable.
   * It should not be able to be inherited from outside of the specified friend
   * classes.
   */
  template <typename T> friend class GeneralSchedule;

private:
  BaseGeneralSchedule() = default;

public:
  virtual ~BaseGeneralSchedule()                               = default;
  [[nodiscard]] virtual bool is_forced_to_stop(int time) const = 0;
};

template <typename T = GeneralScheduledStop>
class GeneralSchedule : BaseGeneralSchedule {
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

  std::pair<int, int> t_0;
  double              v_0;
  size_t              entry;
  std::pair<int, int> t_n;
  double              v_n;
  size_t              exit;
  std::vector<T>      stops = {};

public:
  [[nodiscard]] static auto time_type() -> decltype(T::time_type()) {
    // return the type of the desired time type
    return T::time_type();
  }

  [[nodiscard]] const std::pair<int, int>& get_t_0_range() const { return t_0; }
  [[nodiscard]] double                     get_v_0() const { return v_0; }
  [[nodiscard]] size_t                     get_entry() const { return entry; }
  [[nodiscard]] const std::pair<int, int>& get_t_n_range() const { return t_n; }
  [[nodiscard]] double                     get_v_n() const { return v_n; }
  [[nodiscard]] size_t                     get_exit() const { return exit; }
  [[nodiscard]] const std::vector<T>&      get_stops() const { return stops; }

  [[nodiscard]] virtual bool is_forced_to_stop(int time) const override {
    for (const auto& stop : stops) {
      if (stop.is_forced_to_stop(time)) {
        return true;
      }
    }
    return false;
  }

  void set_t_0_range(std::pair<int, int> t_0) { this->t_0 = std::move(t_0); }
  void set_v_0(double v_0) { this->v_0 = v_0; }
  void set_entry(size_t entry) { this->entry = entry; }
  void set_t_n_range(std::pair<int, int> t_n) { this->t_n = std::move(t_n); }
  void set_v_n(double v_n) { this->v_n = v_n; }
  void set_exit(size_t exit) { this->exit = exit; }
  void set_stops(std::vector<T> stops) { this->stops = std::move(stops); }
  void remove_stop(const std::string& station_name) {
    stops.erase(std::remove_if(stops.begin(), stops.end(),
                               [&station_name](const auto& stop) {
                                 return stop.get_station_name() == station_name;
                               }),
                stops.end());
  }

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
      : BaseGeneralSchedule(), t_0({-1, -1}), v_0(-1), entry(-1), t_n({-1, -1}),
        v_n(-1), exit(-1) {}
  GeneralSchedule(std::pair<int, int> t_0, double v_0, size_t entry,
                  std::pair<int, int> t_n, double v_n, size_t exit,
                  std::vector<T> stops = {})
      : BaseGeneralSchedule(), t_0(std::move(t_0)), v_0(v_0), entry(entry),
        t_n(std::move(t_n)), v_n(v_n), exit(exit), stops(std::move(stops)) {}
};

class BaseTimetable {
  /**
   * This class is only used for static_assert in GeneralTimetable.
   * It should not be able to be inherited from outside of the specified friend
   * classes.
   */

  template <typename T> friend class GeneralTimetable;

private:
  BaseTimetable() = default;

public:
  virtual ~BaseTimetable() = default;

  virtual void               export_timetable(const std::filesystem::path& p,
                                              const Network&               network) const = 0;
  [[nodiscard]] virtual bool is_forced_to_stop(const std::string& tr_name,
                                               int time) const              = 0;
};

template <typename T = GeneralSchedule<GeneralScheduledStop>>
class GeneralTimetable : BaseTimetable {
  /**
   * General timetable class
   */
  static_assert(std::is_base_of_v<BaseGeneralSchedule, T>,
                "T must be derived from BaseGeneralSchedule");
  static_assert(HasGetStops<T>::value, "T must have a get_stops() method");
  using stop_type = typename std::remove_reference<
      decltype(std::declval<T>().get_stops())>::type::value_type;
  static_assert(std::is_base_of_v<GeneralScheduledStop, stop_type>,
                "std::vector<S> returned by get_stops must have S being "
                "derived from GeneralScheduledStop");
  static_assert(
      std::is_base_of_v<GeneralSchedule<stop_type>, T>,
      "T must be derived from GeneralSchedule with suitable stop type");

  template <typename U, std::enable_if_t<std::is_same_v<U, int>, int> = 0>
  void add_json_data(json& j, const int i, const Network& network) const {
    const auto& schedule = schedules.at(i);
    json        stops;

    for (const auto& stop : schedule.get_stops()) {
      stops.push_back(
          {{"begin", stop.arrival()},
           {"end", stop.departure()},
           {"station",
            station_list.get_station(stop.get_station_name()).name}});
    }
    j[train_list.get_train(i).name] = {
        {"t_0", schedule.get_t_0()},
        {"v_0", schedule.get_v_0()},
        {"entry", network.get_vertex(schedule.get_entry()).name},
        {"t_n", schedule.get_t_n()},
        {"v_n", schedule.get_v_n()},
        {"exit", network.get_vertex(schedule.get_exit()).name},
        {"stops", stops}};
  }

  template <typename U,
            std::enable_if_t<std::is_same_v<U, std::pair<int, int>>, int> = 0>
  void add_json_data(json& j, const int i, const Network& network) const {
    const auto& schedule = schedules.at(i);
    json        stops;

    for (const auto& stop : schedule.get_stops()) {
      stops.push_back(
          {{"begin", stop.get_begin_range()},
           {"end", stop.get_end_range()},
           {"min_stopping_time", stop.get_min_stopping_time()},
           {"station",
            station_list.get_station(stop.get_station_name()).name}});
    }
    j[train_list.get_train(i).name] = {
        {"t_0", schedule.get_t_0_range()},
        {"v_0", schedule.get_v_0()},
        {"entry", network.get_vertex(schedule.get_entry()).name},
        {"t_n", schedule.get_t_n_range()},
        {"v_n", schedule.get_v_n()},
        {"exit", network.get_vertex(schedule.get_exit()).name},
        {"stops", stops}};
  }

  template <typename U, std::enable_if_t<std::is_same_v<U, int>, int> = 0>
  void parse_schedule_data(const json& schedule_data, const int i) {
    this->schedules.at(i).set_t_0(static_cast<int>(schedule_data["t_0"]));
    this->schedules.at(i).set_t_n(static_cast<int>(schedule_data["t_n"]));
    for (const auto& stop_data : schedule_data["stops"]) {
      this->add_stop(i, stop_data["station"].get<std::string>(), false,
                     stop_data["begin"].get<int>(),
                     stop_data["end"].get<int>());
    }
  }
  template <typename U,
            std::enable_if_t<std::is_same_v<U, std::pair<int, int>>, int> = 0>
  void parse_schedule_data(const json& schedule_data, const int i) {
    this->schedules.at(i).set_t_0_range({schedule_data["t_0"][0].get<int>(),
                                         schedule_data["t_0"][1].get<int>()});
    this->schedules.at(i).set_t_n_range({schedule_data["t_n"][0].get<int>(),
                                         schedule_data["t_n"][1].get<int>()});
    for (const auto& stop_data : schedule_data["stops"]) {
      this->add_stop(
          i, stop_data["station"].get<std::string>(), false,
          {stop_data["begin"][0].get<int>(), stop_data["begin"][1].get<int>()},
          {stop_data["end"][0].get<int>(), stop_data["end"][1].get<int>()},
          stop_data["min_stopping_time"].get<int>());
    }
  }

protected:
  StationList    station_list;
  TrainList      train_list;
  std::vector<T> schedules = {};

  void set_train_list(const TrainList& tl) {
    this->train_list = tl;
    this->schedules  = std::vector<T>(tl.size());
  }

public:
  [[nodiscard]] static auto time_type() -> decltype(T::time_type()) {
    // return the type of the desired time type
    return T::time_type();
  }

  GeneralTimetable() = default;
  GeneralTimetable(const std::filesystem::path& p, const Network& network)
      : BaseTimetable() {
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
      if (!data.contains(tr.name)) {
        throw exceptions::ScheduleNotExistentException(tr.name);
      }

      const auto& schedule_data = data[tr.name];

      this->schedules.at(i).set_v_0(static_cast<double>(schedule_data["v_0"]));
      this->schedules.at(i).set_entry(
          network.get_vertex_index(schedule_data["entry"]));
      this->schedules.at(i).set_v_n(static_cast<double>(schedule_data["v_n"]));
      this->schedules.at(i).set_exit(
          network.get_vertex_index(schedule_data["exit"]));

      parse_schedule_data<decltype(T::time_type())>(schedule_data, i);
    }

    this->sort_stops();
  };
  GeneralTimetable(const std::string& path, const Network& network)
      : GeneralTimetable(std::filesystem::path(path), network) {};
  GeneralTimetable(const char* path, const Network& network)
      : GeneralTimetable(std::filesystem::path(path), network) {};
  GeneralTimetable(StationList station_list, TrainList train_list,
                   const std::vector<T>& schedules)
      : station_list(std::move(station_list)),
        train_list(std::move(train_list)), schedules(schedules) {}

  void export_timetable(const std::string& path, const Network& network) const {
    export_timetable(std::filesystem::path(path), network);
  };
  void export_timetable(const char* path, const Network& network) const {
    export_timetable(std::filesystem::path(path), network);
  };
  void export_timetable(const std::filesystem::path& p,
                        const Network&               network) const override {
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
      add_json_data<decltype(T::time_type())>(j, i, network);
    }

    std::ofstream file(p / "schedules.json");
    file << j << '\n';
  };

  Train& editable_tr(size_t index) { return train_list.editable_tr(index); };
  Train& editable_tr(const std::string& name) {
    return train_list.editable_tr(name);
  };

  void add_station(const std::string& name) { station_list.add_station(name); };

  template <typename StationType = std::string, typename... Args,
            typename             = std::enable_if_t<
                            std::is_convertible_v<StationType, std::string>>>
  void add_stop(size_t train_index, const StationType& station_name, bool sort,
                decltype(T::time_type()) t0, decltype(T::time_type()) tn,
                Args... args) {
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
      throw exceptions::TrainNotExistentException(train_index);
    }
    if (!station_list.has_station(static_cast<std::string>(station_name))) {
      // NOLINTNEXTLINE(google-readability-casting)
      throw exceptions::StationNotExistentException(
          static_cast<std::string>(station_name));
    }

    schedules.at(train_index)
        .add_stop(sort, t0, tn, args...,
                  static_cast<std::string>(station_name));
  }
  template <typename TrainType   = std::string,
            typename StationType = std::string, typename... Args,
            typename             = std::enable_if_t<
                            !std::is_convertible_v<TrainType, size_t> &&
                            std::is_convertible_v<TrainType, std::string> &&
                            std::is_convertible_v<StationType, std::string>>>
  void add_stop(const TrainType& train_name, const StationType& station_name,
                bool sort, decltype(T::time_type()) t0,
                decltype(T::time_type()) tn, Args... args) {
    add_stop(train_list.get_train_index(static_cast<std::string>(train_name)),
             static_cast<std::string>(station_name), sort, t0, tn, args...);
  };
  template <typename StationType = std::string, typename... Args,
            typename             = std::enable_if_t<
                            std::is_convertible_v<StationType, std::string>>>
  void add_stop(size_t train_index, const StationType& station_name,
                decltype(T::time_type()) t0, decltype(T::time_type()) tn,
                Args... args) {
    add_stop(train_index, station_name, true, t0, tn, args...);
  }
  template <typename TrainType   = std::string,
            typename StationType = std::string, typename... Args,
            typename             = std::enable_if_t<
                            !std::is_convertible_v<TrainType, size_t> &&
                            std::is_convertible_v<TrainType, std::string> &&
                            std::is_convertible_v<StationType, std::string>>>
  void add_stop(const TrainType& train_name, const StationType& station_name,
                decltype(T::time_type()) t0, decltype(T::time_type()) tn,
                Args... args) {
    add_stop(train_name, station_name, true, t0, tn, args...);
  };

  void remove_stop(size_t train_index, const std::string& station_name) {
    if (!train_list.has_train(train_index)) {
      throw exceptions::TrainNotExistentException(train_index);
    }
    if (!station_list.has_station(station_name)) {
      throw exceptions::StationNotExistentException(station_name);
    }
    schedules.at(train_index).remove_stop(station_name);
  }
  void remove_stop(const std::string& train_name,
                   const std::string& station_name) {
    remove_stop(train_list.get_train_index(train_name), station_name);
  }

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

  template <
      typename TrainN = std::string,
      typename = std::enable_if_t<!std::is_convertible_v<TrainN, size_t> &&
                                  std::is_convertible_v<TrainN, std::string>>>
  size_t add_train(const TrainN& name, int length, double max_speed,
                   double acceleration, double deceleration,
                   decltype(T::time_type()) t_0, double v_0, size_t entry,
                   decltype(T::time_type()) t_n, double v_n, size_t exit,
                   const Network& network) {
    return add_train(static_cast<std::string>(name), length, max_speed,
                     acceleration, deceleration, true, t_0, v_0, entry, t_n,
                     v_n, exit, network);
  };
  template <
      typename TrainN = std::string, typename EntryN = std::string,
      typename ExitN = std::string,
      typename = std::enable_if_t<!std::is_convertible_v<TrainN, size_t> &&
                                  std::is_convertible_v<TrainN, std::string> &&
                                  !std::is_convertible_v<EntryN, size_t> &&
                                  std::is_convertible_v<EntryN, std::string> &&
                                  !std::is_convertible_v<ExitN, size_t> &&
                                  std::is_convertible_v<ExitN, std::string>>>
  size_t add_train(const TrainN& name, int length, double max_speed,
                   double acceleration, double deceleration,
                   decltype(T::time_type()) t_0, double v_0,
                   const EntryN& entry, decltype(T::time_type()) t_n,
                   double v_n, const ExitN& exit, const Network& network) {
    return add_train(
        static_cast<std::string>(name), length, max_speed, acceleration,
        deceleration, true, t_0, v_0,
        network.get_vertex_index(static_cast<std::string>(entry)), t_n, v_n,
        network.get_vertex_index(static_cast<std::string>(exit)), network);
  };
  template <
      typename TrainN = std::string,
      typename = std::enable_if_t<!std::is_convertible_v<TrainN, size_t> &&
                                  std::is_convertible_v<TrainN, std::string>>>
  size_t add_train(const TrainN& name, int length, double max_speed,
                   double acceleration, double deceleration, bool tim,
                   decltype(T::time_type()) t_0, double v_0, size_t entry,
                   decltype(T::time_type()) t_n, double v_n, size_t exit,
                   const Network& network) {
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
      throw exceptions::VertexNotExistentException(entry);
    }
    if (!network.has_vertex(exit)) {
      throw exceptions::VertexNotExistentException(exit);
    }
    if (train_list.has_train(static_cast<std::string>(name))) {
      throw exceptions::ConsistencyException("Train already exists.");
    }
    auto const index =
        train_list.add_train(static_cast<std::string>(name), length, max_speed,
                             acceleration, deceleration, tim);
    schedules.emplace_back(t_0, v_0, entry, t_n, v_n, exit);
    return index;
  }
  template <
      typename TrainN = std::string, typename EntryN = std::string,
      typename ExitN = std::string,
      typename = std::enable_if_t<!std::is_convertible_v<TrainN, size_t> &&
                                  std::is_convertible_v<TrainN, std::string> &&
                                  !std::is_convertible_v<EntryN, size_t> &&
                                  std::is_convertible_v<EntryN, std::string> &&
                                  !std::is_convertible_v<ExitN, size_t> &&
                                  std::is_convertible_v<ExitN, std::string>>>
  size_t add_train(const TrainN& name, int length, double max_speed,
                   double acceleration, double deceleration, bool tim,
                   decltype(T::time_type()) t_0, double v_0,
                   const EntryN& entry, decltype(T::time_type()) t_n,
                   double v_n, const ExitN& exit, const Network& network) {
    return add_train(
        static_cast<std::string>(name), length, max_speed, acceleration,
        deceleration, tim, t_0, v_0,
        network.get_vertex_index(static_cast<std::string>(entry)), t_n, v_n,
        network.get_vertex_index(static_cast<std::string>(exit)), network);
  };

  [[nodiscard]] const StationList& get_station_list() const {
    return station_list;
  };
  [[nodiscard]] const TrainList& get_train_list() const { return train_list; };
  [[nodiscard]] const T&         get_schedule(size_t index) const {
    /**
     * This method returns the schedule of a train with given index.
     *
     * @param index The index of the train in the train list.
     *
     * @return The schedule of the train.
     */
    if (!train_list.has_train(index)) {
      throw exceptions::TrainNotExistentException(index);
    }
    return schedules.at(index);
  }
  [[nodiscard]] const T& get_schedule(const std::string& train_name) const {
    return get_schedule(train_list.get_train_index(train_name));
  };

  [[nodiscard]] bool is_forced_to_stop(const std::string& train_name,
                                       int                time) const override {
    return get_schedule(train_name).is_forced_to_stop(time);
  }

  [[nodiscard]] int max_t() const {
    /**
     * This method returns the maximum time of all trains, i.e., the time at
     * which the last train leaves the network.
     *
     * @return The maximum time of all trains.
     */
    int ret = 0;
    for (const auto& schedule : schedules) {
      if (schedule.get_t_n_range().second > ret) {
        ret = schedule.get_t_n_range().second;
      }
    }
    return ret;
  }

  [[nodiscard]] std::pair<int, int> time_interval(size_t train_index) const {
    /**
     * This method returns the time interval of a train schedule, i.e., the time
     * at which it enters the network and the time at which it leaves the
     * network.
     *
     * @param train_index The index of the train in the train list.
     * @return A pair of integers (t_0, t_n) where t_0 is the time at which the
     * train enters the network and t_n is the time at which the train leaves
     * the network.
     */

    if (!train_list.has_train(train_index)) {
      throw exceptions::TrainNotExistentException(train_index);
    }

    const auto& schedule = schedules.at(train_index);
    return {schedule.get_t_0_range().first, schedule.get_t_n_range().second};
  };

  template <
      typename TrainN = std::string,
      typename = std::enable_if_t<!std::is_convertible_v<TrainN, size_t> &&
                                  std::is_convertible_v<TrainN, std::string>>>
  [[nodiscard]] std::pair<int, int>
  time_interval(const TrainN& train_name) const {
    return time_interval(
        train_list.get_train_index(static_cast<std::string>(train_name)));
  };

  void sort_stops() {
    /**
     * This methods sorts all stops of all trains according to the operator < of
     * ScheduledStop.
     */

    for (auto& schedule : schedules) {
      schedule.sort_stops();
    }
  }

  void update_after_discretization(
      const std::vector<std::pair<size_t, std::vector<size_t>>>& new_edges) {
    station_list.update_after_discretization(new_edges);
  };

  [[nodiscard]] virtual bool check_consistency(const Network& network) const {
    /**
     * This method checks if the timetable is consistent with the network, i.e.,
     * if the following holds:
     * - All vertices used as entry and exit points are valid vertices of the
     * network
     * - Entry and exit vertices have exactly one neighboring vertex
     * - All edges of stations are valid edges of the network
     * - No two scheduled stops overlap
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
        if (stop.get_begin_range().first < schedule.get_t_0_range().first ||
            stop.get_end_range().second > schedule.get_t_n_range().second ||
            stop.get_end_range().second < stop.get_begin_range().first) {
          return false;
        }
      }
    }

    for (const auto& schedule : schedules) {
      for (size_t i = 0; i < schedule.get_stops().size(); ++i) {
        for (size_t j = i + 1; j < schedule.get_stops().size(); ++j) {
          if (schedule.get_stops().at(i).conflicts(
                  schedule.get_stops().at(j))) {
            return false;
          }
        }
      }
    }

    return true;
  }
};

} // namespace cda_rail
