#pragma once

#include "Definitions.hpp"
#include "datastructure/GeneralTimetable.hpp"
#include "datastructure/RailwayNetwork.hpp"
#include "datastructure/Route.hpp"

#include <filesystem>
#include <string>
#include <type_traits>

namespace cda_rail::instances {

template <typename, typename = void> struct HasTimeType : std::false_type {};
template <typename T>
struct HasTimeType<T, std::void_t<decltype(std::declval<T>().time_type())>>
    : std::true_type {};

template <typename, typename = void> struct HasEditableTr : std::false_type {};
template <typename T>
struct HasEditableTr<T, std::void_t<decltype(std::declval<T>().editable_tr())>>
    : std::true_type {};

template <typename, typename = void> struct HasAddTrain : std::false_type {};
template <typename T>
struct HasAddTrain<T, std::void_t<decltype(std::declval<T>().add_train())>>
    : std::true_type {};

template <typename, typename = void> struct HasAddStation : std::false_type {};
template <typename T>
struct HasAddStation<T, std::void_t<decltype(std::declval<T>().add_station())>>
    : std::true_type {};

template <typename, typename = void> struct HasAddStop : std::false_type {};
template <typename T>
struct HasAddStop<T, std::void_t<decltype(std::declval<T>().add_stop())>>
    : std::true_type {};

template <typename, typename = void>
struct HasGetTrainList : std::false_type {};
template <typename T>
struct HasGetTrainList<
    T, std::void_t<decltype(std::declval<T>().get_train_list())>>
    : std::true_type {};

template <typename, typename = void>
struct HasGetStationList : std::false_type {};
template <typename T>
struct HasGetStationList<
    T, std::void_t<decltype(std::declval<T>().get_station_list())>>
    : std::true_type {};

template <typename, typename = void> struct HasGetSchedule : std::false_type {};
template <typename T>
struct HasGetSchedule<T,
                      std::void_t<decltype(std::declval<T>().get_schedule())>>
    : std::true_type {};

template <typename, typename = void> struct HasMaxT : std::false_type {};
template <typename T>
struct HasMaxT<T, std::void_t<decltype(std::declval<T>().max_t())>>
    : std::true_type {};

template <typename, typename = void>
struct HasTrainInterval : std::false_type {};
template <typename T>
struct HasTrainInterval<
    T, std::void_t<decltype(std::declval<T>().get_train_interval())>>
    : std::true_type {};

template <typename, typename = void> struct HasSortStops : std::false_type {};
template <typename T>
struct HasSortStops<T, std::void_t<decltype(std::declval<T>().sort_stops())>>
    : std::true_type {};

class GeneralProblemInstance {
  Network network;

protected:
  GeneralProblemInstance() = default;
  explicit GeneralProblemInstance(Network network)
      : network(std::move(network)){};
  explicit GeneralProblemInstance(const std::filesystem::path& path)
      : network(Network(path / "Network")){};

  void export_network(const std::filesystem::path& path) const {
    if (!is_directory_and_create(path)) {
      throw std::invalid_argument("Path is not a directory");
    }
    network.export_network(path / "Network");
  }

public:
  // Network functions, i.e., network is accessible via n() as a reference
  [[nodiscard]] Network&       n() { return network; };
  [[nodiscard]] const Network& const_n() const { return network; };

  virtual void export_instance(const std::filesystem::path& path) const = 0;

  virtual void export_instance(const std::string& path) const {
    export_instance(std::filesystem::path(path));
  };
  virtual void export_instance(const char* path) const {
    export_instance(std::filesystem::path(path));
  };

  [[nodiscard]] virtual bool check_consistency() const = 0;

  virtual ~GeneralProblemInstance() = default;
};

template <typename T>
class GeneralProblemInstanceWithScheduleAndRoutes
    : public GeneralProblemInstance {
  static_assert(std::is_base_of_v<BaseTimetable, T>,
                "T must be a child of BaseTimeTable");
  static_assert(HasTimeType<T>::value, "T must have a time_type() method");
  using TimeType = decltype(T::time_type());
  // static_assert(HasEditableTr<T>::value, "T must have an editable_tr()
  // method"); static_assert(HasAddTrain<T>::value, "T must have an add_train()
  // method"); static_assert(HasAddStation<T>::value, "T must have an
  // add_station() method"); static_assert(HasAddStop<T>::value, "T must have an
  // add_stop() method"); static_assert(HasGetTrainList<T>::value, "T must have
  // a get_train_list() method"); static_assert(HasGetStationList<T>::value, "T
  // must have a get_station_list() method");
  // static_assert(HasGetSchedule<T>::value, "T must have a get_schedule()
  // method"); static_assert(HasMaxT<T>::value, "T must have a max_t() method");
  // static_assert(HasTrainInterval<T>::value, "T must have a time_interval()
  // method"); static_assert(HasSortStops<T>::value, "T must have a sort_stops()
  // method");

  template <typename S>
  friend class SolGeneralProblemInstanceWithScheduleAndRoutes;

  T        timetable;
  RouteMap routes;

protected:
  GeneralProblemInstanceWithScheduleAndRoutes() = default;
  explicit GeneralProblemInstanceWithScheduleAndRoutes(Network  network,
                                                       T        timetable,
                                                       RouteMap routes)
      : GeneralProblemInstance(std::move(network)),
        timetable(std::move(timetable)), routes(std::move(routes)){};
  explicit GeneralProblemInstanceWithScheduleAndRoutes(
      const std::filesystem::path& path,
      bool                         every_train_must_have_route = true)
      : GeneralProblemInstance(path),
        timetable(T(path / "timetable", this->const_n())),
        routes(RouteMap(path / "routes", this->const_n())) {
    if (!this->check_consistency(every_train_must_have_route)) {
      throw exceptions::ConsistencyException();
    }
  };

  [[nodiscard]] T&              editable_timetable() { return timetable; };
  [[nodiscard]] RouteMap&       editable_routes() { return routes; };
  [[nodiscard]] const T&        const_timetable() const { return timetable; };
  [[nodiscard]] const RouteMap& const_routes() const { return routes; };

public:
  [[nodiscard]] const auto& get_timetable() const { return timetable; };

  Train& editable_tr(size_t index) { return timetable.editable_tr(index); };
  Train& editable_tr(const std::string& name) {
    return timetable.editable_tr(name);
  };

  template <typename... Args> size_t add_train(Args... args) {
    return timetable.add_train(args..., this->const_n());
  }

  void add_station(const std::string& name) { timetable.add_station(name); };

  void add_track_to_station(const std::string& name, size_t track) {
    timetable.add_track_to_station(name, track, this->const_n());
  };
  void add_track_to_station(const std::string& name, size_t source,
                            size_t target) {
    timetable.add_track_to_station(name, source, target, this->const_n());
  };
  void add_track_to_station(const std::string& name, const std::string& source,
                            const std::string& target) {
    timetable.add_track_to_station(name, source, target, this->const_n());
  };

  template <typename... Args> void add_stop(Args... args) {
    timetable.add_stop(args...);
  }

  void sort_stops() { timetable.sort_stops(); };

  [[nodiscard]] const StationList& get_station_list() const {
    return timetable.get_station_list();
  };
  [[nodiscard]] const TrainList& get_train_list() const {
    return timetable.get_train_list();
  };
  [[nodiscard]] const auto& get_schedule(size_t index) const {
    return timetable.get_schedule(index);
  };
  [[nodiscard]] const auto& get_schedule(const std::string& train_name) const {
    return timetable.get_schedule(train_name);
  };

  [[nodiscard]] int                 max_t() const { return timetable.max_t(); };
  [[nodiscard]] std::pair<int, int> time_interval(size_t train_index) const {
    return timetable.time_interval(train_index);
  };
  [[nodiscard]] std::pair<int, int>
  time_interval(const std::string& train_name) const {
    return timetable.time_interval(train_name);
  };

  // RouteMap functions
  void add_empty_route(const std::string& train_name) {
    routes.add_empty_route(train_name, get_train_list());
  };

  void push_back_edge_to_route(const std::string& train_name,
                               size_t             edge_index) {
    routes.push_back_edge(train_name, edge_index, this->const_n());
  };
  void push_back_edge_to_route(const std::string& train_name, size_t source,
                               size_t target) {
    routes.push_back_edge(train_name, source, target, this->const_n());
  };
  void push_back_edge_to_route(const std::string& train_name,
                               const std::string& source,
                               const std::string& target) {
    routes.push_back_edge(train_name, source, target, this->const_n());
  };

  void push_front_edge_to_route(const std::string& train_name,
                                size_t             edge_index) {
    routes.push_front_edge(train_name, edge_index, this->const_n());
  };
  void push_front_edge_to_route(const std::string& train_name, size_t source,
                                size_t target) {
    routes.push_front_edge(train_name, source, target, this->const_n());
  };
  void push_front_edge_to_route(const std::string& train_name,
                                const std::string& source,
                                const std::string& target) {
    routes.push_front_edge(train_name, source, target, this->const_n());
  };

  void remove_first_edge_from_route(const std::string& train_name) {
    routes.remove_first_edge(train_name);
  };
  void remove_last_edge_from_route(const std::string& train_name) {
    routes.remove_last_edge(train_name);
  };

  [[nodiscard]] bool has_route(const std::string& train_name) const {
    return routes.has_route(train_name);
  };
  [[nodiscard]] size_t       route_map_size() const { return routes.size(); };
  [[nodiscard]] const Route& get_route(const std::string& train_name) const {
    return routes.get_route(train_name);
  };

  [[nodiscard]] double route_length(const std::string& train_name) const {
    return routes.length(train_name, this->const_n());
  };
  [[nodiscard]] std::pair<double, double>
  route_edge_pos(const std::string& train_name, size_t edge) const {
    return routes.edge_pos(train_name, edge, this->const_n());
  };
  [[nodiscard]] std::pair<double, double>
  route_edge_pos(const std::string& train_name, size_t source,
                 size_t target) const {
    return routes.edge_pos(train_name, source, target, this->const_n());
  };
  [[nodiscard]] std::pair<double, double>
  route_edge_pos(const std::string& train_name, const std::string& source,
                 const std::string& target) const {
    return routes.edge_pos(train_name, source, target, this->const_n());
  };
  [[nodiscard]] std::pair<double, double>
  route_edge_pos(const std::string&         train_name,
                 const std::vector<size_t>& edges) const {
    return routes.edge_pos(train_name, edges, this->const_n());
  };

  [[nodiscard]] bool check_consistency() const override {
    return check_consistency(true);
  };

  void export_instance(const std::filesystem::path& path) const override {
    if (!is_directory_and_create(path)) {
      throw std::invalid_argument("Path is not a directory");
    }
    timetable.export_timetable(path / "timetable", this->const_n());
    routes.export_routes(path / "routes", this->const_n());
    export_network(path);
  };

  [[nodiscard]] virtual bool
  check_consistency(bool every_train_must_have_route) const {
    if (!timetable.check_consistency(this->const_n())) {
      return false;
    }
    return (routes.check_consistency(get_train_list(), this->const_n(),
                                     every_train_must_have_route));
  };
};

template <typename T> class SolGeneralProblemInstance {
  static_assert(std::is_base_of<GeneralProblemInstance, T>::value,
                "T must be a child of GeneralProblemInstance");

protected:
  T              instance;
  SolutionStatus status  = SolutionStatus::Unknown;
  double         obj     = -1;
  bool           has_sol = false;

  SolGeneralProblemInstance() = default;
  explicit SolGeneralProblemInstance(T instance)
      : instance(std::move(instance)){};
  SolGeneralProblemInstance(T instance, SolutionStatus status, double obj,
                            bool has_sol)
      : instance(std::move(instance)), status(status), obj(obj),
        has_sol(has_sol){};

public:
  [[nodiscard]] const T&       get_instance() const { return instance; };
  [[nodiscard]] SolutionStatus get_status() const { return status; };
  [[nodiscard]] double         get_obj() const { return obj; };
  [[nodiscard]] bool           has_solution() const { return has_sol; };
  void set_status(SolutionStatus new_status) { status = new_status; };
  void set_obj(double new_obj) { obj = new_obj; };
  void set_solution_found() { has_sol = true; };
  void set_solution_not_found() { has_sol = false; };

  virtual void export_solution(const std::filesystem::path& p,
                               bool export_instance) const = 0;

  void export_solution(const std::filesystem::path& p) const {
    export_solution(p, true);
  };

  void export_solution(const std::string& path,
                       bool               export_instance = true) const {
    export_solution(std::filesystem::path(path), export_instance);
  };
  void export_solution(const char* path, bool export_instance = true) const {
    export_solution(std::filesystem::path(path), export_instance);
  };

  [[nodiscard]] virtual bool check_consistency() const = 0;

  virtual ~SolGeneralProblemInstance() = default;
};

template <typename T>
class SolGeneralProblemInstanceWithScheduleAndRoutes
    : public SolGeneralProblemInstance<T> {
  static_assert(std::is_base_of<GeneralProblemInstance, T>::value,
                "T must be a child of GeneralProblemInstance");

public:
  SolGeneralProblemInstanceWithScheduleAndRoutes() = default;
  explicit SolGeneralProblemInstanceWithScheduleAndRoutes(T instance)
      : SolGeneralProblemInstance<T>(std::move(instance)){};
  SolGeneralProblemInstanceWithScheduleAndRoutes(T              instance,
                                                 SolutionStatus status,
                                                 double obj, bool has_sol)
      : SolGeneralProblemInstance<T>(std::move(instance), status, obj,
                                     has_sol){};

  // RouteMap functions
  void reset_routes() {
    for (const auto& tr : this->instance.get_train_list()) {
      if (this->instance.has_route(tr.name)) {
        this->instance.routes.remove_route(tr.name);
      }
    }
  }
  void add_empty_route(const std::string& train_name) {
    this->instance.add_empty_route(train_name);
  };

  void push_back_edge_to_route(const std::string& train_name,
                               size_t             edge_index) {
    this->instance.push_back_edge_to_route(train_name, edge_index);
  };
  void push_back_edge_to_route(const std::string& train_name, size_t source,
                               size_t target) {
    this->instance.push_back_edge_to_route(train_name, source, target);
  };
  void push_back_edge_to_route(const std::string& train_name,
                               const std::string& source,
                               const std::string& target) {
    this->instance.push_back_edge_to_route(train_name, source, target);
  };

  void push_front_edge_to_route(const std::string& train_name,
                                size_t             edge_index) {
    this->instance.push_front_edge_to_route(train_name, edge_index);
  };
  void push_front_edge_to_route(const std::string& train_name, size_t source,
                                size_t target) {
    this->instance.push_front_edge_to_route(train_name, source, target);
  };
  void push_front_edge_to_route(const std::string& train_name,
                                const std::string& source,
                                const std::string& target) {
    this->instance.push_front_edge_to_route(train_name, source, target);
  };

  void remove_first_edge_from_route(const std::string& train_name) {
    this->instance.remove_first_edge_from_route(train_name);
  };
  void remove_last_edge_from_route(const std::string& train_name) {
    this->instance.remove_last_edge_from_route(train_name);
  };
};
} // namespace cda_rail::instances
