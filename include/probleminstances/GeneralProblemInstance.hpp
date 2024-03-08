#pragma once

#include "Definitions.hpp"
#include "datastructure/GeneralTimetable.hpp"
#include "datastructure/RailwayNetwork.hpp"
#include "datastructure/Route.hpp"
#include "nlohmann/json.hpp"

#include <filesystem>
#include <string>
#include <type_traits>

using json = nlohmann::json;

namespace cda_rail::instances {

template <typename, typename = void> struct HasTimeType : std::false_type {};
template <typename T>
struct HasTimeType<T, std::void_t<decltype(std::declval<T>().time_type())>>
    : std::true_type {};

class GeneralProblemInstance {
  Network network;

protected:
  GeneralProblemInstance() = default;
  explicit GeneralProblemInstance(Network network)
      : network(std::move(network)){};
  explicit GeneralProblemInstance(const std::filesystem::path& path)
      : network(Network(path / "network")){};

  void export_network(const std::filesystem::path& path) const {
    if (!is_directory_and_create(path)) {
      throw std::invalid_argument("Path is not a directory");
    }
    network.export_network(path / "network");
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

  template <typename S>
  friend class SolGeneralProblemInstanceWithScheduleAndRoutes;

  T        timetable;
  RouteMap routes;

protected:
  GeneralProblemInstanceWithScheduleAndRoutes() = default;
  explicit GeneralProblemInstanceWithScheduleAndRoutes(
      const Network& network, const T& timetable,
      RouteMap routes) // according to linter, large objects by const reference,
                       // others using std::move
      : GeneralProblemInstance(network), timetable(timetable),
        routes(std::move(routes)){};
  explicit GeneralProblemInstanceWithScheduleAndRoutes(
      const std::filesystem::path& path)
      : GeneralProblemInstance(path),
        timetable(T(path / "timetable", this->const_n())),
        routes(RouteMap(path / "routes", this->const_n())){};

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

  template <typename TrainN = std::string, typename EntryN = std::string,
            typename ExitN = std::string>
  size_t add_train(const TrainN& name, int length, double max_speed,
                   double acceleration, double deceleration,
                   decltype(T::time_type()) t_0, double v_0,
                   const EntryN& entry, decltype(T::time_type()) t_n,
                   double v_n, const ExitN& exit) {
    return timetable.add_train(name, length, max_speed, acceleration,
                               deceleration, t_0, v_0, entry, t_n, v_n, exit,
                               this->const_n());
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

  [[nodiscard]] bool has_route_for_every_train() const {
    /**
     * Checks if every train has a route.
     *
     * @return true if every train has a route, false otherwise
     */

    return std::all_of(get_train_list().begin(), get_train_list().end(),
                       [this](const auto& tr) {
                         return has_route(tr.name) &&
                                !get_route(tr.name).empty();
                       });
  };
  [[nodiscard]] std::vector<size_t>
  trains_in_section(const std::vector<size_t>& section) const {
    /**
     * Returns the trains that traverse the given section
     *
     * @param section: the section
     * @return the trains that traverse the given section
     */

    std::vector<size_t> tr_in_sec;
    for (size_t i = 0; i < get_train_list().size(); ++i) {
      auto tr_name        = get_train_list().get_train(i).name;
      auto tr_route       = get_route(tr_name).get_edges();
      bool tr_in_sec_flag = false;
      for (size_t j = 0; j < section.size() && !tr_in_sec_flag; ++j) {
        for (size_t k = 0; k < tr_route.size() && !tr_in_sec_flag; ++k) {
          if (section[j] == tr_route[k]) {
            tr_in_sec.emplace_back(i);
            tr_in_sec_flag = true;
          }
        }
      }
    }

    return tr_in_sec;
  };
  [[nodiscard]] std::vector<size_t>
  edges_used_by_train(const std::string& train_name, bool fixed_routes) const {
    /**
     * Returns edges potentially used by a specific train.
     *
     * @param train_name the name of the train
     * @param fixed_routes specifies if the routes are fixed, if not returns all
     * edges
     *
     * @return edges potentially used by a specific train
     */

    if (!fixed_routes) {
      // return vector with values 0, 1, ..., num_edges-1
      std::vector<size_t> return_edges(this->const_n().number_of_edges());
      std::iota(return_edges.begin(), return_edges.end(), 0);
      return return_edges;
    }
    return get_route(train_name).get_edges();
  };
  [[nodiscard]] std::vector<size_t>
  edges_used_by_train(size_t train_id, bool fixed_routes) const {
    return edges_used_by_train(get_train_list().get_train(train_id).name,
                               fixed_routes);
  };
  [[nodiscard]] std::vector<size_t>
  trains_on_edge(size_t edge_id, bool fixed_routes,
                 const std::vector<size_t>& trains_to_consider) const {
    /**
     * Returns all trains that are present on a specific edge, but only consider
     * a subset of trains.
     *
     * @param edge_id the id of the edge
     * @param fixed_routes specifies if the routes are fixed, if not returns all
     * trains
     * @param trains_to_consider the trains to consider
     *
     * @return all trains that are present on a specific edge, but only consider
     * a subset of trains
     */

    if (!this->const_n().has_edge(edge_id)) {
      throw exceptions::EdgeNotExistentException(edge_id);
    }

    if (!fixed_routes) {
      return trains_to_consider;
    }
    std::vector<size_t> return_trains;
    for (const auto tr : trains_to_consider) {
      const auto& tr_route =
          this->get_route(get_train_list().get_train(tr).name);
      if (tr_route.contains_edge(edge_id)) {
        return_trains.push_back(tr);
      }
    }
    return return_trains;
  };
  [[nodiscard]] std::vector<size_t> trains_on_edge(size_t edge_id,
                                                   bool   fixed_routes) const {
    /**
     * Returns all trains that are present on a specific edge at any time.
     *
     * @param edge_id the id of the edge
     * @param fixed_routes specifies if the routes are fixed, if not returns all
     * trains
     *
     * @return all trains that are present on a specific edge at any time
     */
    std::vector<size_t> trains_to_consider(get_train_list().size());
    std::iota(trains_to_consider.begin(), trains_to_consider.end(), 0);
    return trains_on_edge(edge_id, fixed_routes, trains_to_consider);
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
    if (!routes.check_consistency(get_train_list(), this->const_n(),
                                  every_train_must_have_route)) {
      return false;
    };
    for (size_t tr_index = 0; tr_index < timetable.get_train_list().size();
         tr_index++) {
      const auto& tr_name = timetable.get_train_list().get_train(tr_index).name;
      if (routes.has_route(tr_name)) {
        size_t entry = timetable.get_schedule(tr_index).get_entry();
        size_t exit  = timetable.get_schedule(tr_index).get_exit();
        if (routes.get_route(tr_name).get_edge(0, this->const_n()).source !=
            entry) {
          return false;
        }
        if (routes.get_route(tr_name)
                .get_edge(routes.get_route(tr_name).size() - 1, this->const_n())
                .target != exit) {
          return false;
        }
      }
    }
    return true;
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
  explicit SolGeneralProblemInstance(const T& instance) : instance(instance){};
  SolGeneralProblemInstance(const T& instance, SolutionStatus status,
                            double obj, bool has_sol)
      : instance(instance), status(status), obj(obj), has_sol(has_sol){};

  void export_general_solution_data(const std::filesystem::path& p,
                                    bool export_instance,
                                    bool export_data) const {
    if (!is_directory_and_create(p / "solution")) {
      throw exceptions::ExportException("Could not create directory " +
                                        p.string());
    }

    if (export_instance) {
      instance.export_instance(p / "instance");
    }

    if (export_data) {
      json data;
      data["status"]       = static_cast<int>(status);
      data["obj"]          = obj;
      data["has_solution"] = has_sol;
      std::ofstream data_file(p / "solution" / "data.json");
      data_file << data << std::endl;
      data_file.close();
    }
  };

  [[nodiscard]] json get_general_solution_data() const {
    json data;
    data["status"]       = static_cast<int>(status);
    data["obj"]          = obj;
    data["has_solution"] = has_sol;
    return data;
  };

  void set_general_solution_data(const json& data) {
    this->status  = static_cast<SolutionStatus>(data["status"].get<int>());
    this->obj     = data["obj"].get<double>();
    this->has_sol = data["has_solution"].get<bool>();
  };

  [[nodiscard]] bool check_general_solution_data_consistency() const {
    if (status == SolutionStatus::Unknown) {
      return false;
    }
    if (status == SolutionStatus::Infeasible ||
        status == SolutionStatus::Timeout) {
      return true;
    }
    return obj + EPS >= 0;
  }

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

protected:
  void export_general_solution_data_with_routes(const std::filesystem::path& p,
                                                bool export_instance,
                                                bool export_data) const {
    if (!is_directory_and_create(p / "solution")) {
      throw exceptions::ExportException("Could not create directory " +
                                        p.string());
    }

    if (!export_instance) {
      this->get_instance().const_routes().export_routes(
          p / "instance" / "routes", this->get_instance().const_n());
    }

    SolGeneralProblemInstance<T>::export_general_solution_data(
        p, export_instance, export_data);
  }

public:
  SolGeneralProblemInstanceWithScheduleAndRoutes() = default;
  explicit SolGeneralProblemInstanceWithScheduleAndRoutes(const T& instance)
      : SolGeneralProblemInstance<T>(instance){};
  SolGeneralProblemInstanceWithScheduleAndRoutes(const T&       instance,
                                                 SolutionStatus status,
                                                 double obj, bool has_sol)
      : SolGeneralProblemInstance<T>(instance, status, obj, has_sol){};

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
