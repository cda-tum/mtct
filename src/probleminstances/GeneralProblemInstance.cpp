#include "probleminstances/GeneralProblemInstance.hpp"

#include "CustomExceptions.hpp"
#include "GeneralHelper.hpp"
#include "nlohmann/json.hpp"
#include "nlohmann/json_fwd.hpp"

using json = nlohmann::json;

// --------------------
// CONSTRUCTOR / IMPORT
// --------------------

cda_rail::instances::GeneralProblemInstanceWithScheduleAndRoutes::
    GeneralProblemInstanceWithScheduleAndRoutes(
        std::string_view const       instanceName,
        std::string_view const       instanceSubdirectory,
        std::filesystem::path const& workingDirectory)
    : GeneralProblemInstance(instanceName, instanceSubdirectory) {
  // read workingDirectory / instances / instanceSubdirectory / instanceName /
  // network.json
  std::ifstream network_file(workingDirectory / "instances" /
                             instanceSubdirectory / instanceName /
                             "network.json");
  json          network_json = json::parse(network_file);
  network_file.close();
  m_network =
      Network(network_json.at("network").get<std::string>(), workingDirectory);

  // read workingDirectory / instances / instanceSubdirectory / instanceName /
  // timetable and / routes
  m_timetable = Timetable(workingDirectory / "instances" /
                              instanceSubdirectory / instanceName / "timetable",
                          m_network);
  m_routes    = RouteMap(workingDirectory / "instances" / instanceSubdirectory /
                             instanceName / "routes",
                         m_network);
}

// --------------------
// EXPORT
// --------------------

void cda_rail::instances::GeneralProblemInstanceWithScheduleAndRoutes::
    export_instance(std::filesystem::path const& workingDirectory,
                    bool const                   saveNetwork) const {
  if (saveNetwork) {
    m_network.export_network(workingDirectory);
  }
  m_timetable.export_timetable(workingDirectory / "instances" /
                                   get_instance_subdirectory() /
                                   get_instance_name() / "timetable",
                               get_const_network());
  m_routes.export_routes(workingDirectory / "instances" /
                             get_instance_subdirectory() / get_instance_name() /
                             "routes",
                         get_const_network());

  json network_json{};
  network_json["network"] = get_const_network().get_network_name();
  std::ofstream network_file(workingDirectory / "instances" /
                             get_instance_subdirectory() / get_instance_name() /
                             "network.json");
  network_file << network_json << '\n';
  network_file.close();
}

// ---------------------
// GETTER
// ---------------------

bool cda_rail::instances::GeneralProblemInstanceWithScheduleAndRoutes::
    has_route_for_every_train() const {
  /**
   * Checks if every train has a route.
   *
   * @return true if every train has a route, false otherwise
   */

  return std::ranges::all_of(get_const_train_list(), [this](const auto& tr) {
    return get_const_routes().has_route(tr.get_name()) &&
           !get_const_routes().get_route(tr.get_name()).empty();
  });
}

std::optional<double>
cda_rail::instances::GeneralProblemInstanceWithScheduleAndRoutes::
    get_last_stop_position_on_route(size_t             tr_id,
                                    const std::string& station_name) const {
  const auto& tr_obj          = get_const_train_list().get_train(tr_id);
  const auto& route           = get_const_routes().get_route(tr_obj.get_name());
  const auto& route_edges     = route.get_edges();
  const auto  stop_tracks_tmp = get_stop_tracks(
      tr_id, station_name, {route_edges.begin(), route_edges.end()});
  cda_rail::index_set stop_tracks;
  for (const auto& [idx, paths] : stop_tracks_tmp) {
    for (const auto& path : paths) {
      stop_tracks.insert(path.begin(), path.end());
    }
  }
  return route.get_last_pos_on_edges(stop_tracks, this->get_const_network());
}

std::vector<std::pair<size_t, std::vector<cda_rail::index_vector>>>
cda_rail::instances::GeneralProblemInstanceWithScheduleAndRoutes::
    possible_stop_vertices(size_t tr, const std::string& station_name,
                           const cda_rail::index_set& edges_to_consider) const {
  /**
   * This method returns the possible stop vertices for a train at a station
   * together with the respective stop edges
   *
   * @param tr The index of the train
   * @param station_name The name of the station
   * @param edges_to_consider The edges to consider. Default: {}, then all
   * edges
   *
   * @return A vector of pairs:
   * - The first element of the pair is the index of a possible stop vertex
   * - The second element lists all possible stop paths ending in that vertex
   * Note: The train has to use one of the stop paths if it stops at the
   * vertex
   */

  auto stop_tracks = get_stop_tracks(tr, station_name, edges_to_consider);
  std::unordered_map<size_t, std::vector<cda_rail::index_vector>> combined;

  for (const auto& [edge_index, paths] : stop_tracks) {
    size_t const target = this->get_const_network().get_edge(edge_index).target;
    auto&        vec    = combined[target];
    vec.insert(vec.end(), paths.begin(), paths.end());
  }

  std::vector<std::pair<size_t, std::vector<cda_rail::index_vector>>> ret_val;
  ret_val.reserve(combined.size());
  for (auto& [target, paths] : combined) {
    ret_val.emplace_back(target, std::move(paths));
  }

  return ret_val;
}

cda_rail::index_set
cda_rail::instances::GeneralProblemInstanceWithScheduleAndRoutes::
    edges_used_by_train(const std::string& train_name, bool fixed_routes,
                        bool error_if_no_route) const {
  /**
   * Returns edges potentially used by a specific train.
   *
   * @param train_name the name of the train
   * @param fixed_routes specifies if the routes are fixed, if not returns all
   * edges
   *
   * @return edges potentially used by a specific train
   */

  if (!fixed_routes ||
      (!error_if_no_route && !get_const_routes().has_route(train_name))) {
    // return set with values 0, 1, ..., num_edges-1
    auto const          num_edges = get_const_network().get_edges().size();
    auto                edge_ids  = std::views::iota(size_t{0}, num_edges);
    cda_rail::index_set all_edges;
    all_edges.reserve(num_edges);
    all_edges.insert(edge_ids.begin(), edge_ids.end());
    return all_edges;
  }
  auto const& route_edges =
      get_const_routes().get_route(train_name).get_edges();
  return {route_edges.begin(), route_edges.end()};
}

cda_rail::index_set
cda_rail::instances::GeneralProblemInstanceWithScheduleAndRoutes::
    vertices_used_by_train(const std::string& tr_name, bool fixed_routes,
                           bool error_if_no_route) const {
  const auto edges =
      edges_used_by_train(tr_name, fixed_routes, error_if_no_route);
  cda_rail::index_set return_vertices;
  for (const auto& e_id : edges) {
    const auto& edge = this->get_const_network().get_edge(e_id);
    if (!std::ranges::contains(return_vertices, edge.source)) {
      return_vertices.insert(edge.source);
    }
    if (!std::ranges::contains(return_vertices, edge.target)) {
      return_vertices.insert(edge.target);
    }
  }
  return return_vertices;
}

cda_rail::index_set
cda_rail::instances::GeneralProblemInstanceWithScheduleAndRoutes::
    sections_used_by_train(const std::string&                      tr_name,
                           const std::vector<cda_rail::index_set>& sections,
                           bool fixed_routes, bool error_if_no_route) const {
  const auto edges =
      edges_used_by_train(tr_name, fixed_routes, error_if_no_route);
  cda_rail::index_set return_sections;
  for (size_t section_id = 0; section_id < sections.size(); ++section_id) {
    const auto& section     = sections.at(section_id);
    bool        add_section = false;
    for (const auto& e_id : section) {
      if (std::ranges::contains(edges, e_id)) {
        add_section = true;
        break;
      }
    }
    if (add_section) {
      return_sections.insert(section_id);
    }
  }
  return return_sections;
}

cda_rail::index_set
cda_rail::instances::GeneralProblemInstanceWithScheduleAndRoutes::
    trains_in_section(const cda_rail::index_set& section, bool fix_routes,
                      bool error_if_no_route) const {
  cda_rail::index_set tr_in_sec;
  for (size_t i = 0; i < get_const_train_list().size(); ++i) {
    const auto edges_used =
        edges_used_by_train(i, fix_routes, error_if_no_route);

    // do section and edges_used overlap?
    if (std::ranges::any_of(section, [&edges_used](auto edge_id) {
          return edges_used.contains(edge_id);
        })) {
      tr_in_sec.insert(i);
    }
  }
  return tr_in_sec;
}
cda_rail::index_set
cda_rail::instances::GeneralProblemInstanceWithScheduleAndRoutes::
    trains_on_edge(RouteMap const& route_map, size_t edge_id, bool fixed_routes,
                   const cda_rail::index_set& trains_to_consider,
                   bool                       error_if_not_route) const {
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

  if (!this->get_const_network().has_edge(edge_id)) {
    throw exceptions::EdgeNotExistentException(edge_id);
  }

  if (!fixed_routes) {
    return trains_to_consider;
  }
  cda_rail::index_set return_trains;
  for (const auto tr : trains_to_consider) {
    auto const& tr_name = get_const_train_list().get_train(tr).get_name();
    if (!error_if_not_route && !route_map.has_route(tr_name)) {
      return_trains.insert(tr);
    } else {
      const auto& tr_route = route_map.get_route(tr_name);
      if (tr_route.contains_edge(edge_id)) {
        return_trains.insert(tr);
      }
    }
  }
  return return_trains;
}

cda_rail::index_set
cda_rail::instances::GeneralProblemInstanceWithScheduleAndRoutes::
    all_trains_on_edge(RouteMap const& route_map, size_t edge_id,
                       bool fixed_routes, bool error_if_not_route) const {
  auto const number_of_trains = get_const_train_list().size();
  auto const all_trains_view  = std::views::iota(size_t{0}, number_of_trains);
  return trains_on_edge(route_map, edge_id, fixed_routes,
                        {all_trains_view.begin(), all_trains_view.end()},
                        error_if_not_route);
}

// ---------------
// Consistency
// ---------------

bool cda_rail::instances::GeneralProblemInstanceWithScheduleAndRoutes::
    check_consistency(bool every_train_must_have_route) const {
  if (!get_const_timetable().check_consistency(this->get_const_network())) {
    return false;
  }
  if (!get_const_routes().check_consistency(get_const_train_list(),
                                            this->get_const_network(),
                                            every_train_must_have_route)) {
    return false;
  };
  for (size_t tr_index = 0; tr_index < get_const_train_list().size();
       tr_index++) {
    const auto& tr_name = get_const_train_list().get_train(tr_index).get_name();
    if (get_const_routes().has_route(tr_name)) {
      size_t const entry = get_const_schedule(tr_index).get_entry_vertex();
      size_t const exit  = get_const_schedule(tr_index).get_exit_vertex();

      auto const& tr_route = get_const_routes().get_route(tr_name);
      if (tr_route.get_last_edge(this->get_const_network()).target != exit) {
        return false;
      }
      if (tr_route.get_first_edge(this->get_const_network()).source != entry) {
        return false;
      }
    }
  }
  return true;
}

// -------------------
// Solution Objects
// -------------------

// general problem instance

cda_rail::json
cda_rail::instances::SolGeneralProblemInstance::get_general_solution_data()
    const {
  json data;
  data["status"]       = static_cast<int>(m_status);
  data["obj"]          = m_obj;
  data["has_solution"] = m_has_sol;
  return data;
}

void cda_rail::instances::SolGeneralProblemInstance::set_general_solution_data(
    const json& data) {
  this->m_status  = static_cast<SolutionStatus>(data.at("status").get<int>());
  this->m_obj     = data.at("obj").get<double>();
  this->m_has_sol = data.at("has_solution").get<bool>();
}

bool cda_rail::instances::SolGeneralProblemInstance::check_consistency() const {
  if (m_status == SolutionStatus::Unknown) {
    return false;
  }
  if (m_status != SolutionStatus::Infeasible &&
      m_status != SolutionStatus::Timeout && m_obj + EPS < 0) {
    return false;
  }

  return true;
}

void cda_rail::instances::SolGeneralProblemInstance::load_solution(
    const std::filesystem::path&      workingDirectory,
    std::string_view const            solutionSubdirectory,
    std::optional<std::string> const& parameter_identifier) {
  std::filesystem::path const p = get_export_path(
      workingDirectory, solutionSubdirectory, parameter_identifier);

  std::ifstream data_file(p / "solution_data.json");
  if (!data_file.is_open()) {
    throw exceptions::ExportException("Could not open file " + p.string());
  }
  json data;
  data_file >> data;
  data_file.close();

  set_general_solution_data(data);
}

void cda_rail::instances::SolGeneralProblemInstance::export_solution(
    const std::filesystem::path&      workingDirectory,
    std::string_view const            solutionSubdirectory,
    std::optional<std::string> const& parameter_identifier) const {
  std::filesystem::path const p = get_export_path(
      workingDirectory, solutionSubdirectory, parameter_identifier);

  if (!is_directory_and_create(p)) {
    throw exceptions::ExportException("Could not create directory " +
                                      p.string());
  }

  json const    data = get_general_solution_data();
  std::ofstream data_file(p / "solution_data.json");
  data_file << data << '\n';
  data_file.close();
}

// with schedules and routes

void cda_rail::instances::SolGeneralProblemInstanceWithScheduleAndRoutes::
    reset_routes() {
  for (const auto& tr : get_instance()->get_const_train_list()) {
    if (m_solution_routes.has_route(tr.get_name())) {
      m_solution_routes.remove_route(tr.get_name());
    }
  }
}

void cda_rail::instances::SolGeneralProblemInstanceWithScheduleAndRoutes::
    add_empty_route(const std::string& train_name) {
  get_instance()->get_const_train_list().throw_if_train_not_exist(train_name);
  m_solution_routes.add_empty_route(train_name);
}

void cda_rail::instances::SolGeneralProblemInstanceWithScheduleAndRoutes::
    load_solution(const std::filesystem::path&      workingDirectory,
                  std::string_view const            solutionSubdirectory,
                  std::optional<std::string> const& parameter_identifier) {
  SolGeneralProblemInstance::load_solution(
      workingDirectory, solutionSubdirectory, parameter_identifier);

  std::filesystem::path const p = get_export_path(
      workingDirectory, solutionSubdirectory, parameter_identifier);
  m_solution_routes = RouteMap(p, get_instance()->get_const_network());
}

void cda_rail::instances::SolGeneralProblemInstanceWithScheduleAndRoutes::
    export_solution(
        const std::filesystem::path&      workingDirectory,
        std::string_view const            solutionSubdirectory,
        std::optional<std::string> const& parameter_identifier) const {
  SolGeneralProblemInstance::export_solution(
      workingDirectory, solutionSubdirectory, parameter_identifier);

  std::filesystem::path const p = get_export_path(
      workingDirectory, solutionSubdirectory, parameter_identifier);
  m_solution_routes.export_routes(p, this->get_instance()->get_const_network());
}

bool cda_rail::instances::SolGeneralProblemInstanceWithScheduleAndRoutes::
    check_consistency() const {
  if (!SolGeneralProblemInstance::check_consistency()) {
    return false;
  }

  if (!this->get_instance()->check_consistency(false)) {
    return false;
  }

  return true;
}
