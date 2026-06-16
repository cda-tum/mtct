#include "probleminstances/GeneralProblemInstance.hpp"

#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "GeneralHelper.hpp"
#include "datastructure/Timetable.hpp"
#include "nlohmann/json.hpp"

#include <algorithm>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <optional>
#include <ranges>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

// --------------------
// CONSTRUCTOR / IMPORT
/**
 * @brief Initializes an instance with schedule and routes loaded from the
 * working directory.
 *
 * Loads the network definition from the instance's `network.json` file, then
 * constructs the timetable and routes from the corresponding directories within
 * the instance folder.
 */

cda_rail::instances::GeneralProblemInstanceWithScheduleAndRoutes::
    GeneralProblemInstanceWithScheduleAndRoutes(
        std::string_view const       instanceName,
        std::string_view const       instanceSubdirectory,
        std::filesystem::path const& working_directory)
    : GeneralProblemInstance(instanceName, instanceSubdirectory) {
  // read workingDirectory / instances / instanceSubdirectory / instanceName /
  // network.json
  std::ifstream network_file(working_directory / "instances" /
                             instanceSubdirectory / instanceName /
                             "network.json");
  json          network_json = json::parse(network_file);
  network_file.close();
  m_network =
      Network(network_json.at("network").get<std::string>(), working_directory);

  // read workingDirectory / instances / instanceSubdirectory / instanceName /
  // timetable and / routes
  m_timetable = Timetable(working_directory / "instances" /
                              instanceSubdirectory / instanceName / "timetable",
                          m_network);
  m_routes = RouteMap(working_directory / "instances" / instanceSubdirectory /
                          instanceName / "routes",
                      m_network);
}

// --------------------
// EXPORT
/**
 * @brief Exports the instance data.
 *
 * Writes the timetable and routes to disk. Always creates a network reference
 * file. If saveNetwork is true, also exports the network structure.
 *
 * @param working_directory Root directory for instance exports.
 * @param saveNetwork If true, exports the network structure to the working
 *                    directory.
 */

void cda_rail::instances::GeneralProblemInstanceWithScheduleAndRoutes::
    export_instance(std::filesystem::path const& working_directory,
                    bool const                   saveNetwork) const {
  if (saveNetwork) {
    m_network.export_network(working_directory);
  }
  m_timetable.export_timetable(working_directory / "instances" /
                                   get_instance_subdirectory() /
                                   get_instance_name() / "timetable",
                               get_const_network());
  m_routes.export_routes(working_directory / "instances" /
                             get_instance_subdirectory() / get_instance_name() /
                             "routes",
                         get_const_network());

  json network_json{};
  network_json["network"] =
      get_const_network()
          .get_network_name(); // NOLINT(*-pro-bounds-avoid-unchecked-container-access)
  std::ofstream network_file(working_directory / "instances" /
                             get_instance_subdirectory() / get_instance_name() /
                             "network.json");
  network_file << network_json << '\n';
  network_file.close();
}

// ---------------------
// GETTER
/**
 * @brief Determines whether every train has a non-empty route.
 *
 * @return `true` if every train has a non-empty route, `false` otherwise.
 */

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

/**
 * @brief Retrieves the position of the last stop for a train at a specified
 * station along its route.
 *
 * @param tr_id The identifier of the train.
 * @param station_name The name of the station.
 * @return The position on the train's route where it makes its last stop at the
 * specified station, or `std::nullopt` if the train does not stop at that
 * station.
 */
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
  for (const auto& paths : stop_tracks_tmp | std::views::values) {
    for (const auto& path : paths) {
      stop_tracks.insert(path.begin(), path.end());
    }
  }
  return route.get_last_pos_on_edges(stop_tracks, this->get_const_network());
}

/**
 * @brief Groups stop paths for a train at a station by target vertex.
 *
 * @param edges_to_consider Edges to consider when computing stop paths.
 * @return Vector of (vertex_index, stop_paths) pairs, where stop_paths are
 *         all paths ending at that vertex.
 */
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

/**
 * @brief Gets edges used by a train based on route configuration.
 *
 * When @p fixed_routes is false, returns all edges in the network. When @p
 * fixed_routes is true, returns edges from the train's assigned route; if the
 * train lacks a route and @p error_if_no_route is false, returns all edges
 * instead.
 *
 * @param train_name Name of the train.
 * @param fixed_routes If false, returns all edges; if true, returns only the
 * train's route edges.
 * @param error_if_no_route If false and the train has no route, returns all
 * edges; otherwise, the route must exist.
 * @return Set of edge IDs used by the train.
 */
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

/**
 * @brief Collects the vertices used by a train.
 *
 * @param tr_name The name of the train.
 * @param fixed_routes If true, consider only vertices in the train's route; if
 * false, consider all network vertices.
 * @param error_if_no_route If true, throw an exception if the train has no
 * assigned route; if false, include all vertices if no route exists.
 * @return The set of vertex indices used by the train.
 */
cda_rail::index_set
cda_rail::instances::GeneralProblemInstanceWithScheduleAndRoutes::
    vertices_used_by_train(const std::string& tr_name, bool fixed_routes,
                           bool error_if_no_route) const {
  const auto edges =
      edges_used_by_train(tr_name, fixed_routes, error_if_no_route);
  cda_rail::index_set return_vertices;
  for (const auto& e_id : edges) {
    const auto& edge = this->get_const_network().get_edge(e_id);
    return_vertices.insert(edge.source);
    return_vertices.insert(edge.target);
  }
  return return_vertices;
}

/**
 * @brief Determines which sections contain edges used by a train.
 *
 * @param tr_name Train name.
 * @param sections A vector where each element is a set of edge IDs forming a
 * section.
 * @param fixed_routes If true, uses only the train's assigned route; if false,
 * considers all edges.
 * @param error_if_no_route If true and fixed_routes is true, throws when the
 * train has no route; if false, uses all edges.
 * @return A set of section indices containing at least one edge used by the
 * train.
 */
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

/**
 * @brief Identifies trains that use at least one edge from a section.
 *
 * @param section Set of edge IDs representing the section.
 * @param fix_routes If true, only considers trains' assigned routes; if false,
 * considers all edges.
 * @param error_if_no_route If true, throws an exception for trains without
 * routes; if false, includes them.
 * @return Indices of trains using at least one edge in the section.
 */
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
/**
 * @brief Filters trains based on whether their routes contain a specific edge.
 *
 * When routes are not fixed, returns all trains. When routes are fixed,
 * includes trains whose routes contain the edge, plus trains without a route
 * (if `error_if_not_route` is false).
 *
 * @param route_map The route assignments for trains.
 *
 * @return Subset of `trains_to_consider` containing trains matching the
 * criteria.
 *
 * @throws EdgeNotExistentException if the edge does not exist.
 */
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

/**
 * @brief Identifies all trains that use a specific edge.
 *
 * @param route_map The route map to consult.
 * @param edge_id The edge identifier.
 * @param fixed_routes If true, filters trains by whether their routes contain
 * the edge.
 * @param error_if_not_route If false, trains without a route are included when
 * `fixed_routes` is true.
 * @return Set of indices of all trains that use the edge.
 *
 * @throws exceptions::EdgeNotExistentException if the edge does not exist in
 * the network.
 */
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
/**
 * @brief Verifies consistency of the timetable, routes, and schedule entry/exit
 * vertices.
 *
 * Checks that the timetable is consistent with the network, routes are
 * consistent with the train list and network, and for each train with a route,
 * the schedule's entry and exit vertices match the route's first and last edge
 * endpoints.
 *
 * @param every_train_must_have_route If `true`, the route consistency check
 * enforces that every train has a route. If `false`, trains without routes are
 * permitted.
 *
 * @return `true` if all consistency checks pass, `false` otherwise.
 */

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

/**
 * @brief Constructs a JSON representation of the general solution data.
 *
 * @return A JSON object containing the solution status, objective value, and
 * feasibility flag.
 */

cda_rail::json
cda_rail::instances::SolGeneralProblemInstance::get_general_solution_data()
    const {
  json data;
  data["status"] = static_cast<int>(
      m_status);       // NOLINT(*-pro-bounds-avoid-unchecked-container-access)
  data["obj"] = m_obj; // NOLINT(*-pro-bounds-avoid-unchecked-container-access)
  data["has_solution"] =
      m_has_sol; // NOLINT(*-pro-bounds-avoid-unchecked-container-access)
  return data;
}

/**
 * @brief Sets solution metadata from a JSON object.
 *
 * @param data JSON object containing `"status"`, `"obj"`, and `"has_solution"`
 * fields.
 */
void cda_rail::instances::SolGeneralProblemInstance::set_general_solution_data(
    const json& data) {
  this->m_status  = static_cast<SolutionStatus>(data.at("status").get<int>());
  this->m_obj     = data.at("obj").get<double>();
  this->m_has_sol = data.at("has_solution").get<bool>();
}

/**
 * @brief Validates the consistency of the solution status and objective value.
 *
 * @return `true` if the status is defined and the objective is non-negative for
 * non-infeasible/non-timeout solutions, `false` otherwise.
 */
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

/**
 * @brief Loads solution data from disk.
 *
 * Reads the solution data file from the solution directory and populates
 * the current instance with the loaded data.
 *
 * @param working_directory Base directory containing solution files.
 * @param solutionSubdirectory Subdirectory name where the solution is stored.
 * @param parameter_identifier Optional identifier for the parameter set.
 *
 * @throws exceptions::ImportException If the solution data file cannot be
 * opened.
 */
void cda_rail::instances::SolGeneralProblemInstance::load_solution(
    const std::filesystem::path&      working_directory,
    std::string_view const            solutionSubdirectory,
    std::optional<std::string> const& parameter_identifier) {
  std::filesystem::path const p = get_export_path(
      working_directory, solutionSubdirectory, parameter_identifier);

  std::ifstream data_file(p / "solution_data.json");
  if (!data_file.is_open()) {
    throw exceptions::ImportException("Could not open file " + p.string());
  }
  json data;
  data_file >> data;
  data_file.close();

  set_general_solution_data(data);
}

/**
 * @brief Writes the solution data to a JSON file.
 *
 * Creates the export directory if necessary and writes the solution to
 * `solution_data.json`.
 *
 * @param working_directory The base working directory for the export.
 * @param solutionSubdirectory The subdirectory name for the solution.
 * @param parameter_identifier Optional identifier for the solution parameters.
 *
 * @throws exceptions::ExportException If the export directory cannot be
 * created.
 */
void cda_rail::instances::SolGeneralProblemInstance::export_solution(
    const std::filesystem::path&      working_directory,
    std::string_view const            solutionSubdirectory,
    std::optional<std::string> const& parameter_identifier) const {
  std::filesystem::path const p = get_export_path(
      working_directory, solutionSubdirectory, parameter_identifier);

  if (!is_directory_and_create(p)) {
    throw exceptions::ExportException("Could not create directory " +
                                      p.string());
  }

  json const    data = get_general_solution_data();
  std::ofstream data_file(p / "solution_data.json");
  data_file << data << '\n';
  data_file.close();
}

/**
 * @brief Removes all solution routes.
 */

void cda_rail::instances::SolGeneralProblemInstanceWithScheduleAndRoutes::
    reset_routes() {
  for (const auto& tr : get_instance()->get_const_train_list()) {
    if (m_solution_routes.has_route(tr.get_name())) {
      m_solution_routes.remove_route(tr.get_name());
    }
  }
}

/**
 * @brief Adds an empty route for a train to the solution.
 *
 * @param train_name Name of the train.
 * @throws std::exception if the train does not exist in the instance.
 */
void cda_rail::instances::SolGeneralProblemInstanceWithScheduleAndRoutes::
    add_empty_route(const std::string& train_name) {
  get_instance()->get_const_train_list().throw_if_train_not_exist(train_name);
  m_solution_routes.add_empty_route(train_name);
}

/**
 * @brief Loads solution data and routes from disk.
 *
 * Restores the complete solution state by loading general solution data
 * (status, objective, feasibility) and then loading route information
 * from the corresponding export location.
 *
 * @param working_directory Base directory containing solutions.
 * @param solutionSubdirectory Solution subdirectory name.
 * @param parameter_identifier Optional parameter identifier for the solution.
 *
 * @throws exceptions::ImportException if solution data cannot be read from
 * disk.
 */
void cda_rail::instances::SolGeneralProblemInstanceWithScheduleAndRoutes::
    load_solution(const std::filesystem::path&      working_directory,
                  std::string_view const            solutionSubdirectory,
                  std::optional<std::string> const& parameter_identifier) {
  SolGeneralProblemInstance::load_solution(
      working_directory, solutionSubdirectory, parameter_identifier);

  std::filesystem::path const p = get_export_path(
      working_directory, solutionSubdirectory, parameter_identifier);
  m_solution_routes = RouteMap(p, get_instance()->get_const_network());
}

/**
 * @brief Exports the solution to the specified directory, with optional
 * instance export.
 *
 * @param working_directory The base directory for export.
 * @param solutionSubdirectory The subdirectory where the solution will be
 * saved.
 * @param save_instance If true, also exports the problem instance.
 * @param parameter_identifier Optional identifier for the solution export path.
 */
void cda_rail::instances::SolGeneralProblemInstanceWithScheduleAndRoutes::
    export_solution(
        const std::filesystem::path& working_directory,
        std::string_view const solutionSubdirectory, bool save_instance,
        std::optional<std::string> const& parameter_identifier) const {
  SolGeneralProblemInstance::export_solution(
      working_directory, solutionSubdirectory, parameter_identifier);

  std::filesystem::path const p = get_export_path(
      working_directory, solutionSubdirectory, parameter_identifier);
  m_solution_routes.export_routes(p, this->get_instance()->get_const_network());

  if (save_instance) {
    get_instance()->export_instance(working_directory, true);
  }
}

/**
 * @brief Validates the consistency of the solution and its underlying instance.
 *
 * @return bool `true` if the solution and instance are both consistent, `false`
 * otherwise.
 */
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
