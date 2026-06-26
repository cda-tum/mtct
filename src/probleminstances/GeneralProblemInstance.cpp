#include "probleminstances/GeneralProblemInstance.hpp"

#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "EOMHelper.hpp"
#include "GeneralHelper.hpp"
#include "datastructure/Timetable.hpp"
#include "nlohmann/json.hpp"
#include "nlohmann/json_fwd.hpp"

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
  if (!network_file.is_open()) {
    throw exceptions::ImportException(
        (working_directory / "instances" / instanceSubdirectory / instanceName /
         "network.json")
            .string());
  }
  nlohmann::json network_json = nlohmann::json::parse(network_file);
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

// -------------
// HELPER
// -------------
bool cda_rail::instances::GeneralProblemInstanceWithScheduleAndRoutes::
    is_route_end_valid_stop_pos(size_t tr, const cda_rail::index_vector& edges,
                                size_t next_stop_id) const {
  get_const_train_list().throw_if_train_not_exist(tr);

  const auto& tr_length   = get_const_train_list().get_train(tr).get_length();
  const auto& tr_schedule = get_const_schedule(tr).get_stops();
  if (next_stop_id >= tr_schedule.size()) {
    // All stops have been set, hence, no further stop is possible
    return false;
  }
  const auto& next_station = tr_schedule.at(next_stop_id).get_station();

  double len = 0;
  for (auto it = edges.rbegin(); (len < tr_length) && (it != edges.rend());
       ++it) {
    if (!std::ranges::contains(next_station.tracks, *it)) {
      // Track does not belong to the next station
      return false;
    }
    len += get_const_network().get_edge(*it).length;
  }

  return len >= tr_length;
}

// --------------------
// EXPORT
// --------------------

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

  nlohmann::json network_json{};
  network_json["network"] =
      get_const_network()
          .get_network_name(); // NOLINT(*-pro-bounds-avoid-unchecked-container-access)
  std::ofstream network_file(working_directory / "instances" /
                             get_instance_subdirectory() / get_instance_name() /
                             "network.json");
  if (!network_file.is_open()) {
    throw exceptions::ExportException(
        "Could not open network.json for writing");
  }
  if (!(network_file << network_json << '\n')) {
    throw exceptions::ExportException("Failed to write network.json");
  }
}

// ---------------------
// GETTER

bool cda_rail::instances::GeneralProblemInstanceWithScheduleAndRoutes::
    has_route_for_every_train() const {
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
  for (const auto& paths : stop_tracks_tmp | std::views::values) {
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
  get_const_train_list().throw_if_train_not_exist(train_name);
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
    return_vertices.insert(edge.source);
    return_vertices.insert(edge.target);
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

cda_rail::instances::GeneralProblemInstance::FeasibilityCheck
cda_rail::instances::GeneralProblemInstanceWithScheduleAndRoutes::
    is_obviously_infeasible(bool late_entry_allowed) const {
  auto const num_trains = get_const_train_list().get_number_of_trains();

  // Check if routes exist
  for (size_t tr = 0; tr < num_trains; ++tr) {
    auto const& tr_obj      = get_const_train_list().get_train(tr);
    auto const& tr_name     = tr_obj.get_name();
    auto const& tr_schedule = get_const_schedule(tr);
    auto        entry_edges =
        get_const_network().out_edges(tr_schedule.get_entry_vertex());
    if (entry_edges.size() != 1) {
      return {.is_obviously_infeasible = true,
              .reason =
                  "Train " + tr_name + " has more or less than one entry edge"};
    }
    if (auto const& exit_edges =
            get_const_network().in_edges(tr_schedule.get_exit_vertex());
        exit_edges.size() != 1) {
      return {.is_obviously_infeasible = true,
              .reason =
                  "Train " + tr_name + " has more or less than one exit edge"};
    }
    for (auto const& stop : tr_schedule.get_stops()) {
      auto const& stop_tracks = stop.get_station().tracks;
      if (auto const p_len =
              get_const_network().shortest_path_length_between_edge_sets(
                  entry_edges, stop_tracks);
          !p_len.has_value()) {
        return {.is_obviously_infeasible = true,
                .reason = "Train " + tr_name + " cannot reach station " +
                          stop.get_station().name};
      }
      entry_edges = stop_tracks;
    }
    if (auto const p_len =
            get_const_network()
                .shortest_path_length_between_edge_and_vertex_set(
                    entry_edges, {tr_schedule.get_exit_vertex()});
        !p_len.has_value()) {
      return {.is_obviously_infeasible = true,
              .reason = "Train " + tr_name + " cannot reach exit vertex"};
    }
  }

  // Check entry times
  if (late_entry_allowed) {
    return {.is_obviously_infeasible = false};
  }

  struct EntryInformation {
    Train  tr_obj;
    double entry_time;
    double entry_velocity;
  };
  std::vector<std::vector<EntryInformation>> entry_information(
      get_const_network().number_of_vertices());

  for (size_t tr = 0; tr < num_trains; ++tr) {
    auto const& tr_obj      = get_const_train_list().get_train(tr);
    auto const& tr_schedule = get_const_schedule(tr);
    entry_information.at(tr_schedule.get_entry_vertex())
        .emplace_back(tr_obj, tr_schedule.get_entry_time(),
                      tr_schedule.get_initial_velocity());
  }
  for (size_t v = 0; v < get_const_network().number_of_vertices(); ++v) {
    if (entry_information.at(v).empty()) {
      continue;
    }
    // sort entry_information.at(v) by entry_time
    std::ranges::sort(entry_information.at(v),
                      [](const EntryInformation& a, const EntryInformation& b) {
                        return a.entry_time < b.entry_time;
                      });

    auto const vertex_obj  = get_const_network().get_vertex(v);
    auto const entry_edges = get_const_network().out_edges(v);
    assert(entry_edges.size() == 1);
    auto const entry_edge_id = *entry_edges.begin();
    auto const entry_edge    = get_const_network().get_edge(entry_edge_id);

    for (size_t entry_idx = 1; entry_idx < entry_information.at(v).size();
         ++entry_idx) {
      auto const& entry_obj      = entry_information.at(v).at(entry_idx);
      auto const& prev_entry_obj = entry_information.at(v).at(entry_idx - 1);
      if (entry_obj.entry_time <
          prev_entry_obj.entry_time + vertex_obj.headway) {
        return {.is_obviously_infeasible = true,
                .reason = "Entry times at vertex " + vertex_obj.name +
                          " are closer than the vertex headway for trains " +
                          prev_entry_obj.tr_obj.get_name() + " and " +
                          entry_obj.tr_obj.get_name()};
      }
      auto const bd = braking_distance(entry_obj.entry_velocity,
                                       entry_obj.tr_obj.get_deceleration());
      if (auto const max_speed =
              bd <= entry_edge.length
                  ? std::min(entry_edge.max_speed,
                             prev_entry_obj.tr_obj.get_max_speed())
                  : prev_entry_obj.tr_obj.get_max_speed();
          entry_obj.entry_time <
          prev_entry_obj.entry_time +
              min_travel_time_flexible_exit_speed(
                  prev_entry_obj.entry_velocity, max_speed,
                  prev_entry_obj.tr_obj.get_acceleration(),
                  prev_entry_obj.tr_obj.get_length() + bd)) {
        return {
            .is_obviously_infeasible = true,
            .reason =
                "At vertex " + vertex_obj.name + ", train " +
                prev_entry_obj.tr_obj.get_name() +
                " cannot clear the braking distance of the following train " +
                entry_obj.tr_obj.get_name()};
      }
    }
  }

  return {.is_obviously_infeasible = false};
}

// -------------------
// Solution Objects
// -------------------

nlohmann::json
cda_rail::instances::SolGeneralProblemInstance::get_general_solution_data()
    const {
  nlohmann::json data;
  data["status"] = static_cast<int>(
      m_status);       // NOLINT(*-pro-bounds-avoid-unchecked-container-access)
  data["obj"] = m_obj; // NOLINT(*-pro-bounds-avoid-unchecked-container-access)
  data["has_solution"] =
      m_has_sol; // NOLINT(*-pro-bounds-avoid-unchecked-container-access)
  return data;
}

void cda_rail::instances::SolGeneralProblemInstance::set_general_solution_data(
    const nlohmann::json& data) {
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
    const std::filesystem::path&      working_directory,
    std::string_view const            solutionSubdirectory,
    std::optional<std::string> const& parameter_identifier) {
  std::filesystem::path const p = get_export_path(
      working_directory, solutionSubdirectory, parameter_identifier);

  std::ifstream data_file(p / "solution_data.json");
  if (!data_file.is_open()) {
    throw exceptions::ImportException("Could not open file " + p.string());
  }
  nlohmann::json data;
  data_file >> data;
  data_file.close();

  set_general_solution_data(data);
}

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

  nlohmann::json const data = get_general_solution_data();
  std::ofstream        data_file(p / "solution_data.json");
  if (!data_file.is_open()) {
    throw exceptions::ExportException(
        "Could not open solution_data.json for writing");
  }
  if (!(data_file << data << '\n')) {
    throw exceptions::ExportException("Failed to write solution_data.json");
  }
}

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
    load_solution(const std::filesystem::path&      working_directory,
                  std::string_view const            solutionSubdirectory,
                  std::optional<std::string> const& parameter_identifier) {
  SolGeneralProblemInstance::load_solution(
      working_directory, solutionSubdirectory, parameter_identifier);

  std::filesystem::path const p = get_export_path(
      working_directory, solutionSubdirectory, parameter_identifier);
  m_solution_routes = RouteMap(p, get_instance()->get_const_network());
}

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

bool cda_rail::instances::SolGeneralProblemInstanceWithScheduleAndRoutes::
    check_consistency() const {
  if (!SolGeneralProblemInstance::check_consistency()) {
    return false;
  }

  const auto* instance = this->get_instance();
  if (!instance->check_consistency(false)) {
    return false;
  }

  return m_solution_routes.check_consistency(instance->get_const_train_list(),
                                             instance->get_const_network(),
                                             this->has_solution());
}
