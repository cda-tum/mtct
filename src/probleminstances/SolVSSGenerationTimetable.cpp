#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "nlohmann/json.hpp"
#include "probleminstances/VSSGenerationTimetable.hpp"

#include <cmath>
#include <fstream>
#include <plog/Log.h>
#include <unordered_set>
#include <vector>

using json = nlohmann::json;

// NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay)

// NOLINTBEGIN(performance-inefficient-string-concatenation)

cda_rail::instances::SolVSSGenerationTimetable::SolVSSGenerationTimetable(
    const cda_rail::instances::VSSGenerationTimetable& instance, int dt)
    : SolGeneralProblemInstanceWithScheduleAndRoutes<VSSGenerationTimetable>(
          instance),
      dt(dt) {
  this->initialize_vectors();
}

double
cda_rail::instances::SolVSSGenerationTimetable::get_train_pos(size_t train_id,
                                                              int time) const {
  if (!instance.get_train_list().has_train(train_id)) {
    throw exceptions::TrainNotExistentException(train_id);
  }

  const auto& [t0, tn] = instance.time_index_interval(train_id, dt, true);
  if (t0 * dt > time || tn * dt < time) {
    throw exceptions::ConsistencyException("Train " + std::to_string(train_id) +
                                           " is not scheduled at time " +
                                           std::to_string(time));
  }

  if (time % dt == 0) {
    const auto t_index = static_cast<size_t>(time / dt) - t0;
    return train_pos.at(train_id).at(t_index);
  }

  const auto t_1 = static_cast<size_t>(std::floor(time / dt)) - t0;
  const auto t_2 = t_1 + 1;

  const auto x_1 = train_pos.at(train_id).at(t_1);
  const auto v_1 = train_speed.at(train_id).at(t_1);
  const auto x_2 = train_pos.at(train_id).at(t_2);
  const auto v_2 = train_speed.at(train_id).at(t_2);

  if (approx_equal(x_2 - x_1, 0.5 * dt * (v_1 + v_2))) {
    const auto a   = (v_2 - v_1) / dt;
    const auto tau = time - static_cast<double>(t0 + t_1) * dt;
    return x_1 + v_1 * tau + 0.5 * a * tau * tau;
  }

  throw exceptions::ConsistencyException(
      "Train " + std::to_string(train_id) + " is not scheduled at time " +
      std::to_string(time) + " and cannot be inferred by linear interpolation");
}

double cda_rail::instances::SolVSSGenerationTimetable::get_train_speed(
    size_t train_id, int time) const {
  if (!instance.get_train_list().has_train(train_id)) {
    throw exceptions::TrainNotExistentException(train_id);
  }

  const auto& [t0, tn] = instance.time_index_interval(train_id, dt, true);
  if (t0 * dt > time || tn * dt < time) {
    throw exceptions::ConsistencyException("Train " + std::to_string(train_id) +
                                           " is not scheduled at time " +
                                           std::to_string(time));
  }

  if (time % dt == 0) {
    const auto t_index = static_cast<size_t>(time / dt) - t0;
    return train_speed.at(train_id).at(t_index);
  }

  const auto t_1 = static_cast<size_t>(std::floor(time / dt)) - t0;
  const auto t_2 = t_1 + 1;

  const auto x_1 = train_pos.at(train_id).at(t_1);
  const auto v_1 = train_speed.at(train_id).at(t_1);
  const auto x_2 = train_pos.at(train_id).at(t_2);
  const auto v_2 = train_speed.at(train_id).at(t_2);

  if (approx_equal(x_2 - x_1, 0.5 * dt * (v_1 + v_2))) {
    const auto a   = (v_2 - v_1) / dt;
    const auto tau = time - static_cast<double>(t0 + t_1) * dt;
    return v_1 + a * tau;
  }

  throw exceptions::ConsistencyException(
      "Train " + std::to_string(train_id) + " is not scheduled at time " +
      std::to_string(time) + " and cannot be inferred by linear interpolation");
}

void cda_rail::instances::SolVSSGenerationTimetable::add_vss_pos(
    size_t edge_id, double pos, bool reverse_edge) {
  if (!instance.const_n().has_edge(edge_id)) {
    throw exceptions::EdgeNotExistentException(edge_id);
  }

  const auto& edge = instance.const_n().get_edge(edge_id);

  if (pos <= EPS || pos + EPS >= edge.length) {
    throw exceptions::ConsistencyException(
        "VSS position " + std::to_string(pos) + " is not on edge " +
        std::to_string(edge_id));
  }

  vss_pos.at(edge_id).emplace_back(pos);
  std::sort(vss_pos.at(edge_id).begin(), vss_pos.at(edge_id).end());

  if (reverse_edge) {
    const auto reverse_edge_index =
        instance.const_n().get_reverse_edge_index(edge_id);
    if (reverse_edge_index.has_value()) {
      vss_pos.at(reverse_edge_index.value()).emplace_back(edge.length - pos);
      std::sort(vss_pos.at(reverse_edge_index.value()).begin(),
                vss_pos.at(reverse_edge_index.value()).end());
    }
  }
}

void cda_rail::instances::SolVSSGenerationTimetable::set_vss_pos(
    size_t edge_id, std::vector<double> pos) {
  if (!instance.const_n().has_edge(edge_id)) {
    throw exceptions::EdgeNotExistentException(edge_id);
  }

  const auto& edge = instance.const_n().get_edge(edge_id);

  for (const auto& p : pos) {
    if (p <= EPS || p + EPS >= edge.length) {
      throw exceptions::ConsistencyException(
          "VSS position " + std::to_string(p) + " is not on edge " +
          std::to_string(edge_id));
    }
  }

  vss_pos.at(edge_id) = std::move(pos);
}

void cda_rail::instances::SolVSSGenerationTimetable::reset_vss_pos(
    size_t edge_id) {
  if (!instance.const_n().has_edge(edge_id)) {
    throw exceptions::EdgeNotExistentException(edge_id);
  }

  vss_pos.at(edge_id).clear();
}

void cda_rail::instances::SolVSSGenerationTimetable::add_train_pos(
    size_t train_id, int time, double pos) {
  if (pos + EPS < 0) {
    throw exceptions::ConsistencyException(
        "Train position " + std::to_string(pos) + " is negative");
  }

  if (!instance.get_train_list().has_train(train_id)) {
    throw exceptions::TrainNotExistentException(train_id);
  }

  const auto& [t0, tn] = instance.time_index_interval(train_id, dt, true);
  if (t0 * dt > time || tn * dt < time) {
    throw exceptions::ConsistencyException("Train " + std::to_string(train_id) +
                                           " is not scheduled at time " +
                                           std::to_string(time));
  }

  if (time % dt != 0) {
    throw exceptions::ConsistencyException(
        "Time " + std::to_string(time) +
        " is not a multiple of dt = " + std::to_string(dt));
  }

  const auto tr_t0   = instance.time_index_interval(train_id, dt, true).first;
  const auto t_index = static_cast<size_t>(time / dt) - tr_t0;
  train_pos.at(train_id).at(t_index) = pos;
}

void cda_rail::instances::SolVSSGenerationTimetable::add_train_speed(
    size_t train_id, int time, double speed) {
  if (!instance.get_train_list().has_train(train_id)) {
    throw exceptions::TrainNotExistentException(train_id);
  }

  if (speed + EPS < 0) {
    throw exceptions::ConsistencyException(
        "Train speed " + std::to_string(speed) + " is negative");
  }
  if (speed > instance.get_train_list().get_train(train_id).max_speed + EPS) {
    throw exceptions::ConsistencyException(
        "Train speed " + std::to_string(speed) +
        " is greater than the maximum speed of train " +
        std::to_string(train_id) + " (" +
        std::to_string(
            instance.get_train_list().get_train(train_id).max_speed) +
        ")");
  }

  const auto& [t0, tn] = instance.time_index_interval(train_id, dt, true);
  if (t0 * dt > time || tn * dt < time) {
    throw exceptions::ConsistencyException("Train " + std::to_string(train_id) +
                                           " is not scheduled at time " +
                                           std::to_string(time));
  }

  if (time % dt != 0) {
    throw exceptions::ConsistencyException(
        "Time " + std::to_string(time) +
        " is not a multiple of dt = " + std::to_string(dt));
  }

  const auto tr_t0   = instance.time_index_interval(train_id, dt, true).first;
  const auto t_index = static_cast<size_t>(time / dt) - tr_t0;
  train_speed.at(train_id).at(t_index) = speed;
}

bool cda_rail::instances::SolVSSGenerationTimetable::check_consistency() const {
  if (!SolGeneralProblemInstanceWithScheduleAndRoutes<
          VSSGenerationTimetable>::check_general_solution_data_consistency()) {
    return false;
  }

  if (dt < 0) {
    return false;
  }
  if (!instance.check_consistency(true)) {
    return false;
  }
  for (const auto& train_pos_vec : train_pos) {
    for (const auto& pos : train_pos_vec) {
      if (pos + EPS < 0) {
        return false;
      }
    }
  }
  for (size_t tr_id = 0; tr_id < train_speed.size(); ++tr_id) {
    const auto& train = instance.get_train_list().get_train(tr_id);
    for (double v : train_speed.at(tr_id)) {
      if (v + EPS < 0 || v > train.max_speed + EPS) {
        return false;
      }
    }
  }
  for (size_t edge_id = 0; edge_id < vss_pos.size(); ++edge_id) {
    const auto& edge = instance.const_n().get_edge(edge_id);
    for (const auto& pos : vss_pos.at(edge_id)) {
      if (pos + EPS < 0 || pos > edge.length + EPS) {
        return false;
      }
    }
  }
  return true;
}

void cda_rail::instances::SolVSSGenerationTimetable::export_solution(
    const std::filesystem::path& p, bool export_instance) const {
  /**
   * This method exports the solution object to a specific path. This includes
   * the following:
   * - If export_instance is true, the instance is exported to the path p /
   * instance
   * - If export_instance is false, the routes are exported to the path p /
   * instance / routes
   * - dt, status, obj, and postprocessed are exported to p / solution /
   * data.json
   * - vss_pos is exported to p / solution / vss_pos.json
   * - train_pos and train_speed are exported to p / solution / train_pos.json
   * and p / solution / train_speed.json The method throws a
   * ConsistencyException if the solution is not consistent.
   *
   * @param p the path to the folder where the solution should be exported
   * @param export_instance whether the instance should be exported next to the
   * solution
   */

  if (!check_consistency()) {
    throw exceptions::ConsistencyException();
  }

  if (!is_directory_and_create(p / "solution")) {
    throw exceptions::ExportException("Could not create directory " +
                                      p.string());
  }

  SolGeneralProblemInstanceWithScheduleAndRoutes<VSSGenerationTimetable>::
      export_general_solution_data_with_routes(p, export_instance, false);

  json data = SolGeneralProblemInstanceWithScheduleAndRoutes<
      VSSGenerationTimetable>::get_general_solution_data();
  data["dt"]            = dt;
  data["mip_obj"]       = mip_obj;
  data["postprocessed"] = postprocessed;
  std::ofstream data_file(p / "solution" / "data.json");
  data_file << data << std::endl;
  data_file.close();

  json vss_pos_json;
  for (size_t edge_id = 0; edge_id < instance.const_n().number_of_edges();
       ++edge_id) {
    const auto& edge = instance.const_n().get_edge(edge_id);
    const auto& v0   = instance.const_n().get_vertex(edge.source).name;
    const auto& v1   = instance.const_n().get_vertex(edge.target).name;
    vss_pos_json["('" + v0 + "', '" + v1 + "')"] = vss_pos.at(edge_id);
  }

  std::ofstream vss_pos_file(p / "solution" / "vss_pos.json");
  vss_pos_file << vss_pos_json << std::endl;
  vss_pos_file.close();

  json train_pos_json;
  json train_speed_json;
  for (size_t tr_id = 0; tr_id < instance.get_train_list().size(); ++tr_id) {
    const auto& train       = instance.get_train_list().get_train(tr_id);
    const auto  tr_interval = instance.time_index_interval(tr_id, dt, true);
    json        train_pos_json_tmp;
    json        train_speed_json_tmp;
    for (size_t t_id = 0; t_id < train_pos.at(tr_id).size(); ++t_id) {
      const auto t = static_cast<int>(tr_interval.first + t_id) * dt;
      train_pos_json_tmp[std::to_string(t)]   = train_pos.at(tr_id).at(t_id);
      train_speed_json_tmp[std::to_string(t)] = train_speed.at(tr_id).at(t_id);
    }
    train_pos_json[train.name]   = train_pos_json_tmp;
    train_speed_json[train.name] = train_speed_json_tmp;
  }

  std::ofstream train_pos_file(p / "solution" / "train_pos.json");
  train_pos_file << train_pos_json << std::endl;
  train_pos_file.close();

  std::ofstream train_speed_file(p / "solution" / "train_speed.json");
  train_speed_file << train_speed_json << std::endl;
  train_speed_file.close();
}

cda_rail::instances::SolVSSGenerationTimetable::SolVSSGenerationTimetable(
    const std::filesystem::path&                 p,
    const std::optional<VSSGenerationTimetable>& instance) {
  if (!std::filesystem::exists(p)) {
    throw exceptions::ImportException("Path does not exist");
  }
  if (!std::filesystem::is_directory(p)) {
    throw exceptions::ImportException("Path is not a directory");
  }

  bool const import_routes = instance.has_value();
  if (instance.has_value()) {
    this->instance = instance.value();
  } else {
    this->instance = VSSGenerationTimetable(p / "instance");
  }

  if (import_routes) {
    this->instance.editable_routes() =
        RouteMap(p / "instance" / "routes", this->instance.const_n());
  }

  if (!this->instance.check_consistency(true)) {
    throw exceptions::ConsistencyException(
        "Imported instance is not consistent");
  }

  // Read data
  std::ifstream data_file(p / "solution" / "data.json");
  json          data = json::parse(data_file);
  SolGeneralProblemInstanceWithScheduleAndRoutes<
      VSSGenerationTimetable>::set_general_solution_data(data);
  this->dt            = data["dt"].get<int>();
  this->mip_obj       = data["mip_obj"].get<double>();
  this->postprocessed = data["postprocessed"].get<bool>();

  this->initialize_vectors();

  // Read vss_pos
  std::ifstream vss_pos_file(p / "solution" / "vss_pos.json");
  json          vss_pos_json = json::parse(vss_pos_file);
  for (const auto& [key, val] : vss_pos_json.items()) {
    std::string source_name;
    std::string target_name;
    extract_vertices_from_key(key, source_name, target_name);
    const auto vss_pos_vector = val.get<std::vector<double>>();
    set_vss_pos(source_name, target_name, vss_pos_vector);
  }

  // Read train_pos
  std::ifstream train_pos_file(p / "solution" / "train_pos.json");
  json          train_pos_json = json::parse(train_pos_file);
  for (const auto& [tr_name, tr_pos_json] : train_pos_json.items()) {
    for (const auto& [t, pos] : tr_pos_json.items()) {
      this->add_train_pos(tr_name, std::stoi(t), pos.get<double>());
    }
  }

  // Read train_speed
  std::ifstream train_speed_file(p / "solution" / "train_speed.json");
  json          train_speed_json = json::parse(train_speed_file);
  for (const auto& [tr_name, tr_speed_json] : train_speed_json.items()) {
    for (const auto& [t, speed] : tr_speed_json.items()) {
      this->add_train_speed(tr_name, std::stoi(t), speed.get<double>());
    }
  }
}

void cda_rail::instances::SolVSSGenerationTimetable::initialize_vectors() {
  vss_pos = std::vector<std::vector<double>>(
      this->instance.const_n().number_of_edges());
  train_pos.reserve(this->instance.get_train_list().size());
  train_speed.reserve(this->instance.get_train_list().size());

  for (size_t tr = 0; tr < this->instance.get_train_list().size(); ++tr) {
    const auto tr_interval = this->instance.time_index_interval(tr, dt, true);
    const auto tr_interval_size = tr_interval.second - tr_interval.first + 1;
    train_pos.emplace_back(tr_interval_size, -1);
    train_speed.emplace_back(tr_interval_size, -1);
  }
}

std::vector<double>
cda_rail::instances::SolVSSGenerationTimetable::get_valid_border_stops(
    size_t train_id) const {
  const auto& tr_name  = instance.get_train_list().get_train(train_id).name;
  const auto& tr_route = instance.get_route(tr_name);
  const auto& tr_route_edges = tr_route.get_edges();

  std::vector<double> valid_border_stops;
  valid_border_stops.emplace_back(0);
  for (const auto& e : tr_route_edges) {
    const auto& edge            = instance.const_n().get_edge(e);
    const auto& e_target        = instance.const_n().get_vertex(edge.target);
    const auto [e_start, e_end] = tr_route.edge_pos(e, instance.const_n());

    const auto& vss_on_e = get_vss_pos(e);
    for (const auto& vss : vss_on_e) {
      if (vss > EPS && vss < edge.length - EPS) {
        valid_border_stops.emplace_back(e_start + vss);
      }
    }

    if (e_target.type != VertexType::NoBorder) {
      valid_border_stops.emplace_back(e_end);
    }
  }

  // Sort return value
  std::sort(valid_border_stops.begin(), valid_border_stops.end());

  return valid_border_stops;
}

// NOLINTEND(performance-inefficient-string-concatenation)

// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay)
