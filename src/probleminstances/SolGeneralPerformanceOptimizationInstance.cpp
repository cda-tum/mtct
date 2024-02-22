#include "nlohmann/json.hpp"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"

using json = nlohmann::json;

void cda_rail::instances::SolGeneralPerformanceOptimizationInstance::
    export_solution(const std::filesystem::path& p,
                    bool                         export_instance) const {
  /**
   * This method exports the solution object to a specific path. This includes
   * the following:
   * - If export_instance is true, the instance is exported to the path p /
   * instance
   * - If export_instance is false, the routes are exported to the path p /
   * instance / routes
   * - dt, status, obj, and postprocessed are exported to p / solution /
   * data.json
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

  SolGeneralProblemInstanceWithScheduleAndRoutes<
      GeneralPerformanceOptimizationInstance>::
      export_general_solution_data_with_routes(p, export_instance, true);

  json train_pos_json;
  json train_speed_json;
  for (size_t tr_id = 0; tr_id < instance.get_train_list().size(); ++tr_id) {
    const auto& train            = instance.get_train_list().get_train(tr_id);
    train_pos_json[train.name]   = train_pos.at(tr_id);
    train_speed_json[train.name] = train_speed.at(tr_id);
  }

  std::ofstream train_pos_file(p / "solution" / "train_pos.json");
  train_pos_file << train_pos_json << std::endl;
  train_pos_file.close();

  std::ofstream train_speed_file(p / "solution" / "train_speed.json");
  train_speed_file << train_speed_json << std::endl;
  train_speed_file.close();
}

bool cda_rail::instances::SolGeneralPerformanceOptimizationInstance::
    check_consistency() const {
  if (!SolGeneralProblemInstanceWithScheduleAndRoutes<
          GeneralPerformanceOptimizationInstance>::
          check_general_solution_data_consistency()) {
    return false;
  }
  if (!instance.check_consistency(false)) {
    return false;
  }

  for (auto tr_id = 0; tr_id <= train_routed.size(); tr_id++) {
    const auto& tr_name = instance.get_train_list().get_train(tr_id).name;
    if (train_routed.at(tr_id) && !instance.has_route(tr_name)) {
      return false;
    }
    if (!train_routed.at(tr_id) && !instance.get_train_optional().at(tr_id)) {
      return false;
    }
  }

  for (const auto& train_pos_vec : train_pos) {
    for (const auto& [t, pos] : train_pos_vec) {
      if (pos + EPS < 0) {
        return false;
      }
    }
  }
  for (size_t tr_id = 0; tr_id < train_speed.size(); ++tr_id) {
    const auto& train = instance.get_train_list().get_train(tr_id);
    for (const auto& [t, v] : train_speed.at(tr_id)) {
      if (v + EPS < 0 || v > train.max_speed + EPS) {
        return false;
      }
    }
  }
  return true;
}

cda_rail::instances::SolGeneralPerformanceOptimizationInstance::
    SolGeneralPerformanceOptimizationInstance(
        const std::filesystem::path&                                 p,
        const std::optional<GeneralPerformanceOptimizationInstance>& instance) {
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
    this->instance = GeneralPerformanceOptimizationInstance(p / "instance");
  }

  if (import_routes) {
    this->instance.editable_routes() =
        RouteMap(p / "instance" / "routes", this->instance.const_n());
  }

  std::ifstream data_file(p / "solution" / "data.json");
  json          data = json::parse(data_file);
  SolGeneralProblemInstanceWithScheduleAndRoutes<
      GeneralPerformanceOptimizationInstance>::set_general_solution_data(data);

  this->initialize_vectors();

  // Read train_pos
  std::ifstream train_pos_file(p / "solution" / "train_pos.json");
  json          train_pos_json = json::parse(train_pos_file);
  for (const auto& [tr_name, tr_pos_json] : train_pos_json.items()) {
    for (const auto& [t, pos] : tr_pos_json.items()) {
      this->add_train_pos(tr_name, std::stod(t), pos.get<double>());
    }
  }

  // Read train_speed
  std::ifstream train_speed_file(p / "solution" / "train_speed.json");
  json          train_speed_json = json::parse(train_speed_file);
  for (const auto& [tr_name, tr_speed_json] : train_speed_json.items()) {
    for (const auto& [t, speed] : tr_speed_json.items()) {
      this->add_train_speed(tr_name, std::stod(t), speed.get<double>());
    }
  }

  // Read train_routed
  std::ifstream train_routed_file(p / "solution" / "train_routed.json");
  json          train_routed_json = json::parse(train_routed_file);
  for (const auto& [tr_name, routed] : train_routed_json.items()) {
    this->train_routed[instance->get_train_list().get_train_index(tr_name)] =
        routed.get<bool>();
  }
}

void cda_rail::instances::SolGeneralPerformanceOptimizationInstance::
    add_train_pos(const std::string& tr_name, double t, double pos) {
  if (!instance.get_train_list().has_train(tr_name)) {
    throw exceptions::TrainNotExistentException(tr_name);
  }
  if (pos + EPS < 0) {
    throw exceptions::ConsistencyException("Position must be non-negative");
  }
  if (t + EPS < 0) {
    throw exceptions::ConsistencyException("Time must be non-negative");
  }

  const auto tr_id = instance.get_train_list().get_train_index(tr_name);
  if (train_pos.at(tr_id).count(t) > 0) {
    train_pos.at(tr_id).at(t) = pos;
  } else {
    train_pos.at(tr_id).insert({t, pos});
  }
}

void cda_rail::instances::SolGeneralPerformanceOptimizationInstance::
    add_train_speed(const std::string& tr_name, double t, double speed) {
  if (!instance.get_train_list().has_train(tr_name)) {
    throw exceptions::TrainNotExistentException(tr_name);
  }
  if (speed + EPS < 0) {
    throw exceptions::ConsistencyException("Speed must be non-negative");
  }
  if (t + EPS < 0) {
    throw exceptions::ConsistencyException("Time must be non-negative");
  }

  const auto tr_id = instance.get_train_list().get_train_index(tr_name);
  if (train_speed.at(tr_id).count(t) > 0) {
    train_speed.at(tr_id).at(t) = speed;
  } else {
    train_speed.at(tr_id).insert({t, speed});
  }
}

void cda_rail::instances::SolGeneralPerformanceOptimizationInstance::
    set_train_routed_value(const std::string& tr_name, bool val) {
  if (!instance.get_train_list().has_train(tr_name)) {
    throw exceptions::TrainNotExistentException(tr_name);
  }
  train_routed.at(instance.get_train_list().get_train_index(tr_name)) = val;
}
