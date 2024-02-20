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
  }

  for (const auto& train_pos_vec : train_pos) {
    for (const auto& pos : train_pos_vec) {
      if (pos.second + EPS < 0) {
        return false;
      }
    }
  }
  for (size_t tr_id = 0; tr_id < train_speed.size(); ++tr_id) {
    const auto& train = instance.get_train_list().get_train(tr_id);
    for (const auto& v : train_speed.at(tr_id)) {
      if (v.second + EPS < 0 || v.second > train.max_speed + EPS) {
        return false;
      }
    }
  }
  return true;
}
