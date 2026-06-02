#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"

#include "Definitions.hpp"
#include "EOMHelper.hpp"

#include <algorithm>
#include <cstddef>

cda_rail::instances::GeneralPerformanceOptimizationInstance::
    GeneralPerformanceOptimizationInstance(
        std::string_view const       instanceName,
        std::string_view const       instanceSubdirectory,
        std::filesystem::path const& workingDirectory) {
  {
    initialize_vectors();

    std::ifstream file(workingDirectory / "instances" / instanceSubdirectory /
                       instanceName / "problem_data.json");
    json          j = json::parse(file);
    for (const auto& [train_name, weight] : j["train_weights"].items()) {
      set_train_weight(train_name, static_cast<double>(weight));
    }
    for (const auto& [train_name, optional] : j["train_optional"].items()) {
      set_train_optionality_value(train_name, static_cast<bool>(optional));
    }
    m_station_delay_weight = static_cast<double>(j["station_delay_weight"]);
    m_lambda               = static_cast<double>(j["lambda"]);
  }
}

void cda_rail::instances::GeneralPerformanceOptimizationInstance::
    export_instance(const std::filesystem::path& workingDirectory) const {
  GeneralProblemInstanceWithScheduleAndRoutes::export_instance(
      workingDirectory);

  json j;
  for (size_t i = 0; i < m_train_weights.size(); ++i) {
    j["train_weights"][this->get_const_train_list().get_train(i).get_name()] =
        m_train_weights.at(i);
    j["train_optional"][this->get_const_train_list().get_train(i).get_name()] =
        m_train_optional.at(i);
  }
  j["station_delay_weight"] = m_station_delay_weight;
  j["lambda"]               = m_lambda;

  std::ofstream file(workingDirectory / "instances" /
                     get_instance_subdirectory() / get_instance_name() /
                     "problem_data.json");
  file << j << '\n';
}

void cda_rail::instances::GeneralPerformanceOptimizationInstance::
    discretize_stops() {
  /**
   * This method discretizes the network within the stations. It updates the
   * timetable and the routes accordingly.
   */

  for (const auto& station_name :
       this->get_const_station_list().get_station_names()) {
    const auto& station_tracks =
        this->get_const_station_list().get_station(station_name).tracks;
    const auto new_edges =
        this->get_editable_network().separate_stop_edges(station_tracks);
    this->get_editable_timetable().update_after_discretization(new_edges);
    this->get_editable_routes().update_after_discretization(new_edges);
  }
}

bool cda_rail::instances::GeneralPerformanceOptimizationInstance::
    check_consistency(bool every_train_must_have_route) const {
  if (!GeneralProblemInstanceWithScheduleAndRoutes::check_consistency(
          every_train_must_have_route)) {
    return false;
  }
  const auto num_tr = this->get_const_timetable().get_train_list().size();
  if (get_train_weights().size() != num_tr ||
      get_train_optional().size() != num_tr) {
    return false;
  }
  return (get_lambda() >= 0 && get_station_delay_weight() >= 0 &&
          std::ranges::all_of(get_train_weights(),
                              [](double w) { return w >= 0; }));
}

double cda_rail::instances::GeneralPerformanceOptimizationInstance::
    get_approximate_leaving_time(size_t train) const {
  const auto& tr_object = this->get_const_train_list().get_train(train);
  const auto& timetable = this->get_const_timetable().get_schedule(train);
  return tr_object.get_length() / timetable.get_exit_velocity();
}

double cda_rail::instances::GeneralPerformanceOptimizationInstance::
    get_maximal_leaving_time(size_t train, double v) const {
  const auto& tr_object = this->get_const_train_list().get_train(train);
  const auto& timetable = this->get_const_timetable().get_schedule(train);
  return cda_rail::max_travel_time_no_stopping(
      v, timetable.get_exit_velocity(), V_MIN, tr_object.get_acceleration(),
      tr_object.get_deceleration(), tr_object.get_length());
}

double cda_rail::instances::GeneralPerformanceOptimizationInstance::
    get_minimal_leaving_time(size_t train, double v) const {
  const auto& tr_object = this->get_const_train_list().get_train(train);
  const auto& timetable = this->get_const_timetable().get_schedule(train);
  auto        v_max     = std::max(v, timetable.get_exit_velocity());
  if (v_max <= 0) {
    const auto& exit_node =
        this->get_const_timetable().get_schedule(train).get_exit_vertex();
    auto const& in_edges = this->get_const_network().in_edges(exit_node);
    assert(in_edges.size() == 1);
    const auto exit_edge = *in_edges.begin();
    v_max = this->get_const_network().get_edge(exit_edge).max_speed;
  }
  v_max = std::min(v_max, tr_object.get_max_speed());
  return cda_rail::min_travel_time(
      v, timetable.get_exit_velocity(), v_max, tr_object.get_acceleration(),
      tr_object.get_deceleration(), tr_object.get_length());
}
