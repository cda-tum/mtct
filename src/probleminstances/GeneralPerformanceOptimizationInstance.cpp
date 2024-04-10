#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"

#include "Definitions.hpp"
#include "EOMHelper.hpp"
#include "probleminstances/VSSGenerationTimetable.hpp"

void cda_rail::instances::GeneralPerformanceOptimizationInstance::
    discretize_stops() {
  /**
   * This method discretizes the network within the stations. It updates the
   * timetable and the routes accordingly.
   */

  for (const auto& station_name :
       this->get_station_list().get_station_names()) {
    const auto& station_tracks =
        this->get_station_list().get_station(station_name).tracks;
    const auto new_edges = this->n().separate_stop_edges(station_tracks);
    this->editable_timetable().update_after_discretization(new_edges);
    this->editable_routes().update_after_discretization(new_edges);
  }
}

double cda_rail::instances::GeneralPerformanceOptimizationInstance::
    get_approximate_leaving_time(size_t train) const {
  const auto& tr_object = this->get_train_list().get_train(train);
  const auto& timetable = this->get_timetable().get_schedule(train);
  return tr_object.length / timetable.get_v_n();
}

double cda_rail::instances::GeneralPerformanceOptimizationInstance::
    get_maximal_leaving_time(size_t train, double v) const {
  const auto& tr_object = this->get_train_list().get_train(train);
  const auto& timetable = this->get_timetable().get_schedule(train);
  return cda_rail::max_travel_time_no_stopping(
      v, timetable.get_v_n(), V_MIN, tr_object.acceleration,
      tr_object.deceleration, tr_object.length);
}

double cda_rail::instances::GeneralPerformanceOptimizationInstance::
    get_minimal_leaving_time(size_t train, double v) const {
  const auto& tr_object = this->get_train_list().get_train(train);
  const auto& timetable = this->get_timetable().get_schedule(train);
  auto        v_max     = std::max(v, timetable.get_v_n());
  if (v_max <= 0) {
    const auto& exit_node =
        this->get_timetable().get_schedule(train).get_exit();
    const auto exit_edge = this->const_n().in_edges(exit_node).at(0);
    v_max                = this->const_n().get_edge(exit_edge).max_speed;
  }
  if (v_max > tr_object.max_speed) {
    v_max = tr_object.max_speed;
  }
  return cda_rail::min_travel_time(v, timetable.get_v_n(), v_max,
                                   tr_object.acceleration,
                                   tr_object.deceleration, tr_object.length);
}

cda_rail::instances::GeneralPerformanceOptimizationInstance cda_rail::
    instances::GeneralPerformanceOptimizationInstance::cast_from_vss_generation(
        const cda_rail::instances::VSSGenerationTimetable& vss_gen) {
  GeneralPerformanceOptimizationInstance general_instance(
      vss_gen.const_n(), vss_gen.get_timetable().parse_to_general_timetable(),
      vss_gen.get_routes());
  general_instance.initialize_vectors();
  return general_instance;
}
