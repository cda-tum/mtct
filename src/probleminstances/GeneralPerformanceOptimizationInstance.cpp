#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"

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
    get_approximate_leaving_time(size_t train) {
  const auto& tr_object = this->get_train_list().get_train(train);
  const auto& timetable = this->get_timetable().get_schedule(train);
  return tr_object.length / timetable.get_v_n();
}
