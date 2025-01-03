#include "simulation/SimulationInstance.hpp"

double cda_rail::sim::SimulationInstance::get_max_train_speed() const {
  double max_speed = 0;
  for (const auto& train : timetable.get_train_list()) {
    if (train.max_speed > max_speed) {
      max_speed = train.max_speed;
    }
  }
  return max_speed;
}

double cda_rail::sim::SimulationInstance::get_shortest_track() const {
  double shortest_edge_length = DBL_MAX;
  for (const auto& edge : network.get_edges()) {
    if (edge.length < shortest_edge_length) {
      shortest_edge_length = edge.length;
    }
  }
  return shortest_edge_length;
}

u_int64_t cda_rail::sim::SimulationInstance::get_last_train_departure() const {
  u_int64_t last_departure = 1;
  for (auto train : timetable.get_train_list()) {
    // TODO: narrowing cast should be avoided
    u_int64_t departure = timetable.get_schedule(train.name).get_t_n();
    if (departure > last_departure)
      last_departure = departure;
  }
  return last_departure;
}
