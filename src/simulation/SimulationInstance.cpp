#include "simulation/SimulationInstance.hpp"

double cda_rail::SimulationInstance::get_max_train_speed() {
  double max_speed = 0;
  for (const auto& train : timetable.get_train_list()) {
    if (train.max_speed > max_speed) {
      max_speed = train.max_speed;
    }
  }
  return max_speed;
}

double cda_rail::SimulationInstance::get_shortest_track() {
  double shortest_edge_length = DBL_MAX;
  for (const auto& edge : network.get_edges()) {
    if (edge.length < shortest_edge_length) {
      shortest_edge_length = edge.length;
    }
  }
  return shortest_edge_length;
}
