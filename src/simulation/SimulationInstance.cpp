#include "simulation/SimulationInstance.hpp"

cda_rail::sim::SimulationInstance::SimulationInstance(Network   network,
                                                      Timetable timetable,
                                                      bool      allow_reversing)
    : network(network), timetable(timetable),
      shortest_paths(network.all_vertex_pairs_shortest_paths_undirected()),
      n_timesteps(get_last_train_departure()),
      n_switch_vars(std::ceil((get_max_train_speed() * n_timesteps) /
                              get_shortest_track())),
      allow_reversing(allow_reversing) {}

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

size_t cda_rail::sim::SimulationInstance::get_last_train_departure() const {
  size_t last_departure = 1;
  for (auto train : timetable.get_train_list()) {
    // TODO: narrowing cast should be avoided
    size_t departure = timetable.get_schedule(train.name).get_t_n();
    if (departure > last_departure)
      last_departure = departure;
  }
  return last_departure;
}
