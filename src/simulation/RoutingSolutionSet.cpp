#include "simulation/RoutingSolutionSet.hpp"

cda_rail::sim::RoutingSolutionSet::RoutingSolutionSet() {}

cda_rail::sim::RoutingSolutionSet::RoutingSolutionSet(
    const SimulationInstance& instance, std::ranlux24_base& rng_engine) {
  for (const Train& train : instance.timetable.get_train_list()) {
    RoutingSolution sol{instance, train, rng_engine};
    solutions.insert({train.name, sol});
  }
}

cda_rail::sim::RoutingSolutionSet::RoutingSolutionSet(
    const SimulationInstance& instance) {
  for (const Train& train : instance.timetable.get_train_list()) {
    RoutingSolution sol{instance};
    solutions.insert({train.name, sol});
  }
}
