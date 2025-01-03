#include "simulation/RoutingSolutionSet.hpp"

cda_rail::sim::RoutingSolutionSet::RoutingSolutionSet(
    const SimulationInstance& instance, std::ranlux24_base& rng_engine) {
  for (const Train& train : instance.timetable.get_train_list()) {
    RoutingSolution sol{instance.n_v_target_vars, instance.n_switch_vars,
                        instance.n_timesteps, train, rng_engine};
    solutions.insert({train.name, sol});
  }
}
