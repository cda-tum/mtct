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

void cda_rail::sim::RoutingSolutionSet::perturb(
    const SimulationInstance& instance, double fraction,
    std::ranlux24_base& rng_engine) {
  std::uniform_real_distribution<double> uniform_direction_perturbation(
      -fraction, fraction);

  for (auto sol_it = solutions.begin(); sol_it != solutions.end(); sol_it++) {
    double max_speed = instance.timetable.get_train_list()
                           .get_train((*sol_it).first)
                           .max_speed;
    double max_speed_perturbation = fraction * 2 * max_speed;
    size_t max_timestep_perturbation =
        (size_t)std::ceil(fraction * instance.n_timesteps);

    std::uniform_real_distribution<double> uniform_speed_perturbation(
        -max_speed_perturbation, max_speed_perturbation);
    std::uniform_int_distribution<int64_t> uniform_timestep_perturbation(
        -max_timestep_perturbation, max_timestep_perturbation);

    SpeedTargets new_targets;
    for (auto old_target : (*sol_it).second.v_targets.targets) {
      new_targets.targets.insert_or_assign(
          std::clamp(old_target.first +
                         uniform_timestep_perturbation(rng_engine),
                     (size_t)0, instance.n_timesteps - 1),
          std::clamp(old_target.second + uniform_speed_perturbation(rng_engine),
                     -max_speed, max_speed));
    }

    std::vector<double> new_switch_directions;

    for (auto old_direction : (*sol_it).second.switch_directions) {
      new_switch_directions.push_back(
          std::clamp(old_direction + uniform_direction_perturbation(rng_engine),
                     0.0, 1.0));
    }

    (*sol_it).second.v_targets         = new_targets;
    (*sol_it).second.switch_directions = new_switch_directions;
  }
}
