#include "simulation/RoutingSolution.hpp"

cda_rail::sim::RoutingSolution::RoutingSolution(
    const SimulationInstance& instance, const Train& train,
    std::ranlux24_base& rng_engine) {
  std::uniform_real_distribution<double>   uniform(0, 1);
  std::uniform_int_distribution<u_int64_t> uniform_timestep(
      0, instance.n_timesteps - 1);
  double min_speed = 0;
  if (instance.bidirectional_travel) {
    min_speed = -train.max_speed;
  }
  std::uniform_real_distribution<double> uniform_train_speed(min_speed,
                                                             train.max_speed);

  switch_directions.reserve(instance.n_switch_vars);
  while (switch_directions.size() < instance.n_switch_vars) {
    switch_directions.push_back(uniform(rng_engine));
  }

  while (v_targets.targets.size() < instance.n_v_target_vars) {
    v_targets.targets.insert(
        {uniform_timestep(rng_engine), uniform_train_speed(rng_engine)});
  }
}
