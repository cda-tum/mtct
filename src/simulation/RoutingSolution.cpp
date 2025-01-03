#include "simulation/RoutingSolution.hpp"

cda_rail::sim::RoutingSolution::RoutingSolution(
    u_int64_t n_v_target_vars, u_int64_t n_switch_vars, u_int64_t n_timesteps,
    const cda_rail::Train& train, std::ranlux24_base& rng_engine) {
  std::uniform_real_distribution<double>   uniform(0, 1);
  std::uniform_int_distribution<u_int64_t> uniform_timestep(0, n_timesteps - 1);
  std::uniform_real_distribution<double>   uniform_train_speed(-train.max_speed,
                                                               train.max_speed);

  switch_directions.reserve(n_switch_vars);
  while (switch_directions.size() < n_switch_vars) {
    switch_directions.push_back(uniform(rng_engine));
  }

  while (v_targets.targets.size() < n_v_target_vars) {
    v_targets.targets.insert(
        {uniform_timestep(rng_engine), uniform_train_speed(rng_engine)});
  }
}
