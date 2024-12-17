#include "simulation/RoutingSolution.hpp"

cda_rail::RoutingSolution::RoutingSolution(ulong n_v_target_vars,
                                           ulong n_switch_vars,
                                           ulong n_timesteps,
                                           const cda_rail::Train& train,
                                           std::ranlux24_base&    rng_engine) {
  std::uniform_int_distribution<ulong>   uniform_int(1, n_timesteps);
  std::uniform_real_distribution<double> uniform(-train.max_speed,
                                                 train.max_speed);

  switch_directions.reserve(n_switch_vars);
  for (ulong i = 1; i <= n_switch_vars; i++) {
    switch_directions.push_back(uniform(rng_engine));
  }

  for (ulong i = 1; i <= n_v_target_vars; i++) {
    v_targets.targets.insert({uniform_int(rng_engine), uniform(rng_engine)});
  }
}
