#include "simulation/RoutingSolution.hpp"

cda_rail::RoutingSolution::RoutingSolution(uint64_t      n_v_target_vars,
                                           uint64_t      n_switch_vars,
                                           std::mt19937& rng_engine) {
  std::uniform_real_distribution<> uniform(0, 1);

  switch_directions.reserve(n_switch_vars);
  for (uint64_t i = 0; i <= n_switch_vars; i++) {
    switch_directions.push_back(uniform(rng_engine));
  }

  v_targets.reserve(n_v_target_vars);
  for (uint64_t i = 0; i <= n_v_target_vars; i++) {
    v_targets.push_back(
        std::make_tuple(uniform(rng_engine), uniform(rng_engine)));
  }
}
