#include "simulation/RoutingSolution.hpp"

cda_rail::RoutingSolution::RoutingSolution(uint                n_v_target_vars,
                                           uint                n_switch_vars,
                                           std::ranlux24_base& rng_engine) {
  std::uniform_real_distribution<double> uniform(0, 1);

  switch_directions.reserve(n_switch_vars);
  for (uint i = 1; i <= n_switch_vars; i++) {
    switch_directions.push_back(uniform(rng_engine));
  }

  v_targets.reserve(n_v_target_vars);
  for (uint i = 1; i <= n_v_target_vars; i++) {
    v_targets.push_back(
        std::make_tuple(uniform(rng_engine), uniform(rng_engine)));
  }
}

bool cda_rail::RoutingSolution::check_consistency() const {
  // Check range
  for (double direction : switch_directions) {
    if (direction < 0.0 || direction > 1.0)
      return false;
  }

  for (std::tuple<double, double> v_target : v_targets) {
    if (std::get<0>(v_target) < 0.0 || std::get<0>(v_target) > 1.0)
      return false;
    if (std::get<1>(v_target) < 0.0 || std::get<1>(v_target) > 1.0)
      return false;
  }

  return true;
}
