#include "simulation/RoutingSolution.hpp"

cda_rail::sim::RoutingSolution::RoutingSolution(
    const SimulationInstance& instance) {
  switch_directions.reserve(instance.n_switch_vars);
  while (switch_directions.size() < instance.n_switch_vars) {
    switch_directions.push_back(0.5);
  }
  v_targets.targets.insert({0, 0});
}

cda_rail::sim::RoutingSolution::RoutingSolution(
    const SimulationInstance& instance, const Train& train,
    std::ranlux24_base& rng_engine) {
  std::uniform_real_distribution<double>   uniform(0, 1);
  std::uniform_int_distribution<u_int64_t> uniform_timestep(
      0, instance.n_timesteps - 1);
  std::uniform_int_distribution<u_int64_t> uniform_n_v_target_vars(
      1, instance.n_timesteps);
  double min_speed = 0;
  if (instance.allow_reversing)
    min_speed = -train.max_speed;
  std::uniform_real_distribution<double> uniform_train_speed(min_speed,
                                                             train.max_speed);

  switch_directions.reserve(instance.n_switch_vars);
  while (switch_directions.size() < instance.n_switch_vars) {
    switch_directions.push_back(uniform(rng_engine));
  }

  // TODO: ignores duplicates and produces less than n_v_target_vars variables
  while (v_targets.size() < uniform_n_v_target_vars(rng_engine)) {
    v_targets.targets.insert(
        {uniform_timestep(rng_engine), uniform_train_speed(rng_engine)});
  }
}

cda_rail::sim::RoutingSolution::RoutingSolution(
    const SimulationInstance& instance, const Train& train,
    const std::function<double(void)>& rnd01) {
  auto uniform_n_v_target_vars = [instance, rnd01]() {
    return (u_int64_t)std::round(1 + (rnd01() * (instance.n_timesteps - 1)));
  };

  auto uniform_timestep = [instance, rnd01]() {
    return (u_int64_t)std::round((rnd01() * (instance.n_timesteps - 1)));
  };

  std::function<double()> uniform_train_speed;
  if (instance.allow_reversing) {
    uniform_train_speed = [train, rnd01]() {
      return (2 * rnd01() - 1) * train.max_speed;
    };
  } else {
    uniform_train_speed = [train, rnd01]() {
      return rnd01() * train.max_speed;
    };
  }

  switch_directions.reserve(instance.n_switch_vars);
  while (switch_directions.size() < instance.n_switch_vars) {
    switch_directions.push_back(rnd01());
  }

  // TODO: ignores duplicates and produces less than n_v_target_vars variables
  while (v_targets.size() < uniform_n_v_target_vars()) {
    v_targets.targets.insert({uniform_timestep(), uniform_train_speed()});
  }
}

cda_rail::sim::RoutingSolution::RoutingSolution(
    const SimulationInstance& instance, const SpeedTargets& targets,
    std::vector<double> directions, const Train& train)
    : v_targets(targets), switch_directions(directions) {
  if (targets.size() < 1 || directions.size() < instance.n_switch_vars)
    throw std::invalid_argument("Routing solution is not consistent.");

  // Check speed targets
  double min_speed = 0;
  if (instance.allow_reversing)
    min_speed = -train.max_speed;

  for (const auto& target : targets.targets) {
    if (target.second < min_speed || target.second > train.max_speed ||
        target.first > instance.n_timesteps - 1)
      throw std::invalid_argument("Routing solution is not consistent.");
  }

  for (const auto& direction : directions) {
    if (direction > 1 || direction < 0)
      throw std::invalid_argument("Routing solution is not consistent.");
  }
}
