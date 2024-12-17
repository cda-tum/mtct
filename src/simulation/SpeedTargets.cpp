#include "simulation/SpeedTargets.hpp"

cda_rail::SpeedTargets::SpeedTargets(std::vector<uint>   timesteps,
                                     std::vector<double> speeds) {
  for (size_t i = 0; i < timesteps.size(); i++) {
    targets.insert({timesteps.at(i), speeds.at(i)});
  };
};

double cda_rail::SpeedTargets::find_target_speed(uint timestep) {
  if (targets.size() == 0)
    throw std::out_of_range("Speed target set is empty.");

  std::map<uint, double>::iterator it = targets.upper_bound(timestep);
  if (it != targets.begin()) {
    --it;
  }

  if (it == targets.end())
    throw std::out_of_range("Failed to find relevant speed target.");

  return it->second;
};
