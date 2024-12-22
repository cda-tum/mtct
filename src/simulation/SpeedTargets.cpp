#include "simulation/SpeedTargets.hpp"

cda_rail::SpeedTargets::SpeedTargets(std::vector<ulong>  timesteps,
                                     std::vector<double> speeds) {
  for (size_t i = 0; i < timesteps.size(); i++) {
    targets.insert({timesteps.at(i), speeds.at(i)});
  };
};

double cda_rail::SpeedTargets::find_target_speed(ulong timestep) {
  if (targets.size() == 0)
    throw std::out_of_range("Speed target set is empty.");

  std::map<ulong, double>::const_iterator it = targets.upper_bound(timestep);
  if (it != targets.begin()) {
    --it;
  }

  if (it == targets.end())
    throw std::out_of_range("Failed to find relevant speed target.");

  return it->second;
};

void cda_rail::SpeedTargets::limit_speed_after(double maximum, ulong timestep) {
  ;
  for (auto it = targets.lower_bound(timestep); it != targets.end(); it++) {
    if (std::abs((*it).second) > maximum) {
      (*it).second = std::copysign(maximum, (*it).second);
    }
  }
}
