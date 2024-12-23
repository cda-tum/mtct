#include "simulation/SpeedTargets.hpp"

cda_rail::SpeedTargets::SpeedTargets(std::vector<ulong>  timesteps,
                                     std::vector<double> speeds) {
  for (size_t i = 0; i < timesteps.size(); i++) {
    targets.insert({timesteps.at(i), speeds.at(i)});
  };
};

void cda_rail::SpeedTargets::limit_speed_from(double maximum, ulong timestep) {
  // Cap targets in range
  for (auto it = targets.lower_bound(timestep); it != targets.end(); it++) {
    if (std::abs((*it).second) > maximum) {
      (*it).second = std::copysign(maximum, (*it).second);
    }
  }

  // Ensure carried over targets are not too high
  double previous_target = find_target_speed(timestep);
  if (std::abs(previous_target) > maximum)
    targets.insert_or_assign(timestep, std::copysign(maximum, previous_target));
}

void cda_rail::SpeedTargets::insert(std::map<ulong, double> add_targets) {
  targets.insert(add_targets.begin(), add_targets.end());
}

void cda_rail::SpeedTargets::delete_range(ulong start, ulong end) {
  targets.erase(targets.lower_bound(start), targets.upper_bound(end));
}

double cda_rail::SpeedTargets::find_target_speed(ulong timestep) const {
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

std::map<ulong, double> cda_rail::SpeedTargets::copy_range(ulong start,
                                                           ulong end) const {
  std::map<ulong, double> new_map;
  for (auto it = targets.lower_bound(start);
       (*it).first <= end && it != targets.end(); it++) {
    new_map.insert({(*it).first, (*it).second});
  }
  return new_map;
}
