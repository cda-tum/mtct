#include "simulation/SpeedTargets.hpp"

cda_rail::SpeedTargets::SpeedTargets(std::vector<u_int64_t> timesteps,
                                     std::vector<double>    speeds) {
  for (size_t i = 0; i < timesteps.size(); i++) {
    targets.insert({timesteps.at(i), speeds.at(i)});
  };
};

void cda_rail::SpeedTargets::limit_speed_from(double    maximum,
                                              u_int64_t timestep) {
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

void cda_rail::SpeedTargets::insert(std::map<u_int64_t, double> add_targets) {
  targets.insert(add_targets.begin(), add_targets.end());
}

void cda_rail::SpeedTargets::delete_range(u_int64_t start, u_int64_t end) {
  if (start > end)
    throw std::invalid_argument("Invalid timestep range.");

  for (auto it = targets.lower_bound(start); it != targets.end();) {
    if ((*it).first <= end) {
      it = targets.erase(it);
    } else {
      break;
    }
  }
}

std::optional<u_int64_t>
cda_rail::SpeedTargets::find_next_reversal(u_int64_t timestep) const {
  bool previous_direction = std::signbit(find_target_speed(timestep));
  for (auto it = targets.upper_bound(timestep); it != targets.end(); it++) {
    if (std::signbit((*it).second) != previous_direction)
      return (*it).first;
  }
  return {};
}

void cda_rail::SpeedTargets::set_range(u_int64_t start, u_int64_t end,
                                       double value) {
  if (start > end)
    throw std::invalid_argument("Invalid timestep range.");

  delete_range(start, end);
  targets.insert_or_assign(start, value);
}

double cda_rail::SpeedTargets::find_target_speed(u_int64_t timestep) const {
  std::map<u_int64_t, double>::const_iterator it =
      targets.upper_bound(timestep);
  if (it != targets.begin()) {
    --it;
  }

  if (it == targets.end())
    throw std::out_of_range("Failed to find relevant speed target.");

  return it->second;
};

std::map<u_int64_t, double>
cda_rail::SpeedTargets::copy_range(u_int64_t start, u_int64_t end) const {
  if (start > end)
    throw std::invalid_argument("Invalid timestep range.");

  std::map<u_int64_t, double> new_map;
  for (auto it = targets.lower_bound(start);
       (*it).first <= end && it != targets.end(); it++) {
    new_map.insert({(*it).first, (*it).second});
  }
  return new_map;
}

bool cda_rail::SpeedTargets::is_first_target(u_int64_t timestep) const {
  std::map<u_int64_t, double>::const_iterator it =
      targets.upper_bound(timestep);
  if (it == targets.begin() || it-- == targets.begin())
    return true;
  return false;
}
