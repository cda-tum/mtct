#pragma once

#include <cmath>
#include <cstdint>
#include <map>
#include <optional>
#include <stdexcept>
#include <vector>

namespace cda_rail::sim {

struct SpeedTargets {
  /**
   * Timestep-Speed tuples that train acceleration follows
   * Speed target is determined by the previous tuple
   */
  std::map<size_t, double> targets;

public:
  SpeedTargets() = default;
  SpeedTargets(std::vector<size_t> timesteps, std::vector<double> speeds);

  void limit_speed_from(double maximum, size_t timestep);
  void insert(std::map<size_t, double> add_targets); // does not replace
  void delete_range(size_t start, size_t end);
  void set_range(size_t start, size_t end, double value);

  double                   find_target_speed(size_t timestep) const;
  std::optional<size_t>    find_next_reversal(size_t timestep) const;
  std::map<size_t, double> copy_range(size_t start, size_t end) const;
  bool                     is_first_target(size_t timestep) const;
  size_t                   size() const;
};

}; // namespace cda_rail::sim
