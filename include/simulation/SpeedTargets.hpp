#pragma once

#include <cmath>
#include <map>
#include <optional>
#include <stdexcept>
#include <vector>

namespace cda_rail {

struct SpeedTargets {
  /**
   * Timestep-Speed tuples that train acceleration follows
   * Speed target is determined by the previous tuple
   */
  std::map<u_int64_t, double> targets;

public:
  SpeedTargets() = default;
  SpeedTargets(std::vector<u_int64_t> timesteps, std::vector<double> speeds);

  void limit_speed_from(double maximum, u_int64_t timestep);
  void insert(std::map<u_int64_t, double> add_targets); // does not replace
  void delete_range(u_int64_t start, u_int64_t end);
  void set_range(u_int64_t start, u_int64_t end, double value);

  double                      find_target_speed(u_int64_t timestep) const;
  std::optional<u_int64_t>    find_next_reversal(u_int64_t timestep) const;
  std::map<u_int64_t, double> copy_range(u_int64_t start, u_int64_t end) const;
  bool                        is_first_target(u_int64_t timestep) const;
};

}; // namespace cda_rail
