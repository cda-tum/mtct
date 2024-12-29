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
  std::map<ulong, double> targets;

public:
  SpeedTargets() = default;
  SpeedTargets(std::vector<ulong> timesteps, std::vector<double> speeds);

  void limit_speed_from(double maximum, ulong timestep);
  void insert(std::map<ulong, double> add_targets); // does not replace
  void delete_range(ulong start, ulong end);
  void set_range(ulong start, ulong end, double value);

  double                  find_target_speed(ulong timestep) const;
  std::optional<ulong>    find_next_reversal(ulong timestep) const;
  std::map<ulong, double> copy_range(ulong start, ulong end) const;
  bool                    is_first_target(ulong timestep) const;
};

}; // namespace cda_rail
