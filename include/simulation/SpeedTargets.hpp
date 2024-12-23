#include <cmath>
#include <map>
#include <optional>
#include <stdexcept>
#include <vector>

namespace cda_rail {

struct SpeedTargets {
  /**
   * Timestep-Speed tuples that train acceleration follows
   */
  std::map<ulong, double> targets;

public:
  SpeedTargets() = default;
  SpeedTargets(std::vector<ulong> timesteps, std::vector<double> speeds);

  double find_target_speed(ulong timestep) const;
  void   limit_speed_from(double maximum, ulong timestep);

  void                    delete_range(ulong start, ulong end);
  std::map<ulong, double> copy_range(ulong start, ulong end) const;
  // Insert without replacing
  void insert(std::map<ulong, double> add_targets);
};

}; // namespace cda_rail
