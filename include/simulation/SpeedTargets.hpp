#include <cmath>
#include <map>
#include <optional>
#include <stdexcept>
#include <vector>

namespace cda_rail {

struct SpeedTargets {
  /**
   * Timestep-Speed tuples that train acceleration follows
   * Timesteps are in range [1, n_timesteps]
   */
  std::map<ulong, double> targets;

public:
  SpeedTargets() = default;
  SpeedTargets(std::vector<ulong> timesteps, std::vector<double> speeds);

  double find_target_speed(ulong timestep);
  void   limit_speed_after(double maximum, ulong timestep);
};

}; // namespace cda_rail
