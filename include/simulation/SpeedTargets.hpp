#include <map>
#include <optional>
#include <stdexcept>
#include <vector>

namespace cda_rail {

class SpeedTargets {
  /**
   * Timestep-Speed tuples that train acceleration follows
   */
  std::map<uint, double> targets;

public:
  SpeedTargets(std::vector<uint> timesteps, std::vector<double> speeds);

  double find_target_speed(uint timestep);
};

}; // namespace cda_rail
