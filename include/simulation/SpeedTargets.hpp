#include <map>
#include <optional>
#include <stdexcept>
#include <vector>

namespace cda_rail {

struct SpeedTargets {
  /**
   * Timestep-Speed tuples that train acceleration follows
   */
  std::map<uint, double> targets;

  SpeedTargets(std::vector<uint> timesteps, std::vector<double> speeds);
};

}; // namespace cda_rail
