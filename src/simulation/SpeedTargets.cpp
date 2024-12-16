#include "simulation/SpeedTargets.hpp"

cda_rail::SpeedTargets::SpeedTargets(std::vector<uint>   timesteps,
                                     std::vector<double> speeds) {
  for (size_t i = 0; i < timesteps.size(); i++) {
    targets.insert({timesteps.at(i), speeds.at(i)});
  };
};
