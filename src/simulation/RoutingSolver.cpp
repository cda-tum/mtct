#include "simulation/RoutingSolver.hpp"

#include <ctime>

cda_rail::RoutingSolver::RoutingSolver(SimulationInstance instance)
    : instance(instance) {
  rng_engine = std::ranlux24_base(time(NULL));
}
