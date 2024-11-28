#include <array>
#include <sys/types.h>
#include <vector>

namespace cda_rail {

template <size_t N_SWITCH_VARS> struct RoutingSolutionTrain {
  /**
   * Heuristic routing decision variables for one train
   * @param v_targets speed targets to accelerate towards <timestep
   * [0,n_timesteps], speed [-1,1]>
   * @param directions directions to take at vertices [0,1]
   */
  std::vector<std::tuple<u_int64_t, double>> v_targets;
  std::array<double, N_SWITCH_VARS>          directions;

  // Constructors

  // Default constructor generates a random solution
  RoutingSolutionTrain();
  // Alternatively pass in the variables
  RoutingSolutionTrain(std::vector<std::tuple<u_int64_t, double>> v_targets,
                       double                                     directions[])
      : v_targets(v_targets), directions(directions) {};
};

}; // namespace cda_rail
