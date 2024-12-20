#include "simulation/TrainTrajectory.hpp"

// cda_rail::TrainTrajectory::TrainTrajectory(const SimulationInstance&
// instance,
//                                            const Train&              train,
//                                            RoutingSolution solution,
//                                            InitialEdgeState init_state) {
//
//   for (auto switch_direction : solution.switch_directions) {
//     EdgeTrajectory edge_traj(instance, train, solution.v_targets,
//     init_state); std::optional<InitialEdgeState> new_init_state =
//     edge_traj.get_next_edge(instance, switch_direction);
//
//     if (!new_init_state.has_value()) break;
//
//     init_state = new_init_state.value();
//   }
// }
