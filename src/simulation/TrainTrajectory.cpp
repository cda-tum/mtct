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

cda_rail::InitialEdgeState
cda_rail::init_train_state_from_schedule(const SimulationInstance& instance,
                                         const Train&              train) {
  cda_rail::Schedule train_schedule =
      instance.timetable.get_schedule(train.name);
  return InitialEdgeState{
      .timestep = (ulong)train_schedule.get_t_0(),
      .edge =
          instance.network.get_successors(train_schedule.get_entry()).front(),
      .position    = 0,
      .orientation = true,
      .speed       = train_schedule.get_v_0()};
}
