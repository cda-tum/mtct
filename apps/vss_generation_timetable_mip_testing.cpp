#include "solver/mip-based/VSSGenTimetableSolver.hpp"

int main(int argc, char** argv) {
  if (argc != 11) {
    std::cout << "Expected 10 arguments, got " << argc - 1 << std::endl;
    std::exit(-1);
  }

  std::string                                        model_name    = argv[1];
  std::string                                        instance_path = argv[2];
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(instance_path);

  std::cout << "Instance " << model_name << " loaded at " << instance_path
            << std::endl;

  int  delta_t                  = std::stoi(argv[3]);
  bool fix_routes               = std::stoi(argv[4]);
  bool discretize_vss_positions = std::stoi(argv[5]);
  bool include_train_dynamics   = std::stoi(argv[6]);
  bool include_braking_curves   = std::stoi(argv[7]);
  bool use_pwl                  = std::stoi(argv[8]);
  bool use_schedule_cuts        = std::stoi(argv[9]);
  int  timeout                  = std::stoi(argv[10]);

  std::cout << "The following parameters were passed to the toolkit:"
            << std::endl;
  std::cout << "   delta_t: " << delta_t << std::endl;
  if (fix_routes) {
    std::cout << "   routes are fixed" << std::endl;
  }
  if (discretize_vss_positions) {
    std::cout << "   the graph is preprocessed" << std::endl;
  }
  if (include_train_dynamics) {
    std::cout << "   acceleration and deceleration are included" << std::endl;
  }
  if (include_braking_curves) {
    std::cout << "   braking distance is included" << std::endl;
  }
  if (use_pwl) {
    std::cout << "   piecewise linear functions are used" << std::endl;
  }
  if (use_schedule_cuts) {
    std::cout << "   schedule cuts are used" << std::endl;
  }
  std::cout << "   timeout: " << timeout << "s" << std::endl;

  std::string file_name =
      model_name + "_" + std::to_string(delta_t) + "_" +
      std::to_string(fix_routes) + "_" +
      std::to_string(discretize_vss_positions) + "_" +
      std::to_string(include_train_dynamics) + "_" +
      std::to_string(include_braking_curves) + "_" + std::to_string(use_pwl) +
      "_" + std::to_string(use_schedule_cuts) + "_" + std::to_string(timeout);

  solver.solve(delta_t, fix_routes, discretize_vss_positions,
               include_train_dynamics, include_braking_curves, use_pwl,
               use_schedule_cuts, timeout, true, true, file_name);
}
