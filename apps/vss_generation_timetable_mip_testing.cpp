#include "Definitions.hpp"
#include "VSSModel.hpp"
#include "solver/mip-based/VSSGenTimetableSolver.hpp"

#include <gsl/span>

int main(int argc, char** argv) {
  if (argc != 11) {
    std::cout << "Expected 10 arguments, got " << argc - 1 << std::endl;
    std::exit(-1);
  }

  auto args = gsl::span<char*>(argv, argc);

  const std::string                                  model_name    = args[1];
  const std::string                                  instance_path = args[2];
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(instance_path);

  std::cout << "Instance " << model_name << " loaded at " << instance_path
            << std::endl;

  const int  delta_t                  = std::stoi(args[3]);
  const bool fix_routes               = std::stoi(args[4]) != 0;
  const bool discretize_vss_positions = std::stoi(args[5]) != 0;
  const bool include_train_dynamics   = std::stoi(args[6]) != 0;
  const bool include_braking_curves   = std::stoi(args[7]) != 0;
  const bool use_pwl                  = std::stoi(args[8]) != 0;
  const bool use_schedule_cuts        = std::stoi(args[9]) != 0;
  const int  timeout                  = std::stoi(args[10]);

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

  const std::string file_name =
      model_name + "_" + std::to_string(delta_t) + "_" +
      std::to_string(static_cast<int>(fix_routes)) + "_" +
      std::to_string(static_cast<int>(discretize_vss_positions)) + "_" +
      std::to_string(static_cast<int>(include_train_dynamics)) + "_" +
      std::to_string(static_cast<int>(include_braking_curves)) + "_" +
      std::to_string(static_cast<int>(use_pwl)) + "_" +
      std::to_string(static_cast<int>(use_schedule_cuts)) + "_" +
      std::to_string(timeout);

  cda_rail::vss::Model vss_model =
      discretize_vss_positions
          ? cda_rail::vss::Model(cda_rail::vss::ModelType::Discrete,
                                 {&cda_rail::vss::functions::uniform})
          : cda_rail::vss::Model(cda_rail::vss::ModelType::Continuous);

  solver.solve(delta_t, fix_routes, vss_model, include_train_dynamics,
               include_braking_curves, use_pwl, use_schedule_cuts, false,
               timeout, true,
               cda_rail::ExportOption::ExportSolutionWithInstance, file_name);
}
