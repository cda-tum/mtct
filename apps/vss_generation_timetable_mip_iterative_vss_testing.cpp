#include "Definitions.hpp"
#include "VSSModel.hpp"
#include "solver/mip-based/VSSGenTimetableSolver.hpp"

#include <gsl/span>

int main(int argc, char** argv) {
  if (argc < 12 || argc > 13) {
    std::cerr << "Expected 11 or 12 arguments, got " << argc - 1 << std::endl;
    std::exit(-1);
  }

  auto args = gsl::span<char*>(argv, argc);

  const std::string                                  model_name    = args[1];
  const std::string                                  instance_path = args[2];
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(instance_path);

  std::cout << "Instance " << model_name << " loaded at " << instance_path
            << std::endl;

  const int  delta_t                 = std::stoi(args[3]);
  const bool fix_routes              = std::stoi(args[4]) != 0;
  const bool include_train_dynamics  = std::stoi(args[5]) != 0;
  const bool include_braking_curves  = std::stoi(args[6]) != 0;
  const bool use_pwl                 = std::stoi(args[7]) != 0;
  const bool use_schedule_cuts       = std::stoi(args[8]) != 0;
  const bool iterate_vss             = std::stoi(args[9]) != 0;
  const int  optimality_strategy_int = std::stoi(args[10]);
  const auto optimality_strategy =
      static_cast<cda_rail::OptimalityStrategy>(optimality_strategy_int);
  const int         timeout     = std::stoi(args[11]);
  const std::string output_path = (argc == 13 ? args[12] : "");

  std::cout << "The following parameters were passed to the toolkit:"
            << std::endl;
  std::cout << "   delta_t: " << delta_t << std::endl;
  if (fix_routes) {
    std::cout << "   routes are fixed" << std::endl;
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
  if (iterate_vss) {
    std::cout << "   VSS is iterated" << std::endl;
  }
  if (optimality_strategy == cda_rail::OptimalityStrategy::Optimal) {
    std::cout << "   optimality strategy: optimal" << std::endl;
  } else if (optimality_strategy == cda_rail::OptimalityStrategy::TradeOff) {
    std::cout << "   optimality strategy: trade-off" << std::endl;
  } else if (optimality_strategy == cda_rail::OptimalityStrategy::Feasible) {
    std::cout << "   optimality strategy: feasible" << std::endl;
  } else {
    std::cout << "   optimality strategy: unknown" << std::endl;
  }
  std::cout << "   timeout: " << timeout << "s" << std::endl;

  const std::string file_name =
      model_name + "_" + std::to_string(delta_t) + "_" +
      std::to_string(static_cast<int>(fix_routes)) + "_" +
      std::to_string(static_cast<int>(include_train_dynamics)) + "_" +
      std::to_string(static_cast<int>(include_braking_curves)) + "_" +
      std::to_string(static_cast<int>(use_pwl)) + "_" +
      std::to_string(static_cast<int>(use_schedule_cuts)) + "_" +
      std::to_string(static_cast<int>(iterate_vss)) + "_" +
      std::to_string(static_cast<int>(optimality_strategy_int)) + "_" +
      std::to_string(timeout);

  cda_rail::vss::Model vss_model(cda_rail::vss::ModelType::Continuous);

  solver.solve(
      {delta_t, fix_routes, include_train_dynamics, include_braking_curves},
      {vss_model, use_pwl, use_schedule_cuts},
      {iterate_vss, optimality_strategy},
      {false, cda_rail::ExportOption::ExportSolution, file_name, output_path},
      timeout, true);
}
