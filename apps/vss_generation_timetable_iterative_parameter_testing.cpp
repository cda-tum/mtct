#include "Definitions.hpp"
#include "VSSModel.hpp"
#include "solver/mip-based/VSSGenTimetableSolver.hpp"

#include <gsl/span>

int main(int argc, char** argv) {
  if (argc < 13 || argc > 15) {
    std::cout << "Expected 12 or 13 or 14 arguments, got " << argc - 1
              << std::endl;
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
  const bool include_braking_curves  = std::stoi(args[5]) != 0;
  const bool iterate_vss             = std::stoi(args[6]) != 0;
  const int  optimality_strategy_int = std::stoi(args[7]);
  const auto optimality_strategy =
      static_cast<cda_rail::OptimalityStrategy>(optimality_strategy_int);
  const int  update_strategy_int = std::stoi(args[8]);
  const auto update_strategy =
      static_cast<cda_rail::solver::mip_based::UpdateStrategy>(
          update_strategy_int);
  const double      initial_vss  = std::stod(args[9]);
  const double      update_value = std::stod(args[10]);
  const bool        include_cuts = std::stoi(args[11]) != 0;
  const int         timeout      = std::stoi(args[12]);
  const std::string output_path  = (argc >= 14 ? args[13] : "");
  const std::string file_name =
      (argc >= 15
           ? args[14]
           : model_name + "_" + std::to_string(delta_t) + "_" +
                 std::to_string(static_cast<int>(fix_routes)) + "_" +
                 std::to_string(static_cast<int>(include_braking_curves)) +
                 "_" + std::to_string(static_cast<int>(iterate_vss)) + "_" +
                 std::to_string(optimality_strategy_int) + "_" +
                 std::to_string(update_strategy_int) + "_" +
                 std::to_string(initial_vss) + "_" +
                 std::to_string(update_value) + "_" +
                 std::to_string(static_cast<int>(include_cuts)) + "_" +
                 std::to_string(timeout));

  std::cout << "The following parameters were passed to the toolkit:"
            << std::endl;
  std::cout << "   delta_t: " << delta_t << std::endl;
  if (fix_routes) {
    std::cout << "   routes are fixed" << std::endl;
  }
  if (include_braking_curves) {
    std::cout << "   braking distance is included" << std::endl;
  }
  if (iterate_vss) {
    std::cout << "   VSS is iterated to optimality" << std::endl;
    std::cout << "      using initial value " << initial_vss << std::endl;
    std::cout << "      and update value " << update_value << std::endl;
    if (update_strategy == cda_rail::solver::mip_based::UpdateStrategy::Fixed) {
      std::cout << "      with fixed update strategy" << std::endl;
    } else if (update_strategy ==
               cda_rail::solver::mip_based::UpdateStrategy::Relative) {
      std::cout << "      with relative update strategy" << std::endl;
    } else {
      std::cout << "      with unknown update strategy" << std::endl;
    }
    if (include_cuts) {
      std::cout << "      and cuts are used" << std::endl;
    }
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
  std::cout << "   output path: " << output_path << std::endl;
  std::cout << "   file name: " << file_name << std::endl;

  cda_rail::vss::Model vss_model(cda_rail::vss::ModelType::Continuous);

  solver.solve(delta_t, fix_routes, vss_model, true, include_braking_curves,
               false, true,
               {iterate_vss, optimality_strategy, update_strategy, initial_vss,
                update_value, include_cuts},
               false, timeout, true, cda_rail::ExportOption::ExportSolution,
               file_name, output_path);
}
