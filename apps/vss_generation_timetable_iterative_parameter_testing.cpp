#include "Definitions.hpp"
#include "VSSModel.hpp"
#include "solver/mip-based/VSSGenTimetableSolver.hpp"

#include <gsl/span>

int main(int argc, char** argv) {
  if (argc < 11 || argc > 12) {
    std::cout << "Expected 10 or 11 arguments, got " << argc - 1 << std::endl;
    std::exit(-1);
  }

  auto args = gsl::span<char*>(argv, argc);

  const std::string                                  model_name    = args[1];
  const std::string                                  instance_path = args[2];
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(instance_path);

  std::cout << "Instance " << model_name << " loaded at " << instance_path
            << std::endl;

  const int         delta_t                = std::stoi(args[3]);
  const bool        fix_routes             = std::stoi(args[4]) != 0;
  const bool        include_braking_curves = std::stoi(args[5]) != 0;
  const bool        iterate_vss            = std::stoi(args[6]) != 0;
  const int         initial_vss            = std::stoi(args[7]);
  const int         factor                 = std::stoi(args[8]);
  const bool        include_cuts           = std::stoi(args[9]) != 0;
  const int         timeout                = std::stoi(args[10]);
  const std::string output_path            = (argc == 12 ? args[11] : "");

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
    std::cout << "      and factor " << factor << std::endl;
    if (include_cuts) {
      std::cout << "      and cuts are used" << std::endl;
    }
  }
  std::cout << "   timeout: " << timeout << "s" << std::endl;

  const std::string file_name =
      model_name + "_" + std::to_string(delta_t) + "_" +
      std::to_string(static_cast<int>(fix_routes)) + "_" +
      std::to_string(static_cast<int>(include_braking_curves)) + "_" +
      std::to_string(static_cast<int>(iterate_vss)) + "_" +
      std::to_string(initial_vss) + "_" + std::to_string(factor) + "_" +
      std::to_string(static_cast<int>(include_cuts)) + "_" +
      std::to_string(timeout);

  cda_rail::vss::Model vss_model(cda_rail::vss::ModelType::Continuous);

  solver.solve(delta_t, fix_routes, vss_model, true, include_braking_curves,
               false, true,
               {iterate_vss, cda_rail::OptimalityStrategy::Optimal, initial_vss,
                static_cast<double>(factor), include_cuts},
               false, timeout, true, cda_rail::ExportOption::ExportSolution,
               file_name, output_path);
}
