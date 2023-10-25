#include "Definitions.hpp"
#include "VSSModel.hpp"
#include "solver/mip-based/VSSGenTimetableSolver.hpp"

#include <gsl/span>

int main(int argc, char** argv) {
  if (argc < 13 || argc > 14) {
    std::cout << "Expected 12 or 13 arguments, got " << argc - 1 << std::endl;
    std::exit(-1);
  }

  auto args = gsl::span<char*>(argv, argc);

  const std::string                                  model_name    = args[1];
  const std::string                                  instance_path = args[2];
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(instance_path);

  std::cout << "Instance " << model_name << " loaded at " << instance_path
            << std::endl;

  const int         delta_t                 = std::stoi(args[3]);
  const bool        fix_routes              = std::stoi(args[4]) != 0;
  const std::string type                    = args[5];
  const bool        include_train_dynamics  = std::stoi(args[6]) != 0;
  const bool        include_braking_curves  = std::stoi(args[7]) != 0;
  const bool        use_pwl                 = std::stoi(args[8]) != 0;
  const bool        use_schedule_cuts       = std::stoi(args[9]) != 0;
  const bool        iterate_vss             = std::stoi(args[10]) != 0;
  const int         timeout                 = std::stoi(args[11]);
  const int         optimality_strategy_int = std::stoi(args[12]);
  const auto        optimality_strategy =
      static_cast<cda_rail::OptimalityStrategy>(optimality_strategy_int);
  const std::string output_path = (argc == 14 ? args[13] : "");

  std::cout << "The following parameters were passed to the toolkit:"
            << std::endl;
  std::cout << "   delta_t: " << delta_t << std::endl;
  if (fix_routes) {
    std::cout << "   routes are fixed" << std::endl;
  }
  std::cout << "   type is " << type << std::endl;
  std::vector<cda_rail::vss::SeparationFunction> sep_functions;
  if (type == "uniform" || type == "uniform_alt") {
    std::cout << "   uniform separation functions are used" << std::endl;
    sep_functions.emplace_back(&cda_rail::vss::functions::uniform);
  } else if (type == "chebyshev" || type == "chebyshev_alt") {
    std::cout << "   Chebyshev separation functions are used" << std::endl;
    sep_functions.emplace_back(&cda_rail::vss::functions::chebyshev);
  } else if (type == "both" || type == "both_alt") {
    std::cout << "   uniform and Chebyshev separation functions are used"
              << std::endl;
    sep_functions.emplace_back(&cda_rail::vss::functions::uniform);
    sep_functions.emplace_back(&cda_rail::vss::functions::chebyshev);
  } else if (type == "continuous") {
    std::cout << "   no separation functions are used" << std::endl;
  } else {
    std::cout << "   unknown separation functions are used" << std::endl;
    std::exit(-1);
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
  std::cout << "   timeout: " << timeout << "s" << std::endl;

  const std::string file_name =
      model_name + "_" + std::to_string(delta_t) + "_" +
      std::to_string(static_cast<int>(fix_routes)) + "_" + type + "_" +
      std::to_string(static_cast<int>(include_train_dynamics)) + "_" +
      std::to_string(static_cast<int>(include_braking_curves)) + "_" +
      std::to_string(static_cast<int>(use_pwl)) + "_" +
      std::to_string(static_cast<int>(use_schedule_cuts)) + "_" +
      std::to_string(static_cast<int>(iterate_vss)) + "_" +
      std::to_string(timeout);

  std::optional<cda_rail::vss::Model> vss_model;
  if (type == "continuous") {
    vss_model =
        cda_rail::vss::Model(cda_rail::vss::ModelType::Continuous, {}, true);
  } else if (type.length() >= 4 && type.substr(type.length() - 4) == "_alt") {
    vss_model = cda_rail::vss::Model(cda_rail::vss::ModelType::InferredAlt,
                                     sep_functions, true);
  } else {
    vss_model = cda_rail::vss::Model(cda_rail::vss::ModelType::Inferred,
                                     sep_functions, true);
  }

  solver.solve(delta_t, fix_routes, vss_model.value(), include_train_dynamics,
               include_braking_curves, use_pwl, use_schedule_cuts, iterate_vss,
               cda_rail::OptimalityStrategy::Optimal, false, timeout, true,
               cda_rail::ExportOption::ExportSolutionWithInstance, file_name,
               output_path);
}
