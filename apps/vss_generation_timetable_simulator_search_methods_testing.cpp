#include "simulation/RoutingSolver.hpp"

#include <algorithm>
#include <filesystem>
#include <future>
#include <gsl/span>
#include <plog/Appenders/ColorConsoleAppender.h>
#include <plog/Formatters/TxtFormatter.h>
#include <plog/Initializers/ConsoleInitializer.h>
#include <plog/Log.h>
#include <string>
#include <thread>

// NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay,bugprone-exception-escape)

int main(int argc, char** argv) {
  auto args = gsl::span<char*>(argv, argc);

  const std::string model_path  = args[1];
  const std::string output_path = args[2];
  const std::string model_name =
      model_path.substr(model_path.find_last_of("/"), model_path.length());

  cda_rail::Network network =
      cda_rail::Network::import_network(model_path + "/network");
  cda_rail::Timetable timetable =
      cda_rail::Timetable::import_timetable(model_path + "/timetable", network);

  cda_rail::sim::SimulationInstance instance{network, timetable, false};

  size_t processor_count = std::thread::hardware_concurrency();
  if (processor_count == 0)
    processor_count = 1;

  cda_rail::sim::GeneticParams ga_params{
      .is_multithread = false,
      .population     = 1000,
      .gen_max        = 100,
      .stall_max      = 10,
      .n_elite        = 10,
      .xover_frac     = 0.99,
      .mut_rate       = 0.1,
  };

  cda_rail::sim::LocalParams loc_params{
      .start_sampling_range_fraction = 0.4,
      .abort_sampling_range_fraction = 0.001,
      .contraction_coeff             = 0.99,
  };

  std::vector<std::string> methods = {"random", "random+local", "greedy",
                                      "grasp", "genetic"};

  for (std::string method : methods) {
    cda_rail::sim::ScoreHistoryCollection score_coll;
    std::mutex                            hist_mutex;

    std::vector<std::thread> workers;
    for (size_t process = 0; process < processor_count; process++) {
      workers.push_back(std::thread{[&]() {
        cda_rail::sim::RoutingSolver solver{instance};

        int max_samples;
        if (method == "random" || method == "greedy" ||
            method == "random+local" || method == "grasp")
          max_samples = 10;
        else
          max_samples = 3;

        for (size_t sample = 0; sample < max_samples; sample++) {
          // Method here

          std::tuple<std::optional<cda_rail::sim::SolverResult>,
                     cda_rail::sim::ScoreHistory>
              res;
          if (method == "random") {
            res = solver.random_search(std::chrono::seconds{100}, {});
          } else if (method == "greedy") {
            res = solver.greedy_search(std::chrono::seconds{100}, {},
                                       {std::chrono::milliseconds{10}});
          } else if (method == "random+local") {
            res = solver.random_local_search(std::chrono::seconds{100},
                                             loc_params);
          } else if (method == "grasp") {
            res = solver.grasp_search(std::chrono::seconds{100},
                                      {std::chrono::milliseconds{50}},
                                      loc_params);
          } else if (method == "genetic") {
            res = solver.genetic_search(ga_params);
          } else if (method == "genetic+local") {
            res = solver.genetic_search(ga_params, true);
          }

          if (std::get<0>(res)) {
            const std::lock_guard<std::mutex> lock(hist_mutex);
            score_coll.add(std::get<1>(res));
            std::get<0>(res).value().get_trajectories().export_csv(
                output_path + "/results/methods/" + model_name + "/best_traj_" +
                method + ".csv");
          }

          std::cout << "Sample completed." << std::endl;
        }
      }});
    }

    while (workers.size() > 0) {
      workers.back().join();
      workers.pop_back();
    }

    cda_rail::is_directory_and_create(output_path + "/results/methods/" +
                                      model_name);
    score_coll.export_csv(output_path + "/results/methods/" + model_name +
                          "/score_hist_" + method + ".csv");
  }
}

// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay,bugprone-exception-escape)
