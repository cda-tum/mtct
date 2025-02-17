#include "simulation/RoutingSolver.hpp"

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
      .gen_max        = 20,
      .stall_max      = 5,
      .n_elite        = 100,
      .xover_frac     = 0.7,
      .mut_rate       = 0.1,
  };

  std::vector<double> xover_fractions = {0.1, 0.5, 0.7, 0.9};

  for (double xover_fraction : xover_fractions) {
    cda_rail::sim::ScoreHistoryCollection score_coll;
    std::mutex                            hist_mutex;

    ga_params.xover_frac = xover_fraction;

    std::vector<std::thread> workers;
    for (size_t process = 0; process < processor_count; process++) {
      workers.push_back(std::thread{[&]() {
        cda_rail::sim::RoutingSolver solver{instance};

        for (size_t sample_runs = 0; sample_runs < 1; sample_runs++) {
          auto res = solver.genetic_search(ga_params);

          if (std::get<0>(res)) {
            const std::lock_guard<std::mutex> lock(hist_mutex);
            score_coll.add(std::get<1>(res));
          }
        }
      }});
    }

    while (workers.size() > 0) {
      workers.back().join();
      workers.pop_back();
    }

    score_coll.export_csv(output_path + "/score_hist_" +
                          std::to_string(xover_fraction).substr(0, 5) + ".csv");
  }
}

// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay,bugprone-exception-escape)
