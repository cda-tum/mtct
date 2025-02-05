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

  std::vector<std::tuple<double, double, double>> params = {
      {0.1, 0.001, 0.5}, {0.05, 0.001, 0.9}, {0.3, 0.01, 0.99}};

  for (std::tuple<double, double, double> param : params) {
    cda_rail::sim::ScoreHistoryCollection score_coll;
    std::mutex                            hist_mutex;

    std::vector<std::thread> workers;
    for (size_t process = 0; process < processor_count; process++) {
      workers.push_back(std::thread{[&]() {
        cda_rail::sim::RoutingSolver solver{instance};

        for (size_t sample = 0; sample < std::floor(100 / processor_count);
             sample++) {
          auto res = solver.random_local_search(
              std::chrono::seconds{10}, std::get<0>(param), std::get<1>(param),
              std::get<2>(param));

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

    score_coll.export_csv(
        output_path + "/score_hist_" +
        std::to_string(std::get<0>(param)).substr(0, 5) + "-" +
        std::to_string(std::get<1>(param)).substr(0, 5) + "-" +
        std::to_string((std::get<2>(param))).substr(0, 5) + ".csv");
  }
}

// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay,bugprone-exception-escape)
