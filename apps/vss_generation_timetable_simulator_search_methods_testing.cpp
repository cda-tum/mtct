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

  std::vector<std::string> methods = {"random", "greedy", "random+local",
                                      "greedy+local_grasp"};

  for (std::string method : methods) {
    cda_rail::sim::ScoreHistoryCollection score_coll;
    std::mutex                            hist_mutex;

    std::vector<std::thread> workers;
    for (size_t process = 0; process < processor_count; process++) {
      workers.push_back(std::thread{[&]() {
        cda_rail::sim::RoutingSolver solver{instance};

        for (size_t sample = 0; sample < std::floor(100 / processor_count);
             sample++) {
          // Method here

          std::tuple<std::optional<cda_rail::sim::SolverResult>,
                     cda_rail::sim::ScoreHistory>
              res;
          if (method == "random") {
            res = solver.random_search(std::chrono::seconds{10}, {});
          } else if (method == "greedy") {
            res = solver.greedy_search(std::chrono::seconds{10}, {},
                                       std::chrono::milliseconds{50});
          } else if (method == "random+local") {
            res = solver.random_local_search(std::chrono::seconds{10}, 0.1,
                                             1e-4, 0.95);
          } else if (method == "greedy+local_grasp") {
            res = solver.grasp_search(std::chrono::seconds{10},
                                      std::chrono::milliseconds{50}, 0.1, 1e-4,
                                      0.95);
          }

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

    score_coll.export_csv(output_path + "/score_hist_" + method + ".csv");
  }
}

// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay,bugprone-exception-escape)
