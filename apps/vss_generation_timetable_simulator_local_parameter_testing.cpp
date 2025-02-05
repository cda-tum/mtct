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

  std::vector<double> contr_coeffs = {0.5, 0.7, 0.9, 0.95, 0.99};

  for (double contr_coeff : contr_coeffs) {
    cda_rail::sim::ScoreHistoryCollection score_coll;
    std::mutex                            hist_mutex;

    std::vector<std::thread> workers;
    for (size_t process = 0; process < processor_count; process++) {
      workers.push_back(std::thread{[&]() {
        cda_rail::sim::RoutingSolver solver{instance};

        for (size_t sample = 0; sample < std::floor(100 / processor_count);
             sample++) {
          auto res = solver.random_local_search(std::chrono::seconds{8}, 0.1,
                                                1e-4, contr_coeff);

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
                          std::to_string(contr_coeff) + ".csv");
  }

  std::vector<double> stop_sampl_fracs = {1e-2, 1e-3, 1e-4, 1e-5};

  for (double stop_sampl_frac : stop_sampl_fracs) {
    cda_rail::sim::ScoreHistoryCollection score_coll;
    std::mutex                            hist_mutex;

    std::vector<std::thread> workers;
    for (size_t process = 0; process < processor_count; process++) {
      workers.push_back(std::thread{[&]() {
        cda_rail::sim::RoutingSolver solver{instance};

        for (size_t sample = 0; sample < std::floor(100 / processor_count);
             sample++) {
          auto res = solver.random_local_search(std::chrono::seconds{8}, 0.1,
                                                stop_sampl_frac, 0.95);

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
                          std::to_string(stop_sampl_frac) + ".csv");
  }

  std::vector<double> start_sampl_fracs = {0.001, 0.01, 0.1, 0.5};

  for (double start_sampl_frac : start_sampl_fracs) {
    cda_rail::sim::ScoreHistoryCollection score_coll;
    std::mutex                            hist_mutex;

    std::vector<std::thread> workers;
    for (size_t process = 0; process < processor_count; process++) {
      workers.push_back(std::thread{[&]() {
        cda_rail::sim::RoutingSolver solver{instance};

        for (size_t sample = 0; sample < std::floor(100 / processor_count);
             sample++) {
          auto res = solver.random_local_search(std::chrono::seconds{8},
                                                start_sampl_frac, 1e-3, 0.95);

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
                          std::to_string(start_sampl_frac) + ".csv");
  }
}

// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay,bugprone-exception-escape)
