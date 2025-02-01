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
  // Only log to console using std::cerr and std::cout respectively unless
  // initialized differently
  if (plog::get() == nullptr) {
    static plog::ColorConsoleAppender<plog::TxtFormatter> console_appender;
    plog::init(plog::debug, &console_appender);
  }

  if (strcmp(argv[1], "--help") == 0 || strcmp(argv[1], "-h") == 0 ||
      argc != 3) {
    PLOGI << "Usage: vss_generation_timetable_simulator [NETWORK PATH] [OUTPUT "
             "PATH]"
          << std::endl;
    std::exit(0);
  }

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

  for (size_t train_to = 10; train_to <= 1000; train_to = train_to * 2) {
    cda_rail::sim::ScoreHistoryCollection score_coll;
    std::mutex                            hist_mutex;

    std::vector<std::thread> workers;
    for (size_t process = 0; process < processor_count; process++) {
      workers.push_back(std::thread{[&]() {
        cda_rail::sim::RoutingSolver solver{instance};

        for (size_t sample = 0; sample < std::floor(100 / processor_count);
             sample++) {
          auto res = solver.greedy_search(std::chrono::seconds{4},
                                          std::chrono::milliseconds{train_to});

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
                          std::to_string(train_to) + ".csv");
  }
}

// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay,bugprone-exception-escape)
