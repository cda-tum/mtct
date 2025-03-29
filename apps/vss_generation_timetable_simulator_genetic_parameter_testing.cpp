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
      .gen_max        = 20,
      .stall_max      = 5,
      .n_elite        = 10,
      .xover_frac     = 0.7,
      .mut_rate       = 0.1,
  };

  {
    std::vector<double> xover_fractions = {0.01, 0.025, 0.1, 0.5, 0.7, 0.99};

    for (double xover_fraction : xover_fractions) {
      cda_rail::sim::ScoreHistoryCollection score_coll;
      std::mutex                            hist_mutex;

      ga_params.xover_frac = xover_fraction;

      std::vector<std::thread> workers;
      for (size_t process = 0; process < processor_count; process++) {
        workers.push_back(std::thread{[&]() {
          cda_rail::sim::RoutingSolver solver{instance};

          for (size_t sample_runs = 0; sample_runs < 3; sample_runs++) {
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

      auto save_path =
          output_path + "/results/genetic_params/crossover/" + model_name;
      cda_rail::is_directory_and_create(save_path);
      score_coll.export_csv(save_path + "/score_hist_" +
                            std::to_string(xover_fraction).substr(0, 5) +
                            ".csv");
    }
  }

  {
    std::vector<double> mut_rates = {0.01, 0.1, 0.25, 0.5, 0.75, 0.9, 0.99};

    for (double mut_rate : mut_rates) {
      cda_rail::sim::ScoreHistoryCollection score_coll;
      std::mutex                            hist_mutex;

      ga_params.mut_rate = mut_rate;

      std::vector<std::thread> workers;
      for (size_t process = 0; process < processor_count; process++) {
        workers.push_back(std::thread{[&]() {
          cda_rail::sim::RoutingSolver solver{instance};

          for (size_t sample_runs = 0; sample_runs < 3; sample_runs++) {
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

      auto save_path =
          output_path + "/results/genetic_params/mut_rate/" + model_name;
      cda_rail::is_directory_and_create(save_path);
      score_coll.export_csv(save_path + "/score_hist_" +
                            std::to_string(mut_rate).substr(0, 5) + ".csv");
    }
  }

  {
    std::vector<size_t> pops = {10, 100, 1000};

    for (size_t pop : pops) {
      cda_rail::sim::ScoreHistoryCollection score_coll;
      std::mutex                            hist_mutex;

      ga_params.population = pop;
      ga_params.n_elite    = (size_t)std::round(0.1 * pop);

      std::vector<std::thread> workers;
      for (size_t process = 0; process < processor_count; process++) {
        workers.push_back(std::thread{[&]() {
          cda_rail::sim::RoutingSolver solver{instance};

          for (size_t sample_runs = 0; sample_runs < 3; sample_runs++) {
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

      auto save_path =
          output_path + "/results/genetic_params/pop/" + model_name;
      cda_rail::is_directory_and_create(save_path);
      score_coll.export_csv(save_path + "/score_hist_" +
                            std::to_string(pop).substr(0, 5) + ".csv");
    }
  }

  {
    std::vector<double> elites = {0.01, 0.05, 0.1, 0.25, 0.5};

    for (double elite : elites) {
      cda_rail::sim::ScoreHistoryCollection score_coll;
      std::mutex                            hist_mutex;

      ga_params.n_elite =
          (size_t)std::round(elite * (double)ga_params.population);

      std::vector<std::thread> workers;
      for (size_t process = 0; process < processor_count; process++) {
        workers.push_back(std::thread{[&]() {
          cda_rail::sim::RoutingSolver solver{instance};

          for (size_t sample_runs = 0; sample_runs < 3; sample_runs++) {
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

      auto save_path =
          output_path + "/results/genetic_params/elite/" + model_name;
      cda_rail::is_directory_and_create(save_path);
      score_coll.export_csv(save_path + "/score_hist_" +
                            std::to_string(elite).substr(0, 5) + ".csv");
    }
  }

  {
    std::vector<bool> multithreads = {false, true};

    for (bool multithread : multithreads) {
      cda_rail::sim::ScoreHistoryCollection score_coll;
      std::mutex                            hist_mutex;

      ga_params.is_multithread = multithread;

      cda_rail::sim::RoutingSolver solver{instance};

      for (size_t sample_runs = 0; sample_runs < 5; sample_runs++) {
        auto res = solver.genetic_search(ga_params);
        score_coll.add(std::get<1>(res));
      }

      auto save_path =
          output_path + "/results/genetic_params/multithread/" + model_name;
      cda_rail::is_directory_and_create(save_path);
      score_coll.export_csv(save_path + "/score_hist_" +
                            std::to_string(multithread) + ".csv");
    }
  }
}

// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay,bugprone-exception-escape)
