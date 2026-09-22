#include "CLI/CLI.hpp"
#include "CommandInterpreter.hpp"
#include "Formatting.hpp"
#include "Session.hpp"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"

#include <iostream>
#include <memory>
#include <string>

namespace {
using cda_rail::cli::format_double;
using cda_rail::cli::Session;

void list_stations(const Session& session) {
  const auto& instance = session.const_instance();
  const auto& network  = instance.get_const_network();
  for (const auto& [name, station] : instance.get_const_station_list()) {
    std::cout << name << ':';
    if (station->tracks.empty()) {
      std::cout << " (no tracks)";
    }
    for (const auto& track : station->tracks) {
      std::cout << ' ' << network.get_edge_name(track);
    }
    std::cout << '\n';
  }
}

void list_trains(const Session& session) {
  const auto& instance = session.const_instance();
  const auto& network  = instance.get_const_network();
  for (const auto& train : instance.get_const_train_list()) {
    const auto& schedule = instance.get_const_schedule(train.get_name());
    std::cout << train.get_name() << ": length "
              << format_double(train.get_length()) << " m, max speed "
              << format_double(train.get_max_speed()) << " m/s, a "
              << format_double(train.get_acceleration()) << " m/s^2, d "
              << format_double(train.get_deceleration()) << " m/s^2, tim "
              << (train.has_tim() ? "yes" : "no") << ", weight "
              << format_double(instance.get_train_weight(train.get_name()))
              << '\n';
    std::cout << "  enters at "
              << network.get_vertex(schedule.get_entry_vertex()).name
              << " at t = " << format_double(schedule.get_entry_time())
              << " s with v = "
              << format_double(schedule.get_initial_velocity()) << " m/s\n";
    std::cout << "  leaves at "
              << network.get_vertex(schedule.get_exit_vertex()).name
              << " at t = " << format_double(schedule.get_exit_time())
              << " s with v = " << format_double(schedule.get_exit_velocity())
              << " m/s\n";
    for (const auto& stop : schedule.get_stops()) {
      std::cout << "  stops at " << stop.get_station().name
                << " from t = " << format_double(stop.get_service_time())
                << " s for " << format_double(stop.get_service_duration())
                << " s\n";
    }
  }
}
} // namespace

void cda_rail::cli::add_timetable_commands(CLI::App& app, Session& session) {
  // --------------------------------------------------------------- stations

  auto* station_cmd =
      app.add_subcommand("station", "Add stations and assign tracks to them");
  station_cmd->require_subcommand(1);

  auto* station_add_cmd =
      station_cmd->add_subcommand("add", "Add an empty station");
  auto station_name = std::make_shared<std::string>();
  station_add_cmd->add_option("name", *station_name, "Name of the station")
      ->required();
  station_add_cmd->callback([&session, station_name]() {
    session.instance().add_empty_station(*station_name);
    session.mark_instance_modified();
    std::cout << "Added station " << *station_name << '\n';
  });

  auto* station_track_cmd = station_cmd->add_subcommand(
      "add-track", "Add an edge as a track of a station");
  struct TrackOptions {
    std::string station;
    std::string source;
    std::string target;
  };
  auto track_options = std::make_shared<TrackOptions>();
  station_track_cmd
      ->add_option("station", track_options->station, "Name of the station")
      ->required();
  station_track_cmd
      ->add_option("source", track_options->source,
                   "Name of the source vertex of the track")
      ->required();
  station_track_cmd
      ->add_option("target", track_options->target,
                   "Name of the target vertex of the track")
      ->required();
  station_track_cmd->callback([&session, track_options]() {
    session.instance().add_track_to_station(
        track_options->station,
        edge_input(track_options->source, track_options->target));
    session.mark_instance_modified();
    std::cout << "Added track " << track_options->source << "-"
              << track_options->target << " to station "
              << track_options->station << '\n';
  });

  station_cmd->add_subcommand("list", "List all stations and their tracks")
      ->callback([&session]() { list_stations(session); });

  // ----------------------------------------------------------------- trains

  auto* train_cmd = app.add_subcommand(
      "train", "Add trains, change their properties, schedules and stops");
  train_cmd->require_subcommand(1);

  auto* train_add_cmd = train_cmd->add_subcommand(
      "add", "Add a train together with its schedule");
  struct TrainAddOptions {
    std::string name;
    double      length{0};
    double      max_speed{0};
    double      acceleration{0};
    double      deceleration{0};
    bool        tim{true};
    std::string entry_vertex;
    double      entry_time{0};
    double      initial_velocity{0};
    std::string exit_vertex;
    double      exit_time{0};
    double      exit_velocity{0};
    double      weight{1};
  };
  auto train_add = std::make_shared<TrainAddOptions>();
  train_add_cmd->add_option("name", train_add->name, "Name of the train")
      ->required();
  train_add_cmd
      ->add_option("-l,--length", train_add->length, "Length of the train in m")
      ->check(CLI::NonNegativeNumber)
      ->required();
  train_add_cmd
      ->add_option("-v,--max-speed", train_add->max_speed,
                   "Maximal speed of the train in m/s")
      ->check(CLI::PositiveNumber)
      ->required();
  train_add_cmd
      ->add_option("-a,--acceleration", train_add->acceleration,
                   "Maximal acceleration of the train in m/s^2")
      ->check(CLI::PositiveNumber)
      ->required();
  train_add_cmd
      ->add_option("-d,--deceleration", train_add->deceleration,
                   "Maximal deceleration of the train in m/s^2")
      ->check(CLI::PositiveNumber)
      ->required();
  train_add_cmd
      ->add_flag("!-p,!--no-tim,--tim", train_add->tim,
                 "The train has train integrity monitoring by default. If "
                 "this flag is negated (-p or --no-tim), it does not.")
      ->capture_default_str();
  train_add_cmd
      ->add_option("-e,--entry-vertex", train_add->entry_vertex,
                   "Name of the vertex at which the train enters the network")
      ->required();
  train_add_cmd
      ->add_option("-t,--entry-time", train_add->entry_time,
                   "Earliest time in s at which the train enters the network")
      ->check(CLI::NonNegativeNumber)
      ->capture_default_str();
  train_add_cmd
      ->add_option("-i,--initial-velocity", train_add->initial_velocity,
                   "Velocity in m/s at which the train enters the network")
      ->check(CLI::NonNegativeNumber)
      ->capture_default_str();
  train_add_cmd
      ->add_option("-x,--exit-vertex", train_add->exit_vertex,
                   "Name of the vertex at which the train leaves the network")
      ->required();
  train_add_cmd
      ->add_option("-y,--exit-time", train_add->exit_time,
                   "Desired time in s at which the train leaves the network")
      ->check(CLI::NonNegativeNumber)
      ->required();
  train_add_cmd
      ->add_option("-z,--exit-velocity", train_add->exit_velocity,
                   "Desired velocity in m/s at which the train leaves the "
                   "network")
      ->check(CLI::NonNegativeNumber)
      ->capture_default_str();
  train_add_cmd
      ->add_option("-w,--weight", train_add->weight,
                   "Weight of the train in the objective function")
      ->check(CLI::NonNegativeNumber)
      ->capture_default_str();
  train_add_cmd->callback([&session, train_add]() {
    const auto index = session.instance().add_train(
        train_add->name, train_add->length, train_add->max_speed,
        train_add->acceleration, train_add->deceleration, train_add->tim,
        train_add->entry_time, train_add->initial_velocity,
        vertex_input(train_add->entry_vertex), train_add->exit_time,
        train_add->exit_velocity, vertex_input(train_add->exit_vertex),
        train_add->weight);
    session.mark_instance_modified();
    std::cout << "Added train " << train_add->name << " with index " << index
              << '\n';
  });

  auto* train_change_cmd = train_cmd->add_subcommand(
      "change", "Change the properties of an existing train");
  struct TrainChangeOptions {
    std::string name;
    double      length{0};
    double      max_speed{0};
    double      acceleration{0};
    double      deceleration{0};
    bool        tim{true};
  };
  auto train_change = std::make_shared<TrainChangeOptions>();
  train_change_cmd->add_option("name", train_change->name, "Name of the train")
      ->required();
  auto* change_length_opt =
      train_change_cmd
          ->add_option("-l,--length", train_change->length, "New length in m")
          ->check(CLI::NonNegativeNumber);
  auto* change_speed_opt =
      train_change_cmd
          ->add_option("-v,--max-speed", train_change->max_speed,
                       "New maximal speed in m/s")
          ->check(CLI::PositiveNumber);
  auto* change_acceleration_opt =
      train_change_cmd
          ->add_option("-a,--acceleration", train_change->acceleration,
                       "New maximal acceleration in m/s^2")
          ->check(CLI::PositiveNumber);
  auto* change_deceleration_opt =
      train_change_cmd
          ->add_option("-d,--deceleration", train_change->deceleration,
                       "New maximal deceleration in m/s^2")
          ->check(CLI::PositiveNumber);
  auto* change_tim_opt = train_change_cmd->add_flag(
      "!-p,!--no-tim,--tim", train_change->tim,
      "Give the train train integrity monitoring, or take it away if the flag "
      "is negated (-p or --no-tim)");
  train_change_cmd->callback([&session, train_change, change_length_opt,
                              change_speed_opt, change_acceleration_opt,
                              change_deceleration_opt, change_tim_opt]() {
    auto& train = session.instance().editable_train(train_change->name);
    if (change_length_opt->count() > 0) {
      train.set_length(train_change->length);
    }
    if (change_speed_opt->count() > 0) {
      train.set_max_speed(train_change->max_speed);
    }
    if (change_acceleration_opt->count() > 0) {
      train.set_acceleration(train_change->acceleration);
    }
    if (change_deceleration_opt->count() > 0) {
      train.set_deceleration(train_change->deceleration);
    }
    if (change_tim_opt->count() > 0) {
      train.set_tim_value(train_change->tim);
    }
    session.mark_instance_modified();
    std::cout << "Changed train " << train_change->name << '\n';
  });

  auto* schedule_cmd = train_cmd->add_subcommand(
      "schedule", "Change the entry and exit data of a train");
  struct ScheduleOptions {
    std::string name;
    std::string entry_vertex;
    double      entry_time{0};
    double      initial_velocity{0};
    std::string exit_vertex;
    double      exit_time{0};
    double      exit_velocity{0};
  };
  auto schedule = std::make_shared<ScheduleOptions>();
  schedule_cmd->add_option("name", schedule->name, "Name of the train")
      ->required();
  auto* entry_vertex_opt = schedule_cmd->add_option(
      "-e,--entry-vertex", schedule->entry_vertex, "New entry vertex");
  auto* entry_time_opt =
      schedule_cmd
          ->add_option("-t,--entry-time", schedule->entry_time,
                       "New entry time in s")
          ->check(CLI::NonNegativeNumber);
  auto* initial_velocity_opt =
      schedule_cmd
          ->add_option("-i,--initial-velocity", schedule->initial_velocity,
                       "New entry velocity in m/s")
          ->check(CLI::NonNegativeNumber);
  auto* exit_vertex_opt = schedule_cmd->add_option(
      "-x,--exit-vertex", schedule->exit_vertex, "New exit vertex");
  auto* exit_time_opt = schedule_cmd
                            ->add_option("-y,--exit-time", schedule->exit_time,
                                         "New exit time in s")
                            ->check(CLI::NonNegativeNumber);
  auto* exit_velocity_opt =
      schedule_cmd
          ->add_option("-z,--exit-velocity", schedule->exit_velocity,
                       "New exit velocity in m/s")
          ->check(CLI::NonNegativeNumber);
  schedule_cmd->callback([&session, schedule, entry_vertex_opt, entry_time_opt,
                          initial_velocity_opt, exit_vertex_opt, exit_time_opt,
                          exit_velocity_opt]() {
    auto&       instance = session.instance();
    const auto& network  = instance.get_const_network();
    // The vertices are resolved before the schedule is touched, so that an
    // unknown name leaves the schedule unchanged.
    const auto entry_vertex =
        entry_vertex_opt->count() > 0
            ? network.get_vertex_index(schedule->entry_vertex)
            : 0;
    const auto exit_vertex =
        exit_vertex_opt->count() > 0
            ? network.get_vertex_index(schedule->exit_vertex)
            : 0;
    auto& train_schedule = instance.editable_schedule(schedule->name);
    if (entry_vertex_opt->count() > 0) {
      train_schedule.set_entry_vertex(entry_vertex, network);
    }
    if (exit_vertex_opt->count() > 0) {
      train_schedule.set_exit_vertex(exit_vertex, network);
    }
    // The setters enforce entry time <= exit time, so the two are written in
    // whichever order keeps that invariant intact in between.
    const double new_entry_time = entry_time_opt->count() > 0
                                      ? schedule->entry_time
                                      : train_schedule.get_entry_time();
    const double new_exit_time  = exit_time_opt->count() > 0
                                      ? schedule->exit_time
                                      : train_schedule.get_exit_time();
    if (new_entry_time <= train_schedule.get_exit_time()) {
      train_schedule.set_entry_time(new_entry_time);
      train_schedule.set_exit_time(new_exit_time);
    } else {
      train_schedule.set_exit_time(new_exit_time);
      train_schedule.set_entry_time(new_entry_time);
    }
    if (initial_velocity_opt->count() > 0) {
      train_schedule.set_initial_velocity(schedule->initial_velocity);
    }
    if (exit_velocity_opt->count() > 0) {
      train_schedule.set_exit_velocity(schedule->exit_velocity);
    }
    session.mark_instance_modified();
    std::cout << "Changed the schedule of train " << schedule->name << '\n';
  });

  // ------------------------------------------------------------------ stops

  auto* stop_cmd =
      train_cmd->add_subcommand("stop", "Add and remove scheduled stops");
  stop_cmd->require_subcommand(1);

  auto* stop_add_cmd =
      stop_cmd->add_subcommand("add", "Add a stop to the schedule of a train");
  struct StopOptions {
    std::string train;
    std::string station;
    double      service_time{0};
    double      duration{0};
  };
  auto stop_add = std::make_shared<StopOptions>();
  stop_add_cmd->add_option("train", stop_add->train, "Name of the train")
      ->required();
  stop_add_cmd->add_option("station", stop_add->station, "Name of the station")
      ->required();
  stop_add_cmd
      ->add_option("-t,--service-time", stop_add->service_time,
                   "Time in s at which the service starts at the earliest")
      ->check(CLI::NonNegativeNumber)
      ->required();
  stop_add_cmd
      ->add_option("-d,--duration", stop_add->duration,
                   "Minimal duration of the service in s")
      ->check(CLI::NonNegativeNumber)
      ->required();
  stop_add_cmd->callback([&session, stop_add]() {
    session.instance().insert_stop(stop_add->train, stop_add->station,
                                   stop_add->service_time, stop_add->duration);
    session.mark_instance_modified();
    std::cout << "Added a stop of " << stop_add->train << " at "
              << stop_add->station << '\n';
  });

  auto* stop_remove_cmd = stop_cmd->add_subcommand(
      "remove", "Remove a stop from the schedule of a train");
  auto stop_remove = std::make_shared<StopOptions>();
  stop_remove_cmd->add_option("train", stop_remove->train, "Name of the train")
      ->required();
  stop_remove_cmd
      ->add_option("station", stop_remove->station, "Name of the station")
      ->required();
  stop_remove_cmd->callback([&session, stop_remove]() {
    session.instance()
        .editable_schedule(stop_remove->train)
        .remove_stop(stop_remove->station);
    session.mark_instance_modified();
    std::cout << "Removed the stop of " << stop_remove->train << " at "
              << stop_remove->station << '\n';
  });

  // ----------------------------------------------------------------- weight

  auto* weight_cmd = train_cmd->add_subcommand(
      "weight", "Show or set the weight of a train in the objective");
  struct WeightOptions {
    std::string name;
    double      weight{1};
  };
  auto weight_options = std::make_shared<WeightOptions>();
  weight_cmd->add_option("name", weight_options->name, "Name of the train")
      ->required();
  auto* weight_opt =
      weight_cmd
          ->add_option("weight", weight_options->weight,
                       "New weight. If omitted, the current one is printed.")
          ->check(CLI::NonNegativeNumber);
  weight_cmd->callback([&session, weight_options, weight_opt]() {
    if (weight_opt->count() > 0) {
      session.instance().set_train_weight(weight_options->name,
                                          weight_options->weight);
      session.mark_instance_modified();
    }
    std::cout << "Weight of " << weight_options->name << ": "
              << session.const_instance().get_train_weight(weight_options->name)
              << '\n';
  });

  train_cmd->add_subcommand("list", "List all trains with their schedules")
      ->callback([&session]() { list_trains(session); });
}
