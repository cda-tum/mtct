#include "CLI/CLI.hpp"
#include "CommandInterpreter.hpp"
#include "Session.hpp"

#include <filesystem>
#include <iostream>
#include <memory>
#include <string>

namespace {
/**
 * @brief Lists every `<subdirectory>/<name>` below `<working>/instances`.
 *
 * A directory counts as an instance if it contains a `network.json`, which is
 * the file the instance loader reads first.
 */
void list_instances(const cda_rail::cli::Session& session) {
  const auto instances_directory = session.working_directory() / "instances";
  if (!std::filesystem::is_directory(instances_directory)) {
    std::cout << "No instances directory in "
              << std::filesystem::absolute(session.working_directory()).string()
              << '\n';
    return;
  }

  bool any = false;
  for (const auto& subdirectory :
       std::filesystem::directory_iterator(instances_directory)) {
    if (!subdirectory.is_directory()) {
      continue;
    }
    for (const auto& instance :
         std::filesystem::directory_iterator(subdirectory.path())) {
      if (!instance.is_directory() ||
          !std::filesystem::exists(instance.path() / "network.json")) {
        continue;
      }
      std::cout << instance.path().filename().string() << "  (-s "
                << subdirectory.path().filename().string() << ")\n";
      any = true;
    }
  }
  if (!any) {
    std::cout << "No instances found in " << instances_directory.string()
              << '\n';
  }
}

void warn_about_discarded(bool const had_unsaved_changes) {
  if (had_unsaved_changes) {
    std::cout << "Warning: unsaved changes were discarded.\n";
  }
}

void print_instance_info(const cda_rail::cli::Session& session) {
  const auto& instance = session.const_instance();
  std::cout << "Instance:             " << instance.get_instance_name()
            << " (subdirectory " << instance.get_instance_subdirectory()
            << ")\n";
  std::cout << "Network:              "
            << instance.get_const_network().get_network_name() << '\n';
  std::cout << "Trains:               "
            << instance.get_const_train_list().size() << '\n';
  std::cout << "Stations:             "
            << instance.get_const_station_list().size() << '\n';
  std::cout << "Routes:               " << instance.get_const_routes().size()
            << '\n';
  std::cout << "Station delay weight: " << instance.get_station_delay_weight()
            << '\n';
}
} // namespace

void cda_rail::cli::add_instance_commands(CLI::App& app, Session& session) {
  auto* instance_cmd =
      app.add_subcommand("instance", "Load, create, check and save instances");
  instance_cmd->require_subcommand(1);

  // ------------------------------------------------------------------- new
  auto* new_cmd = instance_cmd->add_subcommand(
      "new", "Create a new empty instance on top of an existing network");
  struct NewOptions {
    std::string name;
    std::string subdirectory;
    std::string network_name;
  };
  auto new_options = std::make_shared<NewOptions>();
  new_cmd->add_option("name", new_options->name, "Name of the new instance")
      ->required();
  new_cmd
      ->add_option("-s,--instance-subdirectory", new_options->subdirectory,
                   "Subdirectory the instance belongs to")
      ->required();
  new_cmd
      ->add_option("-n,--network", new_options->network_name,
                   "Name of the network the instance uses. If it is the "
                   "standalone network of this session, that in-memory "
                   "network is adopted instead of being read from disk.")
      ->required();
  new_cmd->callback([&session, new_options]() {
    const bool had_unsaved_changes = session.has_unsaved_changes();
    session.new_instance(new_options->name, new_options->subdirectory,
                         new_options->network_name);
    warn_about_discarded(had_unsaved_changes);
    std::cout << "Created instance " << new_options->name << " using network "
              << session.const_current_network().get_network_name() << '\n';
  });

  // ------------------------------------------------------------------ load
  auto* load_cmd =
      instance_cmd->add_subcommand("load", "Load an instance from disk");
  struct LoadOptions {
    std::string name;
    std::string subdirectory;
  };
  auto load_options = std::make_shared<LoadOptions>();
  load_cmd->add_option("name", load_options->name, "Name of the instance")
      ->required();
  load_cmd
      ->add_option("-s,--instance-subdirectory", load_options->subdirectory,
                   "Subdirectory of the instance")
      ->required();
  load_cmd->callback([&session, load_options]() {
    const bool had_unsaved_changes = session.has_unsaved_changes();
    session.load_instance(load_options->name, load_options->subdirectory);
    warn_about_discarded(had_unsaved_changes);
    std::cout << "Loaded instance " << load_options->name << " using network "
              << session.const_current_network().get_network_name() << '\n';
  });

  // ------------------------------------------------------------------ list
  instance_cmd
      ->add_subcommand("list", "List the instances in the working directory")
      ->callback([&session]() { list_instances(session); });

  // ------------------------------------------------------------------ save
  auto* save_cmd = instance_cmd->add_subcommand(
      "save", "Write the instance to the working directory");
  struct SaveOptions {
    std::string name;
    bool        with_network{false};
  };
  auto  save_options = std::make_shared<SaveOptions>();
  auto* save_as_opt  = save_cmd->add_option(
      "-a,--as", save_options->name,
      "Save under this name instead of the current one. The instance is "
      "renamed, so subsequent saves use the new name as well.");
  save_cmd->add_flag(
      "-w,--with-network", save_options->with_network,
      "Also write the network. Without this flag the network on disk is left "
      "untouched, even if it was edited in this session.");
  save_cmd->callback([&session, save_options, save_as_opt]() {
    if (save_as_opt->count() > 0) {
      session.instance().set_instance_name(save_options->name);
    }
    const auto path = session.save_instance(save_options->with_network);
    std::cout << "Wrote instance to " << path.string() << '\n';
    if (save_options->with_network) {
      std::cout << "Wrote network to "
                << (session.working_directory() / "networks" /
                    session.const_current_network().get_network_name())
                       .string()
                << '\n';
    } else if (session.network_modified()) {
      std::cout << "Note: the network has unsaved changes. Use 'network save' "
                   "or 'instance save --with-network' to write it.\n";
    }
  });

  // ---------------------------------------------------------------- reload
  instance_cmd
      ->add_subcommand("reload",
                       "Reread the instance from disk, discarding unsaved "
                       "changes")
      ->callback([&session]() {
        const bool had_unsaved_changes = session.has_unsaved_changes();
        session.reload_instance();
        warn_about_discarded(had_unsaved_changes);
        std::cout << "Reloaded instance "
                  << session.const_instance().get_instance_name() << '\n';
      });

  // ----------------------------------------------------------------- close
  instance_cmd
      ->add_subcommand("close", "Unload the instance without writing it")
      ->callback([&session]() {
        const bool had_unsaved_changes = session.has_unsaved_changes();
        session.close_instance();
        warn_about_discarded(had_unsaved_changes);
        std::cout << "Closed the instance.\n";
      });

  // ------------------------------------------------------------------ info
  instance_cmd->add_subcommand("info", "Show what the loaded instance contains")
      ->callback([&session]() { print_instance_info(session); });

  // ----------------------------------------------------------------- check
  auto* check_cmd = instance_cmd->add_subcommand(
      "check", "Check the instance for consistency and obvious infeasibility");
  auto late_entry = std::make_shared<bool>(false);
  check_cmd->add_flag(
      "-l,--allow-late-entry", *late_entry,
      "Allow trains to enter the network later than scheduled when judging "
      "whether the instance is obviously infeasible");
  check_cmd->callback([&session, late_entry]() {
    const auto& instance = session.const_instance();
    std::cout << "Consistent (every train routed): "
              << (instance.check_consistency(true) ? "yes" : "no") << '\n';
    std::cout << "Consistent (routes optional):    "
              << (instance.check_consistency(false) ? "yes" : "no") << '\n';
    const auto feasibility = instance.is_obviously_infeasible(*late_entry);
    if (feasibility.is_obviously_infeasible) {
      std::cout << "Obviously infeasible:            yes ("
                << feasibility.reason << ")\n";
    } else {
      std::cout << "Obviously infeasible:            no\n";
    }
  });

  // -------------------------------------------------- station-delay-weight
  auto* weight_cmd = instance_cmd->add_subcommand(
      "station-delay-weight",
      "Show or set the weight with which station delays enter the objective");
  auto  weight     = std::make_shared<double>(0);
  auto* weight_opt = weight_cmd->add_option(
      "weight", *weight, "New weight. If omitted, the current one is printed.");
  weight_cmd->callback([&session, weight, weight_opt]() {
    if (weight_opt->count() > 0) {
      session.instance().set_station_delay_weight(*weight);
      session.mark_instance_modified();
    }
    std::cout << "Station delay weight: "
              << session.const_instance().get_station_delay_weight() << '\n';
  });
}
