#include "CLI/CLI.hpp"
#include "CommandInterpreter.hpp"
#include "Session.hpp"

#include <filesystem>
#include <iostream>
#include <memory>
#include <string>

namespace {
// Answers where the session currently is, in three lines. Everything beyond
// that, i.e. what the two objects contain, belongs to `instance info` and
// `network info`.
void print_status(const cda_rail::cli::Session& session) {
  std::cout << "Working directory: "
            << std::filesystem::absolute(session.working_directory()).string()
            << '\n';

  if (session.has_instance()) {
    const auto& instance = session.const_instance();
    std::cout << "Instance:          " << instance.get_instance_name()
              << " (subdirectory " << instance.get_instance_subdirectory()
              << ")" << (session.instance_modified() ? "  [modified]" : "")
              << '\n';
  } else {
    std::cout << "Instance:          none\n";
  }

  if (session.has_network()) {
    std::cout << "Network:           "
              << session.const_current_network().get_network_name()
              << (session.has_instance() ? " (of the instance)"
                                         : " (standalone)")
              << (session.network_modified() ? "  [modified]" : "") << '\n';
  } else {
    std::cout << "Network:           none\n";
  }
}

} // namespace

void cda_rail::cli::add_session_commands(CLI::App& app, Session& session,
                                         bool& exit_requested) {
  app.add_subcommand("status",
                     "Show which instance and network the session holds")
      ->callback([&session]() { print_status(session); });

  app.add_subcommand("help", "Print the list of all commands")
      ->callback([&app]() {
        // App::help() delegates to the selected subcommand, which here is
        // 'help' itself; the formatter is asked for the whole tree instead.
        std::cout << app.get_formatter()->make_help(&app, "",
                                                    CLI::AppFormatMode::Normal)
                  << '\n';
      });

  auto* exit_cmd =
      app.add_subcommand("exit", "End the session (also available as 'quit')");
  exit_cmd->alias("quit");
  auto exit_forced = std::make_shared<bool>(false);
  exit_cmd->add_flag("-f,--force", *exit_forced,
                     "End the session even if there are unsaved changes");
  exit_cmd->callback([&session, &exit_requested, exit_forced]() {
    if (session.has_unsaved_changes() && !*exit_forced) {
      std::cout << "There are unsaved changes. Save them, or use 'exit "
                   "--force' to discard them.\n";
      return;
    }
    exit_requested = true;
  });

  auto* workdir_cmd = app.add_subcommand(
      "working-directory", "Show or change the working directory");
  auto new_workdir = std::make_shared<std::string>();
  workdir_cmd->add_option(
      "path", *new_workdir,
      "New working directory. If omitted, the current one is printed.");
  workdir_cmd->callback([&session, new_workdir]() {
    if (!new_workdir->empty()) {
      session.set_working_directory(std::filesystem::path(*new_workdir));
    }
    std::cout << std::filesystem::absolute(session.working_directory()).string()
              << '\n';
  });
}
