#include "CLI/CLI.hpp"
#include "CommandInterpreter.hpp"
#include "Session.hpp"
#include "plog/Init.h"
#include "plog/Severity.h"

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <plog/Appenders/ColorConsoleAppender.h>
#include <plog/Formatters/TxtFormatter.h>
#include <plog/Log.h>
#include <plog/Logger.h>
#include <string>
#include <vector>

#ifdef _WIN32
#include <io.h>
#define CDA_RAIL_ISATTY _isatty
#define CDA_RAIL_FILENO _fileno
#else
#include <unistd.h>
#define CDA_RAIL_ISATTY isatty
#define CDA_RAIL_FILENO fileno
#endif

// NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay,bugprone-exception-escape)

int main(int argc, char** argv) {
  // Only log to console using std::cerr and std::cout respectively unless
  // initialized differently
  if (plog::get() == nullptr) {
    static plog::ColorConsoleAppender<plog::TxtFormatter> console_appender;
    plog::init(plog::info, &console_appender);
  }

  CLI::App app{"Interactive session for building, editing and solving railway "
               "instances"};
  argv = app.ensure_utf8(argv);
  app.option_defaults()->multi_option_policy(CLI::MultiOptionPolicy::Throw);

  std::string              working_directory{"."};
  std::vector<std::string> scripts;
  bool                     debug_output{false};

  app.add_option("-d,--working-directory", working_directory,
                 "Working directory holding the 'networks' and 'instances' "
                 "folders. Can be changed later by the 'working-directory' "
                 "command.")
      ->capture_default_str();
  app.add_option("-c,--script", scripts,
                 "File of commands to run, one per line. These are the very "
                 "same commands that can be typed at the prompt, so a script "
                 "can build an instance, edit it, and solve it. The session "
                 "continues at the prompt once the file has been run, hence a "
                 "script can also be used to set up an instance that is then "
                 "worked on by hand. Can be passed multiple times, in which "
                 "case the files are run in the given order. Empty lines and "
                 "lines starting with '#' are ignored.")
      ->multi_option_policy(CLI::MultiOptionPolicy::TakeAll)
      ->check(CLI::ExistingFile);
  app.add_flag("-v,--verbose,--debug", debug_output,
               "Whether to output debug information. Default: no debug "
               "output.");

  CLI11_PARSE(app, argc, argv);

  if (debug_output) {
    plog::get()->setMaxSeverity(plog::debug);
  }

  cda_rail::cli::Session session{std::filesystem::path(working_directory)};

  for (const auto& script : scripts) {
    std::ifstream script_file(script);
    if (!script_file.is_open()) {
      PLOGE << "Could not open script " << script;
      return 1;
    }
    if (!run_stream(script_file, session, false, true)) {
      return 0;
    }
  }

  const bool interactive = CDA_RAIL_ISATTY(CDA_RAIL_FILENO(stdin)) != 0;
  if (interactive) {
    std::cout << "Type 'help' for the list of commands and 'exit' to end the "
                 "session.\n";
  }
  run_stream(std::cin, session, interactive, false);

  return 0;
}

// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay,bugprone-exception-escape)
