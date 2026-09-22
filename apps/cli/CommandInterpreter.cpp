#include "CommandInterpreter.hpp"

#include "CLI/CLI.hpp"
#include "Session.hpp"

#include <exception>
#include <iostream>
#include <istream>
#include <plog/Log.h>
#include <string>

// The reinterpret_cast warnings are false positives stemming from the plog
// macros.
// NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast)

cda_rail::cli::CommandInterpreter::CommandInterpreter(Session& session)
    : m_session(session) {
  m_app.option_defaults()->multi_option_policy(CLI::MultiOptionPolicy::Throw);
  m_app.require_subcommand(0, 1);
  // The commands are read from a terminal, where a failed command should not
  // print the full help text of the whole tree.
  m_app.failure_message(CLI::FailureMessage::simple);

  add_session_commands(m_app, m_session, m_exit_requested);
  add_instance_commands(m_app, m_session);
  add_network_commands(m_app, m_session);
  add_timetable_commands(m_app, m_session);
  add_route_commands(m_app, m_session);
  add_solve_commands(m_app, m_session);
}

bool cda_rail::cli::CommandInterpreter::run_line(const std::string& input) {
  // Editors on Windows like to put a byte order mark in front of a script.
  static const std::string BYTE_ORDER_MARK{"\xEF\xBB\xBF"};
  const std::string        line = input.starts_with(BYTE_ORDER_MARK)
                                      ? input.substr(BYTE_ORDER_MARK.size())
                                      : input;

  if (line.find_first_not_of(" \t\r\n") == std::string::npos) {
    return true;
  }
  // '#' starts a comment, so that scripts can be annotated.
  if (line.at(line.find_first_not_of(" \t\r\n")) == '#') {
    return true;
  }

  try {
    m_app.parse(line);
  } catch (const CLI::ParseError& e) {
    // Prints the message (and the help text for --help), does not terminate.
    m_app.exit(e);
  } catch (const std::exception& e) {
    // A library exception must not kill the session.
    PLOGE << e.what();
  }

  return !m_exit_requested;
}

bool cda_rail::cli::run_stream(std::istream& input, Session& session,
                               bool const prompt, bool const echo) {
  std::string line;
  while (true) {
    if (prompt) {
      std::cout << session.prompt() << std::flush;
    }
    if (!std::getline(input, line)) {
      return true;
    }
    if (echo) {
      std::cout << session.prompt() << line << '\n';
    }
    CommandInterpreter interpreter{session};
    if (!interpreter.run_line(line)) {
      return false;
    }
  }
}

// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast)
