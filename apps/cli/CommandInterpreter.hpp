#pragma once

#include "CLI/CLI.hpp"
#include "Session.hpp"

#include <istream>
#include <string>

namespace cda_rail::cli {

/**
 * @brief Parses and executes a single command line of the `rail_cli` session.
 *
 * One interpreter is constructed per input line. Building the subcommand tree
 * costs microseconds, which is irrelevant next to anything the commands do,
 * and it avoids the two CLI11 pitfalls that would otherwise bite here:
 * `App::clear()` resets the parse state but not the bound variables, so a
 * reused app silently carries an option of the previous command into the next
 * one, and a `CLI::App` must not be moved after construction because its
 * options hold references into the owning object.
 */
class CommandInterpreter {
  Session& m_session;
  CLI::App m_app{"Interactive session for building, editing and solving "
                 "railway instances"};
  bool     m_exit_requested{false};

public:
  explicit CommandInterpreter(Session& session);

  // A CLI::App must not be moved after construction.
  CommandInterpreter(const CommandInterpreter&)                = delete;
  CommandInterpreter& operator=(const CommandInterpreter&)     = delete;
  CommandInterpreter(CommandInterpreter&&) noexcept            = delete;
  CommandInterpreter& operator=(CommandInterpreter&&) noexcept = delete;
  ~CommandInterpreter()                                        = default;

  /**
   * @brief Executes one command line.
   *
   * Neither a parse error nor an exception thrown by the library terminates
   * the session; both are reported and leave the session usable.
   *
   * @return `true` if the session should continue, `false` after `exit`.
   */
  bool run_line(const std::string& line);

  /** @brief Whether the last executed line asked the session to end. */
  [[nodiscard]] bool exit_requested() const { return m_exit_requested; }
};

/**
 * @brief Runs every line of @p input through a fresh interpreter.
 *
 * The loop ends when the input is exhausted or a line ends the session, so a
 * session started on a script or on a pipe always terminates.
 *
 * @param prompt Whether the prompt is printed before every line, which is
 *        wanted when the commands are typed but not when they are read from a
 *        file or a pipe.
 * @param echo Whether the commands themselves are printed, which is the other
 *        way round.
 * @return `false` if the session was ended by `exit`, `true` if the input ran
 *         out.
 */
bool run_stream(std::istream& input, Session& session, bool prompt, bool echo);

// The command groups, each defined in its own translation unit. Every one of
// them adds its subcommands to @p app and captures @p session by reference;
// the interpreter owns both for the duration of a line.
void add_session_commands(CLI::App& app, Session& session,
                          bool& exit_requested);
void add_instance_commands(CLI::App& app, Session& session);
void add_network_commands(CLI::App& app, Session& session);
void add_timetable_commands(CLI::App& app, Session& session);
void add_route_commands(CLI::App& app, Session& session);
void add_solve_commands(CLI::App& app, Session& session);

} // namespace cda_rail::cli
