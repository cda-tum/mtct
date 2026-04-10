#pragma once
#include <algorithm>
#include <array>
#include <cctype>
#include <cstddef>
#include <exception>
#include <string>
#include <string_view>
#include <utility>

namespace cda_rail::exceptions {
class CustomException : public std::exception {};

class ModelCreationException : public CustomException {
public:
  ModelCreationException() : error_message("Model creation failed.") {}
  explicit ModelCreationException(std::string message)
      : error_message(std::move(message)) {}
  [[nodiscard]] const char* what() const noexcept override {
    return error_message.c_str();
  }

private:
  std::string error_message;
};

class ExportException : public CustomException {
public:
  ExportException() : error_message("Export failed.") {}
  explicit ExportException(std::string message)
      : error_message(std::move(message)) {}
  [[nodiscard]] const char* what() const noexcept override {
    return error_message.c_str();
  }

private:
  std::string error_message;
};

class ConsistencyException : public CustomException {
public:
  ConsistencyException() : error_message("Consistency check failed.") {}
  explicit ConsistencyException(std::string message)
      : error_message(std::move(message)) {}
  [[nodiscard]] const char* what() const noexcept override {
    return error_message.c_str();
  }

private:
  std::string error_message;
};

class InvalidInputException : public CustomException {
public:
  InvalidInputException() : error_message("Invalid input.") {}
  explicit InvalidInputException(std::string message)
      : error_message(std::move(message)) {}
  [[nodiscard]] const char* what() const noexcept override {
    return error_message.c_str();
  }

private:
  std::string error_message;
};

class ImportException : public CustomException {
public:
  ImportException() : error_message("Import failed.") {}
  explicit ImportException(const std::string& import_name)
      : error_message("Import of " + import_name + " failed.") {}
  [[nodiscard]] const char* what() const noexcept override {
    return error_message.c_str();
  }

private:
  std::string error_message;
};

class VertexNotExistentException : public CustomException {
public:
  VertexNotExistentException()
      : error_message("Some vertex specified does not exist.") {}
  explicit VertexNotExistentException(const std::string& vertex_name)
      : error_message("Vertex " + vertex_name + " does not exist") {}
  explicit VertexNotExistentException(size_t vertex_id)
      : error_message("Vertex with ID " + std::to_string(vertex_id) +
                      " does not exist") {}
  [[nodiscard]] const char* what() const noexcept override {
    return error_message.c_str();
  }

private:
  std::string error_message;
};

class EdgeNotExistentException : public CustomException {
public:
  EdgeNotExistentException()
      : error_message("Some edge specified does not exist.") {}
  explicit EdgeNotExistentException(const std::string& edge_name)
      : error_message("Edge " + edge_name + " does not exist.") {}
  explicit EdgeNotExistentException(size_t edge_id)
      : error_message("Edge with ID " + std::to_string(edge_id) +
                      " does not exist.") {}
  explicit EdgeNotExistentException(size_t source, size_t target)
      : error_message("Edge connecting vertices with IDs " +
                      std::to_string(source) + "->" + std::to_string(target) +
                      " does not exist.") {}
  explicit EdgeNotExistentException(const std::string& source,
                                    const std::string& target)
      : error_message("Edge connecting " + source + "->" + target +
                      " does not exist.") {}
  [[nodiscard]] const char* what() const noexcept override {
    return error_message.c_str();
  }

private:
  std::string error_message;
};

class TrainNotExistentException : public CustomException {
public:
  TrainNotExistentException()
      : error_message("Some train specified does not exist.") {}
  explicit TrainNotExistentException(const std::string& train_name)
      : error_message("Train " + train_name + " does not exist.") {}
  explicit TrainNotExistentException(size_t train_id)
      : error_message("Train with ID " + std::to_string(train_id) +
                      " does not exist.") {}
  [[nodiscard]] const char* what() const noexcept override {
    return error_message.c_str();
  }

private:
  std::string error_message;
};

class StationNotExistentException : public CustomException {
public:
  StationNotExistentException()
      : error_message("Some station specified does not exist.") {}
  explicit StationNotExistentException(const std::string& station_name)
      : error_message("Station " + station_name + " does not exist.") {}
  [[nodiscard]] const char* what() const noexcept override {
    return error_message.c_str();
  }

private:
  std::string error_message;
};

class ScheduleNotExistentException : public CustomException {
public:
  ScheduleNotExistentException()
      : error_message("Some schedule specified does not exist.") {}
  explicit ScheduleNotExistentException(const std::string& schedule_name)
      : error_message("Schedule " + schedule_name + " does not exist.") {}
  explicit ScheduleNotExistentException(size_t schedule_id)
      : error_message("Schedule with ID " + std::to_string(schedule_id) +
                      " does not exist.") {}
  [[nodiscard]] const char* what() const noexcept override {
    return error_message.c_str();
  }

private:
  std::string error_message;
};

static void throw_if_less(double const value, double const threshold,
                          std::string const& name, bool inclusive) {
  if (inclusive) {
    if (value <= threshold) {
      throw InvalidInputException(name + " must be strictly larger than " +
                                  std::to_string(threshold) + ", but is " +
                                  std::to_string(value) + ".");
    }
  } else {
    if (value < threshold) {
      throw InvalidInputException(name + " must be at least " +
                                  std::to_string(threshold) + ", but is " +
                                  std::to_string(value) + ".");
    }
  }
};

static void throw_if_less_than(double const value, double const threshold,
                               std::string const& name) {
  throw_if_less(value, threshold, name, false);
};
static void throw_if_less_than_or_equal(double const       value,
                                        double const       threshold,
                                        std::string const& name) {
  throw_if_less(value, threshold, name, true);
};

static void throw_if_negative(double const value, std::string const& name) {
  throw_if_less_than(value, 0, name);
};

static void throw_if_non_positive(double const value, double tolerance,
                                  const std::string& name) {
  throw_if_less_than_or_equal(value, tolerance, name);
};
static void throw_if_non_positive(double const value, const std::string& name) {
  throw_if_non_positive(value, 0.0, name);
};
static void throw_if_non_positive(int const value, const std::string& name) {
  throw_if_non_positive(static_cast<double>(value), 0.0, name);
};

/**
 * @brief Validates that a folder name is conservatively portable across Linux,
 Windows, and macOS.

 * Checks whether a folder name is portable across Linux, Windows, and macOS.
 *
 * This method intentionally uses a conservative allowlist. A folder name is
 * accepted only if all characters are in:
 * - A-Z,
 * - a-z,
 * - 0-9,
 * - underscore (_), hyphen (-), dot (.), or space ( ).
 *
 * In addition, the name is rejected if any of the following applies:
 * - it is empty,
 * - it is longer than 255 characters,
 * - it contains any character outside the allowlist above,
 * - it starts with a space,
 * - it ends with a space or a dot,
 *   ending with a dot also rejects "." and "..",
 * - its Windows base name (part before first '.', or the whole name if there
 *   is no dot) is a reserved device name
 *   (case-insensitive): CON, PRN, AUX, NUL, COM1..COM9, LPT1..LPT9.
 *
 * These constraints are intentionally conservative so that every accepted name
 * can be used as a directory name on all three operating systems.
 *
 * @param folderName The folder name to validate
 *
 * @throws InvalidInputException if the folder name is not valid
 */
static void throw_if_invalid_folder_name(std::string_view const folderName) {
  if (folderName.empty()) {
    throw InvalidInputException("Folder name must not be empty.");
  }

  if (folderName.size() > 255) {
    throw InvalidInputException(
        "Folder name must not be longer than 255 characters.");
  }

  const auto is_allowed_character = [](char const c) {
    const bool is_uppercase_letter = (c >= 'A' && c <= 'Z');
    const bool is_lowercase_letter = (c >= 'a' && c <= 'z');
    const bool is_digit            = (c >= '0' && c <= '9');
    const bool is_allowed_symbol =
        (c == '_') || (c == '-') || (c == '.') || (c == ' ');

    return is_uppercase_letter || is_lowercase_letter || is_digit ||
           is_allowed_symbol;
  };

  if (!std::ranges::all_of(folderName, is_allowed_character)) {
    throw InvalidInputException(
        "Folder name contains unsupported characters. Allowed are A-Z, a-z, "
        "0-9, '_', '-', '.', and space.");
  }

  if (folderName.front() == ' ') {
    throw InvalidInputException("Folder name must not start with a space.");
  }

  if (folderName.back() == ' ' || folderName.back() == '.') {
    throw InvalidInputException(
        "Folder name must not end with a space or a dot.");
  }

  std::string_view windows_base_name =
      folderName.substr(0, folderName.find('.'));

  // Trim trailing spaces/dots: find last char that is neither ' ' nor '.'.
  std::size_t const last_non_trim_character =
      windows_base_name.find_last_not_of(" .");
  windows_base_name =
      (last_non_trim_character == std::string_view::npos)
          ? std::string_view{}
          : windows_base_name.substr(0, last_non_trim_character + 1);

  const auto matches_reserved_name = [&windows_base_name](
                                         std::string_view reserved_name) {
    if (windows_base_name.size() != reserved_name.size()) {
      return false;
    }

    // Safe here by design: input was validated against a conservative
    // ASCII allowlist, so byte-wise ASCII case folding is sufficient.
    return std::ranges::equal(
        windows_base_name, reserved_name,
        [](char const inputChar, char const reservedChar) {
          return static_cast<char>(::toupper(
                     static_cast<unsigned char>(inputChar))) == reservedChar;
        });
  };

  static constexpr std::array<std::string_view, 22> RESERVED_NAMES = {
      "CON",  "PRN",  "AUX",  "NUL",  "COM1", "COM2", "COM3", "COM4",
      "COM5", "COM6", "COM7", "COM8", "COM9", "LPT1", "LPT2", "LPT3",
      "LPT4", "LPT5", "LPT6", "LPT7", "LPT8", "LPT9"};

  const bool is_reserved_name =
      std::ranges::any_of(RESERVED_NAMES, matches_reserved_name);

  if (is_reserved_name) {
    throw InvalidInputException(
        "Folder name must not be a Windows reserved device name.");
  }
}

} // namespace cda_rail::exceptions
