#pragma once
#include "Definitions.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstddef>
#include <exception>
#include <string>
#include <string_view>
#include <utility>

namespace cda_rail::exceptions {
class CustomException : public std::exception {
private:
  std::string m_error_message;

protected:
  explicit CustomException(std::string_view const message)
      : m_error_message(message) {};

public:
  [[nodiscard]] const char* what() const noexcept override {
    return m_error_message.c_str();
  }
};

class ModelCreationException : public CustomException {
public:
  ModelCreationException() : CustomException("Model creation failed.") {}
  explicit ModelCreationException(std::string_view const message)
      : CustomException(message) {}
};

class ExportException : public CustomException {
public:
  ExportException() : CustomException("Export failed.") {}
  explicit ExportException(std::string_view const message)
      : CustomException(message) {}
};

class ConsistencyException : public CustomException {
public:
  ConsistencyException() : CustomException("Consistency check failed.") {}
  explicit ConsistencyException(std::string_view const message)
      : CustomException(message) {}
};

class InvalidInputException : public CustomException {
public:
  InvalidInputException() : CustomException("Invalid input.") {}
  explicit InvalidInputException(std::string_view const message)
      : CustomException(message) {}
};

class ImportException : public CustomException {
public:
  ImportException() : CustomException("Import failed.") {}
  explicit ImportException(std::string_view const importName)
      : CustomException(
            concatenate_string_views({"Import of ", importName, " failed."})) {}
};

class VertexNotExistentException : public CustomException {
public:
  VertexNotExistentException()
      : CustomException("Some vertex specified does not exist.") {}
  explicit VertexNotExistentException(std::string_view const vertexName)
      : CustomException(concatenate_string_views(
            {"Vertex ", vertexName, " does not exist"})) {}
  explicit VertexNotExistentException(size_t const vertexId)
      : CustomException(concatenate_string_views(
            {"Vertex with ID ", std::to_string(vertexId), " does not exist"})) {
  }
};

class EdgeNotExistentException : public CustomException {
public:
  EdgeNotExistentException()
      : CustomException("Some edge specified does not exist.") {}
  explicit EdgeNotExistentException(std::string_view const edgeName)
      : CustomException(
            concatenate_string_views({"Edge ", edgeName, " does not exist."})) {
  }
  explicit EdgeNotExistentException(size_t const edgeId)
      : CustomException(concatenate_string_views(
            {"Edge with ID ", std::to_string(edgeId), " does not exist."})) {}
  explicit EdgeNotExistentException(size_t const source, size_t const target)
      : CustomException(concatenate_string_views(
            {"Edge connecting vertices with IDs ", std::to_string(source), "->",
             std::to_string(target), " does not exist."})) {}
  explicit EdgeNotExistentException(const std::string_view source,
                                    const std::string_view target)
      : CustomException(concatenate_string_views(
            {"Edge connecting ", source, "->", target, " does not exist."})) {}
};

class TrainNotExistentException : public CustomException {
public:
  TrainNotExistentException()
      : CustomException("Some train specified does not exist.") {}
  explicit TrainNotExistentException(std::string_view const trainName)
      : CustomException(concatenate_string_views(
            {"Train ", trainName, " does not exist."})) {}
  explicit TrainNotExistentException(size_t const trainId)
      : CustomException(concatenate_string_views(
            {"Train with ID ", std::to_string(trainId), " does not exist."})) {}
};

class StationNotExistentException : public CustomException {
public:
  StationNotExistentException()
      : CustomException("Some station specified does not exist.") {}
  explicit StationNotExistentException(std::string_view const stationName)
      : CustomException(concatenate_string_views(
            {"Station ", stationName, " does not exist."})) {}
};

class ScheduleNotExistentException : public CustomException {
public:
  ScheduleNotExistentException()
      : CustomException("Some schedule specified does not exist.") {}
  explicit ScheduleNotExistentException(std::string_view const scheduleName)
      : CustomException(concatenate_string_views(
            {"Schedule ", scheduleName, " does not exist."})) {}
  explicit ScheduleNotExistentException(size_t const scheduleId)
      : CustomException(concatenate_string_views({"Schedule with ID ",
                                                  std::to_string(scheduleId),
                                                  " does not exist."})) {}
};

static void throw_if_less(double const value, double const threshold,
                          std::string_view const name, bool inclusive) {
  if (inclusive) {
    if (value <= threshold) {
      throw InvalidInputException(concatenate_string_views(
          {name, " must be strictly larger than ", std::to_string(threshold),
           ", but is ", std::to_string(value), "."}));
    }
  } else {
    if (value < threshold) {
      throw InvalidInputException(concatenate_string_views(
          {name, " must be at least ", std::to_string(threshold), ", but is ",
           std::to_string(value), "."}));
    }
  }
};

static void throw_if_less_than(double const value, double const threshold,
                               std::string_view const name) {
  throw_if_less(value, threshold, name, false);
};
static void throw_if_less_than_or_equal(double const           value,
                                        double const           threshold,
                                        std::string_view const name) {
  throw_if_less(value, threshold, name, true);
};

static void throw_if_negative(double const value, std::string_view const name) {
  throw_if_less_than(value, 0, name);
};

static void throw_if_non_positive(double const value, double tolerance,
                                  std::string_view const name) {
  throw_if_less_than_or_equal(value, tolerance, name);
};
static void throw_if_non_positive(double const           value,
                                  std::string_view const name) {
  throw_if_non_positive(value, 0.0, name);
};
static void throw_if_non_positive(int const              value,
                                  std::string_view const name) {
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
