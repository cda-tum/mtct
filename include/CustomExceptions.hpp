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
/**
 * @brief Base class for project-specific exceptions.
 *
 * Stores a human-readable error message and exposes it through @ref what().
 */
class CustomException : public std::exception {
private:
  std::string m_error_message;

protected:
  /**
   * @brief Constructs the exception with a custom error message.
   * @param message Message returned by @ref what().
   */
  explicit CustomException(std::string_view const message)
      : m_error_message(message) {};

public:
  /**
   * @brief Returns the explanatory string.
   * @return Null-terminated error message.
   */
  [[nodiscard]] const char* what() const noexcept override {
    return m_error_message.c_str();
  }
};

/**
 * @brief Thrown when model creation fails.
 */
class ModelCreationException : public CustomException {
public:
  /**
   * @brief Constructs with the default model creation error message.
   */
  ModelCreationException() : CustomException("Model creation failed.") {}
  /**
   * @brief Constructs with a custom model creation error message.
   * @param message Custom error message.
   */
  explicit ModelCreationException(std::string_view const message)
      : CustomException(message) {}
};

/**
 * @brief Thrown when export fails.
 */
class ExportException : public CustomException {
public:
  /**
   * @brief Constructs with the default export error message.
   */
  ExportException() : CustomException("Export failed.") {}
  /**
   * @brief Constructs with a custom export error message.
   * @param message Custom error message.
   */
  explicit ExportException(std::string_view const message)
      : CustomException(message) {}
};

/**
 * @brief Thrown when a consistency check fails.
 */
class ConsistencyException : public CustomException {
public:
  /**
   * @brief Constructs with the default consistency-check error message.
   */
  ConsistencyException() : CustomException("Consistency check failed.") {}
  /**
   * @brief Constructs with a custom consistency-check error message.
   * @param message Custom error message.
   */
  explicit ConsistencyException(std::string_view const message)
      : CustomException(message) {}
};

/**
 * @brief Thrown when input values do not satisfy expected constraints.
 */
class InvalidInputException : public CustomException {
public:
  /**
   * @brief Constructs with the default invalid-input error message.
   */
  InvalidInputException() : CustomException("Invalid input.") {}
  /**
   * @brief Constructs with a custom invalid-input error message.
   * @param message Custom error message.
   */
  explicit InvalidInputException(std::string_view const message)
      : CustomException(message) {}
};

/**
 * @brief Thrown when importing data fails.
 */
class ImportException : public CustomException {
public:
  /**
   * @brief Constructs with the default import error message.
   */
  ImportException() : CustomException("Import failed.") {}
  /**
   * @brief Constructs an error for a specific import target.
   * @param importName Name of the entity that could not be imported.
   */
  explicit ImportException(std::string_view const importName)
      : CustomException(
            concatenate_string_views({"Import of ", importName, " failed."})) {}
};

/**
 * @brief Thrown when a referenced vertex does not exist.
 */
class VertexNotExistentException : public CustomException {
public:
  /**
   * @brief Constructs with a generic vertex-not-found message.
   */
  VertexNotExistentException()
      : CustomException("Some vertex specified does not exist.") {}
  /**
   * @brief Constructs for a missing vertex identified by name.
   * @param vertexName Vertex name that was not found.
   */
  explicit VertexNotExistentException(std::string_view const vertexName)
      : CustomException(concatenate_string_views(
            {"Vertex ", vertexName, " does not exist"})) {}
  /**
   * @brief Constructs for a missing vertex identified by ID.
   * @param vertexId Vertex ID that was not found.
   */
  explicit VertexNotExistentException(size_t const vertexId)
      : CustomException(concatenate_string_views(
            {"Vertex with ID ", std::to_string(vertexId), " does not exist"})) {
  }
};

/**
 * @brief Thrown when a referenced edge does not exist.
 */
class EdgeNotExistentException : public CustomException {
public:
  /**
   * @brief Constructs with a generic edge-not-found message.
   */
  EdgeNotExistentException()
      : CustomException("Some edge specified does not exist.") {}
  /**
   * @brief Constructs for a missing edge identified by name.
   * @param edgeName Edge name that was not found.
   */
  explicit EdgeNotExistentException(std::string_view const edgeName)
      : CustomException(
            concatenate_string_views({"Edge ", edgeName, " does not exist."})) {
  }
  /**
   * @brief Constructs for a missing edge identified by ID.
   * @param edgeId Edge ID that was not found.
   */
  explicit EdgeNotExistentException(size_t const edgeId)
      : CustomException(concatenate_string_views(
            {"Edge with ID ", std::to_string(edgeId), " does not exist."})) {}
  /**
   * @brief Constructs for a missing edge identified by source/target IDs.
   * @param source Source vertex ID.
   * @param target Target vertex ID.
   */
  explicit EdgeNotExistentException(size_t const source, size_t const target)
      : CustomException(concatenate_string_views(
            {"Edge connecting vertices with IDs ", std::to_string(source), "->",
             std::to_string(target), " does not exist."})) {}
  /**
   * @brief Constructs for a missing edge identified by source/target names.
   * @param source Source vertex name.
   * @param target Target vertex name.
   */
  explicit EdgeNotExistentException(const std::string_view source,
                                    const std::string_view target)
      : CustomException(concatenate_string_views(
            {"Edge connecting ", source, "->", target, " does not exist."})) {}
};

/**
 * @brief Thrown when a referenced train does not exist.
 */
class TrainNotExistentException : public CustomException {
public:
  /**
   * @brief Constructs with a generic train-not-found message.
   */
  TrainNotExistentException()
      : CustomException("Some train specified does not exist.") {}
  /**
   * @brief Constructs for a missing train identified by name.
   * @param trainName Train name that was not found.
   */
  explicit TrainNotExistentException(std::string_view const trainName)
      : CustomException(concatenate_string_views(
            {"Train ", trainName, " does not exist."})) {}
  /**
   * @brief Constructs for a missing train identified by ID.
   * @param trainId Train ID that was not found.
   */
  explicit TrainNotExistentException(size_t const trainId)
      : CustomException(concatenate_string_views(
            {"Train with ID ", std::to_string(trainId), " does not exist."})) {}
};

/**
 * @brief Thrown when a referenced station does not exist.
 */
class StationNotExistentException : public CustomException {
public:
  /**
   * @brief Constructs with a generic station-not-found message.
   */
  StationNotExistentException()
      : CustomException("Some station specified does not exist.") {}
  /**
   * @brief Constructs for a missing station identified by name.
   * @param stationName Station name that was not found.
   */
  explicit StationNotExistentException(std::string_view const stationName)
      : CustomException(concatenate_string_views(
            {"Station ", stationName, " does not exist."})) {}
};

/**
 * @brief Thrown when a referenced schedule does not exist.
 */
class ScheduleNotExistentException : public CustomException {
public:
  /**
   * @brief Constructs with a generic schedule-not-found message.
   */
  ScheduleNotExistentException()
      : CustomException("Some schedule specified does not exist.") {}
  /**
   * @brief Constructs for a missing schedule identified by name.
   * @param scheduleName Schedule name that was not found.
   */
  explicit ScheduleNotExistentException(std::string_view const scheduleName)
      : CustomException(concatenate_string_views(
            {"Schedule ", scheduleName, " does not exist."})) {}
  /**
   * @brief Constructs for a missing schedule identified by ID.
   * @param scheduleId Schedule ID that was not found.
   */
  explicit ScheduleNotExistentException(size_t const scheduleId)
      : CustomException(concatenate_string_views({"Schedule with ID ",
                                                  std::to_string(scheduleId),
                                                  " does not exist."})) {}
};

/**
 * @brief Throws if a value is below or equal to a threshold, depending on
 *        inclusiveness.
 * @param value Value to validate.
 * @param threshold Lower bound to enforce.
 * @param name Human-readable parameter name used in the error message.
 * @param inclusive If true, enforces @p value > @p threshold; otherwise
 *                  enforces @p value >= @p threshold.
 * @throws InvalidInputException If the constraint is violated.
 */
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

/**
 * @brief Throws if a value is less than a threshold.
 * @param value Value to validate.
 * @param threshold Inclusive lower bound.
 * @param name Human-readable parameter name used in the error message.
 * @throws InvalidInputException If @p value < @p threshold.
 */
static void throw_if_less_than(double const value, double const threshold,
                               std::string_view const name) {
  throw_if_less(value, threshold, name, false);
};
/**
 * @brief Throws if a value is less than or equal to a threshold.
 * @param value Value to validate.
 * @param threshold Strict lower bound.
 * @param name Human-readable parameter name used in the error message.
 * @throws InvalidInputException If @p value <= @p threshold.
 */
static void throw_if_less_than_or_equal(double const           value,
                                        double const           threshold,
                                        std::string_view const name) {
  throw_if_less(value, threshold, name, true);
};

/**
 * @brief Throws if a value is negative.
 * @param value Value to validate.
 * @param name Human-readable parameter name used in the error message.
 * @throws InvalidInputException If @p value < 0.
 */
static void throw_if_negative(double const value, std::string_view const name) {
  throw_if_less_than(value, 0, name);
};

/**
 * @brief Throws if a value is not strictly greater than a tolerance.
 * @param value Value to validate.
 * @param tolerance Strict lower bound.
 * @param name Human-readable parameter name used in the error message.
 * @throws InvalidInputException If @p value <= @p tolerance.
 */
static void throw_if_non_positive(double const value, double tolerance,
                                  std::string_view const name) {
  throw_if_less_than_or_equal(value, tolerance, name);
};
/**
 * @brief Throws if a value is not strictly positive.
 * @param value Value to validate.
 * @param name Human-readable parameter name used in the error message.
 * @throws InvalidInputException If @p value <= 0.
 */
static void throw_if_non_positive(double const           value,
                                  std::string_view const name) {
  throw_if_non_positive(value, 0.0, name);
};
/**
 * @brief Throws if an integer value is not strictly positive.
 * @param value Value to validate.
 * @param name Human-readable parameter name used in the error message.
 * @throws InvalidInputException If @p value <= 0.
 */
static void throw_if_non_positive(int const              value,
                                  std::string_view const name) {
  throw_if_non_positive(static_cast<double>(value), 0.0, name);
};

/**
 * @brief Validates that a folder name is conservatively portable across Linux,
 Windows, and macOS.
 *
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
