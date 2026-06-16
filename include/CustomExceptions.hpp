#pragma once
#include "StringHelper.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstddef>
#include <exception>
#include <initializer_list>
#include <string>
#include <string_view>

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
   * @brief Provides the error message stored in this exception.
   * @return Null-terminated C string containing the error message.
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
       * @brief Creates an exception with a custom export error message.
       * @param message Error message to store.
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
       * @brief Constructs an InvalidInputException with a custom error message.
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
             * @brief Creates an ImportException with a formatted message that names the failed import.
             * @param importName The name of the entity that could not be imported.
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
       * @brief Creates an exception indicating a vertex does not exist.
       */
  VertexNotExistentException()
      : CustomException("Some vertex specified does not exist.") {}
  /**
             * @brief Exception indicating a vertex with the specified name does not exist.
             */
            ```
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
   * @brief Raised when an edge with the specified name does not exist.
   * @param edgeName The name of the edge that was not found.
   */
  explicit EdgeNotExistentException(std::string_view const edgeName)
      : CustomException(
            concatenate_string_views({"Edge ", edgeName, " does not exist."})) {
  }
  /**
             * @brief Creates an exception for an edge that does not exist, identified by its ID.
             */
  explicit EdgeNotExistentException(size_t const edgeId)
      : CustomException(concatenate_string_views(
            {"Edge with ID ", std::to_string(edgeId), " does not exist."})) {}
  /**
              * @brief Exception for a missing edge between two vertices.
              * @param source Source vertex ID.
              * @param target Target vertex ID.
              */
  explicit EdgeNotExistentException(size_t const source, size_t const target)
      : CustomException(concatenate_string_views(
            {"Edge connecting vertices with IDs ", std::to_string(source), "->",
             std::to_string(target), " does not exist."})) {}
  /**
             * @brief Creates an exception for a missing edge between two named vertices.
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
             * @brief Creates an exception for a missing train identified by name.
             */
            ```
  explicit TrainNotExistentException(std::string_view const trainName)
      : CustomException(concatenate_string_views(
            {"Train ", trainName, " does not exist."})) {}
  /**
             * @brief Creates an exception for a train not found by ID.
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
       * @brief Initializes the exception with a default message indicating no station was found.
       */
  StationNotExistentException()
      : CustomException("Some station specified does not exist.") {}
  /**
             * @brief Constructs an exception for a station that does not exist, identified by name.
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
       * @brief Constructs the exception with a default message indicating a schedule does not exist.
       */
  ScheduleNotExistentException()
      : CustomException("Some schedule specified does not exist.") {}
  /**
             * @brief Constructs an exception for a schedule that does not exist.
             * @param scheduleName Name of the schedule that was not found.
             */
  explicit ScheduleNotExistentException(std::string_view const scheduleName)
      : CustomException(concatenate_string_views(
            {"Schedule ", scheduleName, " does not exist."})) {}
  /**
                                                   * @brief Constructs an exception for a missing schedule identified by ID.
                                                   * @param scheduleId The ID of the schedule.
                                                   */
  explicit ScheduleNotExistentException(size_t const scheduleId)
      : CustomException(concatenate_string_views({"Schedule with ID ",
                                                  std::to_string(scheduleId),
                                                  " does not exist."})) {}
};

/**
 * @brief Validates that a value meets a minimum threshold constraint.
 *
 * @param value Value to validate.
 * @param threshold Lower bound to enforce.
 * @param name Human-readable parameter name for the error message.
 * @param inclusive If `true`, requires `value > threshold`; if `false`, requires `value >= threshold`.
 *
 * @throws InvalidInputException If the constraint is violated.
 */
inline void throw_if_less(double const value, double const threshold,
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
inline void throw_if_less_than(double const value, double const threshold,
                               std::string_view const name) {
  throw_if_less(value, threshold, name, false);
};
/**
 * @brief Validates that a value is greater than a threshold.
 *
 * @param value Value to validate.
 * @param threshold Strict lower bound (exclusive).
 * @param name Parameter name for the error message.
 *
 * @throws InvalidInputException If @p value is less than or equal to @p threshold.
 */
inline void throw_if_less_than_or_equal(double const           value,
                                        double const           threshold,
                                        std::string_view const name) {
  throw_if_less(value, threshold, name, true);
};

/**
 * @brief Validates that a value is non-negative.
 *
 * @param name Human-readable parameter name used in the error message.
 * @throws InvalidInputException If the value is negative.
 */
inline void throw_if_negative(double const value, std::string_view const name) {
  throw_if_less_than(value, 0, name);
};

/**
 * @brief Ensures a value is strictly greater than a tolerance.
 * @param value Value to validate.
 * @param tolerance Strict lower bound.
 * @param name Human-readable parameter name used in the error message.
 * @throws InvalidInputException If @p value <= @p tolerance.
 */
inline void throw_if_non_positive(double const value, double tolerance,
                                  std::string_view const name) {
  throw_if_less_than_or_equal(value, tolerance, name);
};
/**
 * @brief Validates that a value is strictly positive.
 *
 * @param value Value to validate.
 * @param name Parameter name used in the error message.
 * @throws InvalidInputException If value is less than or equal to zero.
 */
inline void throw_if_non_positive(double const           value,
                                  std::string_view const name) {
  throw_if_non_positive(value, 0.0, name);
};
/**
 * @brief Throws if an integer value is not strictly positive.
 * @param value Value to validate.
 * @param name Human-readable parameter name used in the error message.
 * @throws InvalidInputException If @p value <= 0.
 */
inline void throw_if_non_positive(int const              value,
                                  std::string_view const name) {
  throw_if_non_positive(static_cast<double>(value), 0.0, name);
};

/**
 * @brief Validates that a folder name is portable across Linux, Windows, and macOS.
 *
 * This function uses a conservative allowlist to ensure portability. A folder name is
 * accepted only if:
 * - All characters are in: A-Z, a-z, 0-9, underscore (_), hyphen (-), dot (.), or space ( ).
 * - It is not empty.
 * - It is at most 255 characters.
 * - It does not start with a space.
 * - It does not end with a space or dot.
 * - Its Windows base name (the part before the first dot, or the whole name if no dot exists)
 *   is not a reserved device name (case-insensitive): CON, PRN, AUX, NUL, COM1–COM9, LPT1–LPT9.
 *
 * @param folderName The folder name to validate.
 *
 * @throws InvalidInputException if the folder name violates any of the above constraints.
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
