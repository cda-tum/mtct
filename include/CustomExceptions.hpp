#pragma once
#include <cstddef>
#include <exception>
#include <string>
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

} // namespace cda_rail::exceptions
