#pragma once
#include <cstddef>
#include <exception>
#include <string>
#include <utility>

namespace cda_rail::exceptions {
class ModelCreationException : public std::exception {
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

class ExportException : public std::exception {
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

class ConsistencyException : public std::exception {
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

class InvalidInputException : public std::exception {
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

class ImportException : public std::exception {
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

class VertexNotExistentException : public std::exception {
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

class EdgeNotExistentException : public std::exception {
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

class TrainNotExistentException : public std::exception {
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

class StationNotExistentException : public std::exception {
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

class ScheduleNotExistentException : public std::exception {
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

static void throw_if_negative(double val, const std::string& name) {
  if (val < 0) {
    throw InvalidInputException(name + " must be non-negative, but is " +
                                std::to_string(val) + ".");
  }
};

static void throw_if_non_positive(double val, double tol,
                                  const std::string& name) {
  if (val <= tol) {
    throw InvalidInputException(name + " must be positive, but is " +
                                std::to_string(val) + ".");
  }
};

} // namespace cda_rail::exceptions
