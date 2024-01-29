#pragma once

#include "Definitions.hpp"
#include "datastructure/RailwayNetwork.hpp"

#include <filesystem>
#include <string>
#include <type_traits>

namespace cda_rail::instances {
class GeneralProblemInstance {
protected:
  Network network;

public:
  // Network functions, i.e., network is accessible via n() as a reference
  [[nodiscard]] Network&       n() { return network; };
  [[nodiscard]] const Network& const_n() const { return network; };

  virtual void export_instance(const std::filesystem::path& path) const = 0;

  void export_instance(const std::string& path) const {
    export_instance(std::filesystem::path(path));
  };
  void export_instance(const char* path) const {
    export_instance(std::filesystem::path(path));
  };

  virtual bool check_consistency() const = 0;
};

template <typename T> class SolGeneralProblemInstance {
  static_assert(std::is_base_of<GeneralProblemInstance, T>::value,
                "T must be a child of GeneralProblemInstance");

protected:
  T              instance;
  SolutionStatus status  = SolutionStatus::Unknown;
  double         obj     = -1;
  bool           has_sol = false;

  SolGeneralProblemInstance() = default;
  SolGeneralProblemInstance(T instance) : instance(std::move(instance)){};
  SolGeneralProblemInstance(T instance, SolutionStatus status, double obj,
                            bool has_sol)
      : instance(std::move(instance)), status(status), obj(obj),
        has_sol(has_sol){};

public:
  [[nodiscard]] const T&       get_instance() const { return instance; };
  [[nodiscard]] SolutionStatus get_status() const { return status; };
  [[nodiscard]] double         get_obj() const { return obj; };
  [[nodiscard]] bool           has_solution() const { return has_sol; };
  void set_status(SolutionStatus new_status) { status = new_status; };
  void set_obj(double new_obj) { obj = new_obj; };
  void set_solution_found() { has_sol = true; };
  void set_solution_not_found() { has_sol = false; };

  virtual void export_solution(const std::filesystem::path& p,
                               bool export_instance = true) const = 0;

  void export_solution(const std::string& path,
                       bool               export_instance = true) const {
    export_solution(std::filesystem::path(path), export_instance);
  };
  void export_solution(const char* path, bool export_instance = true) const {
    export_solution(std::filesystem::path(path), export_instance);
  };

  virtual bool check_consistency() const = 0;
};
} // namespace cda_rail::instances
