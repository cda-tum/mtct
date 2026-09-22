#include "Session.hpp"

#include "CustomExceptions.hpp"
#include "StringHelper.hpp"
#include "datastructure/RailwayNetwork.hpp"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"

#include <filesystem>
#include <optional>
#include <string>
#include <utility>

namespace {
[[noreturn]] void throw_no_instance() {
  throw cda_rail::exceptions::InvalidInputException(
      "No instance loaded. Use 'instance new' or 'instance load' first.");
}
[[noreturn]] void throw_no_network() {
  throw cda_rail::exceptions::InvalidInputException(
      "No network loaded. Use 'network new', 'network load', or load an "
      "instance first.");
}
} // namespace

std::string cda_rail::cli::Session::prompt() const {
  if (m_instance.has_value()) {
    return cda_rail::concatenate_string_views(
        {"rail (", m_instance->get_instance_name(), ")> "});
  }
  if (m_standalone_network.has_value()) {
    return cda_rail::concatenate_string_views(
        {"rail [", m_standalone_network->get_network_name(), "]> "});
  }
  return "rail> ";
}

// ---------------------------------------------------------------- instance

cda_rail::instances::GeneralPerformanceOptimizationInstance&
cda_rail::cli::Session::instance() {
  if (!m_instance.has_value()) {
    throw_no_instance();
  }
  return m_instance.value();
}

const cda_rail::instances::GeneralPerformanceOptimizationInstance&
cda_rail::cli::Session::const_instance() const {
  if (!m_instance.has_value()) {
    throw_no_instance();
  }
  return m_instance.value();
}

void cda_rail::cli::Session::new_instance(const std::string& name,
                                          const std::string& subdirectory,
                                          const std::string& network_name) {
  // A network built in this session can be used without saving it first.
  const bool adopt_standalone_network =
      m_standalone_network.has_value() &&
      m_standalone_network->get_network_name() == network_name;
  auto network = adopt_standalone_network
                     ? std::move(m_standalone_network.value())
                     : Network(network_name, m_working_directory);

  instances::GeneralPerformanceOptimizationInstance new_instance(
      std::move(network));
  new_instance.set_instance_name(name);
  new_instance.set_instance_subdirectory(subdirectory);

  m_instance          = std::move(new_instance);
  m_instance_modified = true;
  // The network is now the instance's; whether it still has unsaved changes
  // does not depend on where it came from.
  m_network_modified   = adopt_standalone_network && m_network_modified;
  m_standalone_network = std::nullopt;
}

void cda_rail::cli::Session::load_instance(const std::string& name,
                                           const std::string& subdirectory) {
  m_instance = instances::GeneralPerformanceOptimizationInstance(
      name, subdirectory, m_working_directory);
  m_instance_modified  = false;
  m_standalone_network = std::nullopt;
  m_network_modified   = false;
}

void cda_rail::cli::Session::reload_instance() {
  if (!m_instance.has_value()) {
    throw_no_instance();
  }
  load_instance(m_instance->get_instance_name(),
                m_instance->get_instance_subdirectory());
}

void cda_rail::cli::Session::close_instance() {
  if (!m_instance.has_value()) {
    throw_no_instance();
  }
  m_instance          = std::nullopt;
  m_instance_modified = false;
  m_network_modified  = false;
}

std::filesystem::path
cda_rail::cli::Session::save_instance(bool const with_network) {
  auto& inst = instance();
  inst.export_instance(m_working_directory, with_network);
  m_instance_modified = false;
  if (with_network) {
    m_network_modified = false;
  }
  return m_working_directory / "instances" / inst.get_instance_subdirectory() /
         inst.get_instance_name();
}

// ----------------------------------------------------------------- network

cda_rail::Network& cda_rail::cli::Session::current_network() {
  if (m_instance.has_value()) {
    return m_instance->get_editable_network();
  }
  if (m_standalone_network.has_value()) {
    return m_standalone_network.value();
  }
  throw_no_network();
}

const cda_rail::Network& cda_rail::cli::Session::const_current_network() const {
  if (m_instance.has_value()) {
    return m_instance->get_const_network();
  }
  if (m_standalone_network.has_value()) {
    return m_standalone_network.value();
  }
  throw_no_network();
}

void cda_rail::cli::Session::new_network(const std::string& name) {
  if (m_instance.has_value()) {
    throw exceptions::InvalidInputException(
        "An instance is loaded, whose network would be replaced. Use "
        "'instance close' first.");
  }
  m_standalone_network = Network(name);
  m_network_modified   = true;
}

void cda_rail::cli::Session::load_network(const std::string& name) {
  if (m_instance.has_value()) {
    throw exceptions::InvalidInputException(
        "An instance is loaded, whose network would be replaced. Use "
        "'instance close' first.");
  }
  m_standalone_network = Network(name, m_working_directory);
  m_network_modified   = false;
}

void cda_rail::cli::Session::reload_network() {
  if (!m_standalone_network.has_value()) {
    throw exceptions::InvalidInputException(
        m_instance.has_value()
            ? "The current network belongs to the loaded instance. Use "
              "'instance reload' instead."
            : "No standalone network loaded.");
  }
  load_network(m_standalone_network->get_network_name());
}

void cda_rail::cli::Session::close_network() {
  if (!m_standalone_network.has_value()) {
    throw exceptions::InvalidInputException("No standalone network loaded.");
  }
  m_standalone_network = std::nullopt;
  m_network_modified   = false;
}

std::filesystem::path cda_rail::cli::Session::save_network(
    const std::optional<std::string>& new_name) {
  auto& network = current_network();
  if (new_name.has_value()) {
    network.set_network_name(new_name.value());
  }
  network.export_network(m_working_directory);
  m_network_modified = false;
  return m_working_directory / "networks" / network.get_network_name();
}
