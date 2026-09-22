#include "CLI/CLI.hpp"
#include "CommandInterpreter.hpp"
#include "Formatting.hpp"
#include "Session.hpp"

#include <cstddef>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace {
using cda_rail::cli::edge_input;
using cda_rail::cli::Session;

void list_routes(const Session& session) {
  const auto& instance = session.const_instance();
  const auto& network  = instance.get_const_network();
  for (const auto& train : instance.get_const_train_list()) {
    std::cout << train.get_name() << ':';
    if (!instance.get_const_routes().has_route(train.get_name())) {
      std::cout << " (no route)\n";
      continue;
    }
    const auto& route = instance.get_const_routes().get_route(train.get_name());
    if (route.empty()) {
      std::cout << " (empty)";
    }
    for (const auto& edge : route.get_edges()) {
      std::cout << ' ' << network.get_edge_name(edge);
    }
    std::cout << '\n';
  }
}
} // namespace

void cda_rail::cli::add_route_commands(CLI::App& app, Session& session) {
  auto* route_cmd =
      app.add_subcommand("route", "Set and inspect the routes of the trains");
  route_cmd->require_subcommand(1);

  auto* route_add_cmd = route_cmd->add_subcommand(
      "add", "Append a path of vertices to the route of a train");
  struct RouteAddOptions {
    std::string              train;
    std::vector<std::string> path;
  };
  auto route_add = std::make_shared<RouteAddOptions>();
  route_add_cmd->add_option("train", route_add->train, "Name of the train")
      ->required();
  route_add_cmd
      ->add_option("path", route_add->path,
                   "Names of the vertices the train passes, in order. Every "
                   "pair of consecutive vertices has to be an edge of the "
                   "network; these edges are appended to the route.")
      ->required()
      ->expected(2, -1)
      ->multi_option_policy(CLI::MultiOptionPolicy::TakeAll);
  route_add_cmd->callback([&session, route_add]() {
    auto& instance = session.instance();
    if (!instance.get_const_routes().has_route(route_add->train)) {
      instance.add_empty_route(route_add->train);
    }
    for (std::size_t i = 0; i + 1 < route_add->path.size(); ++i) {
      instance.push_back_edge_to_route(
          route_add->train,
          edge_input(route_add->path.at(i), route_add->path.at(i + 1)));
    }
    session.mark_instance_modified();
    std::cout << "Route of " << route_add->train << " now has "
              << instance.get_const_routes().get_route(route_add->train).size()
              << " edges\n";
  });

  auto* route_clear_cmd =
      route_cmd->add_subcommand("clear", "Empty the route of a train");
  auto clear_train = std::make_shared<std::string>();
  route_clear_cmd->add_option("train", *clear_train, "Name of the train")
      ->required();
  route_clear_cmd->callback([&session, clear_train]() {
    auto& instance = session.instance();
    if (!instance.get_const_routes().has_route(*clear_train)) {
      instance.add_empty_route(*clear_train);
    }
    while (!instance.get_const_routes().get_route(*clear_train).empty()) {
      instance.remove_last_edge_from_route(*clear_train);
    }
    session.mark_instance_modified();
    std::cout << "Cleared the route of " << *clear_train << '\n';
  });

  route_cmd->add_subcommand("list", "List the route of every train")
      ->callback([&session]() { list_routes(session); });
}
