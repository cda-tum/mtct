#include "CLI/CLI.hpp"
#include "CommandInterpreter.hpp"
#include "Definitions.hpp"
#include "Formatting.hpp"
#include "Session.hpp"
#include "datastructure/RailwayNetwork.hpp"

#include <cstddef>
#include <filesystem>
#include <iostream>
#include <memory>
#include <optional>
#include <string>

namespace {
using cda_rail::cli::format_double;
using cda_rail::cli::Session;

void list_networks(const Session& session) {
  const auto networks_directory = session.working_directory() / "networks";
  if (!std::filesystem::is_directory(networks_directory)) {
    std::cout << "No networks directory in "
              << std::filesystem::absolute(session.working_directory()).string()
              << '\n';
    return;
  }
  bool any = false;
  for (const auto& network :
       std::filesystem::directory_iterator(networks_directory)) {
    if (network.is_directory() &&
        std::filesystem::exists(network.path() / "tracks.graphml")) {
      std::cout << network.path().filename().string() << '\n';
      any = true;
    }
  }
  if (!any) {
    std::cout << "No networks found in " << networks_directory.string() << '\n';
  }
}

void print_network_info(const cda_rail::Network& network) {
  std::cout << "Network:   " << network.get_network_name() << '\n';

  // The vertex types are listed in the order of the option map, so that the
  // breakdown reads the same way as '--type' is given.
  std::string type_breakdown;
  for (const auto& [name, type] : cda_rail::cli::vertex_type_map()) {
    std::size_t count = 0;
    for (std::size_t i = 0; i < network.number_of_vertices(); ++i) {
      if (network.get_vertex(i).type == type) {
        ++count;
      }
    }
    if (count > 0) {
      if (!type_breakdown.empty()) {
        type_breakdown += ", ";
      }
      type_breakdown += std::to_string(count) + " " + name;
    }
  }
  std::cout << "Vertices:  " << network.number_of_vertices();
  if (!type_breakdown.empty()) {
    std::cout << " (" << type_breakdown << ")";
  }
  std::cout << '\n';

  std::size_t breakable = 0;
  for (std::size_t i = 0; i < network.number_of_edges(); ++i) {
    if (network.get_edge(i).breakable) {
      ++breakable;
    }
  }
  std::cout << "Edges:     " << network.number_of_edges() << '\n';
  std::cout << "Breakable: " << breakable << '\n';
}

void list_vertices(const cda_rail::Network& network) {
  std::cout << "index  name  type  headway\n";
  for (std::size_t i = 0; i < network.number_of_vertices(); ++i) {
    const auto& vertex = network.get_vertex(i);
    std::cout << i << "  " << vertex.name << "  "
              << cda_rail::cli::vertex_type_to_string(vertex.type) << "  "
              << format_double(vertex.headway) << '\n';
  }
}

void list_edges(const cda_rail::Network& network) {
  std::cout << "index  source  target  length  max_speed  breakable  "
               "min_block_length  min_stop_block_length\n";
  for (std::size_t i = 0; i < network.number_of_edges(); ++i) {
    const auto& edge = network.get_edge(i);
    std::cout << i << "  " << network.get_vertex(edge.source).name << "  "
              << network.get_vertex(edge.target).name << "  "
              << format_double(edge.length) << "  "
              << format_double(edge.max_speed) << "  "
              << (edge.breakable ? "yes" : "no") << "  "
              << format_double(edge.min_block_length) << "  "
              << format_double(edge.min_stop_block_length) << '\n';
  }
}

void list_successors(const cda_rail::Network& network) {
  for (std::size_t i = 0; i < network.number_of_edges(); ++i) {
    std::cout << network.get_edge_name(i) << " ->";
    const auto successors = network.get_successors(i);
    if (successors.empty()) {
      std::cout << " (none)";
    }
    for (const auto& successor : successors) {
      std::cout << ' ' << network.get_edge_name(successor);
    }
    std::cout << '\n';
  }
}

/** @brief The options shared by `edge add` and `edge add-bidirectional`. */
struct EdgeAddOptions {
  std::string source;
  std::string target;
  double      length{0};
  double      max_speed{0};
  bool        breakable{cda_rail::Edge::BREAKABLE_DEFAULT};
  double      min_block_length{cda_rail::Edge::MIN_BLOCK_LENGTH_DEFAULT};
  double min_stop_block_length{cda_rail::Edge::MIN_STOP_BLOCK_LENGTH_DEFAULT};
};

void add_edge_options(CLI::App& cmd, EdgeAddOptions& options) {
  cmd.add_option("source", options.source, "Name of the source vertex")
      ->required();
  cmd.add_option("target", options.target, "Name of the target vertex")
      ->required();
  cmd.add_option("-l,--length", options.length, "Length of the edge in m")
      ->check(CLI::PositiveNumber)
      ->required();
  cmd.add_option("-v,--max-speed", options.max_speed,
                 "Maximal speed on the edge in m/s")
      ->check(CLI::PositiveNumber)
      ->required();
  cmd.add_flag("!-u,!--unbreakable,--breakable", options.breakable,
               "VSS borders may be placed on the edge by default. If this "
               "flag is negated (-u or --unbreakable), they may not.")
      ->capture_default_str();
  cmd.add_option("-m,--min-block-length", options.min_block_length,
                 "Minimal block length in m")
      ->check(CLI::PositiveNumber)
      ->capture_default_str();
  cmd.add_option("-o,--min-stop-block-length", options.min_stop_block_length,
                 "Minimal block length inside a station stop zone in m")
      ->check(CLI::PositiveNumber)
      ->capture_default_str();
}
} // namespace

void cda_rail::cli::add_network_commands(CLI::App& app, Session& session) {
  auto* network_cmd = app.add_subcommand(
      "network", "Create, load, edit and save railway networks");
  network_cmd->require_subcommand(1);

  // ------------------------------------------------------- object commands

  auto* new_cmd =
      network_cmd->add_subcommand("new", "Create a new empty network");
  auto new_name = std::make_shared<std::string>();
  new_cmd->add_option("name", *new_name, "Name of the new network")->required();
  new_cmd->callback([&session, new_name]() {
    session.new_network(*new_name);
    std::cout << "Created network " << *new_name << '\n';
  });

  auto* load_cmd =
      network_cmd->add_subcommand("load", "Load a network from disk");
  auto load_name = std::make_shared<std::string>();
  load_cmd->add_option("name", *load_name, "Name of the network")->required();
  load_cmd->callback([&session, load_name]() {
    session.load_network(*load_name);
    std::cout << "Loaded network " << *load_name << " with "
              << session.const_current_network().number_of_vertices()
              << " vertices and "
              << session.const_current_network().number_of_edges()
              << " edges\n";
  });

  network_cmd
      ->add_subcommand("list", "List the networks in the working directory")
      ->callback([&session]() { list_networks(session); });

  network_cmd->add_subcommand("info", "Show what the current network contains")
      ->callback([&session]() {
        print_network_info(session.const_current_network());
      });

  auto* save_cmd = network_cmd->add_subcommand(
      "save", "Write the current network to the working directory");
  auto  save_name = std::make_shared<std::string>();
  auto* save_as_opt =
      save_cmd->add_option("-a,--as", *save_name,
                           "Rename the network before writing it. Instances "
                           "referencing the old name keep pointing there.");
  save_cmd->callback([&session, save_name, save_as_opt]() {
    const auto path = session.save_network(
        save_as_opt->count() > 0 ? std::optional<std::string>(*save_name)
                                 : std::nullopt);
    std::cout << "Wrote network to " << path.string() << '\n';
    if (session.has_instance()) {
      std::cout << "Note: this network belongs to instance "
                << session.const_instance().get_instance_name()
                << " and is shared by every instance referencing it.\n";
    }
  });

  network_cmd
      ->add_subcommand("reload",
                       "Reread the standalone network from disk, discarding "
                       "unsaved changes")
      ->callback([&session]() {
        session.reload_network();
        std::cout << "Reloaded network "
                  << session.const_current_network().get_network_name() << '\n';
      });

  network_cmd->add_subcommand("close", "Unload the standalone network")
      ->callback([&session]() {
        session.close_network();
        std::cout << "Closed the standalone network.\n";
      });

  auto* rename_cmd =
      network_cmd->add_subcommand("rename", "Rename the current network");
  auto rename_to = std::make_shared<std::string>();
  rename_cmd->add_option("new-name", *rename_to, "New name of the network")
      ->required();
  rename_cmd->callback([&session, rename_to]() {
    const auto old_name = session.const_current_network().get_network_name();
    session.current_network().set_network_name(*rename_to);
    session.mark_network_modified();
    std::cout << "Renamed network " << old_name << " to " << *rename_to
              << ".\nIt will be written to a new folder; the network.json of "
                 "every other instance still points at "
              << old_name << ".\n";
  });

  // --------------------------------------------------------------- vertices

  auto* vertex_cmd =
      network_cmd->add_subcommand("vertex", "Add, change and list vertices");
  vertex_cmd->require_subcommand(1);

  auto* vertex_add_cmd =
      vertex_cmd->add_subcommand("add", "Add a vertex to the network");
  struct VertexAddOptions {
    std::string name;
    VertexType  type{VertexType::NoBorder};
    double      headway{Vertex::HEADWAY_DEFAULT};
  };
  auto vertex_add = std::make_shared<VertexAddOptions>();
  vertex_add_cmd->add_option("name", vertex_add->name, "Name of the vertex")
      ->required();
  vertex_add_cmd
      ->add_option("-t,--type", vertex_add->type,
                   "Type of the vertex. One of 'NoBorder', 'VSS', 'TTD', and "
                   "'NoBorderVSS'.")
      ->transform(CLI::CheckedTransformer(vertex_type_map(), CLI::ignore_case))
      ->required();
  vertex_add_cmd
      ->add_option("-w,--headway", vertex_add->headway,
                   "Additional minimal headway at this vertex in s")
      ->check(CLI::NonNegativeNumber)
      ->capture_default_str();
  vertex_add_cmd->callback([&session, vertex_add]() {
    const auto index = session.current_network().add_vertex(
        vertex_add->name, vertex_add->type, vertex_add->headway);
    session.mark_network_modified();
    std::cout << "Added vertex " << vertex_add->name << " with index " << index
              << '\n';
  });

  auto* vertex_change_cmd = vertex_cmd->add_subcommand(
      "change", "Change the properties of an existing vertex");
  struct VertexChangeOptions {
    std::string vertex;
    std::string name;
    VertexType  type{VertexType::NoBorder};
    double      headway{Vertex::HEADWAY_DEFAULT};
  };
  auto vertex_change = std::make_shared<VertexChangeOptions>();
  vertex_change_cmd
      ->add_option("vertex", vertex_change->vertex,
                   "Name of the vertex to change")
      ->required();
  auto* vertex_name_opt = vertex_change_cmd->add_option(
      "-n,--name", vertex_change->name, "New name of the vertex");
  auto* vertex_type_opt =
      vertex_change_cmd
          ->add_option("-t,--type", vertex_change->type, "New type")
          ->transform(
              CLI::CheckedTransformer(vertex_type_map(), CLI::ignore_case));
  auto* vertex_headway_opt =
      vertex_change_cmd
          ->add_option("-w,--headway", vertex_change->headway,
                       "New additional minimal headway in s")
          ->check(CLI::NonNegativeNumber);
  vertex_change_cmd->callback([&session, vertex_change, vertex_name_opt,
                               vertex_type_opt, vertex_headway_opt]() {
    auto& network = session.current_network();
    if (vertex_type_opt->count() > 0) {
      network.change_vertex_type(vertex_input(vertex_change->vertex),
                                 vertex_change->type);
    }
    if (vertex_headway_opt->count() > 0) {
      network.change_vertex_headway(vertex_input(vertex_change->vertex),
                                    vertex_change->headway);
    }
    // Renaming last, so that the lookups above still find the old name.
    if (vertex_name_opt->count() > 0) {
      network.change_vertex_name(vertex_input(vertex_change->vertex),
                                 vertex_change->name);
    }
    session.mark_network_modified();
    std::cout << "Changed vertex " << vertex_change->vertex << '\n';
  });

  vertex_cmd->add_subcommand("list", "List all vertices")
      ->callback(
          [&session]() { list_vertices(session.const_current_network()); });

  // ------------------------------------------------------------------ edges

  auto* edge_cmd =
      network_cmd->add_subcommand("edge", "Add, change and list edges");
  edge_cmd->require_subcommand(1);

  auto* edge_add_cmd =
      edge_cmd->add_subcommand("add", "Add a directed edge to the network");
  auto edge_add = std::make_shared<EdgeAddOptions>();
  add_edge_options(*edge_add_cmd, *edge_add);
  edge_add_cmd->callback([&session, edge_add]() {
    const auto index = session.current_network().add_edge(
        vertex_input(edge_add->source), vertex_input(edge_add->target),
        edge_add->length, edge_add->max_speed, edge_add->breakable,
        edge_add->min_block_length, edge_add->min_stop_block_length);
    session.mark_network_modified();
    std::cout << "Added edge " << edge_add->source << "-" << edge_add->target
              << " with index " << index << '\n';
  });

  auto* edge_add_bi_cmd = edge_cmd->add_subcommand(
      "add-bidirectional", "Add an edge in both directions");
  auto edge_add_bi = std::make_shared<EdgeAddOptions>();
  add_edge_options(*edge_add_bi_cmd, *edge_add_bi);
  edge_add_bi_cmd->callback([&session, edge_add_bi]() {
    const auto [forward, reverse] =
        session.current_network().add_bidirectional_edge(
            vertex_input(edge_add_bi->source),
            vertex_input(edge_add_bi->target), edge_add_bi->length,
            edge_add_bi->max_speed, edge_add_bi->breakable,
            edge_add_bi->min_block_length, edge_add_bi->min_stop_block_length);
    session.mark_network_modified();
    std::cout << "Added edges " << edge_add_bi->source << "-"
              << edge_add_bi->target << " (index " << forward << ") and "
              << edge_add_bi->target << "-" << edge_add_bi->source << " (index "
              << reverse << ")\n";
  });

  auto* edge_change_cmd = edge_cmd->add_subcommand(
      "change", "Change the properties of an existing edge");
  struct EdgeChangeOptions {
    std::string source;
    std::string target;
    double      length{0};
    double      max_speed{0};
    bool        breakable{Edge::BREAKABLE_DEFAULT};
    double      min_block_length{0};
    double      min_stop_block_length{0};
  };
  auto edge_change = std::make_shared<EdgeChangeOptions>();
  edge_change_cmd
      ->add_option("source", edge_change->source, "Source vertex of the edge")
      ->required();
  edge_change_cmd
      ->add_option("target", edge_change->target, "Target vertex of the edge")
      ->required();
  auto* edge_length_opt =
      edge_change_cmd
          ->add_option("-l,--length", edge_change->length, "New length in m")
          ->check(CLI::PositiveNumber);
  auto* edge_speed_opt =
      edge_change_cmd
          ->add_option("-v,--max-speed", edge_change->max_speed,
                       "New maximal speed in m/s")
          ->check(CLI::PositiveNumber);
  auto* edge_breakable_opt = edge_change_cmd->add_flag(
      "!-u,!--unbreakable,--breakable", edge_change->breakable,
      "Make the edge breakable, or unbreakable if the flag is negated (-u or "
      "--unbreakable)");
  auto* edge_min_block_opt =
      edge_change_cmd
          ->add_option("-m,--min-block-length", edge_change->min_block_length,
                       "New minimal block length in m")
          ->check(CLI::PositiveNumber);
  auto* edge_min_stop_block_opt =
      edge_change_cmd
          ->add_option("-o,--min-stop-block-length",
                       edge_change->min_stop_block_length,
                       "New minimal block length inside a station stop zone "
                       "in m")
          ->check(CLI::PositiveNumber);
  edge_change_cmd->callback([&session, edge_change, edge_length_opt,
                             edge_speed_opt, edge_breakable_opt,
                             edge_min_block_opt, edge_min_stop_block_opt]() {
    auto&      network = session.current_network();
    const auto edge    = edge_input(edge_change->source, edge_change->target);
    if (edge_length_opt->count() > 0) {
      network.change_edge_length(edge, edge_change->length);
    }
    if (edge_speed_opt->count() > 0) {
      network.change_edge_max_speed(edge, edge_change->max_speed);
    }
    if (edge_breakable_opt->count() > 0) {
      network.change_edge_breakable_status(edge, edge_change->breakable);
    }
    if (edge_min_block_opt->count() > 0) {
      network.change_edge_min_block_length(edge, edge_change->min_block_length);
    }
    if (edge_min_stop_block_opt->count() > 0) {
      network.change_edge_min_stop_block_length(
          edge, edge_change->min_stop_block_length);
    }
    session.mark_network_modified();
    std::cout << "Changed edge " << edge_change->source << "-"
              << edge_change->target << '\n';
  });

  edge_cmd->add_subcommand("list", "List all edges")->callback([&session]() {
    list_edges(session.const_current_network());
  });

  // ------------------------------------------------------------- successors

  auto* successor_cmd = network_cmd->add_subcommand(
      "successor", "Declare which edge may follow which other edge");
  successor_cmd->require_subcommand(1);

  auto* successor_add_cmd = successor_cmd->add_subcommand(
      "add", "Allow the second edge to follow the first one");
  struct SuccessorOptions {
    std::string in_source;
    std::string in_target;
    std::string out_source;
    std::string out_target;
  };
  auto successor_add = std::make_shared<SuccessorOptions>();
  successor_add_cmd
      ->add_option("in-source", successor_add->in_source,
                   "Source vertex of the incoming edge")
      ->required();
  successor_add_cmd
      ->add_option("in-target", successor_add->in_target,
                   "Target vertex of the incoming edge")
      ->required();
  successor_add_cmd
      ->add_option("out-source", successor_add->out_source,
                   "Source vertex of the outgoing edge")
      ->required();
  successor_add_cmd
      ->add_option("out-target", successor_add->out_target,
                   "Target vertex of the outgoing edge")
      ->required();
  successor_add_cmd->callback([&session, successor_add]() {
    session.current_network().add_successor(
        edge_input(successor_add->in_source, successor_add->in_target),
        edge_input(successor_add->out_source, successor_add->out_target));
    session.mark_network_modified();
    std::cout << "Added successor " << successor_add->out_source << "-"
              << successor_add->out_target << " of " << successor_add->in_source
              << "-" << successor_add->in_target << '\n';
  });

  successor_cmd->add_subcommand("list", "List the successors of every edge")
      ->callback(
          [&session]() { list_successors(session.const_current_network()); });
}
