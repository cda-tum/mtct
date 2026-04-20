#pragma once
#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "VSSModel.hpp"

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <numeric>
#include <optional>
#include <ranges>
#include <string>
#include <tinyxml2.h>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <variant>
#include <vector>

namespace cda_rail {

struct Vertex {
  /**
   * Vertex object
   * @param name Name of the vertex
   * @param type Type of the vertex (NoBorder, VSS, TTD, NoBorderVSS)
   * @param headway Additional headway imposed on vertex, default 0.0
   */

  constexpr static double HEADWAY_DEFAULT{0.0};

  std::string name;
  VertexType  type;
  double      headway{HEADWAY_DEFAULT};

  // Constructors
  Vertex(std::string_view const name, VertexType const type,
         double const headway = HEADWAY_DEFAULT)
      : name(name), type(type), headway(headway) {};
  Vertex(std::string_view const name, VertexType const type,
         std::optional<double> const& headway)
      : name(name), type(type), headway(headway.value_or(HEADWAY_DEFAULT)) {};

  // Operators
  bool operator==(Vertex const& other) const {
    return name == other.name && type == other.type && headway == other.headway;
  }
};

struct Edge {
  /**
   * Edge object
   * @param source Source vertex index
   * @param target Target vertex index
   * @param length Length of vertex (in m)
   * @param max_speed Speed limit of vertex (in m/s)
   * @param breakable Boolean indicating if VSS can be placed on this edge
   * @param min_block_length Minimum block length (in m). Optional, default 1.
   * @param min_stop_block_length Minimum block length (in m) within a station.
   * Optional, default 100.
   */

  constexpr static bool   BREAKABLE_DEFAULT             = true;
  constexpr static double MIN_BLOCK_LENGTH_DEFAULT      = 1.0;
  constexpr static double MIN_STOP_BLOCK_LENGTH_DEFAULT = 100.0;

  size_t source;
  size_t target;
  double length;
  double max_speed;
  bool   breakable{BREAKABLE_DEFAULT};
  double min_block_length{MIN_BLOCK_LENGTH_DEFAULT};
  double min_stop_block_length{MIN_STOP_BLOCK_LENGTH_DEFAULT};

  // Constructors
  Edge(size_t const source, size_t const target, double const length,
       double const maxSpeed, bool const breakable = BREAKABLE_DEFAULT,
       double const minBlockLength     = MIN_BLOCK_LENGTH_DEFAULT,
       double const minStopBlockLength = MIN_STOP_BLOCK_LENGTH_DEFAULT)
      : source(source), target(target), length(length), max_speed(maxSpeed),
        breakable(breakable), min_block_length(minBlockLength),
        min_stop_block_length(minStopBlockLength) {};
  Edge(size_t const source, size_t const target, double const length,
       double const maxSpeed, std::optional<bool> const& breakable,
       std::optional<double> const& min_block_length,
       std::optional<double> const& min_stop_block_length)
      : source(source), target(target), length(length), max_speed(maxSpeed),
        breakable(breakable.value_or(BREAKABLE_DEFAULT)),
        min_block_length(min_block_length.value_or(MIN_BLOCK_LENGTH_DEFAULT)),
        min_stop_block_length(
            min_stop_block_length.value_or(MIN_STOP_BLOCK_LENGTH_DEFAULT)) {};

  // Operators
  bool operator==(const Edge& other) const {
    return (source == other.source && target == other.target &&
            length == other.length && max_speed == other.max_speed &&
            breakable == other.breakable &&
            min_block_length == other.min_block_length &&
            min_stop_block_length == other.min_stop_block_length);
  }
};

class Network {
  /**
   * Graph class
   *
   */
public:
  // Helper struct
  struct VertexInput {
  private:
    friend class Network;
    std::variant<size_t, std::string_view, Vertex> m_data;

  public:
    // Implicit constructors: no explicit keyword on purpose
    // NOLINTBEGIN(google-explicit-constructor)
    VertexInput(size_t i) : m_data(i) {}
    VertexInput(std::string_view v_name) : m_data(v_name) {}
    VertexInput(Vertex vertex) : m_data(std::move(vertex)) {}
    // NOLINTEND(google-explicit-constructor)

  private:
    // Resolve edge index
    size_t resolve(Network const* const network) const;
  };

  struct EdgeInput {
  private:
    friend class Network;

    std::variant<size_t, std::pair<size_t, size_t>,
                 std::pair<std::string_view, std::string_view>, Edge>
        m_data;

  public:
    // Implicit constructors: no explicit keyword on purpose
    // NOLINTBEGIN(google-explicit-constructor)
    EdgeInput(size_t i) : m_data(i) {}
    EdgeInput(std::pair<size_t, size_t> p) : m_data(p) {}
    EdgeInput(size_t a, size_t b) : m_data(std::pair{a, b}) {}
    EdgeInput(std::pair<std::string_view, std::string_view> p) : m_data(p) {}
    EdgeInput(std::string_view a, std::string_view b)
        : m_data(std::pair{a, b}) {}
    EdgeInput(Edge edge) : m_data(std::move(edge)) {}
    // NOLINTEND(google-explicit-constructor)

  private:
    // Resolve edge index
    size_t resolve(Network const* const network) const;
  };

private:
  // the class ensures that m_network_name is a valid folder name on any
  // operating system as enforced by throw_if_invalid_folder_name
  std::string m_network_name{"UnnamedNetwork"};

  std::vector<Vertex> m_vertices;
  std::vector<Edge>   m_edges;
  std::vector<cda_rail::index_set>
      m_successors; // for every edge, set of possible successor edges
  std::unordered_map<std::string, size_t> m_vertex_name_to_index;

  std::unordered_map<std::size_t, std::pair<size_t, double>>
      m_new_edge_to_old_edge_after_transform; // needed if edges are discretized

public:
  // -----------------------------
  // CONSTRUCTORS
  // -----------------------------

  Network() = default;

  explicit Network(std::filesystem::path const& working_directory,
                   std::string_view const       networkName = "UnnamedNetwork");
  explicit Network(std::string const&     working_directory,
                   std::string_view const networkName = "UnnamedNetwork")
      : Network(std::filesystem::path(working_directory), networkName) {};
  explicit Network(char const* const      working_directory,
                   std::string_view const networkName = "UnnamedNetwork")
      : Network(std::filesystem::path(working_directory), networkName) {};

  // Rule of 0 suffices

  // -----------------------------
  // IMPORT / EXPORT
  // -----------------------------

  [[nodiscard]] static Network
  import_network(std::filesystem::path const& working_directory,
                 std::string_view const       networkName = "UnnamedNetwork") {
    return Network(working_directory, networkName);
  };
  [[nodiscard]] static Network
  import_network(std::string const&     working_directory,
                 std::string_view const networkName = "UnnamedNetwork") {
    return Network(working_directory, networkName);
  };
  [[nodiscard]] static Network
  import_network(char const* const      workingDirectory,
                 std::string_view const networkName = "UnnamedNetwork") {
    return Network(workingDirectory, networkName);
  };
  void export_network(std::filesystem::path const& working_directory) const;
  void export_network(std::string const& working_directory) const {
    export_network(std::filesystem::path(working_directory));
  };
  void export_network(char const* const workingDirectory) const {
    export_network(std::filesystem::path(workingDirectory));
  };

  // -----------------------------
  // GETTER
  // -----------------------------

  // Existence Helper
  [[nodiscard]] bool has_vertex(size_t index) const {
    return (index < m_vertices.size());
  };
  [[nodiscard]] bool has_vertex(std::string_view const name) const {
    return m_vertex_name_to_index.contains(std::string{name});
  };

private:
  [[nodiscard]] bool has_edge_helper(size_t source_id, size_t target_id) const;

public:
  [[nodiscard]] bool has_edge(size_t const index) const {
    return (index < m_edges.size());
  };
  [[nodiscard]] bool has_edge(VertexInput const& source,
                              VertexInput const& target) const {
    return has_edge_helper(source.resolve(this), target.resolve(this));
  }

  // Property Existence Helpers

private:
  [[nodiscard]] bool is_valid_successor_helper(size_t e0, size_t e1) const;

public:
  [[nodiscard]] bool is_valid_successor(EdgeInput const& edge_in,
                                        EdgeInput const& edge_out) const {
    return is_valid_successor_helper(edge_in.resolve(this),
                                     edge_out.resolve(this));
  };

private:
  [[nodiscard]] bool is_adjustable_helper(size_t vertex_id) const;

public:
  [[nodiscard]] bool is_adjustable(VertexInput const& vertex) const {
    return is_adjustable_helper(vertex.resolve(this));
  };

  [[nodiscard]] bool is_consistent_for_transformation() const;

  // Simple Getter

  [[nodiscard]] size_t number_of_vertices() const { return m_vertices.size(); };
  [[nodiscard]] size_t number_of_edges() const { return m_edges.size(); };

  [[nodiscard]] std::string const& get_network_name() const {
    return m_network_name;
  }

  [[nodiscard]] std::vector<Vertex> const& get_vertices() const {
    return m_vertices;
  };
  [[nodiscard]] cda_rail::index_vector
  get_vertices_by_type(VertexType type) const;

  [[nodiscard]] const std::vector<Edge>& get_edges() const { return m_edges; };

private:
  [[nodiscard]] std::pair<size_t, double>
  get_old_edge_helper(size_t new_edge) const;

public:
  [[nodiscard]] std::pair<size_t, double>
  get_old_edge(EdgeInput const& edge) const {
    return get_old_edge_helper(edge.resolve(this));
  };

  [[nodiscard]] size_t get_vertex_index(std::string_view name) const;

private:
  [[nodiscard]] const Vertex& get_vertex_helper(size_t index) const;

public:
  [[nodiscard]] const Vertex& get_vertex(VertexInput const& vertex) const {
    return get_vertex_helper(vertex.resolve(this));
  }

private:
  [[nodiscard]] size_t get_edge_index_helper(size_t source_id,
                                             size_t target_id) const;

public:
  [[nodiscard]] size_t get_edge_index(VertexInput const& source,
                                      VertexInput const& target) const {
    return get_edge_index_helper(source.resolve(this), target.resolve(this));
  }

private:
  [[nodiscard]] const Edge& get_edge_helper(size_t index) const;

public:
  [[nodiscard]] const Edge& get_edge(EdgeInput const& edge) const {
    return get_edge_helper(edge.resolve(this));
  }

private:
  [[nodiscard]] std::string get_edge_name_helper(std::string_view v1,
                                                 std::string_view v2,
                                                 bool check_existence) const;
  [[nodiscard]] std::string get_edge_name_helper(size_t v0, size_t v1,
                                                 bool check_existence) const {
    return get_edge_name_helper(get_vertex(v0).name, get_vertex(v1).name,
                                check_existence);
  }

public:
  [[nodiscard]] std::string get_edge_name(EdgeInput const& edge) const {
    const auto& edge_object = get_edge(edge.resolve(this));
    return get_edge_name_helper(get_vertex(edge_object.source).name,
                                get_vertex(edge_object.target).name, false);
  }

  // Graph Neighbor Helper

private:
  [[nodiscard]] cda_rail::index_set out_edges_helper(size_t index) const;
  [[nodiscard]] cda_rail::index_set in_edges_helper(size_t index) const;
  [[nodiscard]] cda_rail::index_set
  neighboring_edges_helper(size_t index) const;

public:
  [[nodiscard]] cda_rail::index_set out_edges(VertexInput const& vertex) const {
    return out_edges_helper(vertex.resolve(this));
  };
  [[nodiscard]] cda_rail::index_set in_edges(VertexInput const& vertex) const {
    return in_edges_helper(vertex.resolve(this));
  };
  [[nodiscard]] cda_rail::index_set
  neighboring_edges(VertexInput const& vertex) const {
    return neighboring_edges_helper(vertex.resolve(this));
  };

private:
  [[nodiscard]] cda_rail::index_set get_predecessors_helper(size_t index) const;
  [[nodiscard]] const cda_rail::index_set&
  get_successors_helper(size_t index) const;

public:
  [[nodiscard]] cda_rail::index_set
  get_predecessors(EdgeInput const& edge) const {
    return get_predecessors_helper(edge.resolve(this));
  }
  [[nodiscard]] cda_rail::index_set const&
  get_successors(EdgeInput const& edge) const {
    return get_successors_helper(edge.resolve(this));
  }

private:
  [[nodiscard]] cda_rail::index_set neighbors_helper(size_t index) const;

public:
  [[nodiscard]] cda_rail::index_set neighbors(VertexInput const& vertex) const {
    return neighbors_helper(vertex.resolve(this));
  };

  // Overlap Getters
private:
  [[nodiscard]] std::optional<size_t>
  common_vertex_helper(size_t e_idx_1, size_t e_idx_2) const;

public:
  [[nodiscard]] std::optional<size_t>
  common_vertex(EdgeInput const& edge1, EdgeInput const& edge2) const {
    return common_vertex_helper(edge1.resolve(this), edge2.resolve(this));
  };

  // Reverse Edge Getters

private:
  [[nodiscard]] std::optional<size_t>
  get_reverse_edge_index_helper(size_t edge_index) const;

public:
  [[nodiscard]] std::optional<size_t>
  get_reverse_edge_index(EdgeInput const& edge) const {
    return get_reverse_edge_index_helper(edge.resolve(this));
  };
  [[nodiscard]] std::optional<size_t>
  get_optional_reverse_edge_index(std::optional<size_t> const edgeIndex) const {
    return edgeIndex.has_value()
               ? get_reverse_edge_index_helper(edgeIndex.value())
               : std::optional<size_t>();
  }

  [[nodiscard]] std::vector<
      std::pair<std::optional<size_t>, std::optional<size_t>>>
  combine_reverse_edges(const cda_rail::index_vector& edges_to_consider,
                        bool                          sort = false) const;

private:
  [[nodiscard]] size_t get_track_index_helper(size_t edge_index) const;

public:
  [[nodiscard]] size_t get_track_index(EdgeInput const& edge) const {
    return get_track_index_helper(edge.resolve(this));
  };

  // Special sets of edges

  [[nodiscard]] cda_rail::index_vector breakable_edges() const;
  [[nodiscard]] cda_rail::index_vector relevant_breakable_edges() const;
  [[nodiscard]] std::vector<cda_rail::index_set> unbreakable_sections() const;
  [[nodiscard]] std::vector<cda_rail::index_set> no_border_vss_sections() const;

private:
  [[nodiscard]] cda_rail::index_set
  get_unbreakable_section_containing_edge_helper(size_t e) const;
  [[nodiscard]] bool is_on_same_unbreakable_section_helper(size_t e1,
                                                           size_t e2) const;

public:
  [[nodiscard]] cda_rail::index_set
  get_unbreakable_section_containing_edge(EdgeInput const& edge) const {
    return get_unbreakable_section_containing_edge_helper(edge.resolve(this));
  };
  [[nodiscard]] bool
  is_on_same_unbreakable_section(EdgeInput const& edge1,
                                 EdgeInput const& edge2) const {
    return is_on_same_unbreakable_section_helper(edge1.resolve(this),
                                                 edge2.resolve(this));
  };

  // Other Getters

  [[nodiscard]] cda_rail::index_set
  vertices_used_by_edges(const cda_rail::index_set& edges_tmp) const;

  [[nodiscard]] static std::vector<std::pair<size_t, size_t>>
  get_intersecting_ttd(const cda_rail::index_vector&           edges,
                       const std::vector<cda_rail::index_set>& ttd);

  [[nodiscard]] cda_rail::index_set
  edge_set_complement(const cda_rail::index_set& edge_indices) const;
  [[nodiscard]] cda_rail::index_set
  edge_set_complement(const cda_rail::index_set& edge_indices,
                      const cda_rail::index_set& edges_to_consider) const;

  // Simple Calculation Functions

private:
  [[nodiscard]] double maximal_vertex_speed_helper(
      size_t vertex_id, const cda_rail::index_set& edges_to_consider) const;

public:
  [[nodiscard]] double maximal_vertex_speed(
      VertexInput const&         vertex,
      const cda_rail::index_set& edges_to_consider = {}) const {
    return maximal_vertex_speed_helper(vertex.resolve(this), edges_to_consider);
  };

private:
  [[nodiscard]] double minimal_neighboring_edge_length_helper(
      size_t v, const cda_rail::index_set& edges_to_consider) const;

public:
  [[nodiscard]] double minimal_neighboring_edge_length(
      VertexInput const&         vertex,
      const cda_rail::index_set& edges_to_consider = {}) const {
    return minimal_neighboring_edge_length_helper(vertex.resolve(this),
                                                  edges_to_consider);
  };

private:
  [[nodiscard]] size_t max_vss_on_edge_helper(size_t index) const;

public:
  [[nodiscard]] size_t max_vss_on_edge(EdgeInput const& edge) const {
    return max_vss_on_edge_helper(edge.resolve(this));
  };

  // -----------------------------
  // SETTER
  // -----------------------------

  // Simple Setter

  void set_network_name(std::string_view const networkName) {
    exceptions::throw_if_invalid_folder_name(networkName);
    m_network_name = networkName;
  }

private:
  void change_vertex_name_helper(size_t index, std::string_view new_name);
  void change_vertex_type_helper(size_t index, VertexType new_type);
  void change_vertex_headway_helper(size_t index, double new_headway);

public:
  void change_vertex_name(VertexInput const&     vertex,
                          std::string_view const new_name) {
    change_vertex_name_helper(vertex.resolve(this), new_name);
  };
  void change_vertex_type(VertexInput const& vertex,
                          VertexType const   new_type) {
    change_vertex_type_helper(vertex.resolve(this), new_type);
  };
  void change_vertex_headway(VertexInput const& vertex,
                             double const       new_headway) {
    change_vertex_headway_helper(vertex.resolve(this), new_headway);
  };

private:
  void change_edge_length_helper(size_t index, double new_length);
  void change_edge_max_speed_helper(size_t index, double new_max_speed);
  void change_edge_min_block_length_helper(size_t index,
                                           double new_min_block_length);
  void
  change_edge_min_stop_block_length_helper(size_t index,
                                           double new_min_stop_block_length);
  void change_edge_breakable_helper(size_t index, bool value);

public:
  void change_edge_length(EdgeInput const& edge, double new_length) {
    change_edge_length_helper(edge.resolve(this), new_length);
  };
  void change_edge_max_speed(EdgeInput const& edge, double new_max_speed) {
    change_edge_max_speed_helper(edge.resolve(this), new_max_speed);
  }
  void change_edge_min_block_length(EdgeInput const& edge,
                                    double           new_min_block_length) {
    change_edge_min_block_length_helper(edge.resolve(this),
                                        new_min_block_length);
  }
  void change_edge_min_stop_block_length(EdgeInput const& edge,
                                         double new_min_stop_block_length) {
    change_edge_min_stop_block_length_helper(edge.resolve(this),
                                             new_min_stop_block_length);
  }
  void set_edge_breakable(EdgeInput const& edge) {
    change_edge_breakable_status(edge, true);
  };
  void set_edge_unbreakable(EdgeInput const& edge) {
    change_edge_breakable_status(edge, false);
  };
  void change_edge_breakable_status(EdgeInput const& edge,
                                    bool const       breakable) {
    change_edge_breakable_helper(edge.resolve(this), breakable);
  };

  // Simple Network Editing Functions

  size_t                             add_vertex(Vertex vertex);
  template <typename... Args> size_t add_vertex(Args... args) {
    return add_vertex(Vertex(std::forward<Args>(args)...));
  };

private:
  size_t add_edge_helper(size_t source, size_t target, double length,
                         double max_speed, std::optional<bool> const& breakable,
                         std::optional<double> const& min_block_length,
                         std::optional<double> const& min_stop_block_length);

public:
  size_t add_edge(VertexInput const& source, VertexInput const& target,
                  double const length, double const maxSpeed,
                  std::optional<bool> const&   breakable             = {},
                  std::optional<double> const& min_block_length      = {},
                  std::optional<double> const& min_stop_block_length = {}) {
    return add_edge_helper(source.resolve(this), target.resolve(this), length,
                           maxSpeed, breakable, min_block_length,
                           min_stop_block_length);
  }

private:
  void add_successor_helper(size_t edge_in, size_t edge_out);

public:
  void add_successor(EdgeInput const& edge_in, EdgeInput const& edge_out) {
    // Pass 'this' to the inputs so they can resolve themselves
    add_successor_helper(edge_in.resolve(this), edge_out.resolve(this));
  }

  // ----------------------
  // Transformation Functions
  // ----------------------

  std::pair<cda_rail::index_vector, cda_rail::index_vector> separate_edge(
      EdgeInput const&               edge,
      const vss::SeparationFunction& sep_func = &vss::functions::uniform) {
    auto const edge_index = edge.resolve(this);
    return separate_edge_private_helper(
        edge_index, get_edge(edge_index).min_block_length, sep_func);
  };
  std::pair<cda_rail::index_vector, cda_rail::index_vector>
  separate_stop_edge(EdgeInput const& edge) {
    auto const edge_index = edge.resolve(this);
    return separate_edge_private_helper(
        edge_index, get_edge(edge_index).min_stop_block_length,
        &vss::functions::uniform, true);
  };
  std::vector<std::pair<size_t, cda_rail::index_vector>>
  separate_stop_edges(const cda_rail::index_vector& stop_edges);

  std::vector<std::pair<size_t, cda_rail::index_vector>> discretize(
      const vss::SeparationFunction& sep_func = &vss::functions::uniform);

  // ------------------------
  // Path Finding Algorithms
  // ------------------------

  [[nodiscard]] std::vector<cda_rail::index_vector>
  all_paths_of_length_starting_in_vertex(
      size_t v, double desired_len, std::optional<size_t> exit_node = {},
      cda_rail::index_set edges_to_consider         = {},
      bool                return_successors_if_zero = false) const {
    return all_routes_of_given_length(v, std::nullopt, desired_len, false,
                                      exit_node, std::move(edges_to_consider),
                                      return_successors_if_zero);
  };
  [[nodiscard]] std::vector<cda_rail::index_vector>
  all_paths_of_length_starting_in_edge(
      size_t e, double desired_len, std::optional<size_t> exit_node = {},
      cda_rail::index_set edges_to_consider = {}) const {
    return all_routes_of_given_length(std::nullopt, e, desired_len, false,
                                      exit_node, std::move(edges_to_consider));
  };
  [[nodiscard]] std::vector<cda_rail::index_vector>
  all_paths_of_length_ending_in_vertex(
      size_t v, double desired_len, std::optional<size_t> exit_node = {},
      cda_rail::index_set edges_to_consider = {}) const {
    return all_routes_of_given_length(v, std::nullopt, desired_len, true,
                                      exit_node, std::move(edges_to_consider));
  };
  [[nodiscard]] std::vector<cda_rail::index_vector>
  all_paths_of_length_ending_in_edge(
      size_t const e, double const desiredLen,
      std::optional<size_t> exit_node         = {},
      cda_rail::index_set   edges_to_consider = {}) const {
    return all_routes_of_given_length(std::nullopt, e, desiredLen, true,
                                      std::move(exit_node),
                                      std::move(edges_to_consider));
  }
  [[nodiscard]] std::vector<cda_rail::index_vector>
  all_paths_ending_at_ttd(size_t                                  e_0,
                          const std::vector<cda_rail::index_set>& ttd_sections,
                          std::optional<size_t> exit_node) const;

  // Shortest Paths

  [[nodiscard]] double length_of_path(const cda_rail::index_vector& path) const;

  [[nodiscard]] std::vector<std::vector<double>>
  all_edge_pairs_shortest_paths() const;

private:
  [[nodiscard]] std::optional<double>
  shortest_path_helper(size_t source_edge_id, size_t target_id,
                       bool target_is_edge     = false,
                       bool include_first_edge = false,
                       bool use_minimal_time = false, double max_v = INF) const;

public:
  [[nodiscard]] std::optional<double> shortest_path_from_edge_to_edge(
      EdgeInput const& source_edge, EdgeInput const& target_edge,
      bool include_first_edge = false, bool use_minimal_time = false,
      double max_v = INF) const {
    return shortest_path_helper(source_edge.resolve(this),
                                target_edge.resolve(this), true,
                                include_first_edge, use_minimal_time, max_v);
  };
  [[nodiscard]] std::optional<double> shortest_path_from_edge_to_vertex(
      EdgeInput const& source_edge, VertexInput const& target_vertex,
      bool include_first_edge = false, bool use_minimal_time = false,
      double max_v = INF) const {
    return shortest_path_helper(source_edge.resolve(this),
                                target_vertex.resolve(this), false,
                                include_first_edge, use_minimal_time, max_v);
  };

private:
  [[nodiscard]] std::pair<std::optional<double>, cda_rail::index_vector>
  shortest_path_using_edges_helper(size_t source_edge_id,
                                   size_t target_vertex_id,
                                   bool   only_use_valid_successors   = true,
                                   cda_rail::index_set edges_to_use   = {},
                                   bool                target_is_edge = false,
                                   bool   include_first_edge          = false,
                                   bool   use_minimal_time            = false,
                                   double max_v = INF) const {
    return shortest_path_between_sets_using_edges_helper(
        cda_rail::index_set{source_edge_id},
        cda_rail::index_set{target_vertex_id}, only_use_valid_successors,
        std::move(edges_to_use), target_is_edge, include_first_edge,
        use_minimal_time, max_v);
  };
  [[nodiscard]] std::pair<std::optional<double>, cda_rail::index_vector>
  shortest_path_between_sets_using_edges_helper(
      cda_rail::index_set source_edge_ids, cda_rail::index_set target_ids,
      bool                only_use_valid_successors = true,
      cda_rail::index_set edges_to_use = {}, bool target_is_edge = false,
      bool include_first_edge = false, bool use_minimal_time = false,
      double max_v = INF) const;
  [[nodiscard]] std::optional<double> shortest_path_length_between_sets_helper(
      cda_rail::index_set source_edge_ids, cda_rail::index_set target_ids,
      bool target_is_edge = false, bool include_first_edge = false,
      bool use_minimal_time = false, double max_v = INF) const {
    return shortest_path_between_sets_using_edges_helper(
               std::move(source_edge_ids), std::move(target_ids), true, {},
               target_is_edge, include_first_edge, use_minimal_time, max_v)
        .first;
  };

public:
  [[nodiscard]] std::pair<std::optional<double>, cda_rail::index_vector>
  shortest_edge_to_edge_path(EdgeInput const& source_edge,
                             EdgeInput const& target_edge,
                             bool             only_use_valid_successors = true,
                             cda_rail::index_set edges_to_use           = {},
                             bool                include_first_edge     = false,
                             bool                use_minimal_time       = false,
                             double              max_v = INF) const {
    return shortest_path_using_edges_helper(
        source_edge.resolve(this), target_edge.resolve(this),
        only_use_valid_successors, std::move(edges_to_use), true,
        include_first_edge, use_minimal_time, max_v);
  };
  [[nodiscard]] std::pair<std::optional<double>, cda_rail::index_vector>
  shortest_edge_to_vertex_path(EdgeInput const&   source_edge,
                               VertexInput const& target_vertex,
                               bool only_use_valid_successors         = true,
                               cda_rail::index_set edges_to_use       = {},
                               bool                include_first_edge = false,
                               bool                use_minimal_time   = false,
                               double              max_v = INF) const {
    return shortest_path_using_edges_helper(
        source_edge.resolve(this), target_vertex.resolve(this),
        only_use_valid_successors, std::move(edges_to_use), false,
        include_first_edge, use_minimal_time, max_v);
  };
  [[nodiscard]] std::pair<std::optional<double>, cda_rail::index_vector>
  shortest_path_between_edge_sets(cda_rail::index_set source_edge_ids,
                                  cda_rail::index_set target_edge_ids,
                                  bool only_use_valid_successors   = true,
                                  cda_rail::index_set edges_to_use = {},
                                  bool   include_first_edge        = false,
                                  bool   use_minimal_time          = false,
                                  double max_v = INF) const {
    return shortest_path_between_sets_using_edges_helper(
        std::move(source_edge_ids), std::move(target_edge_ids),
        only_use_valid_successors, std::move(edges_to_use), true,
        include_first_edge, use_minimal_time, max_v);
  };
  [[nodiscard]] std::pair<std::optional<double>, cda_rail::index_vector>
  shortest_path_between_edge_and_vertex_set(
      cda_rail::index_set source_edge_ids,
      cda_rail::index_set target_vertex_ids,
      bool                only_use_valid_successors = true,
      cda_rail::index_set edges_to_use = {}, bool include_first_edge = false,
      bool use_minimal_time = false, double max_v = INF) const {
    return shortest_path_between_sets_using_edges_helper(
        std::move(source_edge_ids), std::move(target_vertex_ids),
        only_use_valid_successors, std::move(edges_to_use), false,
        include_first_edge, use_minimal_time, max_v);
  };
  [[nodiscard]] std::optional<double> shortest_path_length_between_edge_sets(
      cda_rail::index_set source_edge_ids, cda_rail::index_set target_ids,
      bool include_first_edge = false, bool use_minimal_time = false,
      double max_v = INF) const {
    return shortest_path_length_between_sets_helper(
        std::move(source_edge_ids), std::move(target_ids), false,
        include_first_edge, use_minimal_time, max_v);
  };
  [[nodiscard]] std::optional<double>
  shortest_path_length_between_edge_and_vertex_set(
      cda_rail::index_set source_edge_ids,
      cda_rail::index_set target_vertex_ids, bool include_first_edge = false,
      bool use_minimal_time = false, double max_v = INF) const {
    return shortest_path_length_between_sets_helper(
        std::move(source_edge_ids), std::move(target_vertex_ids), false,
        include_first_edge, use_minimal_time, max_v);
  };

private:
  // ---------------------
  // Private Helper
  // ---------------------

  // Import helper
  void        read_graphml(const std::filesystem::path& p);
  static void get_keys_inplace(
      tinyxml2::XMLElement* graphml_body, std::optional<std::string>& breakable,
      std::optional<std::string>& length, std::optional<std::string>& max_speed,
      std::optional<std::string>& min_block_length,
      std::optional<std::string>& min_stop_block_length,
      std::optional<std::string>& type, std::optional<std::string>& headway);
  void add_vertices_from_graphml(const tinyxml2::XMLElement*       graphml_node,
                                 const std::optional<std::string>& type,
                                 const std::optional<std::string>& headway);
  static void extract_vertices_from_key_inplace(const std::string& key,
                                                std::string&       source_name,
                                                std::string&       target_name);
  void        add_edges_from_graphml(
      const tinyxml2::XMLElement*       graphml_edge,
      const std::optional<std::string>& breakable,
      const std::optional<std::string>& length,
      const std::optional<std::string>& max_speed,
      const std::optional<std::string>& min_block_length,
      const std::optional<std::string>& min_stop_block_length);
  void read_successors(const std::filesystem::path& p);

  // Export helper
  void export_graphml(const std::filesystem::path& p) const;
  void export_successors_python(const std::filesystem::path& p) const;
  void export_successors_cpp(const std::filesystem::path& p) const;
  void write_successor_set_to_file(std::ofstream& file, size_t i) const;

  // getter helper
  [[nodiscard]] size_t other_vertex(size_t e, size_t v) const {
    return get_edge(e).source == v ? get_edge(e).target : get_edge(e).source;
  };

  // edge separation/changing helper
  std::pair<cda_rail::index_vector, cda_rail::index_vector>
  separate_edge_private_helper(
      size_t edge_index, double min_length,
      const vss::SeparationFunction& sep_func = &vss::functions::uniform,
      bool                           new_edge_breakable = false);
  std::pair<cda_rail::index_vector, cda_rail::index_vector>
       separate_edge_at(size_t                     edge_index,
                        const std::vector<double>& distances_from_source,
                        bool                       new_edge_breakable = false);
  void update_new_old_edge(size_t new_edge, size_t old_edge, double position);
  std::vector<std::pair<std::optional<size_t>, std::optional<size_t>>>
  sort_edge_pairs(
      std::vector<std::pair<std::optional<size_t>, std::optional<size_t>>>&
          edge_pairs) const;

  // validity helper
  void check_new_edge_requirements(size_t source, size_t target) const;

  // path finding algorithm helper
  [[nodiscard]] static double delta_dist_helper(const Edge& successor_edge,
                                                double      max_v,
                                                bool        use_minimal_time);

  // private path finding algorithms
  void
  dfs_inplace(std::vector<cda_rail::index_set>&     ret_val,
              std::unordered_set<size_t>&           vertices_to_visit,
              const VertexType&                     section_type,
              const std::unordered_set<VertexType>& error_types = {}) const;
  [[nodiscard]] std::vector<cda_rail::index_vector>
  all_routes_of_given_length(std::optional<size_t> v_0,
                             std::optional<size_t> e_0, double desired_length,
                             bool                  reverse_direction,
                             std::optional<size_t> exit_node           = {},
                             cda_rail::index_set   edges_used_by_train = {},
                             bool return_successors_if_zero = false) const;
  [[nodiscard]] std::vector<cda_rail::index_vector> all_paths_ending_at_ttd(
      size_t e_0, const std::vector<cda_rail::index_set>& ttd_sections,
      std::optional<size_t> exit_node, std::optional<size_t> safe_ttd,
      bool first_edge) const;
};
} // namespace cda_rail
