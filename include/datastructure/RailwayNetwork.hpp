#pragma once
#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "VSSModel.hpp"

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <numeric>
#include <optional>
#include <string>
#include <tinyxml2.h>
#include <unordered_map>
#include <unordered_set>
#include <utility>
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
};

class Network {
  /**
   * Graph class
   *
   */
private:
  // the class ensures that m_network_name is a valid folder name on any
  // operating system as enforced by throw_if_invalid_folder_name
  std::string m_network_name{"UnnamedNetwork"};

  std::vector<Vertex>                     m_vertices;
  std::vector<Edge>                       m_edges;
  std::vector<cda_rail::index_set>        m_successors;
  std::unordered_map<std::string, size_t> m_vertex_name_to_index;

  std::unordered_map<std::size_t, std::pair<size_t, double>>
      m_new_edge_to_old_edge_after_transform;

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

  void add_edges_from_graphml(
      const tinyxml2::XMLElement*       graphml_edge,
      const std::optional<std::string>& breakable,
      const std::optional<std::string>& length,
      const std::optional<std::string>& max_speed,
      const std::optional<std::string>& min_block_length,
      const std::optional<std::string>& min_stop_block_length);
  void read_successors(const std::filesystem::path& p);

  void export_graphml(const std::filesystem::path& p) const;
  void export_successors_python(const std::filesystem::path& p) const;
  void export_successors_cpp(const std::filesystem::path& p) const;
  void write_successor_set_to_file(std::ofstream& file, size_t i) const;

  void update_new_old_edge(size_t new_edge, size_t old_edge, double position);

  std::pair<cda_rail::index_vector, cda_rail::index_vector>
  separate_edge_private_helper(
      size_t edge_index, double min_length,
      const vss::SeparationFunction& sep_func = &vss::functions::uniform,
      bool                           new_edge_breakable = false);

  std::pair<cda_rail::index_vector, cda_rail::index_vector>
  separate_edge_at(size_t                     edge_index,
                   const std::vector<double>& distances_from_source,
                   bool                       new_edge_breakable = false);

  // helper function
  void check_new_edge_requirements(size_t source, size_t target) const;

  void dfs(std::vector<cda_rail::index_vector>& ret_val,
           std::unordered_set<size_t>&          vertices_to_visit,
           const VertexType&                    section_type) const {
    dfs(ret_val, vertices_to_visit, section_type, {});
  };
  void dfs(std::vector<cda_rail::index_vector>& ret_val,
           std::unordered_set<size_t>&          vertices_to_visit,
           const VertexType&                    section_type,
           const std::vector<VertexType>&       error_types) const;
  std::vector<std::pair<std::optional<size_t>, std::optional<size_t>>>
  sort_edge_pairs(
      std::vector<std::pair<std::optional<size_t>, std::optional<size_t>>>&
          edge_pairs) const;

  [[nodiscard]] std::vector<cda_rail::index_vector>
  all_routes_of_given_length(std::optional<size_t> v_0,
                             std::optional<size_t> e_0, double desired_length,
                             bool                  reverse_direction,
                             std::optional<size_t> exit_node           = {},
                             cda_rail::index_set   edges_used_by_train = {},
                             bool return_successors_if_zero = false) const;

  [[nodiscard]] size_t other_vertex(size_t e, size_t v) const {
    return get_edge(e).source == v ? get_edge(e).target : get_edge(e).source;
  };

  [[nodiscard]] static double delta_dist_helper(const Edge& successor_edge,
                                                double      max_v,
                                                bool        use_minimal_time);

  [[nodiscard]] std::vector<cda_rail::index_vector> all_paths_ending_at_ttd(
      size_t e_0, const std::vector<cda_rail::index_vector>& ttd_sections,
      std::optional<size_t> exit_node, std::optional<size_t> safe_ttd,
      bool first_edge) const;

public:
  // Constructors
  Network() = default;

  explicit Network(std::filesystem::path const& working_directory,
                   std::string_view const       networkName = "UnnamedNetwork");
  explicit Network(std::string const& working_directory,
                   std::string_view   networkName = "UnnamedNetwork")
      : Network(std::filesystem::path(working_directory), networkName) {};
  explicit Network(char const* const working_directory,
                   std::string_view  networkName = "UnnamedNetwork")
      : Network(std::filesystem::path(working_directory), networkName) {};

  // Rule of 0 suffices

  [[nodiscard]] std::string const& get_network_name() const {
    return m_network_name;
  }
  void set_network_name(std::string_view const networkName) {
    exceptions::throw_if_invalid_folder_name(networkName);
    m_network_name = networkName;
  }

  [[nodiscard]] const std::vector<Vertex>& get_vertices() const {
    return m_vertices;
  };
  [[nodiscard]] const std::vector<Edge>& get_edges() const { return m_edges; };

  [[nodiscard]] double
  maximal_vertex_speed(size_t                     v,
                       const cda_rail::index_set& edges_to_consider = {}) const;
  [[nodiscard]] double maximal_vertex_speed(
      const std::string&         v_name,
      const cda_rail::index_set& edges_to_consider = {}) const {
    return maximal_vertex_speed(get_vertex_index(v_name), edges_to_consider);
  };
  [[nodiscard]] double minimal_neighboring_edge_length(
      size_t v, const cda_rail::index_set& edges_to_consider = {}) const;
  [[nodiscard]] double minimal_neighboring_edge_length(
      const std::string&         v_name,
      const cda_rail::index_set& edges_to_consider = {}) const {
    return minimal_neighboring_edge_length(get_vertex_index(v_name),
                                           edges_to_consider);
  };

  [[nodiscard]] cda_rail::index_vector
  get_vertices_by_type(VertexType type) const;

  [[nodiscard]] std::pair<size_t, double> get_old_edge(size_t new_edge) const;
  [[nodiscard]] std::pair<size_t, double> get_old_edge(size_t source,
                                                       size_t target) const {
    return get_old_edge(get_edge_index(source, target));
  };
  [[nodiscard]] std::pair<size_t, double>
  get_old_edge(const std::string& source, const std::string& target) const {
    return get_old_edge(get_edge_index(source, target));
  };

  size_t                             add_vertex(Vertex vertex);
  template <typename... Args> size_t add_vertex(Args... args) {
    return add_vertex(Vertex(std::forward<Args>(args)...));
  };
  size_t
  add_edge(size_t source, size_t target, double length, double max_speed,
           bool   breakable             = Edge::BREAKABLE_DEFAULT,
           double min_block_length      = Edge::MIN_BLOCK_LENGTH_DEFAULT,
           double min_stop_block_length = Edge::MIN_STOP_BLOCK_LENGTH_DEFAULT);
  size_t add_edge(size_t source, size_t target, double length, double max_speed,
                  std::optional<bool> const&   breakable,
                  std::optional<double> const& min_block_length,
                  std::optional<double> const& min_stop_block_length);
  size_t add_edge(
      std::string_view const sourceName, std::string_view const targetName,
      double const length, double const maxSpeed,
      bool const   breakable          = Edge::BREAKABLE_DEFAULT,
      double const minBlockLength     = Edge::MIN_BLOCK_LENGTH_DEFAULT,
      double const minStopBlockLength = Edge::MIN_STOP_BLOCK_LENGTH_DEFAULT) {
    return add_edge(get_vertex_index(sourceName), get_vertex_index(targetName),
                    length, maxSpeed, breakable, minBlockLength,
                    minStopBlockLength);
  };
  size_t add_edge(std::string_view const sourceName,
                  std::string_view const targetName, double const length,
                  double const maxSpeed, std::optional<bool> const& breakable,
                  std::optional<double> const& min_block_length,
                  std::optional<double> const& min_stop_block_length) {
    return add_edge(get_vertex_index(sourceName), get_vertex_index(targetName),
                    length, maxSpeed, breakable, min_block_length,
                    min_stop_block_length);
  }

  void add_successor(size_t edge_in, size_t edge_out);
  void add_successor(const std::pair<size_t, size_t>& edge_in,
                     const std::pair<size_t, size_t>& edge_out) {
    add_successor(get_edge_index(edge_in.first, edge_in.second),
                  get_edge_index(edge_out.first, edge_out.second));
  };
  void
  add_successor(const std::pair<std::string_view, std::string_view>& edge_in,
                const std::pair<std::string_view, std::string_view>& edge_out) {
    add_successor(get_edge_index(edge_in.first, edge_in.second),
                  get_edge_index(edge_out.first, edge_out.second));
  };
  void add_successor(size_t const                     edge_in,
                     const std::pair<size_t, size_t>& edge_out) {
    add_successor(edge_in, get_edge_index(edge_out.first, edge_out.second));
  };
  void
  add_successor(size_t                                               edge_in,
                const std::pair<std::string_view, std::string_view>& edge_out) {
    add_successor(edge_in, get_edge_index(edge_out.first, edge_out.second));
  };
  void add_successor(const std::pair<size_t, size_t>& edge_in,
                     size_t                           edge_out) {
    add_successor(get_edge_index(edge_in.first, edge_in.second), edge_out);
  };
  void
  add_successor(const std::pair<size_t, size_t>&                     edge_in,
                const std::pair<std::string_view, std::string_view>& edge_out) {
    add_successor(get_edge_index(edge_in.first, edge_in.second),
                  get_edge_index(edge_out.first, edge_out.second));
  };
  void
  add_successor(const std::pair<std::string_view, std::string_view>& edge_in,
                size_t                                               edge_out) {
    add_successor(get_edge_index(edge_in.first, edge_in.second), edge_out);
  };
  void
  add_successor(const std::pair<std::string_view, std::string_view>& edge_in,
                const std::pair<size_t, size_t>&                     edge_out) {
    add_successor(get_edge_index(edge_in.first, edge_in.second),
                  get_edge_index(edge_out.first, edge_out.second));
  };

  [[nodiscard]] const Vertex& get_vertex(size_t index) const;
  [[nodiscard]] const Vertex& get_vertex(std::string_view const name) const {
    return m_vertices.at(get_vertex_index(name));
  };
  [[nodiscard]] size_t      get_vertex_index(std::string_view name) const;
  [[nodiscard]] const Edge& get_edge(size_t index) const;
  [[nodiscard]] const Edge& get_edge(size_t source_id, size_t target_id) const;
  [[nodiscard]] const Edge& get_edge(const std::string_view sourceName,
                                     const std::string_view targetName) const {
    return get_edge(get_vertex_index(sourceName), get_vertex_index(targetName));
  };
  [[nodiscard]] size_t get_edge_index(size_t source_id, size_t target_id) const;
  [[nodiscard]] size_t get_edge_index(const std::string_view sourceName,
                                      const std::string_view targetName) const {
    return get_edge_index(get_vertex_index(sourceName),
                          get_vertex_index(targetName));
  };
  [[nodiscard]] std::string get_edge_name(size_t const index) const {
    const auto& edge_object = get_edge(index);
    return get_edge_name(get_vertex(edge_object.source).name,
                         get_vertex(edge_object.target).name);
  }
  [[nodiscard]] std::string get_edge_name(size_t v0, size_t v1,
                                          bool check_existence = false) const {
    return get_edge_name(get_vertex(v0).name, get_vertex(v1).name,
                         check_existence);
  }
  [[nodiscard]] std::string
  get_edge_name(const std::string_view v1, const std::string_view v2,
                bool const checkExistance = false) const {
    if (checkExistance && !has_vertex(v1)) {
      throw exceptions::VertexNotExistentException(v1);
    }
    if (checkExistance && !has_vertex(v2)) {
      throw exceptions::VertexNotExistentException(v2);
    }
    if (checkExistance && !has_edge(v1, v2)) {
      throw exceptions::EdgeNotExistentException(v1, v2);
    }
    return std::string(v1).append("-").append(v2);
  }

  [[nodiscard]] cda_rail::index_vector
  vertices_used_by_edges(const cda_rail::index_vector& edges_tmp) const;

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
  [[nodiscard]] std::vector<cda_rail::index_vector> all_paths_ending_at_ttd(
      size_t e_0, const std::vector<cda_rail::index_vector>& ttd_sections,
      std::optional<size_t> exit_node) const;

  [[nodiscard]] bool has_vertex(size_t index) const {
    return (index < m_vertices.size());
  };
  [[nodiscard]] bool has_vertex(std::string_view const name) const {
    return m_vertex_name_to_index.contains(std::string{name});
  };
  [[nodiscard]] bool has_edge(size_t index) const {
    return (index < m_edges.size());
  };
  [[nodiscard]] bool has_edge(size_t source_id, size_t target_id) const;
  [[nodiscard]] bool has_edge(std::string_view source_name,
                              std::string_view target_name) const;

  void change_vertex_name(size_t index, std::string_view new_name);
  void change_vertex_name(std::string_view const old_name,
                          std::string_view const new_name) {
    change_vertex_name(get_vertex_index(old_name), new_name);
  };
  void change_vertex_type(size_t index, VertexType new_type);
  void change_vertex_type(std::string_view const name,
                          VertexType const       new_type) {
    change_vertex_type(get_vertex_index(name), new_type);
  };
  void change_vertex_headway(size_t index, double new_headway);
  void change_vertex_headway(std::string_view const name,
                             double const           new_headway) {
    change_vertex_headway(get_vertex_index(name), new_headway);
  };

  void change_edge_length(size_t index, double new_length);
  void change_edge_max_speed(size_t index, double new_max_speed);
  void change_edge_min_block_length(size_t index, double new_min_block_length);
  void change_edge_min_stop_block_length(size_t index,
                                         double new_min_stop_block_length);

  void change_edge_length(size_t source_id, size_t target_id,
                          double new_length) {
    change_edge_length(get_edge_index(source_id, target_id), new_length);
  };
  void change_edge_length(std::string_view const source_name,
                          std::string_view const target_name,
                          double                 new_length) {
    change_edge_length(get_edge_index(source_name, target_name), new_length);
  };
  void change_edge_max_speed(size_t source_id, size_t target_id,
                             double new_max_speed) {
    change_edge_max_speed(get_edge_index(source_id, target_id), new_max_speed);
  }
  void change_edge_max_speed(std::string_view const source_name,
                             std::string_view const target_name,
                             double                 new_max_speed) {
    change_edge_max_speed(get_edge_index(source_name, target_name),
                          new_max_speed);
  }
  void change_edge_min_block_length(size_t source_id, size_t target_id,
                                    double new_min_block_length) {
    change_edge_min_block_length(get_edge_index(source_id, target_id),
                                 new_min_block_length);
  }
  void change_edge_min_block_length(std::string_view const source_name,
                                    std::string_view const target_name,
                                    double new_min_block_length) {
    change_edge_min_block_length(get_edge_index(source_name, target_name),
                                 new_min_block_length);
  }

  void set_edge_breakable(size_t index);
  void set_edge_unbreakable(size_t index);

  void set_edge_breakable(size_t source_id, size_t target_id) {
    set_edge_breakable(get_edge_index(source_id, target_id));
  };
  void set_edge_breakable(std::string_view const source_name,
                          std::string_view const target_name) {
    set_edge_breakable(get_edge_index(source_name, target_name));
  };
  void set_edge_unbreakable(size_t source_id, size_t target_id) {
    set_edge_unbreakable(get_edge_index(source_id, target_id));
  };
  void set_edge_unbreakable(std::string_view const source_name,
                            std::string_view const target_name) {
    set_edge_unbreakable(get_edge_index(source_name, target_name));
  };

  [[nodiscard]] cda_rail::index_set out_edges(size_t index) const;
  [[nodiscard]] cda_rail::index_set
  out_edges(std::string_view const name) const {
    return out_edges(get_vertex_index(name));
  };
  [[nodiscard]] cda_rail::index_set in_edges(size_t index) const;
  [[nodiscard]] cda_rail::index_set
  in_edges(std::string_view const name) const {
    return in_edges(get_vertex_index(name));
  };
  [[nodiscard]] cda_rail::index_set neighboring_edges(size_t index) const;
  [[nodiscard]] cda_rail::index_set
  neighboring_edges(std::string_view const name) const {
    return neighboring_edges(get_vertex_index(name));
  };

  [[nodiscard]] static std::vector<std::pair<size_t, size_t>>
  get_intersecting_ttd(const cda_rail::index_vector&              edges,
                       const std::vector<cda_rail::index_vector>& ttd);

  [[nodiscard]] cda_rail::index_set        get_predecessors(size_t index) const;
  [[nodiscard]] const cda_rail::index_set& get_successors(size_t index) const;
  [[nodiscard]] const cda_rail::index_set&
  get_successors(size_t source_id, size_t target_id) const {
    return get_successors(get_edge_index(source_id, target_id));
  };
  [[nodiscard]] const cda_rail::index_set&
  get_successors(std::string_view const source_name,
                 std::string_view const target_name) const {
    return get_successors(get_edge_index(source_name, target_name));
  };

  [[nodiscard]] cda_rail::index_vector neighbors(size_t index) const;
  [[nodiscard]] cda_rail::index_vector
  neighbors(std::string_view const name) const {
    return neighbors(get_vertex_index(name));
  };

  [[nodiscard]] size_t number_of_vertices() const { return m_vertices.size(); };
  [[nodiscard]] size_t number_of_edges() const { return m_edges.size(); };

  [[nodiscard]] int max_vss_on_edge(size_t index) const;
  [[nodiscard]] int max_vss_on_edge(size_t source, size_t target) const {
    return max_vss_on_edge(get_edge_index(source, target));
  };
  [[nodiscard]] int max_vss_on_edge(std::string_view const source,
                                    std::string_view const target) const {
    return max_vss_on_edge(get_edge_index(source, target));
  };

  [[nodiscard]] static Network
  import_network(std::filesystem::path const& working_directory,
                 std::string_view const       networkName = "UnnamedNetwork") {
    return Network(working_directory, networkName);
  };
  [[nodiscard]] static Network
  import_network(std::string const& working_directory,
                 std::string_view   networkName = "UnnamedNetwork") {
    return Network(working_directory, networkName);
  };
  [[nodiscard]] static Network
  import_network(char const* const working_directory,
                 std::string_view  networkName = "UnnamedNetwork") {
    return Network(working_directory, networkName);
  };
  void export_network(std::filesystem::path const& working_directory) const;
  void export_network(std::string const& working_directory) const {
    export_network(std::filesystem::path(working_directory));
  };
  void export_network(char const* const working_directory) const {
    export_network(std::filesystem::path(working_directory));
  };

  [[nodiscard]] bool is_valid_successor(size_t e0, size_t e1) const;

  [[nodiscard]] bool is_adjustable(size_t vertex_id) const;
  [[nodiscard]] bool is_adjustable(std::string_view const vertex_name) const {
    return is_adjustable(get_vertex_index(vertex_name));
  };

  [[nodiscard]] bool is_consistent_for_transformation() const;

  // Get special edges
  [[nodiscard]] cda_rail::index_vector breakable_edges() const;
  [[nodiscard]] cda_rail::index_vector relevant_breakable_edges() const;
  [[nodiscard]] std::vector<cda_rail::index_vector>
  unbreakable_sections() const;
  [[nodiscard]] std::vector<cda_rail::index_vector>
  no_border_vss_sections() const;
  [[nodiscard]] std::vector<
      std::pair<std::optional<size_t>, std::optional<size_t>>>
  combine_reverse_edges(const cda_rail::index_vector& edges_to_consider,
                        bool                          sort = false) const;
  [[nodiscard]] std::optional<size_t>
  get_reverse_edge_index(size_t edge_index) const;
  [[nodiscard]] std::optional<size_t>
  get_reverse_edge_index(std::optional<size_t> edge_index) const {
    return edge_index.has_value() ? get_reverse_edge_index(edge_index.value())
                                  : std::optional<size_t>();
  }
  [[nodiscard]] size_t                get_track_index(size_t edge_index) const;
  [[nodiscard]] std::optional<size_t> common_vertex(
      const std::pair<std::optional<size_t>, std::optional<size_t>>& pair1,
      const std::pair<std::optional<size_t>, std::optional<size_t>>& pair2)
      const;
  [[nodiscard]] std::optional<size_t>
  common_vertex(const std::pair<size_t, size_t>& pair1,
                const std::pair<size_t, size_t>& pair2) const {
    return common_vertex(std::make_pair(std::optional<size_t>(pair1.first),
                                        std::optional<size_t>(pair1.second)),
                         std::make_pair(std::optional<size_t>(pair2.first),
                                        std::optional<size_t>(pair2.second)));
  }

  [[nodiscard]] cda_rail::index_vector
                     get_unbreakable_section_containing_edge(size_t e) const;
  [[nodiscard]] bool is_on_same_unbreakable_section(size_t e1, size_t e2) const;

  [[nodiscard]] cda_rail::index_vector
  inverse_edges(const cda_rail::index_vector& edge_indices) const {
    const auto&            edge_number = number_of_edges();
    cda_rail::index_vector edges_to_consider(edge_number);
    std::iota(edges_to_consider.begin(), edges_to_consider.end(), 0);
    return inverse_edges(edge_indices, edges_to_consider);
  };
  [[nodiscard]] cda_rail::index_vector
  inverse_edges(const cda_rail::index_vector& edge_indices,
                const cda_rail::index_vector& edges_to_consider) const;

  // Transformation functions
  std::pair<cda_rail::index_vector, cda_rail::index_vector> separate_edge(
      size_t const                   edgeIndex,
      const vss::SeparationFunction& sep_func = &vss::functions::uniform) {
    return separate_edge_private_helper(
        edgeIndex, get_edge(edgeIndex).min_block_length, sep_func);
  };
  std::pair<cda_rail::index_vector, cda_rail::index_vector> separate_edge(
      size_t const sourceId, size_t const targetId,
      const vss::SeparationFunction& sep_func = &vss::functions::uniform) {
    return separate_edge(get_edge_index(sourceId, targetId), sep_func);
  };
  std::pair<cda_rail::index_vector, cda_rail::index_vector> separate_edge(
      std::string_view const sourceName, std::string_view const targetName,
      const vss::SeparationFunction& sep_func = &vss::functions::uniform) {
    return separate_edge(get_edge_index(sourceName, targetName), sep_func);
  };

  std::pair<cda_rail::index_vector, cda_rail::index_vector>
  separate_stop_edge(size_t edge_index) {
    return separate_edge_private_helper(
        edge_index, get_edge(edge_index).min_stop_block_length,
        &vss::functions::uniform, true);
  };
  std::vector<std::pair<size_t, cda_rail::index_vector>>
  separate_stop_edges(const cda_rail::index_vector& stop_edges);

  std::vector<std::pair<size_t, cda_rail::index_vector>> discretize(
      const vss::SeparationFunction& sep_func = &vss::functions::uniform);

  [[nodiscard]] std::vector<std::vector<double>>
  all_edge_pairs_shortest_paths() const;

  [[nodiscard]] std::optional<double>
  shortest_path(size_t source_edge_id, size_t target_id,
                bool target_is_edge = false, bool include_first_edge = false,
                bool use_minimal_time = false, double max_v = INF) const;

  [[nodiscard]] std::optional<double> shortest_path_between_sets(
      cda_rail::index_vector source_edge_ids, cda_rail::index_vector target_ids,
      bool target_is_edge = false, bool include_first_edge = false,
      bool use_minimal_time = false, double max_v = INF) const {
    return shortest_path_between_sets_using_edges(
               std::move(source_edge_ids), std::move(target_ids), true, {},
               target_is_edge, include_first_edge, use_minimal_time, max_v)
        .first;
  };

  [[nodiscard]] std::pair<std::optional<double>, cda_rail::index_vector>
  shortest_path_using_edges(size_t source_edge_id, size_t target_vertex_id,
                            bool only_use_valid_successors            = true,
                            cda_rail::index_vector edges_to_use       = {},
                            bool                   target_is_edge     = false,
                            bool                   include_first_edge = false,
                            bool                   use_minimal_time   = false,
                            double                 max_v = INF) const {
    return shortest_path_between_sets_using_edges(
        cda_rail::index_vector{source_edge_id},
        cda_rail::index_vector{target_vertex_id}, only_use_valid_successors,
        std::move(edges_to_use), target_is_edge, include_first_edge,
        use_minimal_time, max_v);
  };

  [[nodiscard]] std::pair<std::optional<double>, cda_rail::index_vector>
  shortest_path_between_sets_using_edges(
      cda_rail::index_vector source_edge_ids, cda_rail::index_vector target_ids,
      bool                   only_use_valid_successors = true,
      cda_rail::index_vector edges_to_use = {}, bool target_is_edge = false,
      bool include_first_edge = false, bool use_minimal_time = false,
      double max_v = INF) const;

  [[nodiscard]] double length_of_path(const cda_rail::index_vector& path) const;
};
} // namespace cda_rail
