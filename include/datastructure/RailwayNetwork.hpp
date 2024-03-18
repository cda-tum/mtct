#pragma once
#include "Definitions.hpp"
#include "MultiArray.hpp"
#include "VSSModel.hpp"

#include <algorithm>
#include <filesystem>
#include <numeric>
#include <optional>
#include <sstream>
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
   */

  std::string name;
  VertexType  type;
  double      headway;

  // Constructors
  Vertex(std::string name, VertexType type)
      : name(std::move(name)), type(type), headway(0.0){};
  Vertex(std::string name, VertexType type, double headway)
      : name(std::move(name)), type(type), headway(headway){};
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
   */
  size_t source;
  size_t target;
  double length;
  double max_speed;
  bool   breakable;
  double min_block_length      = 1;
  double min_stop_block_length = 100;

  // Constructors
  Edge(size_t source, size_t target, double length, double max_speed,
       bool breakable, double min_block_length = 1,
       double min_stop_block_length = 1)
      : source(source), target(target), length(length), max_speed(max_speed),
        breakable(breakable), min_block_length(min_block_length),
        min_stop_block_length(min_stop_block_length){};
};

class Network {
  /**
   * Graph class
   *
   */
private:
  std::vector<Vertex>                     vertices;
  std::vector<Edge>                       edges;
  std::vector<std::vector<size_t>>        successors;
  std::unordered_map<std::string, size_t> vertex_name_to_index;

  void        read_graphml(const std::filesystem::path& p);
  static void get_keys(tinyxml2::XMLElement* graphml_body,
                       std::string& breakable, std::string& length,
                       std::string& max_speed, std::string& min_block_length,
                       std::string& min_stop_block_length, std::string& type,
                       std::string& headway);
  void add_vertices_from_graphml(const tinyxml2::XMLElement* graphml_node,
                                 const std::string&          type,
                                 const std::string&          headway);
  void add_edges_from_graphml(const tinyxml2::XMLElement* graphml_edge,
                              const std::string&          breakable,
                              const std::string&          length,
                              const std::string&          max_speed,
                              const std::string&          min_block_length,
                              const std::string& min_stop_block_length);
  void read_successors(const std::filesystem::path& p);

  void export_graphml(const std::filesystem::path& p) const;
  void export_successors_python(const std::filesystem::path& p) const;
  void export_successors_cpp(const std::filesystem::path& p) const;
  void write_successor_set_to_file(std::ofstream& file, size_t i) const;

  std::pair<std::vector<size_t>, std::vector<size_t>>
  separate_edge_private_helper(
      size_t edge_index, double min_length,
      const vss::SeparationFunction& sep_func = &vss::functions::uniform,
      bool                           new_edge_breakable = false);

  std::pair<std::vector<size_t>, std::vector<size_t>>
  separate_edge_at(size_t                     edge_index,
                   const std::vector<double>& distances_from_source,
                   bool                       new_edge_breakable = false);

  // helper function
  void dfs(std::vector<std::vector<size_t>>& ret_val,
           std::unordered_set<size_t>&       vertices_to_visit,
           const VertexType&                 section_type) const {
    dfs(ret_val, vertices_to_visit, section_type, {});
  };
  void dfs(std::vector<std::vector<size_t>>& ret_val,
           std::unordered_set<size_t>&       vertices_to_visit,
           const VertexType&                 section_type,
           const std::vector<VertexType>&    error_types) const;
  std::vector<std::pair<std::optional<size_t>, std::optional<size_t>>>
  sort_edge_pairs(
      std::vector<std::pair<std::optional<size_t>, std::optional<size_t>>>&
          edge_pairs) const;

  [[nodiscard]] std::vector<std::vector<size_t>>
  all_routes_of_given_length(std::optional<size_t> v_0,
                             std::optional<size_t> e_0, double desired_length,
                             bool reverse_direction) const;

public:
  // Constructors
  Network() = default;

  explicit Network(const std::filesystem::path& p);
  explicit Network(const std::string& path)
      : Network(std::filesystem::path(path)){};
  explicit Network(const char* path) : Network(std::filesystem::path(path)){};

  // Rule of 5
  Network(const Network& other)                = default;
  Network(Network&& other) noexcept            = default;
  Network& operator=(const Network& other)     = default;
  Network& operator=(Network&& other) noexcept = default;
  ~Network()                                   = default;

  [[nodiscard]] const std::vector<Vertex>& get_vertices() const {
    return vertices;
  };
  [[nodiscard]] const std::vector<Edge>& get_edges() const { return edges; };

  [[nodiscard]] std::vector<size_t> get_vertices_by_type(VertexType type) const;

  size_t add_vertex(const std::string& name, VertexType type,
                    double headway = 0.0);
  size_t add_edge(size_t source, size_t target, double length, double max_speed,
                  bool breakable = true, double min_block_length = 1,
                  double min_stop_block_length = 100);
  size_t add_edge(const std::string& source_name,
                  const std::string& target_name, double length,
                  double max_speed, bool breakable = true,
                  double min_block_length      = 1,
                  double min_stop_block_length = 100) {
    return add_edge(get_vertex_index(source_name),
                    get_vertex_index(target_name), length, max_speed, breakable,
                    min_block_length, min_stop_block_length);
  };
  void add_successor(size_t edge_in, size_t edge_out);
  void add_successor(const std::pair<size_t, size_t>& edge_in,
                     const std::pair<size_t, size_t>& edge_out) {
    add_successor(get_edge_index(edge_in.first, edge_in.second),
                  get_edge_index(edge_out.first, edge_out.second));
  };
  void add_successor(const std::pair<std::string, std::string>& edge_in,
                     const std::pair<std::string, std::string>& edge_out) {
    add_successor(get_edge_index(edge_in.first, edge_in.second),
                  get_edge_index(edge_out.first, edge_out.second));
  };
  void add_successor(size_t                           edge_in,
                     const std::pair<size_t, size_t>& edge_out) {
    add_successor(edge_in, get_edge_index(edge_out.first, edge_out.second));
  };
  void add_successor(size_t                                     edge_in,
                     const std::pair<std::string, std::string>& edge_out) {
    add_successor(edge_in, get_edge_index(edge_out.first, edge_out.second));
  };
  void add_successor(const std::pair<size_t, size_t>& edge_in,
                     size_t                           edge_out) {
    add_successor(get_edge_index(edge_in.first, edge_in.second), edge_out);
  };
  void add_successor(const std::pair<size_t, size_t>&           edge_in,
                     const std::pair<std::string, std::string>& edge_out) {
    add_successor(get_edge_index(edge_in.first, edge_in.second),
                  get_edge_index(edge_out.first, edge_out.second));
  };
  void add_successor(const std::pair<std::string, std::string>& edge_in,
                     size_t                                     edge_out) {
    add_successor(get_edge_index(edge_in.first, edge_in.second), edge_out);
  };
  void add_successor(const std::pair<std::string, std::string>& edge_in,
                     const std::pair<size_t, size_t>&           edge_out) {
    add_successor(get_edge_index(edge_in.first, edge_in.second),
                  get_edge_index(edge_out.first, edge_out.second));
  };

  [[nodiscard]] const Vertex& get_vertex(size_t index) const;
  [[nodiscard]] const Vertex& get_vertex(const std::string& name) const {
    return vertices[get_vertex_index(name)];
  };
  [[nodiscard]] size_t      get_vertex_index(const std::string& name) const;
  [[nodiscard]] const Edge& get_edge(size_t index) const;
  [[nodiscard]] const Edge& get_edge(size_t source_id, size_t target_id) const;
  [[nodiscard]] const Edge& get_edge(const std::string& source_name,
                                     const std::string& target_name) const {
    return get_edge(get_vertex_index(source_name),
                    get_vertex_index(target_name));
  };
  [[nodiscard]] size_t get_edge_index(size_t source_id, size_t target_id) const;
  [[nodiscard]] size_t get_edge_index(const std::string& source_name,
                                      const std::string& target_name) const {
    return get_edge_index(get_vertex_index(source_name),
                          get_vertex_index(target_name));
  };

  [[nodiscard]] std::vector<std::vector<size_t>>
  all_paths_of_length_starting_in_vertex(size_t v, double desired_len) const {
    return all_routes_of_given_length(v, std::nullopt, desired_len, false);
  };
  [[nodiscard]] std::vector<std::vector<size_t>>
  all_paths_of_length_starting_in_edge(size_t e, double desired_len) const {
    return all_routes_of_given_length(std::nullopt, e, desired_len, false);
  };
  [[nodiscard]] std::vector<std::vector<size_t>>
  all_paths_of_length_ending_in_vertex(size_t v, double desired_len) const {
    return all_routes_of_given_length(v, std::nullopt, desired_len, true);
  };
  [[nodiscard]] std::vector<std::vector<size_t>>
  all_paths_of_length_ending_in_edge(size_t e, double desired_len) const {
    return all_routes_of_given_length(std::nullopt, e, desired_len, true);
  }

  [[nodiscard]] bool has_vertex(size_t index) const {
    return (index < vertices.size());
  };
  [[nodiscard]] bool has_vertex(const std::string& name) const {
    return vertex_name_to_index.find(name) != vertex_name_to_index.end();
  };
  [[nodiscard]] bool has_edge(size_t index) const {
    return (index < edges.size());
  };
  [[nodiscard]] bool has_edge(size_t source_id, size_t target_id) const;
  [[nodiscard]] bool has_edge(const std::string& source_name,
                              const std::string& target_name) const;

  void change_vertex_name(size_t index, const std::string& new_name);
  void change_vertex_name(const std::string& old_name,
                          const std::string& new_name) {
    change_vertex_name(get_vertex_index(old_name), new_name);
  };
  void change_vertex_type(size_t index, VertexType new_type);
  void change_vertex_type(const std::string& name, VertexType new_type) {
    change_vertex_type(get_vertex_index(name), new_type);
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
  void change_edge_length(const std::string& source_name,
                          const std::string& target_name, double new_length) {
    change_edge_length(get_edge_index(source_name, target_name), new_length);
  };
  void change_edge_max_speed(size_t source_id, size_t target_id,
                             double new_max_speed) {
    change_edge_max_speed(get_edge_index(source_id, target_id), new_max_speed);
  }
  void change_edge_max_speed(const std::string& source_name,
                             const std::string& target_name,
                             double             new_max_speed) {
    change_edge_max_speed(get_edge_index(source_name, target_name),
                          new_max_speed);
  }
  void change_edge_min_block_length(size_t source_id, size_t target_id,
                                    double new_min_block_length) {
    change_edge_min_block_length(get_edge_index(source_id, target_id),
                                 new_min_block_length);
  }
  void change_edge_min_block_length(const std::string& source_name,
                                    const std::string& target_name,
                                    double             new_min_block_length) {
    change_edge_min_block_length(get_edge_index(source_name, target_name),
                                 new_min_block_length);
  }

  void set_edge_breakable(size_t index);
  void set_edge_unbreakable(size_t index);

  void set_edge_breakable(size_t source_id, size_t target_id) {
    set_edge_breakable(get_edge_index(source_id, target_id));
  };
  void set_edge_breakable(const std::string& source_name,
                          const std::string& target_name) {
    set_edge_breakable(get_edge_index(source_name, target_name));
  };
  void set_edge_unbreakable(size_t source_id, size_t target_id) {
    set_edge_unbreakable(get_edge_index(source_id, target_id));
  };
  void set_edge_unbreakable(const std::string& source_name,
                            const std::string& target_name) {
    set_edge_unbreakable(get_edge_index(source_name, target_name));
  };

  [[nodiscard]] std::vector<size_t> out_edges(size_t index) const;
  [[nodiscard]] std::vector<size_t> out_edges(const std::string& name) const {
    return out_edges(get_vertex_index(name));
  };
  [[nodiscard]] std::vector<size_t> in_edges(size_t index) const;
  [[nodiscard]] std::vector<size_t> in_edges(const std::string& name) const {
    return in_edges(get_vertex_index(name));
  };
  [[nodiscard]] std::vector<size_t>        get_predecessors(size_t index) const;
  [[nodiscard]] const std::vector<size_t>& get_successors(size_t index) const;
  [[nodiscard]] const std::vector<size_t>&
  get_successors(size_t source_id, size_t target_id) const {
    return get_successors(get_edge_index(source_id, target_id));
  };
  [[nodiscard]] const std::vector<size_t>&
  get_successors(const std::string& source_name,
                 const std::string& target_name) const {
    return get_successors(get_edge_index(source_name, target_name));
  };

  [[nodiscard]] std::vector<size_t> neighbors(size_t index) const;
  [[nodiscard]] std::vector<size_t> neighbors(const std::string& name) const {
    return neighbors(get_vertex_index(name));
  };

  [[nodiscard]] size_t number_of_vertices() const { return vertices.size(); };
  [[nodiscard]] size_t number_of_edges() const { return edges.size(); };

  [[nodiscard]] int max_vss_on_edge(size_t index) const;
  [[nodiscard]] int max_vss_on_edge(size_t source, size_t target) const {
    return max_vss_on_edge(get_edge_index(source, target));
  };
  [[nodiscard]] int max_vss_on_edge(const std::string& source,
                                    const std::string& target) const {
    return max_vss_on_edge(get_edge_index(source, target));
  };

  [[nodiscard]] static Network import_network(const std::string& path) {
    return Network(path);
  };
  [[nodiscard]] static Network import_network(const char* path) {
    return Network(path);
  };
  [[nodiscard]] static Network import_network(const std::filesystem::path& p) {
    return Network(p);
  };
  void export_network(const std::string& path) const {
    export_network(std::filesystem::path(path));
  };
  void export_network(const char* path) const {
    export_network(std::filesystem::path(path));
  };
  void export_network(const std::filesystem::path& p) const;

  [[nodiscard]] bool is_valid_successor(size_t e0, size_t e1) const;

  [[nodiscard]] bool is_adjustable(size_t vertex_id) const;
  [[nodiscard]] bool is_adjustable(const std::string& vertex_name) const {
    return is_adjustable(get_vertex_index(vertex_name));
  };

  [[nodiscard]] bool is_consistent_for_transformation() const;

  // Get special edges
  [[nodiscard]] std::vector<size_t> breakable_edges() const;
  [[nodiscard]] std::vector<size_t> relevant_breakable_edges() const;
  [[nodiscard]] std::vector<std::vector<size_t>> unbreakable_sections() const;
  [[nodiscard]] std::vector<std::vector<size_t>> no_border_vss_sections() const;
  [[nodiscard]] std::vector<
      std::pair<std::optional<size_t>, std::optional<size_t>>>
  combine_reverse_edges(const std::vector<size_t>& edges_to_consider,
                        bool                       sort = false) const;
  [[nodiscard]] std::optional<size_t>
  get_reverse_edge_index(size_t edge_index) const;
  [[nodiscard]] std::optional<size_t>
  get_reverse_edge_index(std::optional<size_t> edge_index) const {
    return edge_index.has_value() ? get_reverse_edge_index(edge_index.value())
                                  : std::optional<size_t>();
  }
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

  [[nodiscard]] std::vector<size_t>
  inverse_edges(const std::vector<size_t>& edge_indices) const {
    const auto&         edge_number = number_of_edges();
    std::vector<size_t> edges_to_consider(edge_number);
    std::iota(edges_to_consider.begin(), edges_to_consider.end(), 0);
    return inverse_edges(edge_indices, edges_to_consider);
  };
  [[nodiscard]] std::vector<size_t>
  inverse_edges(const std::vector<size_t>& edge_indices,
                const std::vector<size_t>& edges_to_consider) const;

  // Transformation functions
  std::pair<std::vector<size_t>, std::vector<size_t>> separate_edge(
      size_t                         edge_index,
      const vss::SeparationFunction& sep_func = &vss::functions::uniform) {
    return separate_edge_private_helper(
        edge_index, get_edge(edge_index).min_block_length, sep_func);
  };
  std::pair<std::vector<size_t>, std::vector<size_t>> separate_edge(
      size_t source_id, size_t target_id,
      const vss::SeparationFunction& sep_func = &vss::functions::uniform) {
    return separate_edge(get_edge_index(source_id, target_id), sep_func);
  };
  std::pair<std::vector<size_t>, std::vector<size_t>> separate_edge(
      const std::string& source_name, const std::string& target_name,
      const vss::SeparationFunction& sep_func = &vss::functions::uniform) {
    return separate_edge(get_edge_index(source_name, target_name), sep_func);
  };

  std::pair<std::vector<size_t>, std::vector<size_t>>
  separate_stop_edge(size_t edge_index) {
    return separate_edge_private_helper(
        edge_index, get_edge(edge_index).min_stop_block_length,
        &vss::functions::uniform, true);
  };
  std::vector<std::pair<size_t, std::vector<size_t>>>
  separate_stop_edges(const std::vector<size_t>& stop_edges);

  std::vector<std::pair<size_t, std::vector<size_t>>> discretize(
      const vss::SeparationFunction& sep_func = &vss::functions::uniform);

  [[nodiscard]] std::vector<std::vector<double>>
  all_edge_pairs_shortest_paths() const;
};

// HELPER
void to_bool_optional(std::string& s, std::optional<bool>& b);
} // namespace cda_rail
