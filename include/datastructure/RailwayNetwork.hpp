#pragma once
#include "Definitions.hpp"
#include "MultiArray.hpp"

#include <algorithm>
#include <filesystem>
#include <numeric>
#include <optional>
#include <sstream>
#include <string>
#include <tinyxml2.h>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace cda_rail {
struct Vertex {
  /**
   * Vertex object
   * @param name Name of the vertex
   * @param type Type of the vertex (NO_BORDER, VSS, TTD)
   */

  std::string name;
  VertexType  type;

  // Constructors
  Vertex() = default;
  Vertex(const std::string& name, VertexType type) : name(name), type(type){};
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
  int    source;
  int    target;
  double length;
  double max_speed;
  bool   breakable;
  double min_block_length = 1;

  // Constructors
  Edge() = default;
  Edge(int source, int target, double length, double max_speed, bool breakable,
       double min_block_length = 1)
      : source(source), target(target), length(length), max_speed(max_speed),
        breakable(breakable), min_block_length(min_block_length){};
};

class Network {
  /**
   * Graph class
   *
   */
private:
  std::vector<Vertex>                  vertices;
  std::vector<Edge>                    edges;
  std::vector<std::vector<int>>        successors;
  std::unordered_map<std::string, int> vertex_name_to_index;

  void        read_graphml(const std::filesystem::path& p);
  static void get_keys(tinyxml2::XMLElement* graphml_body,
                       std::string& breakable, std::string& length,
                       std::string& max_speed, std::string& min_block_length,
                       std::string& type);
  void add_vertices_from_graphml(const tinyxml2::XMLElement* graphml_node,
                                 const std::string&          type);
  void add_edges_from_graphml(const tinyxml2::XMLElement* graphml_edge,
                              const std::string&          breakable,
                              const std::string&          length,
                              const std::string&          max_speed,
                              const std::string&          min_block_length);
  void read_successors(const std::filesystem::path& p);
  static void extract_vertices_from_key(const std::string& key,
                                        std::string&       source_name,
                                        std::string&       target_name);

  void export_graphml(const std::filesystem::path& p) const;
  void export_successors_python(const std::filesystem::path& p) const;
  void export_successors_cpp(const std::filesystem::path& p) const;
  void write_successor_set_to_file(std::ofstream& file, int i) const;

  std::pair<std::vector<int>, std::vector<int>>
  separate_edge_at(int                        edge_index,
                   const std::vector<double>& distances_from_source);
  std::pair<std::vector<int>, std::vector<int>>
  uniform_separate_edge(int edge_index);
  std::pair<std::vector<int>, std::vector<int>>
  chebychev_separate_edge(int edge_index);

  // helper function
  void dfs(std::vector<std::vector<int>>&          ret_val,
           std::unordered_set<int>&                vertices_to_visit,
           const cda_rail::VertexType&             section_type,
           const std::vector<cda_rail::VertexType> error_types = {}) const;
  std::vector<std::pair<int, int>>
  sort_edge_pairs(std::vector<std::pair<int, int>>& edge_pairs) const;

public:
  // Constructors
  Network() = default;
  Network(const std::filesystem::path& p);
  Network(const std::string& path) : Network(std::filesystem::path(path)){};
  Network(const char* path) : Network(std::filesystem::path(path)){};

  // Rule of 5
  Network(const Network& other)                = default;
  Network(Network&& other) noexcept            = default;
  Network& operator=(const Network& other)     = default;
  Network& operator=(Network&& other) noexcept = default;
  ~Network()                                   = default;

  const std::vector<Vertex>& get_vertices() const { return vertices; };
  const std::vector<Edge>&   get_edges() const { return edges; };

  const std::vector<int> get_vertices_by_type(cda_rail::VertexType type) const;

  int add_vertex(const std::string& name, VertexType type);
  int add_edge(int source, int target, double length, double max_speed,
               bool breakable, double min_block_length = 1);
  int add_edge(const std::string& source_name, const std::string& target_name,
               double length, double max_speed, bool breakable,
               double min_block_length = 1) {
    return add_edge(get_vertex_index(source_name),
                    get_vertex_index(target_name), length, max_speed, breakable,
                    min_block_length);
  };
  void add_successor(int edge_in, int edge_out);
  void add_successor(const std::pair<int, int>& edge_in,
                     const std::pair<int, int>& edge_out) {
    add_successor(get_edge_index(edge_in.first, edge_in.second),
                  get_edge_index(edge_out.first, edge_out.second));
  };
  void add_successor(const std::pair<std::string, std::string>& edge_in,
                     const std::pair<std::string, std::string>& edge_out) {
    add_successor(get_edge_index(edge_in.first, edge_in.second),
                  get_edge_index(edge_out.first, edge_out.second));
  };
  void add_successor(int edge_in, const std::pair<int, int>& edge_out) {
    add_successor(edge_in, get_edge_index(edge_out.first, edge_out.second));
  };
  void add_successor(int                                        edge_in,
                     const std::pair<std::string, std::string>& edge_out) {
    add_successor(edge_in, get_edge_index(edge_out.first, edge_out.second));
  };
  void add_successor(const std::pair<int, int>& edge_in, int edge_out) {
    add_successor(get_edge_index(edge_in.first, edge_in.second), edge_out);
  };
  void add_successor(const std::pair<int, int>&                 edge_in,
                     const std::pair<std::string, std::string>& edge_out) {
    add_successor(get_edge_index(edge_in.first, edge_in.second),
                  get_edge_index(edge_out.first, edge_out.second));
  };
  void add_successor(const std::pair<std::string, std::string>& edge_in,
                     int                                        edge_out) {
    add_successor(get_edge_index(edge_in.first, edge_in.second), edge_out);
  };
  void add_successor(const std::pair<std::string, std::string>& edge_in,
                     const std::pair<int, int>&                 edge_out) {
    add_successor(get_edge_index(edge_in.first, edge_in.second),
                  get_edge_index(edge_out.first, edge_out.second));
  };

  [[nodiscard]] const Vertex& get_vertex(int index) const;
  [[nodiscard]] const Vertex& get_vertex(const std::string& name) const {
    return vertices[get_vertex_index(name)];
  };
  [[nodiscard]] int         get_vertex_index(const std::string& name) const;
  [[nodiscard]] const Edge& get_edge(int index) const;
  [[nodiscard]] const Edge& get_edge(int source_id, int target_id) const;
  [[nodiscard]] const Edge& get_edge(const std::string& source_name,
                                     const std::string& target_name) const {
    return get_edge(get_vertex_index(source_name),
                    get_vertex_index(target_name));
  };
  [[nodiscard]] int get_edge_index(int source_id, int target_id) const;
  [[nodiscard]] int get_edge_index(const std::string& source_name,
                                   const std::string& target_name) const {
    return get_edge_index(get_vertex_index(source_name),
                          get_vertex_index(target_name));
  };

  [[nodiscard]] bool has_vertex(int index) const {
    return (index >= 0 && index < vertices.size());
  };
  [[nodiscard]] bool has_vertex(const std::string& name) const {
    return vertex_name_to_index.find(name) != vertex_name_to_index.end();
  };
  [[nodiscard]] bool has_edge(int index) const {
    return (index >= 0 && index < edges.size());
  };
  [[nodiscard]] bool has_edge(int source_id, int target_id) const;
  [[nodiscard]] bool has_edge(const std::string& source_name,
                              const std::string& target_name) const;

  void change_vertex_name(int index, const std::string& new_name);
  void change_vertex_name(const std::string& old_name,
                          const std::string& new_name) {
    change_vertex_name(get_vertex_index(old_name), new_name);
  };
  void change_vertex_type(int index, cda_rail::VertexType new_type);
  void change_vertex_type(const std::string&   name,
                          cda_rail::VertexType new_type) {
    change_vertex_type(get_vertex_index(name), new_type);
  };

  void change_edge_property(int index, double value,
                            const std::string& property);
  void change_edge_property(int source_id, int target_id, double value,
                            const std::string& property) {
    change_edge_property(get_edge_index(source_id, target_id), value, property);
  };
  void change_edge_property(const std::string& source_name,
                            const std::string& target_name, double value,
                            const std::string& property) {
    change_edge_property(get_edge_index(source_name, target_name), value,
                         property);
  };
  void change_edge_breakable(int index, bool value);
  void change_edge_breakable(int source_id, int target_id, bool value) {
    change_edge_breakable(get_edge_index(source_id, target_id), value);
  };
  void change_edge_breakable(const std::string& source_name,
                             const std::string& target_name, bool value) {
    change_edge_breakable(get_edge_index(source_name, target_name), value);
  };

  [[nodiscard]] std::vector<int> out_edges(int index) const;
  [[nodiscard]] std::vector<int> out_edges(const std::string& name) const {
    return out_edges(get_vertex_index(name));
  };
  [[nodiscard]] std::vector<int> in_edges(int index) const;
  [[nodiscard]] std::vector<int> in_edges(const std::string& name) const {
    return in_edges(get_vertex_index(name));
  };
  [[nodiscard]] const std::vector<int>& get_successors(int index) const;
  [[nodiscard]] const std::vector<int>& get_successors(int source_id,
                                                       int target_id) const {
    return get_successors(get_edge_index(source_id, target_id));
  };
  [[nodiscard]] const std::vector<int>&
  get_successors(const std::string& source_name,
                 const std::string& target_name) const {
    return get_successors(get_edge_index(source_name, target_name));
  };

  [[nodiscard]] std::vector<int> neighbors(int index) const;
  [[nodiscard]] std::vector<int> neighbors(const std::string& name) const {
    return neighbors(get_vertex_index(name));
  };

  [[nodiscard]] int number_of_vertices() const { return vertices.size(); };
  [[nodiscard]] int number_of_edges() const { return edges.size(); };

  [[nodiscard]] int max_vss_on_edge(int index) const;
  [[nodiscard]] int max_vss_on_edge(int source, int target) const {
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

  [[nodiscard]] bool is_valid_successor(int e0, int e1) const;

  [[nodiscard]] bool is_adjustable(int vertex_id) const;
  [[nodiscard]] bool is_adjustable(const std::string& vertex_name) const {
    return is_adjustable(get_vertex_index(vertex_name));
  };

  bool is_consistent_for_transformation() const;

  // Get special edges
  [[nodiscard]] std::vector<int>              breakable_edges() const;
  [[nodiscard]] std::vector<int>              relevant_breakable_edges() const;
  [[nodiscard]] std::vector<std::vector<int>> unbreakable_sections() const;
  [[nodiscard]] std::vector<std::vector<int>> no_border_vss_sections() const;
  [[nodiscard]] std::vector<std::pair<int, int>>
  combine_reverse_edges(const std::vector<int>& edges, bool sort = false) const;
  [[nodiscard]] int get_reverse_edge_index(int edge_index) const;
  [[nodiscard]] std::optional<int>
  common_vertex(const std::pair<int, int>& pair1,
                const std::pair<int, int>& pair2) const;

  std::vector<int> inverse_edges(const std::vector<int>& edge_indices) const {
    const auto&      edge_number = number_of_edges();
    std::vector<int> edges_to_consider(edge_number);
    std::iota(edges_to_consider.begin(), edges_to_consider.end(), 0);
    return inverse_edges(edge_indices, edges_to_consider);
  };
  std::vector<int>
  inverse_edges(const std::vector<int>& edge_indices,
                const std::vector<int>& edges_to_consider) const;

  // Transformation functions
  std::pair<std::vector<int>, std::vector<int>>
  separate_edge(int edge_index, cda_rail::SeparationType separation_type =
                                    cda_rail::SeparationType::UNIFORM);
  std::pair<std::vector<int>, std::vector<int>>
  separate_edge(int source_id, int target_id,
                cda_rail::SeparationType separation_type =
                    cda_rail::SeparationType::UNIFORM) {
    return separate_edge(get_edge_index(source_id, target_id), separation_type);
  };
  std::pair<std::vector<int>, std::vector<int>>
  separate_edge(const std::string& source_name, const std::string& target_name,
                cda_rail::SeparationType separation_type =
                    cda_rail::SeparationType::UNIFORM) {
    return separate_edge(get_edge_index(source_name, target_name),
                         separation_type);
  };

  std::vector<std::pair<int, std::vector<int>>>
  discretize(cda_rail::SeparationType separation_type =
                 cda_rail::SeparationType::UNIFORM);

  MultiArray<double> all_edge_pairs_shortest_paths() const;
};

// HELPER
void to_bool_optional(std::string& s, std::optional<bool>& b);
} // namespace cda_rail
