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

/**
 * @brief Represents a vertex (node) in the railway network graph.
 *
 * A vertex models a physical point on the rail topology, such as a signal,
 * a track-detection boundary, or a plain interior node. The @p type field
 * determines how the vertex is treated by algorithms.
 */
struct Vertex {
  constexpr static double HEADWAY_DEFAULT{0.0};

  std::string name;
  VertexType  type;
  double      headway{HEADWAY_DEFAULT};

  /**
   * @brief Constructs a Vertex with an explicit headway value.
   * @param name  Human-readable identifier for the vertex.
   * @param type  Topological/signal type (`NoBorder`, `VSS`, `TTD`,
   *              `NoBorderVSS`).
   * @param headway Additional minimum headway (in seconds) imposed at this
   *                vertex; defaults to `HEADWAY_DEFAULT` (0.0).
   */
  Vertex(std::string_view const name, VertexType const type,
         double const headway = HEADWAY_DEFAULT)
      : name(name), type(type), headway(headway) {};

  /**
   * @brief Constructs a Vertex with an optional headway value.
   * @param name    Human-readable identifier for the vertex.
   * @param type    Topological/signal type.
   * @param headway Optional headway in seconds; `std::nullopt` falls back to
   *                `HEADWAY_DEFAULT`.
   */
  Vertex(std::string_view const name, VertexType const type,
         std::optional<double> const& headway)
      : name(name), type(type), headway(headway.value_or(HEADWAY_DEFAULT)) {};

  // Operators
  bool operator==(Vertex const& other) const {
    return name == other.name && type == other.type && headway == other.headway;
  }
};

/**
 * @brief Represents a directed edge in the railway network graph.
 *
 * An edge models a physical track segment between two vertices. The
 * @p breakable flag controls whether a Virtual Sub-Section (VSS) border may
 * be placed anywhere on the segment.
 */
struct Edge {
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

  /**
   * @brief Constructs an Edge with explicit attribute values.
   * @param source            Index of the source vertex.
   * @param target            Index of the target vertex.
   * @param length            Length of the edge in metres.
   * @param maxSpeed          Maximum permitted speed on this edge in m/s.
   * @param breakable         `true` if a VSS border may be placed on this
   *                          edge; defaults to `BREAKABLE_DEFAULT` (`true`).
   * @param minBlockLength    Minimum block length in metres; defaults to
   *                          `MIN_BLOCK_LENGTH_DEFAULT` (1.0 m).
   * @param minStopBlockLength Minimum block length inside a station stop zone
   *                           in metres; defaults to
   *                           `MIN_STOP_BLOCK_LENGTH_DEFAULT` (100.0 m).
   */
  Edge(size_t const source, size_t const target, double const length,
       double const maxSpeed, bool const breakable = BREAKABLE_DEFAULT,
       double const minBlockLength     = MIN_BLOCK_LENGTH_DEFAULT,
       double const minStopBlockLength = MIN_STOP_BLOCK_LENGTH_DEFAULT)
      : source(source), target(target), length(length), max_speed(maxSpeed),
        breakable(breakable), min_block_length(minBlockLength),
        min_stop_block_length(minStopBlockLength) {};

  /**
   * @brief Constructs an Edge with optional attribute values falling back to
   *        their defaults when not provided.
   * @param source               Index of the source vertex.
   * @param target               Index of the target vertex.
   * @param length               Length of the edge in metres.
   * @param maxSpeed             Maximum permitted speed in m/s.
   * @param breakable            Optional breakable flag; `std::nullopt` uses
   *                             `BREAKABLE_DEFAULT`.
   * @param min_block_length     Optional minimum block length; `std::nullopt`
   *                             uses `MIN_BLOCK_LENGTH_DEFAULT`.
   * @param min_stop_block_length Optional minimum stop-block length;
   *                              `std::nullopt` uses
   *                              `MIN_STOP_BLOCK_LENGTH_DEFAULT`.
   */
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

/**
 * @brief Directed multigraph representing a railway network.
 *
 * Stores vertices, directed edges, and the edge-successor relation that
 * encodes which continuations are physically possible at each junction. The
 * class also provides import/export, graph queries, network transformations
 * (edge discretization), and shortest-path algorithms.
 */
class Network {
public:
  /**
   * @brief Unified vertex descriptor accepted by all public API functions.
   *
   * Implicitly convertible from a numeric index (`size_t`), a name
   * (`std::string_view`), or a full `Vertex` object, so callers can pass any
   * of these forms without explicit casts.
   */
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
    /**
     * @brief Resolves this descriptor to a numeric vertex index.
     * @param network Owning network used for name look-ups.
     * @return Numeric vertex index.
     * @throws cda_rail::exceptions::VertexNotExistentException If a name or
     *         Vertex object does not match any vertex in @p network.
     * @throws cda_rail::exceptions::ConsistencyException If a Vertex object
     *         matches by name but has different attributes.
     */
    size_t resolve(Network const* const network) const;
  };

  /**
   * @brief Unified edge descriptor accepted by all public API functions.
   *
   * Implicitly convertible from a numeric index (`size_t`), a pair of vertex
   * indices, a pair of vertex names (`std::string_view`), or a full `Edge`
   * object.
   */
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
    EdgeInput(size_t a, size_t b) : m_data(std::pair{a, b}) {}
    EdgeInput(std::string_view a, std::string_view b)
        : m_data(std::pair{a, b}) {}
    template <typename T1, typename T2>
    EdgeInput(const std::pair<T1, T2>& p) : EdgeInput(p.first, p.second) {}
    EdgeInput(Edge edge) : m_data(std::move(edge)) {}
    // NOLINTEND(google-explicit-constructor)

  private:
    /**
     * @brief Resolves this descriptor to a numeric edge index.
     * @param network Owning network used for look-ups.
     * @return Numeric edge index.
     * @throws cda_rail::exceptions::EdgeNotExistentException If the edge
     *         cannot be found.
     * @throws cda_rail::exceptions::ConsistencyException If an Edge object
     *         matches by endpoints but has different attributes.
     */
    size_t resolve(Network const* const network) const;
  };

private:
  // the class ensures that m_network_name is a valid folder name on any
  // operating system as enforced by throw_if_invalid_folder_name
  std::string m_network_name{"UnnamedNetwork"};

  std::vector<Vertex> m_vertices{};
  std::vector<Edge>   m_edges{};
  std::vector<cda_rail::index_set>
      m_successors{}; // for every edge, set of possible successor edges
  std::unordered_map<std::string, size_t> m_vertex_name_to_index{};

  std::unordered_map<std::size_t, std::pair<size_t, double>>
      m_new_edge_to_old_edge_after_transform{}; // needed if edges are
                                                // discretized

public:
  // -----------------------------
  // CONSTRUCTORS
  // -----------------------------

  /**
   * @brief Default-constructs an empty network.
   *
   * @param networkName Name of the network, used for export folder naming;
   * defaults to `"UnnamedNetwork"`.
   * @throws cda_rail::exceptions::InvalidInputException If `networkName` is not
   * a valid folder name.
   */
  explicit Network(std::string_view const networkName = "UnnamedNetwork")
      : m_network_name(networkName) {
    exceptions::throw_if_invalid_folder_name(networkName);
  };

  /**
   * @brief Reads a network from disk and constructs the object.
   *
   * The graph (`tracks.graphml`) and the successor map
   * (`successors_cpp.json`) are read from
   * `working_directory/networks/networkName/`.
   *
  * @param networkName       Name of the network subfolder; defaults to
   *                          `"UnnamedNetwork"`.
   * @param working_directory Root working directory.
   ?
   * @throws cda_rail::exceptions::ImportException If the expected directory
   *         does not exist or the files cannot be parsed.
   */
  explicit Network(std::string_view const       networkName,
                   std::filesystem::path const& working_directory);

  /**
   * @brief Convenience overload accepting a `std::string` path.
   *
   * @param networkName       Name of the network subfolder.
   * @param working_directory Root working directory as a string.
   *
   * @throws cda_rail::exceptions::ImportException If the network cannot be
   *         loaded.
   */
  explicit Network(std::string_view const networkName,
                   std::string const&     working_directory)
      : Network(networkName, std::filesystem::path(working_directory)) {};

  /**
   * @brief Convenience overload accepting a C-string path.
   *
   * @param networkName       Name of the network subfolder.
   * @param working_directory Root working directory as a C-string.
   *
   * @throws cda_rail::exceptions::ImportException If the network cannot be
   *         loaded.
   */
  explicit Network(std::string_view const networkName,
                   char const* const      working_directory)
      : Network(networkName, std::filesystem::path(working_directory)) {};

  // Rule of 0 suffices

  // -----------------------------
  // IMPORT / EXPORT
  // -----------------------------

  /**
   * @brief Loads a network from disk and returns it by value.
   *
   * @param networkName       Name of the network subfolder; defaults to
   *                          `"UnnamedNetwork"`.
   * @param working_directory Root working directory.
   *
   * @return Newly constructed `Network` object.
   * @throws cda_rail::exceptions::ImportException If the network cannot be
   *         loaded.
   */
  [[nodiscard]] static Network
  import_network(std::string_view const       networkName,
                 std::filesystem::path const& working_directory) {
    return Network(networkName, working_directory);
  };

  /**
   * @brief Convenience overload of `import_network` accepting a `std::string`
   *        path.
   *
   * @param networkName       Name of the network subfolder.
   * @param working_directory Root working directory as a string.
   *
   * @return Newly constructed `Network` object.
   * @throws cda_rail::exceptions::ImportException If the network cannot be
   *         loaded.
   */
  [[nodiscard]] static Network
  import_network(std::string_view const networkName,
                 std::string const&     working_directory) {
    return Network(networkName, working_directory);
  };

  /**
   * @brief Convenience overload of `import_network` accepting a C-string path.
   *
   * @param networkName      Name of the network subfolder.
   * @param workingDirectory Root working directory as a C-string.
   *
   * @return Newly constructed `Network` object.
   * @throws cda_rail::exceptions::ImportException If the network cannot be
   *         loaded.
   */
  [[nodiscard]] static Network
  import_network(std::string_view const networkName,
                 char const* const      workingDirectory) {
    return Network(networkName, workingDirectory);
  };

  /**
   * @brief Exports the network to disk.
   *
   * Writes `tracks.graphml`, `successors_cpp.json`, and `successors.txt` to
   * `working_directory/networks/<networkName>/`.
   *
   * @param working_directory Root working directory.
   * @throws cda_rail::exceptions::ExportException If the output directory
   *         cannot be created.
   */
  void export_network(std::filesystem::path const& working_directory) const;

  /**
   * @brief Convenience overload of `export_network` accepting a `std::string`
   *        path.
   * @param working_directory Root working directory as a string.
   * @throws cda_rail::exceptions::ExportException If the output directory
   *         cannot be created.
   */
  void export_network(std::string const& working_directory) const {
    export_network(std::filesystem::path(working_directory));
  };

  /**
   * @brief Convenience overload of `export_network` accepting a C-string path.
   * @param workingDirectory Root working directory as a C-string.
   * @throws cda_rail::exceptions::ExportException If the output directory
   *         cannot be created.
   */
  void export_network(char const* const workingDirectory) const {
    export_network(std::filesystem::path(workingDirectory));
  };

  // -----------------------------
  // GETTER
  // -----------------------------

  /**
   * @brief Checks whether a numeric vertex index is valid.
   * @param index Candidate vertex index.
   * @return `true` if @p index refers to an existing vertex.
   */
  [[nodiscard]] bool has_vertex(size_t index) const {
    return (index < m_vertices.size());
  };

  /**
   * @brief Checks whether a vertex with the given name exists.
   * @param name Vertex name to look up.
   * @return `true` if such a vertex exists in the network.
   */
  [[nodiscard]] bool has_vertex(std::string_view const name) const {
    return m_vertex_name_to_index.contains(std::string{name});
  };

private:
  /**
   * @brief Implementation backing the public `has_edge` overload for vertex
   *        index pairs.
   * @param source_id Index of the source vertex.
   * @param target_id Index of the target vertex.
   * @return `true` if a directed edge from @p source_id to @p target_id
   *         exists.
   * @throws cda_rail::exceptions::VertexNotExistentException If either vertex
   *         index is out of range.
   */
  [[nodiscard]] bool has_edge_helper(size_t source_id, size_t target_id) const;

public:
  /**
   * @brief Checks whether a numeric edge index is valid.
   * @param index Candidate edge index.
   * @return `true` if @p index refers to an existing edge.
   */
  [[nodiscard]] bool has_edge(size_t const index) const {
    return (index < m_edges.size());
  };

  /**
   * @brief Checks whether a directed edge between two vertices exists.
   * @param source Source vertex descriptor.
   * @param target Target vertex descriptor.
   * @return `true` if such a directed edge exists.
   * @throws cda_rail::exceptions::VertexNotExistentException If either vertex
   *         does not exist.
   */
  [[nodiscard]] bool has_edge(VertexInput const& source,
                              VertexInput const& target) const {
    return has_edge_helper(source.resolve(this), target.resolve(this));
  }

private:
  /**
   * @brief Implementation backing `is_valid_successor`.
   * @param e0 Index of the predecessor edge.
   * @param e1 Index of the candidate successor edge.
   * @return `true` if @p e1 is adjacent to and registered as a successor of
   *         @p e0.
   * @throws cda_rail::exceptions::EdgeNotExistentException If either edge
   *         does not exist.
   */
  [[nodiscard]] bool is_valid_successor_helper(size_t e0, size_t e1) const;

public:
  /**
   * @brief Checks whether @p edge_out is a valid successor of @p edge_in.
   *
   * A valid successor must satisfy two conditions:
   * - The source of @p edge_out equals the target of @p edge_in (adjacency).
   * - The pair is registered in the successor map (e.g. physically possible
   *   at a junction).
   *
   * @param edge_in  Predecessor edge.
   * @param edge_out Candidate successor edge.
   * @return `true` if @p edge_out is a valid successor of @p edge_in.
   * @throws cda_rail::exceptions::EdgeNotExistentException If either edge
   *         does not exist.
   */
  [[nodiscard]] bool is_valid_successor(EdgeInput const& edge_in,
                                        EdgeInput const& edge_out) const {
    return is_valid_successor_helper(edge_in.resolve(this),
                                     edge_out.resolve(this));
  };

private:
  /**
   * @brief Implementation backing `is_adjustable`.
   * @param vertex_id Numeric vertex index.
   * @return `true` if the vertex is adjustable.
   * @throws cda_rail::exceptions::VertexNotExistentException If the vertex
   *         does not exist.
   */
  [[nodiscard]] bool is_adjustable_helper(size_t vertex_id) const;

public:
  /**
   * @brief Checks whether a vertex is adjustable by certain algorithmic
   *        approaches.
   *
   * A vertex is adjustable if and only if it is of type `NoBorder` and has
   * exactly two neighboring vertices.
   *
   * @param vertex Vertex descriptor.
   * @return `true` if the vertex satisfies the adjustability criterion.
   * @throws cda_rail::exceptions::VertexNotExistentException If the vertex
   *         does not exist.
   */
  [[nodiscard]] bool is_adjustable(VertexInput const& vertex) const {
    return is_adjustable_helper(vertex.resolve(this));
  };

  /**
   * @brief Checks network consistency required for edge-separation
   *        transformations.
   *
   * The following conditions must all hold:
   * - Every breakable edge has a strictly positive `min_block_length`.
   * - No breakable edge is incident to a `NoBorder` vertex.
   * - For every bidirectional edge pair, both directions share the same
   *   `breakable` flag and `length`.
   * - Every `NoBorderVSS` vertex has at most two neighbors, none of type
   *   `NoBorder`.
   *
   * @return `true` if all consistency conditions are satisfied.
   */
  [[nodiscard]] bool is_consistent_for_transformation() const;

  /**
   * @brief Returns the total number of vertices in the network.
   * @return Number of vertices.
   */
  [[nodiscard]] size_t number_of_vertices() const { return m_vertices.size(); };

  /**
   * @brief Returns the total number of edges in the network.
   * @return Number of directed edges.
   */
  [[nodiscard]] size_t number_of_edges() const { return m_edges.size(); };

  /**
   * @brief Returns the network name.
   * @return Const reference to the network name string; guaranteed to be a
   *         valid directory name on all operating systems.
   */
  [[nodiscard]] std::string const& get_network_name() const {
    return m_network_name;
  }

  /**
   * @brief Returns a const reference to the underlying vertex container.
   * @return Const reference to the vector of all `Vertex` objects, indexed
   *         by vertex index.
   */
  [[nodiscard]] std::vector<Vertex> const& get_vertices() const {
    return m_vertices;
  };

  /**
   * @brief Returns the indices of all vertices of the specified type, ordered
   *        by vertex index.
   * @param type Vertex type to filter by.
   * @return Vector of vertex indices whose type equals @p type.
   */
  [[nodiscard]] cda_rail::index_vector
  get_vertices_by_type(VertexType type) const;

  /**
   * @brief Returns a const reference to the underlying edge container.
   * @return Const reference to the vector of all `Edge` objects, indexed by
   *         edge index.
   */
  [[nodiscard]] const std::vector<Edge>& get_edges() const { return m_edges; };

private:
  /**
   * @brief Implementation backing `get_old_edge`.
   * @param new_edge Numeric edge index (possibly created by a transformation).
   * @return Pair `{original_edge_index, offset_from_source_in_metres}`.
   * @throws cda_rail::exceptions::EdgeNotExistentException If @p new_edge is
   *         out of range.
   */
  [[nodiscard]] std::pair<size_t, double>
  get_old_edge_helper(size_t new_edge) const;

public:
  /**
   * @brief Returns the pre-discretization edge and position for an edge that
   *        may have been created by a transformation.
   *
   * If @p edge has not been produced by any transformation, returns
   * `{edge_index, 0.0}`.
   *
   * @param edge Edge descriptor.
   * @return Pair `{original_edge_index, offset_from_source_in_metres}`.
   * @throws cda_rail::exceptions::EdgeNotExistentException If the edge does
   *         not exist.
   */
  [[nodiscard]] std::pair<size_t, double>
  get_old_edge(EdgeInput const& edge) const {
    return get_old_edge_helper(edge.resolve(this));
  };

  /**
   * @brief Returns the index of the vertex with the given name.
   * @param name Vertex name.
   * @return Numeric vertex index.
   * @throws cda_rail::exceptions::VertexNotExistentException If no vertex
   *         with @p name exists.
   */
  [[nodiscard]] size_t get_vertex_index(std::string_view name) const;

private:
  /**
   * @brief Implementation backing `get_vertex`.
   * @param index Numeric vertex index.
   * @return Const reference to the `Vertex` object.
   * @throws cda_rail::exceptions::VertexNotExistentException If @p index is
   *         out of range.
   */
  [[nodiscard]] const Vertex& get_vertex_helper(size_t index) const;

public:
  /**
   * @brief Returns a const reference to the specified vertex.
   * @param vertex Vertex descriptor (index, name, or `Vertex` object).
   * @return Const reference to the `Vertex`.
   * @throws cda_rail::exceptions::VertexNotExistentException If the vertex
   *         does not exist.
   */
  [[nodiscard]] const Vertex& get_vertex(VertexInput const& vertex) const {
    return get_vertex_helper(vertex.resolve(this));
  }

private:
  /**
   * @brief Implementation backing `get_edge_index`.
   * @param source_id Index of the source vertex.
   * @param target_id Index of the target vertex.
   * @return Index of the directed edge from @p source_id to @p target_id.
   * @throws cda_rail::exceptions::VertexNotExistentException If either vertex
   *         does not exist.
   * @throws cda_rail::exceptions::EdgeNotExistentException If no such edge
   *         exists.
   */
  [[nodiscard]] size_t get_edge_index_helper(size_t source_id,
                                             size_t target_id) const;

public:
  /**
   * @brief Returns the index of the directed edge from @p source to @p target.
   * @param source Source vertex descriptor.
   * @param target Target vertex descriptor.
   * @return Numeric edge index.
   * @throws cda_rail::exceptions::VertexNotExistentException If either vertex
   *         does not exist.
   * @throws cda_rail::exceptions::EdgeNotExistentException If no such directed
   *         edge exists.
   */
  [[nodiscard]] size_t get_edge_index(VertexInput const& source,
                                      VertexInput const& target) const {
    return get_edge_index_helper(source.resolve(this), target.resolve(this));
  }

private:
  /**
   * @brief Implementation backing `get_edge`.
   * @param index Numeric edge index.
   * @return Const reference to the `Edge` object.
   * @throws cda_rail::exceptions::EdgeNotExistentException If @p index is out
   *         of range.
   */
  [[nodiscard]] const Edge& get_edge_helper(size_t index) const;

public:
  /**
   * @brief Returns a const reference to the specified edge.
   * @param edge Edge descriptor (index, vertex-pair, or `Edge` object).
   * @return Const reference to the `Edge`.
   * @throws cda_rail::exceptions::EdgeNotExistentException If the edge does
   *         not exist.
   */
  [[nodiscard]] const Edge& get_edge(EdgeInput const& edge) const {
    return get_edge_helper(edge.resolve(this));
  }

private:
  /**
   * @brief Builds the canonical name string `"v1-v2"` for an edge.
   * @param v1               Name of the source vertex.
   * @param v2               Name of the target vertex.
   * @param check_existence  If `true`, verifies that both vertices and the
   *                         edge exist before returning.
   * @return Edge name string of the form `"<source_name>-<target_name>"`.
   * @throws cda_rail::exceptions::VertexNotExistentException If
   *         @p check_existence is `true` and a vertex is missing.
   * @throws cda_rail::exceptions::EdgeNotExistentException If
   *         @p check_existence is `true` and the edge is missing.
   */
  [[nodiscard]] std::string get_edge_name_helper(std::string_view v1,
                                                 std::string_view v2,
                                                 bool check_existence) const;

  /**
   * @brief Overload resolving vertex indices to names before delegating.
   * @param v0               Index of the source vertex.
   * @param v1               Index of the target vertex.
   * @param check_existence  Forwarded to the string overload.
   * @return Edge name string.
   */
  [[nodiscard]] std::string get_edge_name_helper(size_t v0, size_t v1,
                                                 bool check_existence) const {
    return get_edge_name_helper(get_vertex(v0).name, get_vertex(v1).name,
                                check_existence);
  }

public:
  /**
   * @brief Returns the canonical name of an edge formatted as
   *        `"<source_name>-<target_name>"`.
   * @param edge Edge descriptor.
   * @return Edge name string.
   * @throws cda_rail::exceptions::EdgeNotExistentException If the edge does
   *         not exist.
   */
  [[nodiscard]] std::string get_edge_name(EdgeInput const& edge) const {
    const auto& edge_object = get_edge(edge.resolve(this));
    return get_edge_name_helper(get_vertex(edge_object.source).name,
                                get_vertex(edge_object.target).name, false);
  }

private:
  /**
   * @brief Returns the set of indices of all edges leaving vertex @p index.
   * @param index Numeric vertex index.
   * @return Set of outgoing edge indices.
   * @throws cda_rail::exceptions::VertexNotExistentException If the vertex
   *         does not exist.
   */
  [[nodiscard]] cda_rail::index_set out_edges_helper(size_t index) const;

  /**
   * @brief Returns the set of indices of all edges entering vertex @p index.
   * @param index Numeric vertex index.
   * @return Set of incoming edge indices.
   * @throws cda_rail::exceptions::VertexNotExistentException If the vertex
   *         does not exist.
   */
  [[nodiscard]] cda_rail::index_set in_edges_helper(size_t index) const;

  /**
   * @brief Returns the union of in-edges and out-edges for vertex @p index.
   * @param index Numeric vertex index.
   * @return Set of all incident edge indices (both directions).
   * @throws cda_rail::exceptions::VertexNotExistentException If the vertex
   *         does not exist.
   */
  [[nodiscard]] cda_rail::index_set
  neighboring_edges_helper(size_t index) const;

public:
  /**
   * @brief Returns the set of edge indices of all edges leaving @p vertex.
   * @param vertex Vertex descriptor.
   * @return Set of outgoing edge indices.
   * @throws cda_rail::exceptions::VertexNotExistentException If the vertex
   *         does not exist.
   */
  [[nodiscard]] cda_rail::index_set out_edges(VertexInput const& vertex) const {
    return out_edges_helper(vertex.resolve(this));
  };

  /**
   * @brief Returns the set of edge indices of all edges entering @p vertex.
   * @param vertex Vertex descriptor.
   * @return Set of incoming edge indices.
   * @throws cda_rail::exceptions::VertexNotExistentException If the vertex
   *         does not exist.
   */
  [[nodiscard]] cda_rail::index_set in_edges(VertexInput const& vertex) const {
    return in_edges_helper(vertex.resolve(this));
  };

  /**
   * @brief Returns the set of edge indices of all edges incident to @p vertex
   *        (both in-edges and out-edges).
   * @param vertex Vertex descriptor.
   * @return Set of all incident edge indices.
   * @throws cda_rail::exceptions::VertexNotExistentException If the vertex
   *         does not exist.
   */
  [[nodiscard]] cda_rail::index_set
  neighboring_edges(VertexInput const& vertex) const {
    return neighboring_edges_helper(vertex.resolve(this));
  };

private:
  /**
   * @brief Returns predecessor edge indices for edge @p index.
   *
   * A predecessor of edge @p e is any edge @p e' such that @p e is a valid
   * successor of @p e'.
   *
   * @param index Numeric edge index.
   * @return Set of predecessor edge indices.
   * @throws cda_rail::exceptions::EdgeNotExistentException If the edge does
   *         not exist.
   */
  [[nodiscard]] cda_rail::index_set get_predecessors_helper(size_t index) const;

  /**
   * @brief Returns the registered successor set for edge @p index.
   * @param index Numeric edge index.
   * @return Const reference to the set of valid successor edge indices.
   * @throws cda_rail::exceptions::EdgeNotExistentException If the edge does
   *         not exist.
   */
  [[nodiscard]] const cda_rail::index_set&
  get_successors_helper(size_t index) const;

public:
  /**
   * @brief Returns all predecessor edges of @p edge.
   *
   * An edge @p e' is a predecessor of @p edge if @p edge is a valid successor
   * of @p e'.
   *
   * @param edge Edge descriptor.
   * @return Set of predecessor edge indices.
   * @throws cda_rail::exceptions::EdgeNotExistentException If the edge does
   *         not exist.
   */
  [[nodiscard]] cda_rail::index_set
  get_predecessors(EdgeInput const& edge) const {
    return get_predecessors_helper(edge.resolve(this));
  }

  /**
   * @brief Returns the set of valid successor edges for @p edge.
   * @param edge Edge descriptor.
   * @return Const reference to the set of successor edge indices.
   * @throws cda_rail::exceptions::EdgeNotExistentException If the edge does
   *         not exist.
   */
  [[nodiscard]] cda_rail::index_set const&
  get_successors(EdgeInput const& edge) const {
    return get_successors_helper(edge.resolve(this));
  }

private:
  /**
   * @brief Returns the set of vertex indices adjacent to vertex @p v_index.
   * @param v_index Numeric vertex index.
   * @return Set of neighboring vertex indices.
   * @throws cda_rail::exceptions::VertexNotExistentException If the vertex
   *         does not exist.
   */
  [[nodiscard]] cda_rail::index_set neighbors_helper(size_t v_index) const;

public:
  /**
   * @brief Returns the set of vertex indices adjacent to @p vertex via any
   *        incident edge.
   * @param vertex Vertex descriptor.
   * @return Set of neighboring vertex indices.
   * @throws cda_rail::exceptions::VertexNotExistentException If the vertex
   *         does not exist.
   */
  [[nodiscard]] cda_rail::index_set neighbors(VertexInput const& vertex) const {
    return neighbors_helper(vertex.resolve(this));
  };

private:
  /**
   * @brief Returns the common endpoint of two edges, if one exists.
   * @param e_idx_1 Index of the first edge.
   * @param e_idx_2 Index of the second edge.
   * @return Index of the shared vertex, or `std::nullopt` if the edges share
   *         no endpoint.
   * @throws cda_rail::exceptions::ConsistencyException If the two edges share
   *         more than one vertex (degenerate case).
   */
  [[nodiscard]] std::optional<size_t>
  common_vertex_helper(size_t e_idx_1, size_t e_idx_2) const;

public:
  /**
   * @brief Returns the common endpoint of two edges, if one exists.
   * @param edge1 First edge descriptor.
   * @param edge2 Second edge descriptor.
   * @return Index of the shared vertex, or `std::nullopt` if the edges share
   *         no endpoint.
   * @throws cda_rail::exceptions::ConsistencyException If the edges share
   *         more than one vertex.
   */
  [[nodiscard]] std::optional<size_t>
  common_vertex(EdgeInput const& edge1, EdgeInput const& edge2) const {
    return common_vertex_helper(edge1.resolve(this), edge2.resolve(this));
  };

private:
  /**
   * @brief Returns the index of the reverse edge, or empty if none exists.
   * @param edge_index Numeric edge index.
   * @return Index of the reverse edge, or `std::nullopt`.
   * @throws cda_rail::exceptions::EdgeNotExistentException If @p edge_index
   *         is out of range.
   */
  [[nodiscard]] std::optional<size_t>
  get_reverse_edge_index_helper(size_t edge_index) const;

public:
  /**
   * @brief Returns the index of the edge with reversed endpoints relative to
   *        @p edge.
   * @param edge Edge descriptor.
   * @return Index of the reverse edge, or `std::nullopt` if no such edge
   *         exists.
   * @throws cda_rail::exceptions::EdgeNotExistentException If @p edge does
   *         not exist.
   */
  [[nodiscard]] std::optional<size_t>
  get_reverse_edge_index(EdgeInput const& edge) const {
    return get_reverse_edge_index_helper(edge.resolve(this));
  };

  /**
   * @brief Wraps `get_reverse_edge_index` to propagate an optional input.
   *
   * If @p edgeIndex has no value, `std::nullopt` is returned immediately
   * without any look-up.
   *
   * @param edgeIndex Optional numeric edge index to query.
   * @return Index of the reverse edge, or `std::nullopt`.
   */
  [[nodiscard]] std::optional<size_t>
  get_optional_reverse_edge_index(std::optional<size_t> const edgeIndex) const {
    return edgeIndex.has_value()
               ? get_reverse_edge_index_helper(edgeIndex.value())
               : std::optional<size_t>();
  }

  /**
   * @brief Pairs each edge in @p edges_to_consider with its reverse edge,
   *        eliminating duplicate pairs.
   *
   * Each bidirectional track produces exactly one pair `(e, rev_e)` where the
   * smaller index comes first. Edges without a reverse in edges_to_consider are
   * paired with `std::nullopt` as the second element.
   *
   * If the output is sorted, it is ensured that the first element edges are
   * successors of each other, hence, the inner-pair order might be off. In this
   * case it is also required that edges_to_consider includes every possible
   * reverse edge if existent.
   *
   * @param edges_to_consider Ordered vector of edge indices to process. All
   *                          indices must refer to existing edges.
   * @param sort              If `true`, the returned pairs are reordered so
   *                          that consecutive pairs share a vertex (valid only
   *                          when @p edges_to_consider forms a simple path).
   * @return Vector of `{forward_edge, optional_reverse_edge}` pairs.
   * @throws cda_rail::exceptions::EdgeNotExistentException If any edge in
   *         @p edges_to_consider does not exist.
   */
  [[nodiscard]] std::vector<
      std::pair<std::optional<size_t>, std::optional<size_t>>>
  combine_reverse_edges(const cda_rail::index_vector& edges_to_consider,
                        bool                          sort = false) const;

private:
  /**
   * @brief Returns the canonical track index for edge @p edge_index.
   *
   * For bidirectional tracks the canonical index is the smaller of the two
   * directed edge indices. For unidirectional edges it is the edge index
   * itself.
   *
   * @param edge_index Numeric edge index.
   * @return Canonical track index.
   */
  [[nodiscard]] size_t get_track_index_helper(size_t edge_index) const;

public:
  /**
   * @brief Returns the canonical track index for @p edge.
   *
   * For a bidirectional track the canonical index is the smaller of the two
   * directed edge indices.
   *
   * @param edge Edge descriptor.
   * @return Canonical track index.
   * @throws cda_rail::exceptions::EdgeNotExistentException If the edge does
   *         not exist.
   */
  [[nodiscard]] size_t get_track_index(EdgeInput const& edge) const {
    return get_track_index_helper(edge.resolve(this));
  };

  /**
   * @brief Returns the indices of all breakable edges, sorted by index.
   * @return Vector of breakable edge indices in ascending order.
   */
  [[nodiscard]] cda_rail::index_vector breakable_edges() const;

  /**
   * @brief Returns the indices of breakable edges, counting each
   *        bidirectional track only once.
   *
   * For a bidirectional breakable track only the edge with the smaller index
   * is included.
   *
   * @return Vector of relevant breakable edge indices in ascending order.
   */
  [[nodiscard]] cda_rail::index_vector relevant_breakable_edges() const;

  /**
   * @brief Returns all unbreakable sections of the network.
   *
   * Each section is represented as a set of edge indices. The method covers:
   * - Single-edge sections between two TTD/VSS vertices.
   * - Multi-edge sections passing through one or more `NoBorder` vertices.
   *
   * @return Vector of index sets, each representing one unbreakable section.
   */
  [[nodiscard]] std::vector<cda_rail::index_set> unbreakable_sections() const;

  /**
   * @brief Returns all sections bounded by `NoBorderVSS` vertices.
   *
   * These sections typically result from a prior edge-separation
   * transformation and can be altered by a subsequent VSS-placement algorithm.
   *
   * @return Vector of index sets, each representing one `NoBorderVSS`
   *         section.
   */
  [[nodiscard]] std::vector<cda_rail::index_set> no_border_vss_sections() const;

private:
  /**
   * @brief Returns the set of edge indices that form the unbreakable section
   *        containing edge @p e.
   * @param e Numeric edge index.
   * @return Set of edge indices in the same unbreakable section as @p e, or
   *         an empty set if @p e is breakable.
   */
  [[nodiscard]] cda_rail::index_set
  get_unbreakable_section_containing_edge_helper(size_t e) const;

  /**
   * @brief Returns `true` if and only if edges @p e1 and @p e2 belong to the
   *        same unbreakable section.
   * @param e1 Index of the first edge.
   * @param e2 Index of the second edge.
   * @return `true` if both edges lie in the same unbreakable section.
   */
  [[nodiscard]] bool is_on_same_unbreakable_section_helper(size_t e1,
                                                           size_t e2) const;

public:
  /**
   * @brief Returns the set of edge indices that form the unbreakable section
   *        containing @p edge.
   *
   * If @p edge is breakable the returned set is empty.
   *
   * @param edge Edge descriptor.
   * @return Set of edge indices in the same unbreakable section as @p edge.
   * @throws cda_rail::exceptions::EdgeNotExistentException If the edge does
   *         not exist.
   */
  [[nodiscard]] cda_rail::index_set
  get_unbreakable_section_containing_edge(EdgeInput const& edge) const {
    return get_unbreakable_section_containing_edge_helper(edge.resolve(this));
  };

  /**
   * @brief Checks whether two edges belong to the same unbreakable section.
   * @param edge1 First edge descriptor.
   * @param edge2 Second edge descriptor.
   * @return `true` if both edges lie in the same unbreakable section.
   * @throws cda_rail::exceptions::EdgeNotExistentException If either edge
   *         does not exist.
   */
  [[nodiscard]] bool
  is_on_same_unbreakable_section(EdgeInput const& edge1,
                                 EdgeInput const& edge2) const {
    return is_on_same_unbreakable_section_helper(edge1.resolve(this),
                                                 edge2.resolve(this));
  };

  /**
   * @brief Returns all vertex indices that are endpoints of at least one edge
   *        in @p edges_tmp.
   * @param edges_tmp Set of edge indices to inspect.
   * @return Set of vertex indices referenced as source or target by any edge
   *         in @p edges_tmp.
   */
  [[nodiscard]] cda_rail::index_set
  vertices_used_by_edges(const cda_rail::index_set& edges_tmp) const;

  /**
   * @brief For each TTD section that intersects @p edges, returns the TTD
   *        index together with the position of the first (entering) edge
   *        within @p edges.
   *
   * @param edges Ordered vector of edge indices representing a path.
   * @param ttd   Vector of TTD sections (each a set of edge indices).
   * @return Vector of pairs `{ttd_index, position_of_first_intersecting_edge}`
   *         where the position is an index into @p edges.
   */
  [[nodiscard]] static std::vector<std::pair<size_t, size_t>>
  get_intersecting_ttd(const cda_rail::index_vector&           edges,
                       const std::vector<cda_rail::index_set>& ttd);

  /**
   * @brief Returns the set of all network edge indices not in @p edge_indices.
   * @param edge_indices Set of edge indices to exclude.
   * @return Complement set with respect to all edges in the network.
   * @throws cda_rail::exceptions::EdgeNotExistentException If any index in
   *         @p edge_indices is invalid.
   */
  [[nodiscard]] cda_rail::index_set
  edge_set_complement(const cda_rail::index_set& edge_indices) const;

  /**
   * @brief Returns `edges_to_consider` minus @p edge_indices.
   * @param edge_indices      Set of edge indices to exclude.
   * @param edges_to_consider Set of edge indices to restrict to.
   * @return Set `edges_to_consider \ edge_indices`.
   * @throws cda_rail::exceptions::EdgeNotExistentException If any index in
   *         either argument set is invalid.
   */
  [[nodiscard]] cda_rail::index_set
  edge_set_complement(const cda_rail::index_set& edge_indices,
                      const cda_rail::index_set& edges_to_consider) const;

private:
  /**
   * @brief Implementation backing `maximal_vertex_speed`.
   * @param vertex_id        Numeric vertex index.
   * @param edges_to_consider Subset of incident edges to restrict to; empty
   *                          means all incident edges.
   * @return Maximum traversal speed at the vertex in m/s.
   */
  [[nodiscard]] double maximal_vertex_speed_helper(
      size_t vertex_id, const cda_rail::index_set& edges_to_consider) const;

public:
  /**
   * @brief Returns the maximum speed achievable at @p vertex.
   *
   * If the vertex has exactly one neighbor the speed of the single incident
   * edge is returned. Otherwise the second-highest speed over distinct
   * neighboring vertices is returned, reflecting the maximum speed at which a
   * train can traverse the vertex without reversing.
   *
   * @param vertex           Vertex descriptor.
   * @param edges_to_consider Subset of incident edges to consider; if empty,
   *                          all incident edges are used.
   * @return Maximum speed in m/s, or `0` if no relevant edges exist.
   * @throws cda_rail::exceptions::VertexNotExistentException If the vertex
   *         does not exist.
   */
  [[nodiscard]] double maximal_vertex_speed(
      VertexInput const&         vertex,
      const cda_rail::index_set& edges_to_consider = {}) const {
    return maximal_vertex_speed_helper(vertex.resolve(this), edges_to_consider);
  };

private:
  /**
   * @brief Implementation backing `minimal_neighboring_edge_length`.
   * @param v                Numeric vertex index.
   * @param edges_to_consider Subset of incident edges to restrict to; empty
   *                          means all incident edges.
   * @return Minimum incident edge length in metres.
   */
  [[nodiscard]] double minimal_neighboring_edge_length_helper(
      size_t v, const cda_rail::index_set& edges_to_consider) const;

public:
  /**
   * @brief Returns the length of the shortest edge incident to @p vertex.
   * @param vertex           Vertex descriptor.
   * @param edges_to_consider Subset of incident edges to consider; if empty,
   *                          all incident edges are used.
   * @return Minimum incident edge length in metres, or
   *         `std::numeric_limits<double>::infinity()` if there are no
   *         relevant edges.
   * @throws cda_rail::exceptions::VertexNotExistentException If the vertex
   *         does not exist.
   */
  [[nodiscard]] double minimal_neighboring_edge_length(
      VertexInput const&         vertex,
      const cda_rail::index_set& edges_to_consider = {}) const {
    return minimal_neighboring_edge_length_helper(vertex.resolve(this),
                                                  edges_to_consider);
  };

private:
  /**
   * @brief Returns the maximum number of VSS borders placeable on edge
   *        @p index.
   * @param index Numeric edge index.
   * @return `floor(length / min_block_length)`, or `0` if the edge is not
   *         breakable or `min_block_length` is zero.
   * @throws cda_rail::exceptions::EdgeNotExistentException If the edge does
   *         not exist.
   */
  [[nodiscard]] size_t max_vss_on_edge_helper(size_t index) const;

public:
  /**
   * @brief Returns the maximum number of VSS borders that can be placed on
   *        @p edge.
   *
   * The value equals `floor(length / min_block_length)`. Returns `0` for
   * non-breakable edges or when `min_block_length` is zero.
   *
   * @param edge Edge descriptor.
   * @return Maximum number of VSS borders.
   * @throws cda_rail::exceptions::EdgeNotExistentException If the edge does
   *         not exist.
   */
  [[nodiscard]] size_t max_vss_on_edge(EdgeInput const& edge) const {
    return max_vss_on_edge_helper(edge.resolve(this));
  };

  // -----------------------------
  // SETTER
  // -----------------------------

  /**
   * @brief Sets the network name.
   * @param networkName New name; must be a valid directory name on all
   *                    operating systems.
   * @throws cda_rail::exceptions::InvalidInputException If @p networkName is
   *         not a valid directory name.
   */
  void set_network_name(std::string_view const networkName) {
    exceptions::throw_if_invalid_folder_name(networkName);
    m_network_name = networkName;
  }

private:
  /**
   * @brief Renames vertex @p index to @p new_name.
   * @param index    Numeric vertex index.
   * @param new_name New vertex name.
   * @throws cda_rail::exceptions::VertexNotExistentException If the vertex
   *         does not exist.
   * @throws cda_rail::exceptions::InvalidInputException If a vertex with
   *         @p new_name already exists.
   */
  void change_vertex_name_helper(size_t index, std::string_view new_name);

  /**
   * @brief Changes the type of vertex @p index.
   * @param index    Numeric vertex index.
   * @param new_type New vertex type.
   * @throws cda_rail::exceptions::VertexNotExistentException If the vertex
   *         does not exist.
   */
  void change_vertex_type_helper(size_t index, VertexType new_type);

  /**
   * @brief Updates the headway of vertex @p index.
   * @param index       Numeric vertex index.
   * @param new_headway New headway value in seconds.
   * @throws cda_rail::exceptions::VertexNotExistentException If the vertex
   *         does not exist.
   */
  void change_vertex_headway_helper(size_t index, double new_headway);

public:
  /**
   * @brief Renames a vertex.
   * @param vertex   Vertex descriptor.
   * @param new_name New name for the vertex.
   * @throws cda_rail::exceptions::VertexNotExistentException If the vertex
   *         does not exist.
   * @throws cda_rail::exceptions::InvalidInputException If a vertex with
   *         @p new_name already exists.
   */
  void change_vertex_name(VertexInput const&     vertex,
                          std::string_view const new_name) {
    change_vertex_name_helper(vertex.resolve(this), new_name);
  };

  /**
   * @brief Changes the type of a vertex.
   * @param vertex   Vertex descriptor.
   * @param new_type New vertex type.
   * @throws cda_rail::exceptions::VertexNotExistentException If the vertex
   *         does not exist.
   */
  void change_vertex_type(VertexInput const& vertex,
                          VertexType const   new_type) {
    change_vertex_type_helper(vertex.resolve(this), new_type);
  };

  /**
   * @brief Updates the additional headway imposed on a vertex.
   * @param vertex      Vertex descriptor.
   * @param new_headway New headway value in seconds.
   * @throws cda_rail::exceptions::VertexNotExistentException If the vertex
   *         does not exist.
   */
  void change_vertex_headway(VertexInput const& vertex,
                             double const       new_headway) {
    change_vertex_headway_helper(vertex.resolve(this), new_headway);
  };

private:
  /**
   * @brief Sets the length of edge @p index.
   * @param index      Numeric edge index.
   * @param new_length New length in metres.
   * @throws cda_rail::exceptions::EdgeNotExistentException If the edge does
   *         not exist.
   */
  void change_edge_length_helper(size_t index, double new_length);

  /**
   * @brief Sets the maximum speed of edge @p index.
   * @param index         Numeric edge index.
   * @param new_max_speed New maximum speed in m/s.
   * @throws cda_rail::exceptions::EdgeNotExistentException If the edge does
   *         not exist.
   */
  void change_edge_max_speed_helper(size_t index, double new_max_speed);

  /**
   * @brief Sets the minimum block length of edge @p index.
   * @param index               Numeric edge index.
   * @param new_min_block_length New minimum block length in metres.
   * @throws cda_rail::exceptions::EdgeNotExistentException If the edge does
   *         not exist.
   */
  void change_edge_min_block_length_helper(size_t index,
                                           double new_min_block_length);

  /**
   * @brief Sets the minimum stop-block length of edge @p index.
   * @param index                    Numeric edge index.
   * @param new_min_stop_block_length New minimum stop-block length in metres.
   * @throws cda_rail::exceptions::EdgeNotExistentException If the edge does
   *         not exist.
   */
  void
  change_edge_min_stop_block_length_helper(size_t index,
                                           double new_min_stop_block_length);

  /**
   * @brief Sets the breakable flag of edge @p index.
   * @param index Numeric edge index.
   * @param value New breakable status.
   * @throws cda_rail::exceptions::EdgeNotExistentException If the edge does
   *         not exist.
   */
  void change_edge_breakable_helper(size_t index, bool value);

public:
  /**
   * @brief Changes the length of an edge.
   * @param edge       Edge descriptor.
   * @param new_length New length in metres.
   * @throws cda_rail::exceptions::EdgeNotExistentException If the edge does
   *         not exist.
   */
  void change_edge_length(EdgeInput const& edge, double new_length) {
    change_edge_length_helper(edge.resolve(this), new_length);
  };

  /**
   * @brief Changes the maximum speed of an edge.
   * @param edge          Edge descriptor.
   * @param new_max_speed New maximum speed in m/s.
   * @throws cda_rail::exceptions::EdgeNotExistentException If the edge does
   *         not exist.
   */
  void change_edge_max_speed(EdgeInput const& edge, double new_max_speed) {
    change_edge_max_speed_helper(edge.resolve(this), new_max_speed);
  }

  /**
   * @brief Changes the minimum block length of an edge.
   * @param edge                Edge descriptor.
   * @param new_min_block_length New minimum block length in metres.
   * @throws cda_rail::exceptions::EdgeNotExistentException If the edge does
   *         not exist.
   */
  void change_edge_min_block_length(EdgeInput const& edge,
                                    double           new_min_block_length) {
    change_edge_min_block_length_helper(edge.resolve(this),
                                        new_min_block_length);
  }

  /**
   * @brief Changes the minimum stop-block length of an edge.
   * @param edge                     Edge descriptor.
   * @param new_min_stop_block_length New minimum stop-block length in metres.
   * @throws cda_rail::exceptions::EdgeNotExistentException If the edge does
   *         not exist.
   */
  void change_edge_min_stop_block_length(EdgeInput const& edge,
                                         double new_min_stop_block_length) {
    change_edge_min_stop_block_length_helper(edge.resolve(this),
                                             new_min_stop_block_length);
  }

  /**
   * @brief Marks an edge as breakable (VSS borders may be placed on it).
   * @param edge Edge descriptor.
   * @throws cda_rail::exceptions::EdgeNotExistentException If the edge does
   *         not exist.
   */
  void set_edge_breakable(EdgeInput const& edge) {
    change_edge_breakable_status(edge, true);
  };

  /**
   * @brief Marks an edge as unbreakable (no VSS borders may be placed on it).
   * @param edge Edge descriptor.
   * @throws cda_rail::exceptions::EdgeNotExistentException If the edge does
   *         not exist.
   */
  void set_edge_unbreakable(EdgeInput const& edge) {
    change_edge_breakable_status(edge, false);
  };

  /**
   * @brief Sets the breakable flag of an edge.
   * @param edge      Edge descriptor.
   * @param breakable New breakable status.
   * @throws cda_rail::exceptions::EdgeNotExistentException If the edge does
   *         not exist.
   */
  void change_edge_breakable_status(EdgeInput const& edge,
                                    bool const       breakable) {
    change_edge_breakable_helper(edge.resolve(this), breakable);
  };

  /**
   * @brief Adds a vertex to the network.
   * @param vertex `Vertex` object to add.
   * @return Index of the newly added vertex.
   * @throws cda_rail::exceptions::InvalidInputException If a vertex with the
   *         same name already exists.
   */
  size_t add_vertex(Vertex vertex);

  /**
   * @brief Constructs a vertex from @p args and adds it to the network.
   * @tparam Args Argument types forwarded to the `Vertex` constructor.
   * @param args Arguments forwarded to `Vertex(args...)`.
   * @return Index of the newly added vertex.
   * @throws cda_rail::exceptions::InvalidInputException If a vertex with the
   *         same name already exists.
   */
  template <typename... Args> size_t add_vertex(Args... args) {
    return add_vertex(Vertex(std::forward<Args>(args)...));
  };

private:
  /**
   * @brief Validates constraints and inserts a new directed edge.
   * @param source               Index of the source vertex.
   * @param target               Index of the target vertex.
   * @param length               Edge length in metres.
   * @param max_speed            Maximum speed in m/s.
   * @param breakable            Optional breakable flag.
   * @param min_block_length     Optional minimum block length.
   * @param min_stop_block_length Optional minimum stop-block length.
   * @return Index of the newly added edge.
   * @throws cda_rail::exceptions::InvalidInputException If source equals
   *         target or the edge already exists.
   * @throws cda_rail::exceptions::VertexNotExistentException If either vertex
   *         does not exist.
   */
  size_t add_edge_helper(size_t source, size_t target, double length,
                         double max_speed, std::optional<bool> const& breakable,
                         std::optional<double> const& min_block_length,
                         std::optional<double> const& min_stop_block_length);

public:
  /**
   * @brief Adds a directed edge between two vertices.
   * @param source               Source vertex descriptor.
   * @param target               Target vertex descriptor.
   * @param length               Edge length in metres.
   * @param maxSpeed             Maximum speed in m/s.
   * @param breakable            Whether VSS borders may be placed on this
   *                             edge; `std::nullopt` uses
   *                             `Edge::BREAKABLE_DEFAULT`.
   * @param min_block_length     Minimum block length in metres; `std::nullopt`
   *                             uses `Edge::MIN_BLOCK_LENGTH_DEFAULT`.
   * @param min_stop_block_length Minimum stop-block length in metres;
   *                              `std::nullopt` uses
   *                              `Edge::MIN_STOP_BLOCK_LENGTH_DEFAULT`.
   * @return Index of the newly added edge.
   * @throws cda_rail::exceptions::InvalidInputException If source equals
   *         target or the edge already exists.
   * @throws cda_rail::exceptions::VertexNotExistentException If either vertex
   *         does not exist.
   */
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
  /**
   * @brief Registers @p edge_out as a valid successor of @p edge_in.
   *
   * The edges must be adjacent (target of @p edge_in == source of
   * @p edge_out). If the relation is already registered the call is a no-op.
   *
   * @param edge_in  Index of the predecessor edge.
   * @param edge_out Index of the successor edge to register.
   * @throws cda_rail::exceptions::EdgeNotExistentException If either edge
   *         does not exist.
   * @throws cda_rail::exceptions::ConsistencyException If the edges are not
   *         adjacent.
   */
  void add_successor_helper(size_t edge_in, size_t edge_out);

public:
  /**
   * @brief Registers @p edge_out as a valid successor of @p edge_in.
   *
   * The edges must be adjacent (target of @p edge_in == source of
   * @p edge_out). If the relation already exists the call is a no-op.
   *
   * @param edge_in  Predecessor edge descriptor.
   * @param edge_out Successor edge descriptor to register.
   * @throws cda_rail::exceptions::EdgeNotExistentException If either edge
   *         does not exist.
   * @throws cda_rail::exceptions::ConsistencyException If the edges are not
   *         adjacent.
   */
  void add_successor(EdgeInput const& edge_in, EdgeInput const& edge_out) {
    // Pass 'this' to the inputs so they can resolve themselves
    add_successor_helper(edge_in.resolve(this), edge_out.resolve(this));
  }

  // ----------------------
  // Transformation Functions
  // ----------------------

  /**
   * @brief Separates a breakable edge (and its reverse, if present) into
   *        multiple sub-edges according to @p sep_func and the edge's
   *        `min_block_length`.
   *
   * @param edge     Edge descriptor of the edge to separate.
   * @param sep_func Separation function that positions the new VSS-border
   *                 vertices; defaults to uniform separation.
   * @return Pair `{forward_new_edges, reverse_new_edges}` where each vector
   *         contains the indices of the new sub-edges ordered from source to
   *         target.
   * @throws cda_rail::exceptions::ConsistencyException If the network fails
   *         `is_consistent_for_transformation()` or the edge is not breakable.
   */
  std::pair<cda_rail::index_vector, cda_rail::index_vector> separate_edge(
      EdgeInput const&               edge,
      const vss::SeparationFunction& sep_func = &vss::functions::uniform) {
    auto const edge_index = edge.resolve(this);
    return separate_edge_private_helper(
        edge_index, get_edge(edge_index).min_block_length, sep_func);
  };

  /**
   * @brief Separates a breakable edge using `min_stop_block_length` with
   *        uniform separation.
   *
   * @param edge Edge descriptor of the edge to separate.
   * @return Pair `{forward_new_edges, reverse_new_edges}`.
   * @throws cda_rail::exceptions::ConsistencyException If the network fails
   *         `is_consistent_for_transformation()` or the edge is not breakable.
   */
  std::pair<cda_rail::index_vector, cda_rail::index_vector>
  separate_stop_edge(EdgeInput const& edge) {
    auto const edge_index = edge.resolve(this);
    return separate_edge_private_helper(
        edge_index, get_edge(edge_index).min_stop_block_length,
        &vss::functions::uniform, true);
  };

  /**
   * @brief Separates all edges in @p stop_edges whose length is at least
   *        twice their `min_stop_block_length`.
   *
   * @param stop_edges Vector of edge indices to process.
   * @return Vector of pairs `{last_new_edge_index, all_new_edge_indices}` for
   *         each separated direction; edges that are too short are skipped.
   */
  std::vector<std::pair<size_t, cda_rail::index_vector>>
  separate_stop_edges(const cda_rail::index_set& stop_edges);

  /**
   * @brief Discretizes all relevant breakable edges, inserting new
   *        `NoBorderVSS` vertices so that VSS borders are only permitted at
   *        those vertex positions.
   *
   * @param sep_func Separation function that controls the placement of the
   *                 new vertices; defaults to uniform separation.
   * @return Vector of pairs `{original_edge_index, new_edge_indices}` for
   *         every separated direction.
   */
  std::vector<std::pair<size_t, cda_rail::index_vector>> discretize(
      const vss::SeparationFunction& sep_func = &vss::functions::uniform);

  // ------------------------
  // Path Finding Algorithms
  // ------------------------

  /**
   * @brief Returns all edge paths of length at least @p desired_len starting
   *        at vertex @p v.
   *
   * A path is included if its total length is at least @p desired_len, but
   * removing the last edge would make it too short.
   *
   * @param v                       Starting vertex index.
   * @param desired_len             Minimum path length in metres.
   * @param exit_node               If set, a path may terminate at this
   *                                vertex even before reaching @p desired_len.
   * @param edges_to_consider       If non-empty, only these edges are used.
   * @param return_successors_if_zero If `true` and @p desired_len is `0`,
   *                                  single-edge paths for each outgoing edge
   *                                  are returned.
   * @return Vector of edge-index vectors, each representing one path.
   */
  [[nodiscard]] std::vector<cda_rail::index_vector>
  all_paths_of_length_starting_in_vertex(
      size_t v, double desired_len, std::optional<size_t> exit_node = {},
      cda_rail::index_set edges_to_consider         = {},
      bool                return_successors_if_zero = false) const {
    return all_routes_of_given_length(v, std::nullopt, desired_len, false,
                                      exit_node, std::move(edges_to_consider),
                                      return_successors_if_zero);
  };

  /**
   * @brief Returns all edge paths of length at least @p desired_len starting
   *        immediately after edge @p e (i.e., at the target of @p e).
   *
   * @param e                 Starting edge index.
   * @param desired_len       Minimum path length in metres.
   * @param exit_node         Optional vertex at which a path may terminate
   *                          early.
   * @param edges_to_consider If non-empty, only these edges are used.
   * @return Vector of edge-index vectors, each representing one path.
   */
  [[nodiscard]] std::vector<cda_rail::index_vector>
  all_paths_of_length_starting_in_edge(
      size_t e, double desired_len, std::optional<size_t> exit_node = {},
      cda_rail::index_set edges_to_consider = {}) const {
    return all_routes_of_given_length(std::nullopt, e, desired_len, false,
                                      exit_node, std::move(edges_to_consider));
  };

  /**
   * @brief Returns all edge paths of length at least @p desired_len ending at
   *        vertex @p v (paths are listed from source side towards @p v).
   *
   * @param v                 Terminal vertex index.
   * @param desired_len       Minimum path length in metres.
   * @param exit_node         Optional additional vertex constraint.
   * @param edges_to_consider If non-empty, only these edges are used.
   * @return Vector of edge-index vectors, each representing one path.
   */
  [[nodiscard]] std::vector<cda_rail::index_vector>
  all_paths_of_length_ending_in_vertex(
      size_t v, double desired_len, std::optional<size_t> exit_node = {},
      cda_rail::index_set edges_to_consider = {}) const {
    return all_routes_of_given_length(v, std::nullopt, desired_len, true,
                                      exit_node, std::move(edges_to_consider));
  };

  /**
   * @brief Returns all edge paths of length at least @p desiredLen ending at
   *        edge @p e.
   *
   * @param e                 Terminal edge index.
   * @param desiredLen        Minimum path length in metres.
   * @param exit_node         Optional additional vertex constraint.
   * @param edges_to_consider If non-empty, only these edges are used.
   * @return Vector of edge-index vectors, each representing one path.
   */
  [[nodiscard]] std::vector<cda_rail::index_vector>
  all_paths_of_length_ending_in_edge(
      size_t const e, double const desiredLen,
      std::optional<size_t> exit_node         = {},
      cda_rail::index_set   edges_to_consider = {}) const {
    return all_routes_of_given_length(std::nullopt, e, desiredLen, true,
                                      std::move(exit_node),
                                      std::move(edges_to_consider));
  }

private:
  /**
   * @brief Returns all paths starting immediately after edge @p e_0 and
   *        ending upon entering a TTD section or reaching the exit vertex if
   * specified.
   *
   * @param e_0          Starting edge (paths begin with a valid successor of
   *                     this edge).
   * @param ttd_sections Vector of TTD sections (each a set of edge indices).
   * @param exit_node    Optional vertex at which paths may terminate.
   * @return Vector of edge-index vectors, each representing one such path.
   */
  [[nodiscard]] std::vector<cda_rail::index_vector>
  all_paths_ending_at_ttd_helper(
      size_t e_0, const std::vector<cda_rail::index_set>& ttd_sections,
      std::optional<size_t> exit_node) const;

public:
  /**
   * @brief Returns all paths starting immediately after edge @p e_0 and
   *        ending upon entering a TTD section or reaching the exit vertex if
   * specified.
   *
   * @param e_0          Starting edge (paths begin with a valid successor of
   *                     this edge).
   * @param ttd_sections Vector of TTD sections (each a set of edge indices).
   * @param exit_node    Optional vertex at which paths may terminate.
   * @return Vector of edge-index vectors, each representing one such path.
   */
  [[nodiscard]] std::vector<cda_rail::index_vector>
  all_paths_ending_at_ttd(EdgeInput const&                        edge,
                          const std::vector<cda_rail::index_set>& ttd_sections,
                          std::optional<size_t> exit_node = {}) const {
    return all_paths_ending_at_ttd_helper(edge.resolve(this), ttd_sections,
                                          exit_node);
  };

  /**
   * @brief Computes the total length of a path given as an ordered sequence
   *        of edge indices.
   * @param path Ordered vector of edge indices forming the path.
   * @return Sum of the lengths of all edges in @p path, in metres.
   */
  [[nodiscard]] double length_of_path(const cda_rail::index_vector& path) const;

  /**
   * @brief Computes the shortest-path distance matrix between all ordered
   *        edge pairs using the Floyd–Warshall algorithm.
   *
   * Entry `[u][v]` stores the shortest distance from the target of edge @p u
   * to the target of edge @p v using only valid successors. Entries where no
   * path exists are set to `INF`.
   *
   * @return `n x n` distance matrix, where `n` is the number of edges.
   */
  [[nodiscard]] std::vector<std::vector<double>>
  all_edge_pairs_shortest_paths() const;

private:
  /**
   * @brief Dijkstra-based shortest path from a single source edge to a single
   *        target.
   *
   * If the source edge already reaches the target (without including the
   * first-edge length) the distance is `0`. Only valid successors of the
   * source edge are used as the first continuation.
   *
   * @param source_edge_id   Index of the source edge.
   * @param target_id        Index of the target vertex or edge.
   * @param target_is_edge   If `true`, @p target_id is an edge index;
   *                         otherwise it is a vertex index.
   * @param include_first_edge If `true`, the length of the source edge
   *                           contributes to the distance.
   * @param use_minimal_time If `true`, distances are measured in travel time
   *                         using the maximum allowed speed.
   * @param max_v            Maximum train speed in m/s (used when
   *                         @p use_minimal_time is `true`).
   * @return Shortest distance, or `std::nullopt` if the target is
   *         unreachable.
   */
  [[nodiscard]] std::optional<double>
  shortest_path_helper(size_t source_edge_id, size_t target_id,
                       bool target_is_edge     = false,
                       bool include_first_edge = false,
                       bool use_minimal_time = false, double max_v = INF) const;

public:
  /**
   * @brief Returns the shortest distance from @p source_edge to
   *        @p target_edge.
   *
   * The distance is measured from the target of @p source_edge to the target
   * of @p target_edge using only valid successors.
   *
   * @param source_edge       Starting edge descriptor.
   * @param target_edge       Destination edge descriptor.
   * @param include_first_edge If `true`, the length of @p source_edge is
   *                           included in the distance.
   * @param use_minimal_time  If `true`, distances are measured in travel time.
   * @param max_v             Maximum train speed in m/s.
   * @return Shortest distance, or `std::nullopt` if unreachable.
   * @throws cda_rail::exceptions::EdgeNotExistentException If either edge
   *         does not exist.
   */
  [[nodiscard]] std::optional<double> shortest_path_from_edge_to_edge(
      EdgeInput const& source_edge, EdgeInput const& target_edge,
      bool include_first_edge = false, bool use_minimal_time = false,
      double max_v = INF) const {
    return shortest_path_helper(source_edge.resolve(this),
                                target_edge.resolve(this), true,
                                include_first_edge, use_minimal_time, max_v);
  };

  /**
   * @brief Returns the shortest distance from @p source_edge to
   *        @p target_vertex.
   *
   * The distance is measured from the target of @p source_edge to
   * @p target_vertex using only valid successors.
   *
   * @param source_edge       Starting edge descriptor.
   * @param target_vertex     Destination vertex descriptor.
   * @param include_first_edge If `true`, the length of @p source_edge is
   *                           included.
   * @param use_minimal_time  If `true`, distances are measured in travel time.
   * @param max_v             Maximum train speed in m/s.
   * @return Shortest distance, or `std::nullopt` if unreachable.
   * @throws cda_rail::exceptions::EdgeNotExistentException If the source edge
   *         does not exist.
   * @throws cda_rail::exceptions::VertexNotExistentException If the target
   *         vertex does not exist.
   */
  [[nodiscard]] std::optional<double> shortest_path_from_edge_to_vertex(
      EdgeInput const& source_edge, VertexInput const& target_vertex,
      bool include_first_edge = false, bool use_minimal_time = false,
      double max_v = INF) const {
    return shortest_path_helper(source_edge.resolve(this),
                                target_vertex.resolve(this), false,
                                include_first_edge, use_minimal_time, max_v);
  };

private:
  /**
   * @brief Single-source Dijkstra returning both the shortest distance and
   *        the corresponding path.
   *
   * Delegates to `shortest_path_between_sets_using_edges_helper` with
   * singleton source and target sets.
   *
   * @param source_edge_id           Index of the source edge.
   * @param target_vertex_id         Index of the target vertex or edge.
   * @param only_use_valid_successors If `true`, only registered successors
   *                                  are followed.
   * @param edges_to_use             If non-empty, restricts traversable edges.
   * @param target_is_edge           If `true`, @p target_vertex_id is an edge
   *                                 index.
   * @param include_first_edge       If `true`, includes source edge length.
   * @param use_minimal_time         If `true`, measures in travel time.
   * @param max_v                    Maximum train speed in m/s.
   * @return Pair of `{optional distance, path as edge-index vector}`.
   */
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
  /**
   * @brief Multi-source, multi-target Dijkstra returning the shortest
   *        distance and path.
   *
   * A train may start on any edge in @p source_edge_ids and must reach any
   * element in @p target_ids. Only valid successors are used when
   * @p only_use_valid_successors is `true`. If @p edges_to_use is non-empty,
   * only those edges may be traversed.
   *
   * @param source_edge_ids          Set of starting edge indices.
   * @param target_ids               Set of target vertex or edge indices.
   * @param only_use_valid_successors If `true`, only registered successors
   *                                  are followed.
   * @param edges_to_use             If non-empty, restricts traversable edges.
   * @param target_is_edge           If `true`, elements of @p target_ids are
   *                                 edge indices.
   * @param include_first_edge       If `true`, includes source edge lengths.
   * @param use_minimal_time         If `true`, measures in travel time.
   * @param max_v                    Maximum train speed in m/s.
   * @return Pair of `{optional distance, path as edge-index vector}`.
   * @throws cda_rail::exceptions::InvalidInputException If either set is
   *         empty or @p use_minimal_time is `true` with non-positive @p max_v.
   * @throws cda_rail::exceptions::EdgeNotExistentException If a source edge
   *         does not exist.
   * @throws cda_rail::exceptions::VertexNotExistentException If a target
   *         vertex does not exist (when @p target_is_edge is `false`).
   */
  [[nodiscard]] std::pair<std::optional<double>, cda_rail::index_vector>
  shortest_path_between_sets_using_edges_helper(
      cda_rail::index_set source_edge_ids, cda_rail::index_set target_ids,
      bool                only_use_valid_successors = true,
      cda_rail::index_set edges_to_use = {}, bool target_is_edge = false,
      bool include_first_edge = false, bool use_minimal_time = false,
      double max_v = INF) const;

  /**
   * @brief Returns only the distance component of
   *        `shortest_path_between_sets_using_edges_helper`.
   *
   * @param source_edge_ids  Set of starting edge indices.
   * @param target_ids       Set of target indices.
   * @param target_is_edge   If `true`, elements of @p target_ids are edge
   *                         indices.
   * @param include_first_edge If `true`, includes source edge lengths.
   * @param use_minimal_time If `true`, measures in travel time.
   * @param max_v            Maximum train speed in m/s.
   * @return Shortest distance, or `std::nullopt` if unreachable.
   */
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
  /**
   * @brief Returns the shortest path and its length from @p source_edge to
   *        @p target_edge.
   *
   * @param source_edge              Starting edge descriptor.
   * @param target_edge              Destination edge descriptor.
   * @param only_use_valid_successors If `true`, only registered successors
   *                                  are followed.
   * @param edges_to_use             If non-empty, restricts traversable edges.
   * @param include_first_edge       If `true`, includes @p source_edge length.
   * @param use_minimal_time         If `true`, measures in travel time.
   * @param max_v                    Maximum train speed in m/s.
   * @return Pair of `{optional distance, path as edge-index vector}`.
   * @throws cda_rail::exceptions::EdgeNotExistentException If either edge
   *         does not exist.
   */
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

  /**
   * @brief Returns the shortest path and its length from @p source_edge to
   *        @p target_vertex.
   *
   * @param source_edge              Starting edge descriptor.
   * @param target_vertex            Destination vertex descriptor.
   * @param only_use_valid_successors If `true`, only registered successors
   *                                  are followed.
   * @param edges_to_use             If non-empty, restricts traversable edges.
   * @param include_first_edge       If `true`, includes source edge length.
   * @param use_minimal_time         If `true`, measures in travel time.
   * @param max_v                    Maximum train speed in m/s.
   * @return Pair of `{optional distance, path as edge-index vector}`.
   * @throws cda_rail::exceptions::EdgeNotExistentException If the source edge
   *         does not exist.
   * @throws cda_rail::exceptions::VertexNotExistentException If the target
   *         vertex does not exist.
   */
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

  /**
   * @brief Returns the shortest path and distance from any edge in
   *        @p source_edge_ids to any edge in @p target_edge_ids.
   *
   * @param source_edge_ids          Set of starting edge indices.
   * @param target_edge_ids          Set of destination edge indices.
   * @param only_use_valid_successors If `true`, only registered successors
   *                                  are followed.
   * @param edges_to_use             If non-empty, restricts traversable edges.
   * @param include_first_edge       If `true`, includes source edge length.
   * @param use_minimal_time         If `true`, measures in travel time.
   * @param max_v                    Maximum train speed in m/s.
   * @return Pair of `{optional distance, path as edge-index vector}`.
   * @throws cda_rail::exceptions::EdgeNotExistentException If either edge
   *         does not exist.
   */
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

  /**
   * @brief Returns the shortest path and distance from any edge in
   *        @p source_edge_ids to any vertex in @p target_vertex_ids.
   *
   * @param source_edge_ids          Set of starting edge indices.
   * @param target_vertex_ids        Set of destination vertex indices.
   * @param only_use_valid_successors If `true`, only registered successors
   *                                  are followed.
   * @param edges_to_use             If non-empty, restricts traversable edges.
   * @param include_first_edge       If `true`, includes source edge length.
   * @param use_minimal_time         If `true`, measures in travel time.
   * @param max_v                    Maximum train speed in m/s.
   * @return Pair of `{optional distance, path as edge-index vector}`.
   * @throws cda_rail::exceptions::EdgeNotExistentException If the source edge
   *         does not exist.
   * @throws cda_rail::exceptions::VertexNotExistentException If the target
   *         vertex does not exist.
   */
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

  /**
   * @brief Returns the length of the shortest path from any edge in
   *        @p source_edge_ids to any edge in @p target_ids.
   *
   * @param source_edge_ids  Set of starting edge indices.
   * @param target_ids       Set of destination edge indices.
   * @param include_first_edge If `true`, includes source edge length.
   * @param use_minimal_time If `true`, measures in travel time.
   * @param max_v            Maximum train speed in m/s.
   * @return Shortest distance, or `std::nullopt` if unreachable.
   */
  [[nodiscard]] std::optional<double> shortest_path_length_between_edge_sets(
      cda_rail::index_set source_edge_ids, cda_rail::index_set target_ids,
      bool include_first_edge = false, bool use_minimal_time = false,
      double max_v = INF) const {
    return shortest_path_length_between_sets_helper(
        std::move(source_edge_ids), std::move(target_ids), true,
        include_first_edge, use_minimal_time, max_v);
  };

  /**
   * @brief Returns the length of the shortest path from any edge in
   *        @p source_edge_ids to any vertex in @p target_vertex_ids.
   *
   * @param source_edge_ids   Set of starting edge indices.
   * @param target_vertex_ids Set of destination vertex indices.
   * @param include_first_edge If `true`, includes source edge length.
   * @param use_minimal_time  If `true`, measures in travel time.
   * @param max_v             Maximum train speed in m/s.
   * @return Shortest distance, or `std::nullopt` if unreachable.
   */
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

  /**
   * @brief Reads the graph from `tracks.graphml` in directory @p p and
   *        populates the vertex/edge data structures.
   * @param p Path to the network directory containing `tracks.graphml`.
   * @throws cda_rail::exceptions::ImportException If the file cannot be
   *         loaded or is malformed.
   */
  void read_graphml(const std::filesystem::path& p);

  /**
   * @brief Extracts GraphML attribute key IDs from the `<graphml>` root
   *        element and writes them into the provided optional string
   *        references in place.
   *
   * @param graphml_body          Root `<graphml>` XML element.
   * @param breakable             Receives the key ID for the `breakable`
   *                              attribute.
   * @param length                Receives the key ID for the `length`
   *                              attribute.
   * @param max_speed             Receives the key ID for the `max_speed`
   *                              attribute.
   * @param min_block_length      Receives the key ID for the `min_block_length`
   *                              attribute.
   * @param min_stop_block_length Receives the key ID for the
   *                              `min_stop_block_length` attribute.
   * @param type                  Receives the key ID for the vertex `type`
   *                              attribute.
   * @param headway               Receives the key ID for the `headway`
   *                              attribute.
   */
  static void get_keys_inplace(
      tinyxml2::XMLElement* graphml_body, std::optional<std::string>& breakable,
      std::optional<std::string>& length, std::optional<std::string>& max_speed,
      std::optional<std::string>& min_block_length,
      std::optional<std::string>& min_stop_block_length,
      std::optional<std::string>& type, std::optional<std::string>& headway);

  /**
   * @brief Iterates over all `<node>` elements starting at @p graphml_node
   *        and adds the corresponding vertices to the network in place.
   *
   * @param graphml_node First `<node>` XML element.
   * @param type         Optional key ID for the vertex type attribute.
   * @param headway      Optional key ID for the headway attribute.
   * @throws cda_rail::exceptions::ImportException If a node is missing the
   *         type attribute.
   */
  void add_vertices_from_graphml(const tinyxml2::XMLElement*       graphml_node,
                                 const std::optional<std::string>& type,
                                 const std::optional<std::string>& headway);

  /**
   * @brief Parses a JSON successor-map key of the form
   *        `('source_name','target_name')` and writes the two vertex names
   *        into @p source_name and @p target_name in place.
   *
   * @param key         Key string to parse.
   * @param source_name Receives the source vertex name.
   * @param target_name Receives the target vertex name.
   */
  static void extract_vertices_from_key_inplace(const std::string& key,
                                                std::string&       source_name,
                                                std::string&       target_name);

  /**
   * @brief Iterates over all `<edge>` elements starting at @p graphml_edge
   *        and adds the corresponding directed edges to the network in place.
   *
   * @param graphml_edge          First `<edge>` XML element.
   * @param breakable             Optional key ID for the `breakable`
   *                              attribute.
   * @param length                Optional key ID for the `length` attribute.
   * @param max_speed             Optional key ID for the `max_speed`
   *                              attribute.
   * @param min_block_length      Optional key ID for the `min_block_length`
   *                              attribute.
   * @param min_stop_block_length Optional key ID for the
   *                              `min_stop_block_length` attribute.
   * @throws cda_rail::exceptions::ImportException If length or max_speed is
   *         missing for any edge.
   */
  void add_edges_from_graphml(
      const tinyxml2::XMLElement*       graphml_edge,
      const std::optional<std::string>& breakable,
      const std::optional<std::string>& length,
      const std::optional<std::string>& max_speed,
      const std::optional<std::string>& min_block_length,
      const std::optional<std::string>& min_stop_block_length);

  /**
   * @brief Reads the successor map from `successors_cpp.json` in directory
   *        @p p and registers all successor relations.
   * @param p Path to the network directory containing `successors_cpp.json`.
   */
  void read_successors(const std::filesystem::path& p);

  /**
   * @brief Writes the graph to `tracks.graphml` in directory @p p.
   * @param p Destination directory path.
   */
  void export_graphml(const std::filesystem::path& p) const;

  /**
   * @brief Writes the successor map to `successors.txt` (Python dict format)
   *        in directory @p p.
   * @param p Destination directory path.
   */
  void export_successors_python(const std::filesystem::path& p) const;

  /**
   * @brief Writes the successor map to `successors_cpp.json` (JSON format
   *        used by the C++ import) in directory @p p.
   * @param p Destination directory path.
   */
  void export_successors_cpp(const std::filesystem::path& p) const;

  /**
   * @brief Writes the successor set of edge @p i to @p file in Python set
   *        notation.
   *
   * Produces `set()` for an empty successor set or
   * `{('src','tgt'), ...}` otherwise.
   *
   * @param file Output file stream.
   * @param i    Index of the edge whose successors are written.
   */
  void write_successor_set_to_file(std::ofstream& file, size_t i) const;

  /**
   * @brief Returns the endpoint of edge @p e that is not @p v.
   * @param e Edge index.
   * @param v One endpoint of @p e.
   * @return The other endpoint of @p e.
   */
  [[nodiscard]] size_t other_vertex(size_t e, size_t v) const {
    return get_edge(e).source == v ? get_edge(e).target : get_edge(e).source;
  };

  /**
   * @brief Separates edge @p edge_index (and its reverse, if present) using
   *        @p sep_func with @p min_length as the minimum block size.
   *
   * The number of new blocks is determined by the maximum integer @p n such
   * that `min_length / edge.length` satisfies the separation function for
   * @p n blocks.
   *
   * @param edge_index       Index of the edge to separate.
   * @param min_length       Minimum block length used to compute the number
   *                         of blocks.
   * @param sep_func         Separation function; defaults to uniform.
   * @param new_edge_breakable Whether the new sub-edges should be marked as
   *                           breakable; defaults to `false`.
   * @return Pair `{forward_new_edges, reverse_new_edges}`.
   * @throws cda_rail::exceptions::ConsistencyException If the network is not
   *         consistent for transformation or the edge is not breakable.
   */
  std::pair<cda_rail::index_vector, cda_rail::index_vector>
  separate_edge_private_helper(
      size_t edge_index, double min_length,
      const vss::SeparationFunction& sep_func = &vss::functions::uniform,
      bool                           new_edge_breakable = false);

  /**
   * @brief Splits edge @p edge_index and its reverse (if present) at the
   *        given distances from the source vertex.
   *
   * The following structural changes are performed:
   * - New `NoBorderVSS` vertices are inserted at each split point.
   * - New sub-edges are created and connected.
   * - The successor map is updated accordingly.
   * - The original edge is shortened to span the last segment.
   *
   * @param edge_index            Index of the edge to split.
   * @param distances_from_source Sorted, strictly positive distances (in
   *                               metres) from the source vertex at which to
   *                               split; must be strictly less than the edge
   *                               length.
   * @param new_edge_breakable    Whether new sub-edges are breakable; defaults
   *                              to `false`.
   * @return Pair `{forward_new_edges, reverse_new_edges}` where each vector
   *         lists sub-edge indices in order from source to target.
   * @throws cda_rail::exceptions::EdgeNotExistentException If @p edge_index
   *         is invalid.
   * @throws cda_rail::exceptions::InvalidInputException If
   *         @p distances_from_source is empty.
   * @throws cda_rail::exceptions::ConsistencyException If distances are not
   *         sorted, out of range, or the reverse edge has a different length.
   */
  std::pair<cda_rail::index_vector, cda_rail::index_vector>
  separate_edge_at(size_t                     edge_index,
                   const std::vector<double>& distances_from_source,
                   bool                       new_edge_breakable = false);
  /**
   * @brief Updates the new-to-old edge mapping after a transformation step.
   *
   * If @p old_edge is itself already mapped, the stored offset is added to
   * @p position so that the mapping always refers to the original pre-
   * transformation edge.
   *
   * @param new_edge Index of the new (sub-)edge.
   * @param old_edge Index of the edge it was derived from.
   * @param position Offset from the source of @p old_edge to the source of
   *                 @p new_edge, in metres.
   */
  void update_new_old_edge(size_t new_edge, size_t old_edge, double position);

  /**
   * @brief Sorts a vector of edge pairs so that consecutive pairs share a
   *        vertex, forming an ordered path representation.
   *
   * Each pair must satisfy `first.has_value()` and `second` must equal the
   * reverse of `first` (or be `std::nullopt`). The sort is only valid when
   * the pairs correspond to a simple path.
   *
   * @param edge_pairs Vector of `{forward_edge, optional_reverse_edge}` pairs
   *                   to sort in place (passed by non-const reference).
   * @return Sorted vector of edge pairs.
   * @throws cda_rail::exceptions::InvalidInputException If any first entry is
   *         empty.
   * @throws cda_rail::exceptions::EdgeNotExistentException If any first entry
   *         refers to a non-existent edge.
   * @throws cda_rail::exceptions::ConsistencyException If a pair's second
   *         entry is not the reverse of its first, or if the path is not
   *         simple.
   */
  std::vector<std::pair<std::optional<size_t>, std::optional<size_t>>>
  sort_edge_pairs(
      std::vector<std::pair<std::optional<size_t>, std::optional<size_t>>>&
          edge_pairs) const;

  /**
   * @brief Validates that a new directed edge may be added between @p source
   *        and @p target.
   *
   * Checks that source differs from target, both vertices exist, and no edge
   * between them exists yet.
   *
   * @param source Index of the source vertex.
   * @param target Index of the target vertex.
   * @throws cda_rail::exceptions::InvalidInputException If source equals
   *         target or the edge already exists.
   * @throws cda_rail::exceptions::VertexNotExistentException If either vertex
   *         does not exist.
   */
  void check_new_edge_requirements(size_t source, size_t target) const;

  /**
   * @brief Returns the cost of traversing @p successor_edge, either as
   *        distance (metres) or as minimum travel time (seconds).
   *
   * @param successor_edge The edge being traversed.
   * @param max_v          Maximum train speed in m/s; only used when
   *                       @p use_minimal_time is `true`.
   * @param use_minimal_time If `true`, returns `length / min(max_v,
   *                          max_speed)`; otherwise returns `length`.
   * @return Cost of traversing the edge.
   * @throws cda_rail::exceptions::InvalidInputException If
   *         @p use_minimal_time is `true` and the effective speed is not
   *         strictly positive.
   */
  [[nodiscard]] static double delta_dist_helper(const Edge& successor_edge,
                                                double      max_v,
                                                bool        use_minimal_time);

  /**
   * @brief DFS-based section finder that groups incident edges by inner vertex
   *        type.
   *
   * Starting from each vertex in @p vertices_to_visit, a depth-first search
   * expands through vertices of @p section_type and collects all incident
   * unbreakable edges into a new section appended to @p ret_val.
   *
   * @param ret_val          Result container; each new section is appended as
   *                         an `index_set`.
   * @param vertices_to_visit Set of vertices to use as DFS seeds; visited
   *                          vertices are erased in place.
   * @param section_type     The vertex type that defines inner section
   *                         vertices.
   * @param error_types      Vertex types that must not appear adjacent to a
   *                         section vertex; their presence triggers a
   *                         `ConsistencyException`.
   * @throws cda_rail::exceptions::ConsistencyException If a vertex of an
   *         error type or a breakable edge is encountered inside a section.
   */
  void
  dfs_inplace(std::vector<cda_rail::index_set>&     ret_val,
              std::unordered_set<size_t>&           vertices_to_visit,
              const VertexType&                     section_type,
              const std::unordered_set<VertexType>& error_types = {}) const;
  /**
   * @brief Recursive implementation underlying all `all_paths_of_length_*`
   *        public methods.
   *
   * Finds all routes from a specified starting point in the given direction
   * whose total length is at least @p desired_length (removing the last edge
   * would make the route too short).
   *
   * @param v_0                   Optional starting vertex index; must be
   *                              empty when @p e_0 is set.
   * @param e_0                   Optional starting edge index; must be empty
   *                              when @p v_0 is set.
   * @param desired_length        Required minimum path length in metres.
   * @param reverse_direction     If `true`, paths are explored in reverse
   *                              (predecessor) direction.
   * @param exit_node             Optional vertex at which a path may
   *                              terminate early.
   * @param edges_used_by_train   If non-empty, restricts the traversable
   *                              edges.
   * @param return_successors_if_zero If `true` and @p desired_length is `0`
   *                                  and @p v_0 is set, returns single-edge
   *                                  paths for every (in/out) edge.
   * @return Vector of edge-index vectors representing found paths.
   * @throws cda_rail::exceptions::InvalidInputException If both or neither of
   *         @p v_0 / @p e_0 are set, or @p desired_length is not strictly
   *         positive (unless `return_successors_if_zero` applies).
   */
  [[nodiscard]] std::vector<cda_rail::index_vector>
  all_routes_of_given_length(std::optional<size_t> v_0,
                             std::optional<size_t> e_0, double desired_length,
                             bool                  reverse_direction,
                             std::optional<size_t> exit_node           = {},
                             cda_rail::index_set   edges_used_by_train = {},
                             bool return_successors_if_zero = false) const;

  /**
   * @brief Recursive implementation underlying the public
   *        `all_paths_ending_at_ttd`.
   *
   * Finds all paths starting at edge @p e_0 and ending upon entering a TTD
   * section other than @p safe_ttd.
   *
   * @param e_0          Index of the current edge.
   * @param ttd_sections Vector of TTD sections (each a set of edge indices).
   * @param exit_node    Optional vertex at which paths may terminate.
   * @param safe_ttd     Optional index of the TTD section that @p e_0
   *                     currently belongs to (not treated as a terminating
   *                     section).
   * @param first_edge   `true` on the initial call; `false` in recursive
   *                     calls.
   * @return Vector of edge-index vectors representing found paths.
   * @throws cda_rail::exceptions::EdgeNotExistentException If @p e_0 does
   *         not exist.
   */
  [[nodiscard]] std::vector<cda_rail::index_vector>
  all_paths_ending_at_ttd_recursive_helper(
      size_t e_0, const std::vector<cda_rail::index_set>& ttd_sections,
      std::optional<size_t> exit_node, std::optional<size_t> safe_ttd,
      bool first_edge) const;
};
} // namespace cda_rail
