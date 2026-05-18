#pragma once
#include "Definitions.hpp"
#include "datastructure/RailwayNetwork.hpp"
#include "datastructure/Train.hpp"

#include <algorithm>
#include <cstddef>
#include <filesystem>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace cda_rail {

/**
 * @brief One overlap interval between two routes.
 *
 * For each route, positions are measured in metres from that route's start.
 * The @p edges set stores the track ids involved in this overlap.
 */
struct ConflictPair {
  std::pair<double, double>  pos1;
  std::pair<double, double>  pos2;
  std::unordered_set<size_t> edges;
};

/**
 * @brief Ordered list of network edges that a train follows.
 *
 * For a non-empty route, each next edge should be reachable from the previous
 * one according to the network successor rules.
 */
class Route {
  /**
   * @brief Start and end position of one edge within the route.
   *
   * Both values are in metres from the route start: @p source is where the edge
   * begins and @p target is where it ends.
   */
  struct EdgePosition {
    double source{};
    double target{};
  };

private:
  cda_rail::index_vector m_edges;

  /**
   * @brief Internal helper for finding the first or last matching edge
   * position.
   * @param edge_indices Edge indices to search for in route order.
   * @param network      Network used to read edge lengths.
   * @param first_match  If `true`, returns the start of the first match;
   *                     otherwise returns the end of the last match.
   * @return Matching position in metres, or `std::nullopt` if no edge from
   *         @p edge_indices appears in the route.
   * @throws cda_rail::exceptions::EdgeNotExistentException If the route
   *         contains an edge that does not exist in @p network.
   */
  [[nodiscard]] std::optional<double>
  get_pos_on_edges_impl(const cda_rail::index_set& edge_indices,
                        const Network& network, bool first_match) const;

  /**
   * @brief Checks whether a route index is valid.
   * @param route_index Index into `m_edges`.
   * @pre `route_index < size()`.
   * @throws cda_rail::exceptions::InvalidInputException If @p route_index is
   *         out of range.
   */
  void is_route_index_valid(size_t route_index) const;

  /**
   * @brief Returns whether the route has at least one edge.
   * @return `true` if `size() > 0`, otherwise `false`.
   */
  [[nodiscard]] bool has_edges() const;

public:
  // -----------------------------
  // CONSTRUCTORS
  // -----------------------------
  // Only default constructors (empty)

  // -----------------------------
  // GETTER
  // -----------------------------

  /**
   * @brief Returns the number of edges stored in the route.
   * @return Route length in number of edges.
   */
  [[nodiscard]] size_t size() const { return m_edges.size(); };

  /**
   * @brief Returns the total route length.
   * @param network Network containing all edges referenced by this route.
   * @return Sum of all route edge lengths in metres.
   * @throws cda_rail::exceptions::EdgeNotExistentException If any referenced
   *         edge does not exist in @p network.
   */
  [[nodiscard]] double length(const Network& network) const;

  /**
   * @brief Returns whether the route contains no edges.
   * @return `true` if `size() == 0`, otherwise `false`.
   */
  [[nodiscard]] bool empty() const { return m_edges.empty(); };

  /**
   * @brief Returns the route as edge ids in travel order.
   * @return Const reference to the internal edge-index vector.
   */
  [[nodiscard]] const cda_rail::index_vector& get_edges() const {
    return m_edges;
  };

  /**
   * @brief Returns where a specific edge lies on this route.
   * @param edge    Edge to look up.
   * @param network Network used to resolve @p edge.
   * @return Position interval `{source, target}` in metres from route start.
   * @pre The resolved edge is contained in this route.
   * @throws cda_rail::exceptions::EdgeNotExistentException If @p edge does not
   *         exist in @p network.
   * @throws cda_rail::exceptions::ConsistencyException If the edge exists in
   *         @p network but is not part of this route.
   */
  [[nodiscard]] EdgePosition edge_pos_on_route(Network::EdgeInput const& edge,
                                               const Network& network) const;

  /**
   * @brief Returns one interval that covers all selected edges on the route.
   *
   * The function takes the earliest start and latest end among all route edges
   * whose id is listed in @p edges_to_consider.
   *
   * @param edges_to_consider Edge ids to include.
   * @param network           Network used to read edge lengths.
   * @return Position interval `{source, target}` in metres from route start.
   * @pre At least one edge from @p edges_to_consider occurs in this route.
   * @throws cda_rail::exceptions::ConsistencyException If no edge from
   *         @p edges_to_consider exists in this route.
   * @throws cda_rail::exceptions::EdgeNotExistentException If this route
   *         contains an edge that does not exist in @p network.
   */
  [[nodiscard]] EdgePosition
  edge_set_pos_on_route(const cda_rail::index_vector& edges_to_consider,
                        const Network&                network) const;

  /**
   * @brief Returns where the first matching edge starts.
   * @param edge_indices Edge ids to search for.
   * @param network      Network used to read edge lengths.
   * @return Position (metres from route start) at the beginning of the first
   *         route edge contained in @p edge_indices, or `std::nullopt` if no
   *         such edge exists.
   * @throws cda_rail::exceptions::EdgeNotExistentException If this route
   *         contains an edge that does not exist in @p network.
   */
  [[nodiscard]] std::optional<double>
  get_first_pos_on_edges(const cda_rail::index_set& edge_indices,
                         const Network&             network) const;

  /**
   * @brief Returns where the last matching edge ends.
   * @param edge_indices Edge ids to search for.
   * @param network      Network used to read edge lengths.
   * @return Position (metres from route start) at the end of the last route
   *         edge contained in @p edge_indices, or `std::nullopt` if no such
   *         edge exists.
   * @throws cda_rail::exceptions::EdgeNotExistentException If this route
   *         contains an edge that does not exist in @p network.
   */
  [[nodiscard]] std::optional<double>
  get_last_pos_on_edges(const cda_rail::index_set& edge_indices,
                        const Network&             network) const;

  /**
   * @brief Returns the edge that contains a given position on the route.
   * @param pos     Position in metres from route start.
   * @param network Network used to read edge lengths.
   * @return Network edge id of the edge that contains @p pos.
   * @pre `pos >= 0`.
   * @throws cda_rail::exceptions::InvalidInputException If @p pos is negative.
   * @throws cda_rail::exceptions::ConsistencyException If @p pos lies outside
   *         the route interval `[0, length(network)]`.
   * @throws cda_rail::exceptions::EdgeNotExistentException If this route
   *         contains an edge that does not exist in @p network.
   */
  [[nodiscard]] size_t get_edge_id_at_pos(double         pos,
                                          const Network& network) const;

  /**
   * @brief Returns the edge id at a given route position.
   * @param route_index Index into route order.
   * @return Edge id stored at @p route_index.
   * @pre `route_index < size()`.
   * @throws cda_rail::exceptions::InvalidInputException If @p route_index is
   *         out of range.
   */
  [[nodiscard]] size_t get_edge_id(size_t route_index) const;

  /**
   * @brief Returns the full edge object at a given route position.
   * @param route_index Index into route order.
   * @param network     Network containing all route edges.
   * @return Const reference to the referenced `Edge` object.
   * @pre `route_index < size()`.
   * @throws cda_rail::exceptions::InvalidInputException If @p route_index is
   *         out of range.
   * @throws cda_rail::exceptions::EdgeNotExistentException If the referenced
   *         edge id is not present in @p network.
   */
  [[nodiscard]] const Edge& get_edge(size_t         route_index,
                                     const Network& network) const;

  /**
   * @brief Checks whether an edge id appears in this route.
   * @param edgeIndex Edge id to search.
   * @return `true` if @p edgeIndex is part of this route.
   */
  [[nodiscard]] bool contains_edge(size_t const edgeIndex) const {
    return std::ranges::contains(m_edges, edgeIndex);
  };

  /**
   * @brief Checks whether a network edge appears in this route.
   * @param edge_input Edge input resolved via @p network.
   * @param network    Network used for descriptor resolution.
   * @return `true` if the resolved edge id is part of this route.
   * @throws cda_rail::exceptions::EdgeNotExistentException If @p edge_input
   *         cannot be resolved to an existing edge.
   * @throws cda_rail::exceptions::ConsistencyException If @p edge_input is an
   *         `Edge` object that mismatches the stored edge attributes.
   */
  [[nodiscard]] bool contains_edge(Network::EdgeInput const& edge_input,
                                   const Network&            network) const {
    return contains_edge(edge_input.resolve(&network));
  };

  /**
   * @brief Checks whether an optional edge id is present in this route.
   * @param edgeIndex Optional edge id.
   * @return `false` if @p edgeIndex is empty; otherwise equivalent to
   *         `contains_edge(edgeIndex.value())`.
   */
  [[nodiscard]] bool
  contains_edge(std::optional<size_t> const edgeIndex) const {
    return edgeIndex.has_value() && contains_edge(edgeIndex.value());
  }

  // -----------------------------
  // EDITING FUNCTIONS
  // -----------------------------

  /**
   * @brief Adds an edge at the end of the route.
   *
   * If the route is not empty, the new edge must be a valid next edge of the
   * current last edge.
   *
   * @param new_edge Edge descriptor to append.
   * @param network  Network used to resolve and validate @p new_edge.
   * @throws cda_rail::exceptions::EdgeNotExistentException If @p new_edge does
   *         not exist in @p network.
   * @throws cda_rail::exceptions::ConsistencyException If the route is
   *         non-empty and @p new_edge is not a valid successor of the current
   *         last edge.
   */
  void push_back_edge(Network::EdgeInput const& new_edge,
                      const Network&            network);

  /**
   * @brief Adds an edge at the beginning of the route.
   *
   * If the route is not empty, the new edge must be a valid predecessor of the
   * current first edge.
   *
   * @param new_edge Edge descriptor to prepend.
   * @param network  Network used to resolve and validate @p new_edge.
   * @throws cda_rail::exceptions::EdgeNotExistentException If @p new_edge does
   *         not exist in @p network.
   * @throws cda_rail::exceptions::ConsistencyException If the route is
   *         non-empty and @p new_edge is not a valid predecessor of the current
   *         first edge.
   */
  void push_front_edge(Network::EdgeInput const& new_edge,
                       const Network&            network);

  /**
   * @brief Removes the first edge from the route.
   * @pre `!empty()`.
   * @throws cda_rail::exceptions::ConsistencyException If the route is empty.
   */
  void remove_first_edge();

  /**
   * @brief Removes the last edge from the route.
   * @pre `!empty()`.
   * @throws cda_rail::exceptions::ConsistencyException If the route is empty.
   */
  void remove_last_edge();

  // -----------------------------
  // HELPER
  // -----------------------------

  /**
   * @brief Checks whether the route is connected step by step.
   *
   * The route is consistent when every edge can directly continue to the next
   * edge according to the network. Empty and single-edge routes are valid.
   *
   * @param network Network used for successor checks.
   * @return `true` if all consecutive pairs are valid successors.
   * @throws cda_rail::exceptions::EdgeNotExistentException If one of the
   *         checked route edges does not exist in @p network.
   */
  [[nodiscard]] bool check_consistency(const Network& network) const;

  /**
   * @brief Updates the route after edge splitting in the network.
   *
   * For each pair `(old_edge, replacement_edges)` in @p new_edges, every
   * occurrence of `old_edge` is replaced by all edges in
   * `replacement_edges`, keeping the original travel order.
   *
   * @param new_edges Mapping from original edge id to replacement edge
   * sequence.
   */
  void update_after_discretization(
      const std::vector<std::pair<size_t, cda_rail::index_vector>>& new_edges);
};

class RouteMap {
private:
  std::unordered_map<std::string, Route> routes;

public:
  // Constructors
  RouteMap() = default;
  RouteMap(const std::filesystem::path& p, const Network& network);
  RouteMap(const std::string& path, const Network& network)
      : RouteMap(std::filesystem::path(path), network) {};
  RouteMap(const char* path, const Network& network)
      : RouteMap(std::filesystem::path(path), network) {};

  // Rule of 5
  RouteMap(const RouteMap& other)            = default;
  RouteMap(RouteMap&& other)                 = default;
  RouteMap& operator=(const RouteMap& other) = default;
  RouteMap& operator=(RouteMap&& other)      = default;
  ~RouteMap()                                = default;

  // Iterators (for range-based for loops) that do not allow modification of the
  // underlying data
  [[nodiscard]] constexpr auto cbegin() const { return routes.cbegin(); };
  [[nodiscard]] constexpr auto cend() const { return routes.cend(); };

  // ----------------
  // GETTER
  // ----------------

  // Simple Getter

  [[nodiscard]] size_t       size() const { return routes.size(); };
  [[nodiscard]] bool         empty() const { return routes.empty(); };
  [[nodiscard]] const Route& get_route(const std::string& train_name) const;
  [[nodiscard]] double       route_length(const std::string& train_name,
                                          const Network&     network) const;
  [[nodiscard]] bool         has_route(const std::string& train_name) const {
    return routes.contains(train_name);
  };

  // Overlap functions
  [[nodiscard]] std::vector<ConflictPair>
  get_parallel_overlaps(const std::string& train1, const std::string& train2,
                        const Network& network) const;
  [[nodiscard]] std::vector<ConflictPair>
  get_ttd_overlaps(const std::string& train1, const std::string& train2,
                   const Network& network) const;
  [[nodiscard]] std::vector<ConflictPair>
  get_reverse_overlaps(const std::string& train1, const std::string& train2,
                       const Network& network) const;
  [[nodiscard]] std::vector<ConflictPair>
  get_crossing_overlaps(const std::string& train1, const std::string& train2,
                        const Network& network) const;

  // ------------------
  // EDITING FUNCTIONS
  // ------------------

  void add_empty_route(const std::string& train_name);
  void add_empty_route(const std::string& train_name, const TrainList& trains);

  void push_back_edge(const std::string&        train_name,
                      Network::EdgeInput const& new_edge,
                      const Network&            network);
  void push_front_edge(const std::string&        train_name,
                       Network::EdgeInput const& new_edge,
                       const Network&            network);

  void remove_first_edge(const std::string& train_name);
  void remove_last_edge(const std::string& train_name);
  void remove_route(const std::string& train_name);

  // ------------------
  // IMPORT / EXPORT
  // ------------------

  void export_routes(const std::filesystem::path& p,
                     const Network&               network) const;
  void export_routes(const std::string& path, const Network& network) const {
    export_routes(std::filesystem::path(path), network);
  };
  void export_routes(const char* path, const Network& network) const {
    export_routes(std::filesystem::path(path), network);
  };

  [[nodiscard]] static RouteMap import_routes(const std::filesystem::path& p,
                                              const Network& network) {
    return {p, network};
  };
  [[nodiscard]] static RouteMap import_routes(const std::string& path,
                                              const Network&     network) {
    return {path, network};
  };
  [[nodiscard]] static RouteMap import_routes(const char*    path,
                                              const Network& network) {
    return {path, network};
  };

  // -----------------
  // HELPER
  // -----------------

  [[nodiscard]] bool
  check_consistency(const TrainList& trains, const Network& network,
                    bool every_train_must_have_route = true) const;

  void update_after_discretization(
      const std::vector<std::pair<size_t, cda_rail::index_vector>>& new_edges);

private:
  void throw_if_train_has_no_route(std::string const& train_name) const;
};

} // namespace cda_rail
