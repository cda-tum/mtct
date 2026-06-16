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
public:
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
   * @brief Gets the number of edges in the route.
   * @return The number of edges stored in the route.
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
   * @brief Checks whether the route is empty.
   * @return `true` if the route has no edges, `false` otherwise.
   */
  ``` [[nodiscard]] bool empty() const { return m_edges.empty(); };

  /**
   * @brief Provides the edges forming the route in travel order.
   * @return A const reference to the route's edge indices.
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
   * @brief Returns the first edge in the route.
   *
   * @param network The network containing edge information.
   * @return const Edge& A reference to the first edge.
   *
   * @throws InvalidInputException If the route is empty.
   * @throws EdgeNotExistentException If the first edge's id does not exist in
   * the network.
   */
  [[nodiscard]] const Edge& get_first_edge(const Network& network) const {
    return get_edge(0, network);
  }
  /**
   * @brief Returns the last edge in the route.
   *
   * @return const Edge& The last edge in the route.
   *
   * @throw InvalidInputException If the route is empty.
   * @throw EdgeNotExistentException If the last edge id is not in the network.
   */
  [[nodiscard]] const Edge& get_last_edge(const Network& network) const {
    return get_edge(size() - 1, network);
  }

  /**
   * @brief Checks whether an edge id appears in this route.
   * @param edgeIndex The edge id to search for.
   * @return `true` if the edge is part of this route, `false` otherwise.
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
   * @brief Determines if an optional edge id is present in this route.
   *
   * @return `true` if edgeIndex holds a value and that edge id is in the route,
   *         `false` otherwise.
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

/**
 * @brief Collection of named train routes built on top of a railway network.
 *
 * A `RouteMap` maps each train (identified by its name string) to its `Route`.
 * It supports building routes edge by edge, querying pairwise track conflicts
 * between two trains, and serialising the whole collection to or from a JSON
 * file on disk.
 */
class RouteMap {
private:
  std::unordered_map<std::string, Route> m_routes;

public:
  // ----------------
  // CONSTRUCTORS
  // ----------------

  /**
   * @brief Creates an empty route map with no routes.
   */
  RouteMap() = default;

  /**
   * @brief Loads all routes from a `routes.json` file inside directory @p p.
   *
   * Every route entry in the file is validated against @p network as edges are
   * added: successor constraints are enforced and referenced vertices must
   * exist.
   *
   * @param p       Path to the directory that contains `routes.json`.
   * @param network Network used to resolve and validate each edge.
   * @throws cda_rail::exceptions::ImportException If @p p does not exist or is
   *         not a directory.
   * @throws cda_rail::exceptions::EdgeNotExistentException If the file
   *         references a vertex name that does not exist in @p network.
   * @throws cda_rail::exceptions::ConsistencyException If any loaded edge is
   *         not a valid successor of the previous edge on that route.
   */
  RouteMap(const std::filesystem::path& p, const Network& network);

  /**
   * @brief Convenience overload accepting a `std::string` path.
   * @param path    Path string pointing to the directory with `routes.json`.
   * @param network Network used to validate the loaded routes.
   * @see RouteMap(const std::filesystem::path&, const Network&)
   */
  RouteMap(const std::string& path, const Network& network)
      : RouteMap(std::filesystem::path(path), network) {};

  /**
   * @brief Convenience overload accepting a C-string path.
   * @param path    Null-terminated path string.
   * @param network Network used to validate the loaded routes.
   * @see RouteMap(const std::filesystem::path&, const Network&)
   */
  RouteMap(const char* path, const Network& network)
      : RouteMap(std::filesystem::path(path), network) {};

  // Rule of 5
  RouteMap(const RouteMap& other)            = default;
  RouteMap(RouteMap&& other)                 = default;
  RouteMap& operator=(const RouteMap& other) = default;
  /**
   * @brief Move-assigns another RouteMap to this one.
   *
   * @param other The RouteMap to move from.
   * @return Reference to this RouteMap.
   */
  RouteMap& operator=(RouteMap&& other) = default;
  ~RouteMap()                           = default;

  // ----------------
  // ITERATORS
  // ----------------

  /** @brief Read-only iterator to the first route entry. */
  [[nodiscard]] auto begin() const { return m_routes.cbegin(); };
  /** @brief Read-only iterator past the last route entry. */
  [[nodiscard]] auto end() const { return m_routes.cend(); };
  /**
   * @brief Returns a const iterator to the beginning of the routes.
   */
  [[nodiscard]] auto cbegin() const { return m_routes.cbegin(); };
  /** @brief Read-only iterator past the last route entry (explicit const). */
  [[nodiscard]] auto cend() const { return m_routes.cend(); };

  // ----------------
  // GETTER
  // ----------------

  /**
   * @brief Counts the number of stored train routes.
   * @return size_t The number of stored train routes.
   */
  [[nodiscard]] size_t size() const { return m_routes.size(); };

  /**
   * @brief Determines whether the map contains no routes.
   * @return `true` if no routes are stored, `false` otherwise.
   */
  [[nodiscard]] bool empty() const { return m_routes.empty(); };

  /**
   * @brief Returns the route assigned to the given train.
   * @param train_name Name of the train whose route is requested.
   * @return Const reference to the train's `Route`.
   * @pre A route for @p train_name must exist in this map.
   * @throws cda_rail::exceptions::ConsistencyException If no route for
   *         @p train_name has been registered.
   */
  [[nodiscard]] const Route& get_route(const std::string& train_name) const;

  /**
   * @brief Returns the total length of the route assigned to the given train.
   * @param train_name Name of the train.
   * @param network    Network used to read edge lengths.
   * @return Sum of all route edge lengths in metres.
   * @pre A route for @p train_name must exist in this map.
   * @throws cda_rail::exceptions::ConsistencyException If no route for
   *         @p train_name has been registered.
   * @throws cda_rail::exceptions::EdgeNotExistentException If the route
   *         references an edge that does not exist in @p network.
   */
  [[nodiscard]] double route_length(const std::string& train_name,
                                    const Network&     network) const;

  /**
   * @brief Checks whether a route for the given train is stored in this map.
   * @param train_name Name of the train to look up.
   * @return `true` if a route for @p train_name exists, otherwise `false`.
   */
  [[nodiscard]] bool has_route(const std::string& train_name) const {
    return m_routes.contains(train_name);
  };

  // ----------------
  // OVERLAP FUNCTIONS
  // ----------------

  /**
   * @brief Finds segments where both trains travel in the same direction over
   *        the same track section.
   *
   * Two trains are in a *parallel overlap* whenever consecutive edges on their
   * respective routes are identical (same edge ids in the same order). Each
   * returned `ConflictPair` represents one maximal such shared segment and
   * gives the start/end positions (in metres from each route's beginning) and
   * the set of underlying track ids.
   *
   * @param train1  Name of the first train.
   * @param train2  Name of the second train.
   * @param network Network used to look up edge lengths and track ids.
   * @return List of parallel conflict segments; empty if the routes do not
   *         share any edges in the same direction.
   * @pre Routes for both trains must exist in this map.
   * @throws cda_rail::exceptions::ConsistencyException If either train has no
   *         registered route.
   */
  [[nodiscard]] std::vector<ConflictPair>
  get_parallel_overlaps(const std::string& train1, const std::string& train2,
                        const Network& network) const;

  /**
   * @brief Finds segments where both trains share an unbreakable (TTD) track
   *        section.
   *
   * A *TTD overlap* occurs when both routes use at least one edge that belongs
   * to the same unbreakable section (a section where no VSS border may be
   * placed). Each returned `ConflictPair` covers the full extent of that
   * section on both routes and includes the involved track ids.
   * Only non-breakable edges on `train1`'s route trigger a check.
   *
   * @param train1  Name of the first train.
   * @param train2  Name of the second train.
   * @param network Network used to identify unbreakable sections and edge
   *                lengths.
   * @return List of TTD conflict segments; empty if no unbreakable section is
   *         shared by both routes.
   * @pre Routes for both trains must exist in this map.
   * @throws cda_rail::exceptions::ConsistencyException If either train has no
   *         registered route.
   */
  [[nodiscard]] std::vector<ConflictPair>
  get_ttd_overlaps(const std::string& train1, const std::string& train2,
                   const Network& network) const;

  /**
   * @brief Finds segments where the two trains travel in opposite directions
   *        over the same physical track.
   *
   * A *reverse overlap* occurs when an edge on `train1`'s route has a reverse
   * counterpart (same physical track, opposite direction) on `train2`'s route.
   * Each returned `ConflictPair` covers the maximal consecutive stretch where
   * this head-on situation persists. Positions are measured from each route's
   * own start; note that for `train2` the interval is traversed in reverse, so
   * `pos2.first < pos2.second` still holds but the edges are visited in
   * decreasing order.
   *
   * @param train1  Name of the first train.
   * @param train2  Name of the second train.
   * @param network Network used to find reverse edge pairs and read edge
   *                lengths.
   * @return List of reverse conflict segments; empty if no head-on overlap
   *         exists.
   * @pre Routes for both trains must exist in this map.
   * @throws cda_rail::exceptions::ConsistencyException If either train has no
   *         registered route.
   */
  [[nodiscard]] std::vector<ConflictPair>
  get_reverse_overlaps(const std::string& train1, const std::string& train2,
                       const Network& network) const;

  /**
   * @brief Finds all segments where the two trains could potentially block each
   *        other, regardless of travel direction.
   *
   * Combines the results of `get_ttd_overlaps` and `get_reverse_overlaps`,
   * sorts the merged list by start position on `train1`'s route, and then
   * merges consecutive intervals that overlap on both routes into a single
   * `ConflictPair`. Use this function when you need a complete conflict picture
   * for scheduling or safety checks.
   *
   * @param train1  Name of the first train.
   * @param train2  Name of the second train.
   * @param network Network used for all sub-queries.
   * @return Sorted, de-duplicated list of conflict segments.
   * @pre Routes for both trains must exist in this map.
   * @throws cda_rail::exceptions::ConsistencyException If either train has no
   *         registered route.
   */
  [[nodiscard]] std::vector<ConflictPair>
  get_crossing_overlaps(const std::string& train1, const std::string& train2,
                        const Network& network) const;

  // ------------------
  // EDITING FUNCTIONS
  // ------------------

  /**
   * @brief Registers an empty route for the given train name.
   *
   * The route is initially empty (no edges). Edges can be added afterwards
   * with `push_back_edge` or `push_front_edge`.
   *
   * @param train_name Name to associate the new route with.
   * @throws cda_rail::exceptions::InvalidInputException If a route for
   *         @p train_name is already registered.
   */
  void add_empty_route(const std::string& train_name);

  /**
   * @brief Registers an empty route for a train, verifying it exists in the
   *        train list first.
   *
   * @param train_name Name of the train.
   * @param trains     Train list that @p train_name must be part of.
   * @throws cda_rail::exceptions::TrainNotExistentException If @p train_name
   *         is not present in @p trains.
   * @throws cda_rail::exceptions::InvalidInputException If a route for
   *         @p train_name is already registered.
   */
  void add_empty_route(const std::string& train_name, const TrainList& trains);

  /**
   * @brief Appends an edge at the end of the given train's route.
   * @param train_name Name of the train whose route is extended.
   * @param new_edge   Edge to append.
   * @param network    Network used to resolve and validate @p new_edge.
   * @pre A route for @p train_name must exist in this map.
   * @throws cda_rail::exceptions::ConsistencyException If no route for
   *         @p train_name has been registered or if @p new_edge is not a valid
   *         successor of the current last edge.
   * @throws cda_rail::exceptions::EdgeNotExistentException If @p new_edge does
   *         not exist in @p network.
   */
  void push_back_edge(const std::string&        train_name,
                      Network::EdgeInput const& new_edge,
                      const Network&            network);

  /**
   * @brief Prepends an edge at the beginning of the given train's route.
   * @param train_name Name of the train whose route is extended.
   * @param new_edge   Edge to prepend.
   * @param network    Network used to resolve and validate @p new_edge.
   * @pre A route for @p train_name must exist in this map.
   * @throws cda_rail::exceptions::ConsistencyException If no route for
   *         @p train_name has been registered or if @p new_edge is not a valid
   *         predecessor of the current first edge.
   * @throws cda_rail::exceptions::EdgeNotExistentException If @p new_edge does
   *         not exist in @p network.
   */
  void push_front_edge(const std::string&        train_name,
                       Network::EdgeInput const& new_edge,
                       const Network&            network);

  /**
   * @brief Removes the first edge from the given train's route.
   * @param train_name Name of the train whose route is shortened.
   * @pre A non-empty route for @p train_name must exist in this map.
   * @throws cda_rail::exceptions::ConsistencyException If no route for
   *         @p train_name has been registered or if the route is empty.
   */
  void remove_first_edge(const std::string& train_name);

  /**
   * @brief Removes the last edge from the given train's route.
   * @param train_name Name of the train whose route is shortened.
   * @pre A non-empty route for @p train_name must exist in this map.
   * @throws cda_rail::exceptions::ConsistencyException If no route for
   *         @p train_name has been registered or if the route is empty.
   */
  void remove_last_edge(const std::string& train_name);

  /**
   * @brief Removes the entire route for the given train from the map.
   * @param train_name Name of the train whose route should be deleted.
   * @pre A route for @p train_name must exist in this map.
   * @throws cda_rail::exceptions::ConsistencyException If no route for
   *         @p train_name has been registered.
   */
  void remove_route(const std::string& train_name);

  // ------------------
  // IMPORT / EXPORT
  // ------------------

  /**
   * @brief Writes all routes to a `routes.json` file inside directory @p p.
   *
   * Each route is stored as an ordered list of `[source_vertex_name,
   * target_vertex_name]` pairs. The directory is created if it does not yet
   * exist.
   *
   * @param p       Destination directory for the output file.
   * @param network Network used to look up vertex names for each edge.
   * @throws cda_rail::exceptions::ExportException If the directory cannot be
   *         created.
   */
  void export_routes(const std::filesystem::path& p,
                     const Network&               network) const;

  /**
   * @brief Writes all routes to routes.json in the specified directory.
   * @param path    Destination directory as a string.
   * @param network Network used to look up vertex names.
   */
  void export_routes(const std::string& path, const Network& network) const {
    export_routes(std::filesystem::path(path), network);
  };

  /**
   * @brief Exports routes to a directory specified by a C-string path.
   *
   * @param path    Null-terminated destination directory path.
   * @param network Network used to look up vertex names.
   */
  void export_routes(const char* path, const Network& network) const {
    export_routes(std::filesystem::path(path), network);
  };

  /**
   * @brief Loads routes from a directory into a new `RouteMap`.
   * @param p       Directory containing `routes.json`.
   * @param network Network used to validate the loaded routes.
   * @return RouteMap containing all routes loaded from the file.
   * @see RouteMap(const std::filesystem::path&, const Network&)
   */
  [[nodiscard]] static RouteMap import_routes(const std::filesystem::path& p,
                                              const Network& network) {
    return {p, network};
  };

  /**
   * @brief Static factory: convenience overload accepting a `std::string`
   *        path.
   * @param path    Directory path as a string.
   * @param network Network used to validate the loaded routes.
   * @see RouteMap(const std::string&, const Network&)
   */
  [[nodiscard]] static RouteMap import_routes(const std::string& path,
                                              const Network&     network) {
    return {path, network};
  };

  /**
   * @brief Loads train routes from a file at the given directory path.
   *
   * @param path    Directory path containing routes.json.
   * @param network Network used to validate referenced edges during loading.
   * @return RouteMap containing the loaded routes.
   *
   * @throws ImportException if the directory does not exist or is not a
   * directory.
   * @throws EdgeNotExistentException if the routes file references an unknown
   * vertex or edge.
   * @throws ConsistencyException if any route contains invalid successor
   * relations between consecutive edges.
   */
  [[nodiscard]] static RouteMap import_routes(const char*    path,
                                              const Network& network) {
    return {path, network};
  };

  // -----------------
  // HELPER
  // -----------------

  /**
   * @brief Checks whether all stored routes are valid with respect to the
   *        given train list and network.
   *
   * A `RouteMap` is considered consistent when:
   * - (if @p every_train_must_have_route is `true`) every train in @p trains
   *   has a registered route, and
   * - every registered route belongs to a known train and is internally
   *   connected (each consecutive edge pair satisfies the successor relation).
   *
   * @param trains                      List of all known trains.
   * @param network                     Network used for successor checks.
   * @param every_train_must_have_route If `true` (the default), returns
   *                                    `false` whenever the number of stored
   *                                    routes differs from the number of trains
   *                                    in @p trains.
   * @return `true` if all consistency conditions are met, otherwise `false`.
   */
  [[nodiscard]] bool
  check_consistency(const TrainList& trains, const Network& network,
                    bool every_train_must_have_route = true) const;

  /**
   * @brief Updates all routes after a network discretization step.
   *
   * For each `(old_edge, replacement_edges)` pair in @p new_edges, every
   * occurrence of `old_edge` in every stored route is replaced by the
   * corresponding sequence of replacement edges, preserving travel order.
   * Routes that do not contain any edge from @p new_edges are unchanged.
   *
   * @param new_edges Mapping from each replaced edge id to the ordered list of
   *                  edges that take its place.
   */
  void update_after_discretization(
      const std::vector<std::pair<size_t, cda_rail::index_vector>>& new_edges);

private:
  /**
   * @brief Throws if no route for @p train_name is registered in this map.
   * @param train_name Train name to look up.
   * @throws cda_rail::exceptions::ConsistencyException If the train has no
   *         route.
   */
  void throw_if_train_has_no_route(std::string const& train_name) const;
};

} // namespace cda_rail
