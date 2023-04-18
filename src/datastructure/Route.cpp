#include "datastructure/Route.hpp"
#include "datastructure/RailwayNetwork.hpp"

void cda_rail::Route::push_back_edge(int edge_index, const cda_rail::Network& network) {
    /**
     * Adds the edge to the end of the route.
     * Throws an error if the edge does not exist in the network or is not a valid successor of the last edge.
     *
     * @param edge_index The index of the edge to add.
     * @param network The network to which the edge belongs.
     */

    if (!network.has_edge(edge_index)) {
        throw std::out_of_range("Edge does not exist.");
    }
    if (!edges.empty() && !network.is_valid_successor(edges.back(), edge_index)) {
        throw std::out_of_range("Edge is not a valid successor.");
    }
    edges.push_back(edge_index);
}

void cda_rail::Route::push_back_edge(int source, int target, const cda_rail::Network& network) {
    /**
     * Adds the edge to the end of the route.
     * Throws an error if the edge does not exist in the network or is not a valid successor of the last edge.
     *
     * @param source The source of the edge to add.
     * @param target The target of the edge to add.
     * @param network The network to which the edge belongs.
     */

    if (!network.has_edge(source, target)) {
        throw std::out_of_range("Edge does not exist.");
    }
    push_back_edge(network.get_edge_index(source, target), network);
}

void cda_rail::Route::push_back_edge(const std::string &source, const std::string &target, const cda_rail::Network& network) {
    /**
     * Adds the edge to the end of the route.
     * Throws an error if the edge does not exist in the network or is not a valid successor of the last edge.
     *
     * @param source The source of the edge to add.
     * @param target The target of the edge to add.
     * @param network The network to which the edge belongs.
     */

    if (!network.has_edge(source, target)) {
        throw std::out_of_range("Edge does not exist.");
    }
    push_back_edge(network.get_edge_index(source, target), network);
}

void cda_rail::Route::push_front_edge(int edge_index, const cda_rail::Network& network) {
    /**
     * Adds the edge to the beginning of the route.
     * Throws an error if the edge does not exist in the network or is not a valid predecessor of the first edge.
     *
     * @param edge_index The index of the edge to add.
     * @param network The network to which the edge belongs.
     */

    if (!network.has_edge(edge_index)) {
        throw std::out_of_range("Edge does not exist.");
    }
    if (!edges.empty() && !network.is_valid_successor(edge_index, edges.front())) {
        throw std::out_of_range("Edge is not a valid predecessor.");
    }
    edges.insert(edges.begin(), edge_index);
}

void cda_rail::Route::push_front_edge(int source, int target, const cda_rail::Network& network) {
    /**
     * Adds the edge to the beginning of the route.
     * Throws an error if the edge does not exist in the network or is not a valid predecessor of the first edge.
     *
     * @param source The source of the edge to add.
     * @param target The target of the edge to add.
     * @param network The network to which the edge belongs.
     */

    if (!network.has_edge(source, target)) {
        throw std::out_of_range("Edge does not exist.");
    }
    push_front_edge(network.get_edge_index(source, target), network);
}

void cda_rail::Route::push_front_edge(const std::string &source, const std::string &target, const cda_rail::Network& network) {
    /**
     * Adds the edge to the beginning of the route.
     * Throws an error if the edge does not exist in the network or is not a valid predecessor of the first edge.
     *
     * @param source The source of the edge to add.
     * @param target The target of the edge to add.
     * @param network The network to which the edge belongs.
     */

    if (!network.has_edge(source, target)) {
        throw std::out_of_range("Edge does not exist.");
    }
    push_front_edge(network.get_edge_index(source, target), network);
}

void cda_rail::Route::remove_first_edge() {
    /**
     * Removes the first edge from the route.
     * Throws an error if the route is empty.
     */

    if (edges.empty()) {
        throw std::out_of_range("Route is empty.");
    }
    edges.erase(edges.begin());
}

void cda_rail::Route::remove_last_edge() {
    /**
     * Removes the last edge from the route.
     * Throws an error if the route is empty.
     */

    if (edges.empty()) {
        throw std::out_of_range("Route is empty.");
    }
    edges.pop_back();
}

int cda_rail::Route::get_edge(int route_index) const {
    /**
     * Returns the edge at the given index.
     * Throws an error if the index is out of range.
     *
     * @param route_index The index of the edge to return.
     * @return The edge at the given index.
     */

    if (route_index < 0 || route_index >= edges.size()) {
        throw std::out_of_range("Index out of range.");
    }
    return edges[route_index];
}

const cda_rail::Edge &cda_rail::Route::get_edge(int route_index, const cda_rail::Network& network) const {
    /**
     * Returns the edge at the given index.
     * Throws an error if the index is out of range.
     *
     * @param route_index The index of the edge to return.
     * @param network The network to which the edge belongs.
     * @return The edge at the given index.
     */

    if (route_index < 0 || route_index >= edges.size()) {
        throw std::out_of_range("Index out of range.");
    }
    return network.get_edge(edges[route_index]);
}

int cda_rail::Route::size() const {
    /**
     * Returns the number of edges in the route.
     *
     * @return The number of edges in the route.
     */

    return edges.size();
}

bool cda_rail::Route::assert(const cda_rail::Network& network) const {
    /**
     * Asserts if the route is valid for the given network.
     * A route is valid if all edges exist in the network and if all edges are valid successors of the previous edge.
     * An empty route is valid.
     * Returns true if the route is valid, false otherwise.
     * Throws an error if an edge does not exist in the network.
     *
     * @param network The network to which the route belongs.
     * @return True if the route is valid, false otherwise.
     */

    if (edges.empty()) {
        return true;
    }
    for (int i = 0; i < edges.size() - 1; i++) {
        if (!network.is_valid_successor(edges[i], edges[i + 1])) {
            return false;
        }
    }
}

void cda_rail::RouteMap::add_empty_route(const std::string &train_name) {
    /**
     * Adds an empty route for the given train.
     * Throws an error if the train already has a route.
     *
     * @param train_name The name of the train.
     */

    if (routes.find(train_name) != routes.end()) {
        throw std::out_of_range("Train already has a route.");
    }
    routes[train_name] = Route();
}

void cda_rail::RouteMap::add_empty_route(const std::string &train_name, const cda_rail::TrainList &trains) {
    /**
     * Adds an empty route for the given train.
     * Throws an error if the train already has a route.
     * Throws an error if the train does not exist.
     *
     * @param train_name The name of the train.
     * @param trains The list of trains.
     */

    if (!trains.has_train(train_name)) {
        throw std::out_of_range("Train does not exist.");
    }
    add_empty_route(train_name);
}

void cda_rail::RouteMap::remove_first_edge(const std::string &train_name) {
    /**
     * Removes the first edge from the route of the given train.
     * Throws an error if the train does not have a route.
     *
     * @param train_name The name of the train.
     */

    if (routes.find(train_name) == routes.end()) {
        throw std::out_of_range("Train does not have a route.");
    }
    routes[train_name].remove_first_edge();
}

void cda_rail::RouteMap::remove_last_edge(const std::string &train_name) {
    /**
     * Removes the last edge from the route of the given train.
     * Throws an error if the train does not have a route.
     *
     * @param train_name The name of the train.
     */

    if (routes.find(train_name) == routes.end()) {
        throw std::out_of_range("Train does not have a route.");
    }
    routes[train_name].remove_last_edge();
}

const cda_rail::Route &cda_rail::RouteMap::get_route(const std::string &train_name) const {
    /**
     * Returns the route of the given train.
     * Throws an error if the train does not have a route.
     *
     * @param train_name The name of the train.
     */

    if (routes.find(train_name) == routes.end()) {
        throw std::out_of_range("Train does not have a route.");
    }
    return routes.at(train_name);
}

bool cda_rail::RouteMap::assert(const cda_rail::TrainList &trains, const cda_rail::Network &network,
                                bool every_train_must_have_route) const {
    /**
     * Asserts if the route map is valid for the given trains and network.
     * A route map is valid if all routes are valid for the given network and if all trains with routes exist.
     * If every_train_must_have_route is true, then the route map is only valid if all trains have routes.
     *
     * @param trains The list of trains.
     * @param network The network to which the routes belong.
     * @param every_train_must_have_route If true, then the route map is only valid if all trains have routes.
     * @return True if the route map is valid, false otherwise.
     */

    // If every train must have a route the sizes must be equal.
    if (every_train_must_have_route && routes.size() != trains.size()) {
        return false;
    }

    // Iterate through all elements in the route map.
    for (auto& [name, route] : routes) {
        // If the train does not exist, then the route map is not valid.
        if (!trains.has_train(name)) {
            return false;
        }

        // If the route is not valid, then the route map is not valid.
        if (!route.assert(network)) {
            return false;
        }
    }

    // All checks passed
    return true;
}

template<typename... Ts>
void cda_rail::RouteMap::push_front_edge(const std::string &train_name, Ts &&... args) {
    /**
     * Adds the edge to the beginning of the route of the given train.
     * Throws an error if the train does not have a route.
     *
     * @param train_name The name of the train.
     * @param args The arguments to pass to the push_front_edge method of the Route class.
     */

    if (routes.find(train_name) == routes.end()) {
        throw std::out_of_range("Train does not have a route.");
    }
    routes[train_name].push_front_edge(std::forward<Ts>(args)...);
}

template<typename... Ts>
void cda_rail::RouteMap::push_back_edge(const std::string &train_name, Ts &&... args) {
    /**
     * Adds the edge to the end of the route of the given train.
     * Throws an error if the train does not have a route.
     *
     * @param train_name The name of the train.
     * @param args The arguments to pass to the push_back_edge method of the Route class.
     */

    if (routes.find(train_name) == routes.end()) {
        throw std::out_of_range("Train does not have a route.");
    }
    routes[train_name].push_back_edge(std::forward<Ts>(args)...);
}

