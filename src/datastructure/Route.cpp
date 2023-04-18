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
