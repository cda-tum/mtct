#include "datastructure/Route.hpp"
#include "datastructure/RailwayNetwork.hpp"

void cda_rail::Route::push_back_edge(int edge_index, cda_rail::Network network) {
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
    if (edges.size() > 0 && !network.is_valid_successor(edges.back(), edge_index)) {
        throw std::out_of_range("Edge is not a valid successor.");
    }
    edges.push_back(edge_index);
}

void cda_rail::Route::push_back_edge(int source, int target, cda_rail::Network network) {
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

void cda_rail::Route::push_back_edge(const std::string &source, const std::string &target, cda_rail::Network network) {
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

void cda_rail::Route::push_front_edge(int edge_index, cda_rail::Network network) {
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
    if (edges.size() > 0 && !network.is_valid_successor(edge_index, edges.front())) {
        throw std::out_of_range("Edge is not a valid predecessor.");
    }
    edges.insert(edges.begin(), edge_index);
}

void cda_rail::Route::push_front_edge(int source, int target, cda_rail::Network network) {
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

void cda_rail::Route::push_front_edge(const std::string &source, const std::string &target, cda_rail::Network network) {
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

    if (edges.size() == 0) {
        throw std::out_of_range("Route is empty.");
    }
    edges.erase(edges.begin());
}

void cda_rail::Route::remove_last_edge() {
    /**
     * Removes the last edge from the route.
     * Throws an error if the route is empty.
     */

    if (edges.size() == 0) {
        throw std::out_of_range("Route is empty.");
    }
    edges.pop_back();
}
