#pragma once
#include <string>
#include <vector>
#include <unordered_map>

namespace cda_rail {
    struct Vertex {
        /**
         * Vertex object
         * @param name Name of the vertex
         * @param type Type of the vertex (NO_BORDER, VSS, TTD)
         */

        std::string name;
        int type;
    };

    class Edge {
        /**
         * Edge object
         * @param source Source vertex
         * @param target Target vertex
         * @param length Length of vertex (in m)
         * @param max_speed Speed limit of vertex (in m/s)
         * @param breakable Boolean indicating if VSS can be placed on this edge
         * @param min_block_length Minimum block length (in m). Optional, default 0.
         * @param successors Vector of successor edges
         */
        public:
            Vertex *source, *target;
            double length, max_speed;
            bool breakable;
            double min_block_length = 0;
            std::vector<Edge*> successors;

            std::string toString() const;
    };

    class Network {
        /**
         * Graph object
         * @param vertices Unordered map of vertices
         * @param edges Unordered map of edges
         * @param vertex_names Vector of vertex names
         * @param edge_names Vector of edge names
         */
        public:
            std::unordered_map<std::string, Vertex> vertices;
            std::unordered_map<std::string, Edge> edges;
            std::vector<std::string> vertex_names;
            std::vector<std::string> edge_names;

            static Network read_network(std::string path);
    };
}
