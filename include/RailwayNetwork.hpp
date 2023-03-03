#pragma once
#include <string>
#include <vector>

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

    struct Network {
        /**
         * Graph object
         * @param vertices Vector of vertices
         * @param edges Vector of edges
         */

        std::vector<Vertex> vertices;
        std::vector<Edge> edges;
        // TODO: Might change to unordered_map
    };

    struct SuccessorTuple {
        /**
         * MovementTuple object
         * @param source Source edge
         * @param target Target edge
         */

        Edge source, target;
    };
}
