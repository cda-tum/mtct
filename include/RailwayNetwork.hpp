#pragma once
#include <string>
#include <vector>

namespace cda_rail {
    struct Vertex {
        /**
         * Vertex object
         * @param name Name of the vertex
         * @param type Type of the vertex (NO_BORDER, VSS, TTD)
         * @param successor_tuples Vector of successor tuples related to this vertex. Maybe not here?
         */

        std::string name;
        int type;
        //std::vector<MovementTuple> successor_tuples;
    };

    struct Edge {
        /**
         * Edge object
         * @param source Source vertex
         * @param target Target vertex
         * @param length Length of vertex (in m)
         * @param max_speed Speed limit of vertex (in m/s)
         * @param breakable Boolean indicating if VSS can be placed on this edge
         * @param min_block_length Minimum block length (in m). Optional, default 0.
         */

        Vertex source, target;
        double length, max_speed;
        bool breakable;
        double min_block_length = 0;
    };

    struct Network {
        /**
         * Graph object
         * @param vertices Vector of vertices
         * @param edges Vector of edges
         */

        std::vector<Vertex> vertices;
        std::vector<Edge> edges;
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
