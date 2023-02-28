#pragma once
#include <string>

namespace cda_rail {
    struct Vertex {
        /**
         * Vertex object
         * @param name Name of the vertex
         * @param type Type of the vertex (NO_BORDER, VSS, TTD)
         */

        string name;
        int type;
    };

    struct Edge {
        /**
         * Edge object
         * @param source Pointer to source vertex
         * @param target Pointer to target vertex
         * @param length Length of vertex (in m)
         * @param max_speed Speed limit of vertex (in m/s)
         * @param breakable Boolean indicating if VSS can be placed on this edge
         * @param min_dist_vss Minimum distance between VSS (in m). Optional, default 0.
         */

        Vertex *source, *target;
        double length, max_speed;
        bool breakable;
        double min_dist_vss = 0;
    };
}
