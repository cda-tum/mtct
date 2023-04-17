#pragma once
#include "datastructure/RailwayNetwork.hpp"
#include <vector>
#include <string>
#include <filesystem>

namespace cda_rail {
    class Route {
        private:
            std::vector<int> edges;
        public:
            void push_back_edge(int edge_index, cda_rail::Network network);
            void push_back_edge(int source, int target, cda_rail::Network network);
            void push_back_edge(const std::string& source, const std::string& target, cda_rail::Network network);

            void push_front_edge(int edge_index, cda_rail::Network network);
            void push_front_edge(int source, int target, cda_rail::Network network);
            void push_front_edge(const std::string& source, const std::string& target, cda_rail::Network network);

            void remove_first_edge();
            void remove_last_edge();
    };
}