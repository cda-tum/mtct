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
            void push_back_edge(int source, int target, cda_rail::Network);
            void push_back_edge(const std::string& source, const std::string& target, cda_rail::Network);

            void push_front_edge(int edge_index, cda_rail::Network network);
            void push_front_edge(int source, int target, cda_rail::Network);
            void push_front_edge(const std::string& source, const std::string& target, cda_rail::Network);

            void remove_edge(int edge_index, cda_rail::Network network);
            void remove_edge(int source, int target, cda_rail::Network);
            void remove_edge(const std::string& source, const std::string& target, cda_rail::Network);

            void export_route(const std::filesystem::path& p, cda_rail::Network) const;
            void export_route(const std::string& path, cda_rail::Network) const;
            void export_route(const char* path, cda_rail::Network) const;

            void import_route(const std::filesystem::path& p, cda_rail::Network);
            void import_route(const std::string& path, cda_rail::Network);
            void import_route(const char* path, cda_rail::Network);
    };
}