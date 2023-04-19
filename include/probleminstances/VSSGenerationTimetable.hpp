#pragma once
#include "datastructure/RailwayNetwork.hpp"
#include "datastructure/Timetable.hpp"
#include "datastructure/Route.hpp"

namespace cda_rail::instances{
    class VSSGenerationTimetable {
        private:
            cda_rail::Network network;
            cda_rail::Timetable timetable;
            cda_rail::RouteMap routes;
        public:
            [[nodiscard]] cda_rail::Network& n();
            [[nodiscard]] cda_rail::Timetable& t();
            [[nodiscard]] cda_rail::RouteMap& r();

            void export_instance(const std::filesystem::path& p) const;
            void export_instance(const std::string& path) const;
            void export_instance(const char* path) const;

            [[nodiscard]] static VSSGenerationTimetable import_instance(const std::filesystem::path& p);
            [[nodiscard]] static VSSGenerationTimetable import_instance(const std::string& path);
            [[nodiscard]] static VSSGenerationTimetable import_instance(const char* path);
    };
}