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
            // Network functions, i.e., network is accessible via n() as a reference
            [[nodiscard]] cda_rail::Network& n();

            // Timetable functions
            void add_train(const std::string& name, int length, double max_speed, double acceleration, double deceleration,
                           int t_0, double v_0, int entry, int t_n, double v_n, int exit);
            void add_train(const std::string& name, int length, double max_speed, double acceleration, double deceleration,
                           int t_0, double v_0, const std::string& entry, int t_n, double v_n, const std::string& exit);

            void add_station(const std::string& name, const std::unordered_set<int>& tracks);
            void add_station(const std::string& name);

            void add_track_to_station(int station_index, int track);
            void add_track_to_station(const std::string& name, int track);
            void add_track_to_station(int station_index, int source, int target);
            void add_track_to_station(const std::string& name, int source, int target);
            void add_track_to_station(int station_index, const std::string& source, const std::string& target);
            void add_track_to_station(const std::string& name, const std::string& source, const std::string& target);

            void add_stop(int train_index, int station_index, int begin, int end, bool sort = true);
            void add_stop(const std::string& train_name, int station_index, int begin, int end, bool sort = true);
            void add_stop(int train_index, const std::string& station_name, int begin, int end, bool sort = true);
            void add_stop(const std::string& train_name, const std::string& station_name, int begin, int end, bool sort = true);

            [[nodiscard]] int maxT() const;

            [[nodiscard]] const cda_rail::StationList& get_station_list() const;
            [[nodiscard]] const cda_rail::TrainList& get_train_list() const;
            [[nodiscard]] const Schedule& get_schedule(int index) const;
            [[nodiscard]] const Schedule& get_schedule(const std::string& train_name) const;

            void sort_stops();

            // RouteMap functions
            void add_empty_route(const std::string& train_name);

            void push_back_edge_to_route(const std::string& train_name, int edge_index);
            void push_back_edge_to_route(const std::string& train_name, int source, int target);
            void push_back_edge_to_route(const std::string& train_name, const std::string& source, const std::string& target);

            void push_front_edge_to_route(const std::string& train_name, int edge_index);
            void push_front_edge_to_route(const std::string& train_name, int source, int target);
            void push_front_edge_to_route(const std::string& train_name, const std::string& source, const std::string& target);

            void remove_first_edge_from_route(const std::string& train_name);
            void remove_last_edge_from_route(const std::string& train_name);

            [[nodiscard]] bool has_route(const std::string& train_name) const;
            [[nodiscard]] int route_map_size() const;
            [[nodiscard]] const Route& get_route(const std::string& train_name) const;

            // General Consistency Check
            [[nodiscard]] bool check_consistency(bool every_train_must_have_route = true) const;

            // Export and import functions
            void export_instance(const std::filesystem::path& p) const;
            void export_instance(const std::string& path) const;
            void export_instance(const char* path) const;

            [[nodiscard]] static VSSGenerationTimetable import_instance(const std::filesystem::path& p, bool every_train_must_have_route = true);
            [[nodiscard]] static VSSGenerationTimetable import_instance(const std::string& path, bool every_train_must_have_route = true);
            [[nodiscard]] static VSSGenerationTimetable import_instance(const char* path, bool every_train_must_have_route = true);
    };
}