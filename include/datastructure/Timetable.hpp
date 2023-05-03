#pragma once
#include <string>
#include <unordered_map>
#include <vector>
#include "datastructure/Train.hpp"
#include "datastructure/RailwayNetwork.hpp"
#include "datastructure/Station.hpp"
#include <filesystem>
#include <string>

namespace cda_rail {
    struct ScheduledStop {
        /**
         * A scheduled stop.
         */
        int begin;
        int end;
        std::string station;

        bool operator<(const ScheduledStop& other) const {
            return (end < other.begin);
        }
        bool operator>(const ScheduledStop& other) const {
            return (begin > other.end);
        }
        bool operator==(const ScheduledStop& other) const {
            return (begin == other.begin && end == other.end);
        }
        bool operator<=(const ScheduledStop& other) const {
            return *this < other || *this == other;
        }
        bool operator>=(const ScheduledStop& other) const {
            return *this > other || *this == other;
        }
        bool operator!=(const ScheduledStop& other) const {
            return !(*this == other);
        }

        // Constructor
        ScheduledStop() = default;
        ScheduledStop(int begin, int end, const std::string& station) : begin(begin), end(end), station(station) {}
    };

    struct Schedule {
        /**
         * Schedule object
         * @param t_0 start time of schedule in seconds
         * @param v_0 initial velocity in m/s
         * @param entry entry vertex index of the schedule
         * @param t_n end time of schedule in seconds
         * @param v_n target end velocity in m/s
         * @param exit exit vertex index of the schedule
         * @param stops vector of scheduled stops
         *
         * For stops in stations he has to occupy the station for the entire interval.
         */
        int t_0;
        double v_0;
        int entry;
        int t_n;
        double v_n;
        int exit;
        std::vector<ScheduledStop> stops = {};

        // Constructor
        Schedule() = default;
        Schedule(int t_0, double v_0, int entry, int t_n, double v_n, int exit, const std::vector<ScheduledStop> stops = {}) :
            t_0(t_0), v_0(v_0), entry(entry), t_n(t_n), v_n(v_n), exit(exit), stops(stops) {}
    };

    class Timetable {
        /**
         * Timetable class
         */
        private:
            cda_rail::StationList station_list;
            cda_rail::TrainList train_list;
            std::vector<Schedule> schedules;

            void set_train_list(const cda_rail::TrainList& tl);

        public:
            // Constructors
            Timetable() = default;
            Timetable(const std::filesystem::path& p, const cda_rail::Network& network);
            Timetable(const std::string& path, const cda_rail::Network& network) : Timetable(std::filesystem::path(path), network) {};
            Timetable(const char* path, const cda_rail::Network& network) : Timetable(std::filesystem::path(path), network) {};

            // Rule of 5
            Timetable(const Timetable& other) = default;
            Timetable(Timetable&& other) noexcept = default;
            Timetable& operator=(const Timetable& other) = default;
            Timetable& operator=(Timetable&& other) noexcept = default;
            ~Timetable() = default;

            int add_train(const std::string& name, int length, double max_speed, double acceleration, double deceleration,
                           int t_0, double v_0, int entry, int t_n, double v_n, int exit, const cda_rail::Network& network);
            int add_train(const std::string& name, int length, double max_speed, double acceleration, double deceleration,
                           int t_0, double v_0, const std::string& entry, int t_n, double v_n, const std::string& exit,
                           const cda_rail::Network& network) {
                return add_train(name, length, max_speed, acceleration, deceleration, t_0, v_0, network.get_vertex_index(entry),
                                 t_n, v_n,network.get_vertex_index(exit), network);
            };

            void add_station(const std::string& name) {station_list.add_station(name);};

            void add_track_to_station(const std::string& name, int track, const cda_rail::Network& network) {station_list.add_track_to_station(name, track, network);};
            void add_track_to_station(const std::string& name, int source, int target, const cda_rail::Network& network) {station_list.add_track_to_station(name, source, target, network);};
            void add_track_to_station(const std::string& name, const std::string& source, const std::string& target, const cda_rail::Network& network) {station_list.add_track_to_station(name, source, target, network);};

            void add_stop(int train_index, const std::string& station_name, int begin, int end, bool sort = true);
            void add_stop(const std::string& train_name, const std::string& station_name, int begin, int end, bool sort = true) {
                add_stop(train_list.get_train_index(train_name), station_name, begin, end, sort);
            };

            [[nodiscard]] const cda_rail::StationList& get_station_list() const {return station_list;};
            [[nodiscard]] const cda_rail::TrainList& get_train_list() const {return train_list;};
            [[nodiscard]] const Schedule& get_schedule(int index) const;
            [[nodiscard]] const Schedule& get_schedule(const std::string& train_name) const {return get_schedule(train_list.get_train_index(train_name));};

            void sort_stops();

            [[nodiscard]] bool check_consistency(const cda_rail::Network& network) const;

            void export_timetable(const std::string& path, const cda_rail::Network& network) const {export_timetable(std::filesystem::path(path), network);};
            void export_timetable(const char* path, const cda_rail::Network& network) const {export_timetable(std::filesystem::path(path), network);};
            void export_timetable(const std::filesystem::path& p, const cda_rail::Network& network) const;
            [[nodiscard]] static cda_rail::Timetable import_timetable(const std::string& path, const cda_rail::Network& network) {return Timetable(path, network);};
            [[nodiscard]] static cda_rail::Timetable import_timetable(const std::filesystem::path& p, const cda_rail::Network& network) {return Timetable(p, network);};
            [[nodiscard]] static cda_rail::Timetable import_timetable(const char* path, const cda_rail::Network& network) {return Timetable(path, network);};

    };
}