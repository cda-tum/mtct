#include "Timetable.hpp"
#include <filesystem>
#include <string>
#include "parsing/json.hpp"
#include <fstream>
#include <algorithm>

using json = nlohmann::json;

void cda_rail::Timetable::add_station(const std::string &name, const std::unordered_set<int> &tracks) {
    stations.push_back(cda_rail::Station{name, tracks});
    station_name_to_index[name] = stations.size() - 1;
}

void cda_rail::Timetable::add_station(const std::string &name) {
    stations.push_back(cda_rail::Station{name});
    station_name_to_index[name] = stations.size() - 1;
}

int cda_rail::Timetable::get_station_index(const std::string &name) const {
    if (!has_station(name)) {
        throw std::out_of_range("Station does not exist.");
    }
    return station_name_to_index.at(name);
}

const cda_rail::Station &cda_rail::Timetable::get_station(int index) const {
    if (!has_station(index)) {
        throw std::out_of_range("Station does not exist.");
    }
    return stations.at(index);
}

const cda_rail::Station &cda_rail::Timetable::get_station(const std::string &name) const {
    return get_station(get_station_index(name));
}

bool cda_rail::Timetable::has_station(const std::string &name) const {
    return station_name_to_index.find(name) != station_name_to_index.end();
}

bool cda_rail::Timetable::has_station(int index) const {
    return (index >= 0 && index < stations.size());
}

const cda_rail::Schedule &cda_rail::Timetable::get_schedule(int index) const {
    if (!train_list.has_train(index)) {
        throw std::out_of_range("Train does not exist.");
    }
    return schedules.at(index);
}

const cda_rail::Schedule &cda_rail::Timetable::get_schedule(const std::string &train_name) const {
    if (!train_list.has_train(train_name)) {
        throw std::out_of_range("Train does not exist.");
    }
    return get_schedule(train_list.get_train_index(train_name));
}

void cda_rail::Timetable::add_track_to_station(int station_index, int track, const cda_rail::Network &network) {
    if (!has_station(station_index)) {
        throw std::out_of_range("Station does not exist.");
    }
    if (!network.has_edge(track)) {
        throw std::out_of_range("Track does not exist.");
    }
    stations.at(station_index).tracks.insert(track);
}

void cda_rail::Timetable::add_track_to_station(const std::string &name, int track, const cda_rail::Network &network) {
    if (!has_station(name)) {
        throw std::out_of_range("Station does not exist.");
    }
    if (!network.has_edge(track)) {
        throw std::out_of_range("Track does not exist.");
    }
    add_track_to_station(get_station_index(name), track, network);
}

void
cda_rail::Timetable::add_track_to_station(int station_index, int source, int target, const cda_rail::Network &network) {
    if (!has_station(station_index)) {
        throw std::out_of_range("Station does not exist.");
    }
    if (!network.has_edge(source, target)) {
        throw std::out_of_range("Track does not exist.");
    }
    add_track_to_station(station_index, network.get_edge_index(source, target), network);
}

void
cda_rail::Timetable::add_track_to_station(const std::string &name, int source, int target, const cda_rail::Network &network) {
    if (!has_station(name)) {
        throw std::out_of_range("Station does not exist.");
    }
    if (!network.has_edge(source, target)) {
        throw std::out_of_range("Track does not exist.");
    }
    add_track_to_station(get_station_index(name), network.get_edge_index(source, target), network);
}

void cda_rail::Timetable::add_track_to_station(int station_index, const std::string &source, const std::string &target,
                                               const cda_rail::Network &network) {
    if (!has_station(station_index)) {
        throw std::out_of_range("Station does not exist.");
    }
    if (!network.has_edge(source, target)) {
        throw std::out_of_range("Track does not exist.");
    }
    add_track_to_station(station_index, network.get_edge_index(source, target), network);
}

void cda_rail::Timetable::add_train(const std::string &name, int length, double max_speed, double acceleration,
                                    double deceleration, int t_0, double v_0, int entry, int t_n, double v_n, int exit,
                                    const cda_rail::Network &network) {
    if (!network.has_vertex(entry)) {
        throw std::out_of_range("Entry vertex does not exist.");
    }
    if (!network.has_vertex(exit)) {
        throw std::out_of_range("Exit vertex does not exist.");
    }
    if (train_list.has_train(name)) {
        throw std::out_of_range("Train already exists.");
    }
    train_list.add_train(name, length, max_speed, acceleration, deceleration);
    schedules.push_back(cda_rail::Schedule{t_0, v_0, entry, t_n, v_n, exit});
}

void cda_rail::Timetable::add_train(const std::string &name, int length, double max_speed, double acceleration,
                                    double deceleration, int t_0, double v_0, const std::string &entry, int t_n,
                                    double v_n, const std::string &exit, const cda_rail::Network &network) {
    if (!network.has_vertex(entry)) {
        throw std::out_of_range("Entry vertex does not exist.");
    }
    if (!network.has_vertex(exit)) {
        throw std::out_of_range("Exit vertex does not exist.");
    }
    if (train_list.has_train(name)) {
        throw std::out_of_range("Train already exists.");
    }
    add_train(name, length, max_speed, acceleration, deceleration, t_0, v_0, network.get_vertex_index(entry), t_n, v_n,
              network.get_vertex_index(exit), network);
}

void cda_rail::Timetable::add_stop(int train_index, int station_index, int begin, int end, bool sort) {
    if (!train_list.has_train(train_index)) {
        throw std::out_of_range("Train does not exist.");
    }
    if (!has_station(station_index)) {
        throw std::out_of_range("Station does not exist.");
    }
    if (begin < 0 || end < 0) {
        throw std::invalid_argument("Time cannot be negative.");
    }
    if (begin >= end) {
        throw std::invalid_argument("End time has to be after the start time.");
    }
    auto& stops_reference = schedules.at(train_index).stops;
    for (const auto& stop : stops_reference) {
        if (stop.station == station_index) {
            throw std::out_of_range("Train already stops at station.");
        }
        // Check if [begin, end] and [stop.begin, stop.end] overlap
        if (begin <= stop.end && end >= stop.begin) {
            throw std::invalid_argument("Train has another stop at this time.");
        }
    }

    stops_reference.push_back(cda_rail::ScheduledStop{begin, end, station_index});
    if (sort) {
        std::sort(stops_reference.begin(), stops_reference.end());
    }
}

void cda_rail::Timetable::add_stop(const std::string &train_name, int station_index, int begin, int end, bool sort) {
    if (!train_list.has_train(train_name)) {
        throw std::out_of_range("Train does not exist.");
    }
    add_stop(train_list.get_train_index(train_name), station_index, begin, end, sort);
}

void cda_rail::Timetable::add_stop(int train_index, const std::string &station_name, int begin, int end, bool sort) {
    if (!has_station(station_name)) {
        throw std::out_of_range("Station does not exist.");
    }
    add_stop(train_index, station_name_to_index.at(station_name), begin, end, sort);
}

void cda_rail::Timetable::add_stop(const std::string &train_name, const std::string &station_name, int begin, int end, bool sort) {
    if (!train_list.has_train(train_name)) {
        throw std::out_of_range("Train does not exist.");
    }
    if (!has_station(station_name)) {
        throw std::out_of_range("Station does not exist.");
    }
    add_stop(train_list.get_train_index(train_name), station_name_to_index.at(station_name), begin, end, sort);
}

void cda_rail::Timetable::export_stations(const std::filesystem::path &p, const cda_rail::Network &network) const {
    /**
     * This private method exports all stations to a file. The file is a json file with the following structure:
     * {"station1_name": [["edge1_0", "edge1_1"], ["edge2_0", "edge2_1"], ...], "station2_name": ...}
     * Hereby the names stored in the network reference are used for the edges.
     *
     * @param p The path to the file directory to export to.
     * @param network The network reference to use for the edge names.
     */

    if (!std::filesystem::exists(p)) {
        throw std::invalid_argument("Path does not exist.");
    }
    if (!std::filesystem::is_directory(p)) {
        throw std::invalid_argument("Path is not a directory.");
    }

    json j;
    for (const auto& station : stations) {
        std::vector<std::pair<std::string, std::string>> edges;
        for (const auto& track : station.tracks) {
            const auto& edge = network.get_edge(track);
            edges.emplace_back(network.get_vertex(edge.source).name, network.get_vertex(edge.target).name);
        }
        j[station.name] = edges;
    }

    std::ofstream file(p / "stations.json");
    file << j << std::endl;
}

void cda_rail::Timetable::sort_stops() {
    /**
     * This methods sorts all stops of all trains according to the operator < of ScheduledStop.
     */

    for (auto& schedule : schedules) {
        std::sort(schedule.stops.begin(), schedule.stops.end());
    }
}

const cda_rail::TrainList &cda_rail::Timetable::get_train_list() const {
    return train_list;
}
