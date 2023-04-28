#include "datastructure/Timetable.hpp"
#include "datastructure/Station.hpp"
#include "Definitions.hpp"
#include <filesystem>
#include <string>
#include "nlohmann/json.hpp"
#include <fstream>
#include <algorithm>

using json = nlohmann::json;

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

void cda_rail::Timetable::add_stop(int train_index, const std::string &station_name, int begin, int end, bool sort) {
    if (!train_list.has_train(train_index)) {
        throw std::out_of_range("Train does not exist.");
    }
    if (!station_list.has_station(station_name)) {
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
        if (stop.station == station_name) {
            throw std::out_of_range("Train already stops at station.");
        }
        // Check if [begin, end] and [stop.begin, stop.end] overlap
        if (begin <= stop.end && end >= stop.begin) {
            throw std::invalid_argument("Train has another stop at this time.");
        }
    }

    stops_reference.push_back(cda_rail::ScheduledStop{begin, end, station_name});
    if (sort) {
        std::sort(stops_reference.begin(), stops_reference.end());
    }
}

void cda_rail::Timetable::add_stop(const std::string &train_name, const std::string &station_name, int begin, int end, bool sort) {
    if (!train_list.has_train(train_name)) {
        throw std::out_of_range("Train does not exist.");
    }
    if (!station_list.has_station(station_name)) {
        throw std::out_of_range("Station does not exist.");
    }
    add_stop(train_list.get_train_index(train_name), station_name, begin, end, sort);
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

const cda_rail::StationList &cda_rail::Timetable::get_station_list() const {
    return station_list;
}

void cda_rail::Timetable::export_timetable(const std::filesystem::path &p, const cda_rail::Network &network) const {
    /**
     * This method exports the timetable to a directory. In particular the following files are created:
     * - trains.json according to the function defined in cda_rail::TrainList::export_trains
     * - stations.json according to the function defined in cda_rail::StationList::export_stations
     * - schedules.json of the following format:
     *  {"tr1": {"t_0": t_0, "v_0": v_0, "entry": v_name, "t_n": t_n, "v_n": v_n, "exit": v_name,
     *         "stops": [{"begin": t_b, "end": t_e, "station": s_name}, ...]}, ...}
     *
     *  @param p The path to the directory where the files should be created.
     */

    if (!cda_rail::is_directory_and_create(p)) {
        throw std::runtime_error("Could not create directory " + p.string());
    }

    train_list.export_trains(p);
    station_list.export_stations(p, network);

    json j;
    for (int i = 0; i < schedules.size(); ++i) {
        const auto& schedule = schedules.at(i);
        json stops;

        for (const auto& stop : schedule.stops) {
            stops.push_back({{"begin", stop.begin}, {"end", stop.end}, {"station", station_list.get_station(stop.station).name}});
        }
        j[train_list.get_train(i).name] = {{"t_0", schedule.t_0}, {"v_0", schedule.v_0},
                                                {"entry", network.get_vertex(schedule.entry).name},
                                                {"t_n", schedule.t_n}, {"v_n", schedule.v_n},
                                                {"exit", network.get_vertex(schedule.exit).name},
                                                {"stops", stops}};
    }

    std::ofstream file(p / "schedules.json");
    file << j << std::endl;
}

void cda_rail::Timetable::export_timetable(const std::string &path, const cda_rail::Network &network) const {
    export_timetable(std::filesystem::path(path), network);
}

cda_rail::Timetable cda_rail::Timetable::import_timetable(const std::filesystem::path &p, const cda_rail::Network &network) {
    /**
     * This method imports a timetable from a directory. In particular the following files are read:
     * - trains.json according to the function defined in cda_rail::TrainList::import_trains
     * - stations.json according to the function defined in cda_rail::StationList::import_stations
     * - schedules.json of the format described in cda_rail::Timetable::export_timetable
     *
     * @param p The path to the directory where the files should be read from.
     * @param network The network to which the timetable belongs.
     */

    // Check if the path exists and is a directory
    if (!std::filesystem::exists(p)) {
        throw std::invalid_argument("Path does not exist.");
    }
    if (!std::filesystem::is_directory(p)) {
        throw std::invalid_argument("Path is not a directory.");
    }

    // Import the train list and the station list
    cda_rail::Timetable timetable;
    timetable.set_train_list(cda_rail::TrainList::import_trains(p));
    timetable.station_list = cda_rail::StationList::import_stations(p, network);

    // Read the schedules file
    std::ifstream f(p / "schedules.json");
    json data = json::parse(f);

    // Parse the schedules
    for (int i = 0; i < timetable.train_list.size(); i++) {
        auto& tr = timetable.train_list.get_train(i);
        if (!data.contains(tr.name)) {
            throw std::invalid_argument("Schedule for train " + tr.name + " not found.");
        }
        auto& schedule_data = data[tr.name];
        timetable.schedules.at(i).t_0 = schedule_data["t_0"];
        timetable.schedules.at(i).v_0 = schedule_data["v_0"];
        timetable.schedules.at(i).entry = network.get_vertex_index(schedule_data["entry"]);
        timetable.schedules.at(i).t_n = schedule_data["t_n"];
        timetable.schedules.at(i).v_n = schedule_data["v_n"];
        timetable.schedules.at(i).exit = network.get_vertex_index(schedule_data["exit"]);

        for (const auto& stop_data : schedule_data["stops"]) {
            timetable.add_stop(i, stop_data["station"].get<std::string>(),
                        stop_data["begin"].get<int>(), stop_data["end"].get<int>(), false);
        }
    }

    // Sort the stops
    timetable.sort_stops();

    return timetable;
}

void cda_rail::Timetable::set_train_list(const cda_rail::TrainList &tl) {
    train_list = tl;
    schedules = std::vector<cda_rail::Schedule>(tl.size());
}

cda_rail::Timetable cda_rail::Timetable::import_timetable(const std::string &path, const cda_rail::Network &network) {
    return import_timetable(std::filesystem::path(path), network);
}

cda_rail::Timetable cda_rail::Timetable::import_timetable(const char *path, const cda_rail::Network &network) {
    return import_timetable(std::filesystem::path(path), network);
}

void cda_rail::Timetable::export_timetable(const char *path, const cda_rail::Network &network) const {
    export_timetable(std::filesystem::path(path), network);
}

bool cda_rail::Timetable::check_consistency(const cda_rail::Network &network) const {
    /**
     * This method checks if the timetable is consistent with the network, i.e., if the following holds:
     * - All vertices used as entry and exit points are valid vertices of the network
     * - Entry and exit vertices have exactly one neighboring vertex
     * - All edges of stations are valid edges of the network
     * - All scheduled stops are comparable by < or >, hence not overlapping
     */

    // Check if the entry and exit vertices are valid vertices of the network and if they have exactly one neighbor
    for (const auto& schedule : schedules) {
        if (!network.has_vertex(schedule.entry) || !network.has_vertex(schedule.exit)) {
            return false;
        }
        if (network.neighbors(schedule.entry).size() != 1 || network.neighbors(schedule.exit).size() != 1) {
            return false;
        }
    }

    // Check if all edges of stations are valid edges of the network
    for (auto station_name : station_list.get_station_names()) {
        const auto& station = station_list.get_station(station_name);
        for (auto track : station.tracks) {
            if (!network.has_edge(track)) {
                return false;
            }
        }
    }

    // Check if all stops are comparable by < or >
    for (const auto& schedule : schedules) {
        for (int i = 0; i < schedule.stops.size(); ++i) {
            for (int j = i + 1; j < schedule.stops.size(); ++j) {
                if (!(schedule.stops.at(i) < schedule.stops.at(j)) && !(schedule.stops.at(j) < schedule.stops.at(i))) {
                    return false;
                }
            }
        }
    }

    return true;
}

void cda_rail::Timetable::add_station(const std::string &name, const std::unordered_set<int> &tracks) {
    station_list.add_station(name, tracks);
}

void cda_rail::Timetable::add_station(const std::string &name) {
    station_list.add_station(name);
}

void cda_rail::Timetable::add_track_to_station(const std::string &name, int track, const cda_rail::Network &network) {
    station_list.add_track_to_station(name, track, network);
}

void cda_rail::Timetable::add_track_to_station(const std::string &name, int source, int target,
                                               const cda_rail::Network &network) {
    station_list.add_track_to_station(name, source, target, network);
}

void
cda_rail::Timetable::add_track_to_station(const std::string &name, const std::string &source, const std::string &target,
                                          const cda_rail::Network &network) {
    station_list.add_track_to_station(name, source, target, network);
}
