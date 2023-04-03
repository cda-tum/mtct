#include "Timetable.hpp"
#include "Station.hpp"
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

void cda_rail::Timetable::add_stop(int train_index, int station_index, int begin, int end, bool sort) {
    if (!train_list.has_train(train_index)) {
        throw std::out_of_range("Train does not exist.");
    }
    if (!station_list.has_station(station_index)) {
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
    if (!station_list.has_station(station_name)) {
        throw std::out_of_range("Station does not exist.");
    }
    add_stop(train_index, station_list.get_station_index(station_name), begin, end, sort);
}

void cda_rail::Timetable::add_stop(const std::string &train_name, const std::string &station_name, int begin, int end, bool sort) {
    if (!train_list.has_train(train_name)) {
        throw std::out_of_range("Train does not exist.");
    }
    if (!station_list.has_station(station_name)) {
        throw std::out_of_range("Station does not exist.");
    }
    add_stop(train_list.get_train_index(train_name), station_list.get_station_index(station_name), begin, end, sort);
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

cda_rail::Timetable::Timetable(const cda_rail::Network &network) : station_list(network) {}

const cda_rail::StationList &cda_rail::Timetable::get_station_list() const {
    return station_list;
};
