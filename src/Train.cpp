#include "Train.hpp"
#include <filesystem>
#include <string>
#include "nlohmann/json.hpp"
#include <fstream>
#include <vector>
#include <unordered_map>

using json = nlohmann::json;

bool cda_rail::TrainList::has_train(const std::string &name) const {
    return train_name_to_index.find(name) != train_name_to_index.end();
}

bool cda_rail::TrainList::has_train(int index) const {
    return (index >= 0 && index < trains.size());
}

void cda_rail::TrainList::add_train(const std::string &name, int length, double max_speed, double acceleration,
                                    double deceleration) {
    if (has_train(name)) {
        throw std::out_of_range("Train already exists.");
    }
    trains.push_back(cda_rail::Train{name, length, max_speed, acceleration, deceleration});
}

int cda_rail::TrainList::size() const {
    return trains.size();
}

int cda_rail::TrainList::get_train_index(const std::string &name) const {
    if (!has_train(name)) {
        throw std::out_of_range("Train does not exist.");
    }
    return train_name_to_index.at(name);
}

const cda_rail::Train &cda_rail::TrainList::get_train(int index) const {
    if (!has_train(index)) {
        throw std::out_of_range("Train does not exist.");
    }
    return trains.at(index);
}

const cda_rail::Train &cda_rail::TrainList::get_train(const std::string &name) const {
    if (!has_train(name)) {
        throw std::out_of_range("Train does not exist.");
    }
    return get_train(get_train_index(name));
}

void cda_rail::TrainList::export_trains(const std::string &path) const {
    /**
     * This method exports all trains to a file. The file is a json file with the following structure:
     * {"train1_name": {"length": train1_length, "max_speed": train1_max_speed, "acceleration": train1_acceleration,
     *                  "deceleration": train1_deceleration}, "train2_name": ...}
     *
     * @param p The path to the file directory to export to.
     */

    std::filesystem::path p(path);
    export_trains(p);
}

cda_rail::TrainList cda_rail::TrainList::import_trains(const std::string &path) {
    /**
     * Imports trains from a file in the format specified in export_trains.
     *
     * @param path The path to the file to import from.
     */

    std::filesystem::path p(path);
    return import_trains(p);
}

void cda_rail::TrainList::export_trains(const std::filesystem::path &p) const {
    if (!std::filesystem::is_directory(p)) {
        throw std::invalid_argument("Path is not a directory.");
    }
    if (!std::filesystem::exists(p)) {
        // Create the directory if it doesn't exist
        std::filesystem::create_directory(p);
    }

    json j;
    for (const auto& train : trains) {
        j[train.name] = {{"length", train.length}, {"max_speed", train.max_speed}, {"acceleration", train.acceleration},
                         {"deceleration", train.deceleration}};
    }

    std::ofstream file(p / "trains.json");
    file << j << std::endl;
}

cda_rail::TrainList cda_rail::TrainList::import_trains(const std::filesystem::path &p) {
    if (!std::filesystem::is_directory(p)) {
        throw std::invalid_argument("Path is not a directory.");
    }
    if (!std::filesystem::exists(p)) {
        throw std::invalid_argument("Path does not exist.");
    }

    // Read the file
    std::ifstream f((p / "trains.json"));
    json data = json::parse(f);

    // Create the train list
    cda_rail::TrainList train_list;
    for (auto& [name, train] : data.items()) {
        train_list.add_train(name, train["length"], train["max_speed"], train["acceleration"], train["deceleration"]);
    }

    return train_list;
}

void cda_rail::TrainList::export_trains(const char *path) const {
    return export_trains(std::filesystem::path(path));
}

cda_rail::TrainList cda_rail::TrainList::import_trains(const char *path) {
    return import_trains(std::filesystem::path(path));
}
