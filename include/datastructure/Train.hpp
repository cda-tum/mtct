#pragma once
#include <string>
#include <vector>
#include <unordered_map>
#include <filesystem>

namespace cda_rail {
    struct Train {
        /**
         * Train struct with relevant properties
         * @param name Name of the train
         * @param length Length of the train (in m)
         * @param max_speed Maximum speed of the train (in m/s)
         * @param acceleration Acceleration of the train (in m/s^2)
         * @param deceleration Deceleration of the train (in m/s^2)
         */

        std::string name;
        int length;
        double max_speed;
        double acceleration;
        double deceleration;
    };

    class TrainList {
        /**
         * TrainList class
         */
        private:
            std::vector<Train> trains;
            std::unordered_map<std::string, int> train_name_to_index;

        public:
            void add_train(const std::string& name, int length, double max_speed, double acceleration, double deceleration);
            [[nodiscard]] int size() const;

            [[nodiscard]] int get_train_index(const std::string& name) const;
            [[nodiscard]] const Train& get_train(int index) const;
            [[nodiscard]] const Train& get_train(const std::string& name) const;

            [[nodiscard]] bool has_train(const std::string& name) const;
            [[nodiscard]] bool has_train(int index) const;

            void export_trains(const std::string& path) const;
            void export_trains(const char* path) const;
            void export_trains(const std::filesystem::path& p) const;
            [[nodiscard]] static cda_rail::TrainList import_trains(const std::string& path);
            [[nodiscard]] static cda_rail::TrainList import_trains(const char* path);
            [[nodiscard]] static cda_rail::TrainList import_trains(const std::filesystem::path& p);
    };
}