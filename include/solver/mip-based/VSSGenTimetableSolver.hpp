#pragma once
#include <string>
#include <filesystem>
#include "probleminstances/VSSGenerationTimetable.hpp"
#include "gurobi_c++.h"
#include "unordered_map"
#include "MultiArray.hpp"

namespace cda_rail::solver::mip_based {
    class VSSGenTimetableSolver {
        // TODO: Maybe add an abstract parent class later on
        private:
            cda_rail::instances::VSSGenerationTimetable instance;

            // Gurobi variables
            GRBEnv env = nullptr;
            GRBModel model = nullptr;
            std::unordered_map<std::string, MultiArray<GRBVar>> vars;

        public:
            // Constructors
            explicit VSSGenTimetableSolver(const cda_rail::instances::VSSGenerationTimetable& instance);
            explicit VSSGenTimetableSolver(const std::filesystem::path& instance_path);
            explicit VSSGenTimetableSolver(const std::string& instance_path);
            explicit VSSGenTimetableSolver(const char* instance_path);

            // Methods
            void solve(); // to be added with parameters and return type later
    };
}