#pragma once
#include <string>
#include <filesystem>
#include "probleminstances/VSSGenerationTimetable.hpp"

namespace cda_rail::solver::mip_based {
    class VSSGenTimetableSolver {
        private:
            cda_rail::instances::VSSGenerationTimetable instance;

        public:
            // Constructors
            explicit VSSGenTimetableSolver(const cda_rail::instances::VSSGenerationTimetable& instance);
            explicit VSSGenTimetableSolver(const std::filesystem::path& instance_path);
            explicit VSSGenTimetableSolver(const std::string& instance_path);
            explicit VSSGenTimetableSolver(const char* instance_path);
    };
}