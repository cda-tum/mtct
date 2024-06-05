#pragma once

#include "Definitions.hpp"
#include "GeneralPerformanceOptimizationInstance.hpp"
#include "VSSGenerationTimetable.hpp"
#include "datastructure/GeneralTimetable.hpp"
#include "datastructure/RailwayNetwork.hpp"
#include "datastructure/Route.hpp"
#include "nlohmann/json.hpp"

#include <filesystem>
#include <fstream>
#include <map>
#include <string>
#include <type_traits>
#include <vector>

using json = nlohmann::json;

namespace cda_rail::instances {
class VSSTimetableOptimizationInstance
    : public GeneralPerformanceOptimizationInstance {};
} // namespace cda_rail::instances
