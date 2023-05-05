#pragma once
#include <filesystem>

namespace cda_rail {
    // Constants for vertex type

    enum class VertexType {NO_BORDER = 0, VSS = 1, TTD = 2};
    enum class SeparationType {UNIFORM, CHEBYCHEV};

    static bool is_directory_and_create(const std::filesystem::path& p) {
        /**
         * Checks if a directory exists and creates it if it doesn't.
         * Returns true if the directory exists or was created successfully.
         * Returns false if the directory could not be created, probably because the path is not a directory.
         *
         * @param p Path to the directory
         */

        if (!std::filesystem::exists(p)) {
            std::error_code error_code;
            std::filesystem::create_directories(p, error_code);
            if (error_code) {
                return false;
            }
        }
        return std::filesystem::is_directory(p);
    };
}
