#pragma once
#include <filesystem>

namespace cda_rail {
    // Constants for vertex type
    static const int NO_BORDER = 0;
    static const int VSS = 1;
    static const int TTD = 2;

    static bool is_directory_and_create(const std::filesystem::path& p) {
        /**
         * Checks if a directory exists and creates it if it doesn't.
         * Returns true if the directory exists or was created successfully.
         * Returns false if the directory could not be created or is a file instead of a directory.
         *
         * @param p Path to the directory
         */

        // Check if the last component of the path has a file extension
        const auto ext = p.filename().extension();
        if (!ext.empty()) {
            return false;
        }

        // Check if the directory exists and create it if it doesn't
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
