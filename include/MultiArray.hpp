#pragma once
#include <memory>
#include <vector>
#include <stdexcept>
#include <sstream>

namespace cda_rail {
    template<typename T>
    class MultiArray {
        private:
            std::vector<size_t> shape;
            std::vector<T> data;
            std::vector<MultiArray<T>> rows;
        public:
            // Constructor with arbitrary number of size_t parameters
            template<typename... Args>
            explicit MultiArray(size_t first, Args... args);
            // Need for initialization
            MultiArray() = default;

            // getter with arbitrary number of size_t parameters
            template<typename... Args>
            [[nodiscard]] T& operator()(size_t first, Args... args);
            // Only for compilation
            [[nodiscard]] T& operator()() { throw std::overflow_error("Something terribly went wrong. This function should have never been called"); };
    };

    template<typename T>
    template<typename... Args>
    T &MultiArray<T>::operator()(size_t first, Args... args) {
        /**
         * Getter for an arbitrary number of dimensions.
         * The first parameter is the index of the first dimension.
         * The remaining parameters are the indices of the remaining dimensions.
         * The number of parameters must coincide with the number of dimensions specified in shape.
         * The value of each parameter must be smaller than the size of the corresponding dimension.
         *
         * @param first Index of the first dimension
         * @param args Indices of the remaining dimensions
         */

        // If the number of dimensions and number of arguments does not coincide throw an error
        if (shape.size() != sizeof...(args) + 1) {
            throw std::invalid_argument("Number of dimensions and number of arguments do not coincide.");
        }
        // If the value first is too large throw an error
        if (first >= shape[0]) {
            std::stringstream ss;
            ss << "Index " << first << " of dimension " << shape.size() << " counted from the rear end is too large.";
            throw std::out_of_range(ss.str());
        }

        // If shape has only one element, return data
        if (shape.size() == 1) {
            return data[first];
        } else {
            // Otherwise, return rows
            return rows[first](args...);
        }
    }

    template<typename T>
    template<typename... Args>
    MultiArray<T>::MultiArray(size_t first, Args... args) {
        /**
         * Constructor for an arbitrary number of dimensions.
         * The first parameter is the size of the first dimension.
         * The remaining parameters are the sizes of the remaining dimensions.
         *
         * @param first Size of the first dimension
         * @param args Sizes of the remaining dimensions
         */

        // Set the shape
        shape = {first, static_cast<size_t>(args)...};

        // If shape has only one element, allocate data
        if (shape.size() == 1) {
            data = std::vector<T>(shape[0]);
        } else {
            // Otherwise, allocate rows
            rows = std::vector<MultiArray<T>>(shape[0]);
            for (size_t i = 0; i < shape[0]; i++) {
                rows[i] = MultiArray<T>(args...);
            }
        }
    }
}