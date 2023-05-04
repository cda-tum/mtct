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
        public:
            // Constructor with arbitrary number of size_t parameters
            template<typename... Args>
            explicit MultiArray(Args... args);

            // getter with arbitrary number of size_t parameters
            template<typename... Args>
            T& operator()(Args... args);

            // Function to obtain shape, size and dimensions
            const std::vector<size_t>& get_shape() const { return shape; };
            size_t size() const { return data.size(); };
            size_t dimensions() const { return shape.size(); };
    };

    template<typename T>
    template<typename... Args>
    T &MultiArray<T>::operator()(Args... args) {
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
        if (shape.size() != sizeof...(args)) {
            throw std::invalid_argument("Number of dimensions and number of arguments do not coincide.");
        }
        // If the value of any argument is too large throw an error
        std::vector<size_t> arg_tuple = {static_cast<size_t>(args)...};
        for (size_t i = 0; i < sizeof...(args); ++i) {
            if (arg_tuple[i] >= shape[i]) {
                std::stringstream ss;
                ss << "Index " << arg_tuple[i] << " is too large for dimension " << i;
                throw std::invalid_argument(ss.str());
            }
        }

        // Get the index of the element in the data respecting the row-major order
        size_t index = 0;
        size_t multiplier = 1;
        for (size_t i = 0; i < sizeof...(args); ++i) {
            index += arg_tuple[i] * multiplier;
            multiplier *= shape[i];
        }

        return data[index];
    }

    template<typename T>
    template<typename... Args>
    MultiArray<T>::MultiArray(Args... args) {
        /**
         * Constructor for an arbitrary number of dimensions.
         * The first parameter is the size of the first dimension.
         * The remaining parameters are the sizes of the remaining dimensions.
         *
         * @param first Size of the first dimension
         * @param args Sizes of the remaining dimensions
         */

        // Set the shape
        shape = {static_cast<size_t>(args)...};

        // If shape has only one element, allocate data
        // The overall size of the array is the product of all elements in shape.
        size_t cap = 1;
        for (size_t i = 0; i < shape.size(); ++i) {
            cap *= shape[i];
        }
        data = std::vector<T>(cap);
    }
}