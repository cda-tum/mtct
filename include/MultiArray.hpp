#pragma once
#include <memory>
#include <vector>

namespace cda_rail {
    template<typename T>
    class MultiArray {
        private:
            std::vector<int> shape;
            std::unique_ptr<T[]> data = nullptr;
            std::unique_ptr<MultiArray<T>[]> rows = nullptr;
        public:
            // Constructor with arbitrary number of size_t parameters
            template<typename... Args>
            MultiArray(size_t first, Args... args);

            // getter with arbitrary number of size_t parameters
            template<typename... Args>
            [[nodiscard]] const T& get(size_t first, Args... args) const;

            // setter with arbitrary number of size_t parameters
            template<typename... Args>
            void set(const T& value, size_t first, Args... args);
    };
};