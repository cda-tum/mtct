#pragma once

#include <string>

namespace cda_rail::test {

/**
 * @brief Adds a non-failing warning for the currently running GoogleTest test.
 *
 * The warning is stored globally and shown once in a summary section at the end
 * of the complete test run.
 */
class TestingWarning {
public:
  static void add_warning(const std::string& message);

private:
  static void                      register_listener_once();
  [[nodiscard]] static std::string current_test_name();
};

} // namespace cda_rail::test
