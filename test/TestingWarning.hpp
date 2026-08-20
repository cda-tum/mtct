#pragma once

#include "gtest/gtest.h"
#include <iostream>
#include <mutex>
#include <string>
#include <vector>

namespace cda_rail::test {

/**
 * @brief Adds a non-failing warning for the currently running GoogleTest test.
 *
 * The warning is stored globally and shown once in a summary section at the end
 * of the complete test run.
 */
class TestingWarning {
public:
  static void AddWarning(const std::string& message) {
    std::call_once(listener_registration_flag(), []() {
      ::testing::UnitTest::GetInstance()->listeners().Append(
          new warning_summary_listener());
    });

    std::scoped_lock lock(warning_mutex());
    warning_entries().push_back(
        warning_entry{.test_name = current_test_name(), .message = message});
  }

private:
  struct warning_entry {
    std::string test_name{};
    std::string message{};
  };

  class warning_summary_listener final
      : public ::testing::EmptyTestEventListener {
  public:
    void OnTestProgramEnd(const ::testing::UnitTest&) override {
      std::vector<warning_entry> warnings;
      {
        std::scoped_lock lock(warning_mutex());
        warnings = warning_entries();
      }

      if (warnings.empty()) {
        return;
      }

      std::cout << "\n[ WARNING SUMMARY ]\n";
      for (const auto& warning : warnings) {
        std::cout << warning.test_name << " : " << warning.message << '\n';
      }
    }
  };

  [[nodiscard]] static std::string current_test_name() {
    const auto* test_info =
        ::testing::UnitTest::GetInstance()->current_test_info();
    if (test_info == nullptr) {
      return "UnknownTest";
    }
    return std::string(test_info->test_suite_name()) + "." + test_info->name();
  }

  [[nodiscard]] static std::once_flag& listener_registration_flag() {
    static std::once_flag flag;
    return flag;
  }

  [[nodiscard]] static std::mutex& warning_mutex() {
    static std::mutex mutex;
    return mutex;
  }

  [[nodiscard]] static std::vector<warning_entry>& warning_entries() {
    static std::vector<warning_entry> entries;
    return entries;
  }
};

} // namespace cda_rail::test
