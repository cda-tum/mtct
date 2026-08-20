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
  static void add_warning(const std::string& message) {
    std::call_once(listener_registration_flag(), []() {
      // NOLINTNEXTLINE(cppcoreguidelines-owning-memory)
      ::testing::UnitTest::GetInstance()->listeners().Append(
          new WarningSummaryListener());
    });

    const std::scoped_lock lock(warning_mutex());
    warning_entries().push_back(
        WarningEntry{.test_name = current_test_name(), .message = message});
  }

private:
  struct WarningEntry {
    std::string test_name{};
    std::string message{};
  };

  class WarningSummaryListener final
      : public ::testing::EmptyTestEventListener {
  public:
    void OnTestProgramEnd(const ::testing::UnitTest& /*unused*/) override {
      std::vector<WarningEntry> warnings;
      {
        const std::scoped_lock lock(warning_mutex());
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

  [[nodiscard]] static std::vector<WarningEntry>& warning_entries() {
    static std::vector<WarningEntry> entries;
    return entries;
  }
};

} // namespace cda_rail::test
