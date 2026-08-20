#include "TestingWarning.hpp"

#include "gtest/gtest.h"
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace cda_rail::test {

namespace {

struct WarningEntry {
  std::string test_name{};
  std::string message{};
};

std::once_flag& listener_registration_flag() {
  static std::once_flag flag;
  return flag;
}

std::mutex& warning_mutex() {
  static std::mutex mutex;
  return mutex;
}

std::vector<WarningEntry>& warning_entries() {
  static std::vector<WarningEntry> entries;
  return entries;
}

std::string warning_file_path_from_env() {
  const char* warning_file = std::getenv("CDA_RAIL_WARNING_FILE");
  if (warning_file == nullptr) {
    return "";
  }
  return warning_file;
}

void append_warning_to_file(const std::string& warning_file,
                            const std::string& warning_line) {
  for (std::size_t attempt = 0; attempt < 100; ++attempt) {
    std::ofstream output(warning_file, std::ios::app);
    if (output.is_open()) {
      output << warning_line << '\n';
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  std::cerr << "Failed to write test warning to file: " << warning_file
            << std::endl;
}

class WarningSummaryListener final : public ::testing::EmptyTestEventListener {
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

} // namespace

void TestingWarning::add_warning(const std::string& message) {
  const auto warning_file = warning_file_path_from_env();
  if (!warning_file.empty()) {
    append_warning_to_file(warning_file, current_test_name() + " : " + message);
    return;
  }

  register_listener_once();

  const std::scoped_lock lock(warning_mutex());
  warning_entries().push_back(
      WarningEntry{.test_name = current_test_name(), .message = message});
}

void TestingWarning::register_listener_once() {
  std::call_once(listener_registration_flag(), []() {
    ::testing::UnitTest::GetInstance()->listeners().Append(
        new WarningSummaryListener()); // NOLINT(cppcoreguidelines-owning-memory)
  });
}

std::string TestingWarning::current_test_name() {
  const auto* test_info =
      ::testing::UnitTest::GetInstance()->current_test_info();
  if (test_info == nullptr) {
    return "UnknownTest";
  }
  return std::string(test_info->test_suite_name()) + "." + test_info->name();
}

} // namespace cda_rail::test
