#include "TestingWarning.hpp"

#include <iostream>
#include <mutex>
#include <vector>

namespace cda_rail::test {

namespace {

struct WarningEntry {
  std::string test_name{};
  std::string message{};
};

std::once_flag            listener_registration_flag;
std::mutex                warning_mutex;
std::vector<WarningEntry> warning_entries;

class WarningSummaryListener final : public ::testing::EmptyTestEventListener {
public:
  void OnTestProgramEnd(const ::testing::UnitTest& /*unused*/) override {
    std::vector<WarningEntry> warnings;
    {
      const std::scoped_lock lock(warning_mutex);
      warnings = warning_entries;
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
  register_listener_once();

  const std::scoped_lock lock(warning_mutex);
  warning_entries.push_back(
      WarningEntry{.test_name = current_test_name(), .message = message});
}

void TestingWarning::register_listener_once() {
  std::call_once(listener_registration_flag, []() {
    // NOLINTNEXTLINE(cppcoreguidelines-owning-memory)
    ::testing::UnitTest::GetInstance()->listeners().Append(
        new WarningSummaryListener());
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
