#include "TestingWarning.hpp"

#include <iostream>
#include <mutex>
#include <utility>
#include <vector>

namespace cda_rail::test {
namespace {

struct warning_entry {
  std::string test_name{};
  std::string message{};
};

std::mutex& warning_mutex() {
  static std::mutex mutex;
  return mutex;
}

std::vector<warning_entry>& warning_entries() {
  static std::vector<warning_entry> entries;
  return entries;
}

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

const bool warning_listener_registered = []() {
  ::testing::UnitTest::GetInstance()->listeners().Append(
      new warning_summary_listener());
  return true;
}();

[[nodiscard]] std::string current_test_name() {
  const auto* test_info =
      ::testing::UnitTest::GetInstance()->current_test_info();
  if (test_info == nullptr) {
    return "UnknownTest";
  }
  return std::string(test_info->test_suite_name()) + "." + test_info->name();
}

} // namespace

void TestingWarning::AddWarning(const std::string& message) {
  std::scoped_lock lock(warning_mutex());
  warning_entries().push_back(
      warning_entry{.test_name = current_test_name(), .message = message});
}

void register_warning_summary_listener(::testing::UnitTest& unit_test) {
  unit_test.listeners().Append(new warning_summary_listener());
}

} // namespace cda_rail::test
