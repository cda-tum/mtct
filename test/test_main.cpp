#include "TestingWarning.hpp"

#include "gtest/gtest.h"

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  cda_rail::test::register_warning_summary_listener(
      *::testing::UnitTest::GetInstance());
  return RUN_ALL_TESTS();
}
