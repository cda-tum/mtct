#include "solver/mip-based/VSSGenTimetableSolver.hpp"

#include "gtest/gtest.h"
#include <filesystem>
#include <iostream>
#include <plog/Formatters/TxtFormatter.h>
#include <plog/Initializers/RollingFileInitializer.h>
#include <plog/Log.h>
#include <plog/Logger.h>
#include <string>

TEST(Logging, VSSGenDefaultLogging) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  EXPECT_EQ(plog::get()->getMaxSeverity(), plog::debug);

  solver.solve();

  EXPECT_EQ(plog::get()->getMaxSeverity(), plog::info);

  solver.solve({}, {}, {}, {}, -1, true);

  EXPECT_EQ(plog::get()->getMaxSeverity(), plog::debug);
}

TEST(Logging, FileLogging) {
  // Remove tmp_log.log if it exists
  if (std::filesystem::exists("tmp_log.log")) {
    std::filesystem::remove("tmp_log.log");
  }

  std::unique_ptr<plog::IAppender> g_appender;
  std::unique_ptr<plog::Logger<0>> g_logger;

  g_appender.reset(
      new plog::RollingFileAppender<plog::TxtFormatter>("tmp_log.log"));
  g_logger.reset(new plog::Logger<0>(plog::debug));
  g_logger->addAppender(g_appender.get());

  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");
  solver.solve();

  g_logger.reset();
  g_appender.reset();

  // Check that tmp_log.log exists
  EXPECT_TRUE(std::filesystem::exists("tmp_log.log"));
  // Check that tmp_log.log is not empty
  EXPECT_GT(std::filesystem::file_size("tmp_log.log"), 0);
  // Remove tmp_log.log
  std::filesystem::remove("tmp_log.log");
}
