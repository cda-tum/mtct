#include "solver/mip-based/VSSGenTimetableSolver.hpp"

#include "gtest/gtest.h"
#include <filesystem>
#include <fstream>
#include <memory>
#include <plog/Appenders/RollingFileAppender.h>
#include <plog/Formatters/TxtFormatter.h>
#include <plog/Logger.h>
#include <plog/Severity.h>
#include <string>

TEST(Logging, VSSGenDefaultLogging) {
  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  EXPECT_EQ(plog::get()->getMaxSeverity(), plog::debug);

  // NOLINTNEXTLINE(clang-diagnostic-unused-result)
  solver.solve();

  EXPECT_EQ(plog::get()->getMaxSeverity(), plog::info);

  // NOLINTNEXTLINE(clang-diagnostic-unused-result)
  solver.solve(-1, true);

  EXPECT_EQ(plog::get()->getMaxSeverity(), plog::debug);
}

TEST(Logging, FileLogging) {
  // Remove tmp_log.log if it exists
  if (std::filesystem::exists("tmp_log.log")) {
    std::filesystem::remove("tmp_log.log");
  }

  auto g_appender =
      std::make_unique<plog::RollingFileAppender<plog::TxtFormatter>>(
          "tmp_log.log");
  auto g_logger = std::make_unique<plog::Logger<0>>(plog::debug);
  g_logger->addAppender(g_appender.get());

  cda_rail::solver::mip_based::VSSGenTimetableSolver solver(
      "./example-networks/SimpleStation/");

  // NOLINTNEXTLINE(clang-diagnostic-unused-result)
  solver.solve();

  g_logger.reset();
  g_appender.reset();

  // Check that tmp_log.log exists
  EXPECT_TRUE(std::filesystem::exists("tmp_log.log"));
  // Check that tmp_log.log is not empty
  EXPECT_GT(std::filesystem::file_size("tmp_log.log"), 0);
  // Check that at least one line contains "Gurobi Optimizer version"
  std::ifstream log_file("tmp_log.log");
  std::string   line;
  bool          found = false;
  while (std::getline(log_file, line)) {
    if (line.find("Gurobi Optimizer version") != std::string::npos) {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found);
  log_file.close();
  // Remove tmp_log.log
  std::filesystem::remove("tmp_log.log");
}
