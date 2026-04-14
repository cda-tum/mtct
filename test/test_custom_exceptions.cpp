#include "CustomExceptions.hpp"

#include "gtest/gtest.h"
#include <string>

// NOLINTBEGIN(clang-diagnostic-unused-result)

namespace {

using namespace cda_rail::exceptions;

template <typename Exception, typename Callable>
void expect_exception_message(Callable const&    callable,
                              std::string const& expected_message) {
  try {
    callable();
    FAIL() << "Expected exception was not thrown.";
  } catch (Exception const& exception) {
    EXPECT_STREQ(expected_message.c_str(), exception.what());
  } catch (...) {
    FAIL() << "Unexpected exception type thrown.";
  }
}

} // namespace

TEST(CustomExceptions, ModelCreationExceptionDefaultMessage) {
  ModelCreationException const exception;
  EXPECT_STREQ("Model creation failed.", exception.what());
}

TEST(CustomExceptions, ModelCreationExceptionCustomMessage) {
  ModelCreationException const exception("Could not create model for train 7.");
  EXPECT_STREQ("Could not create model for train 7.", exception.what());
}

TEST(CustomExceptions, ExportExceptionMessages) {
  ExportException const default_exception;
  ExportException const custom_exception("Export to JSON failed.");
  EXPECT_STREQ("Export failed.", default_exception.what());
  EXPECT_STREQ("Export to JSON failed.", custom_exception.what());
}

TEST(CustomExceptions, ConsistencyExceptionMessages) {
  ConsistencyException const default_exception;
  ConsistencyException const custom_exception("Timetable contains cycles.");
  EXPECT_STREQ("Consistency check failed.", default_exception.what());
  EXPECT_STREQ("Timetable contains cycles.", custom_exception.what());
}

TEST(CustomExceptions, InvalidInputExceptionMessages) {
  InvalidInputException const default_exception;
  InvalidInputException const custom_exception("Headway must be positive.");
  EXPECT_STREQ("Invalid input.", default_exception.what());
  EXPECT_STREQ("Headway must be positive.", custom_exception.what());
}

TEST(CustomExceptions, ImportExceptionMessages) {
  ImportException const default_exception;
  ImportException const named_exception("schedule");
  EXPECT_STREQ("Import failed.", default_exception.what());
  EXPECT_STREQ("Import of schedule failed.", named_exception.what());
}

TEST(CustomExceptions, VertexNotExistentExceptionMessages) {
  VertexNotExistentException const default_exception;
  VertexNotExistentException const name_exception("V42");
  VertexNotExistentException const id_exception(42);
  EXPECT_STREQ("Some vertex specified does not exist.",
               default_exception.what());
  EXPECT_STREQ("Vertex V42 does not exist", name_exception.what());
  EXPECT_STREQ("Vertex with ID 42 does not exist", id_exception.what());
}

TEST(CustomExceptions, EdgeNotExistentExceptionMessages) {
  EdgeNotExistentException const default_exception;
  EdgeNotExistentException const name_exception("E1");
  EdgeNotExistentException const id_exception(5);
  EdgeNotExistentException const id_pair_exception(2, 3);
  EdgeNotExistentException const name_pair_exception("A", "B");
  EXPECT_STREQ("Some edge specified does not exist.", default_exception.what());
  EXPECT_STREQ("Edge E1 does not exist.", name_exception.what());
  EXPECT_STREQ("Edge with ID 5 does not exist.", id_exception.what());
  EXPECT_STREQ("Edge connecting vertices with IDs 2->3 does not exist.",
               id_pair_exception.what());
  EXPECT_STREQ("Edge connecting A->B does not exist.",
               name_pair_exception.what());
}

TEST(CustomExceptions, TrainNotExistentExceptionMessages) {
  TrainNotExistentException const default_exception;
  TrainNotExistentException const name_exception("ICE100");
  TrainNotExistentException const id_exception(8);
  EXPECT_STREQ("Some train specified does not exist.",
               default_exception.what());
  EXPECT_STREQ("Train ICE100 does not exist.", name_exception.what());
  EXPECT_STREQ("Train with ID 8 does not exist.", id_exception.what());
}

TEST(CustomExceptions, StationNotExistentExceptionMessages) {
  StationNotExistentException const default_exception;
  StationNotExistentException const name_exception("Berlin Hbf");
  EXPECT_STREQ("Some station specified does not exist.",
               default_exception.what());
  EXPECT_STREQ("Station Berlin Hbf does not exist.", name_exception.what());
}

TEST(CustomExceptions, ScheduleNotExistentExceptionMessages) {
  ScheduleNotExistentException const default_exception;
  ScheduleNotExistentException const name_exception("MorningPlan");
  ScheduleNotExistentException const id_exception(13);
  EXPECT_STREQ("Some schedule specified does not exist.",
               default_exception.what());
  EXPECT_STREQ("Schedule MorningPlan does not exist.", name_exception.what());
  EXPECT_STREQ("Schedule with ID 13 does not exist.", id_exception.what());
}

TEST(CustomExceptions, ThrowIfLessInclusiveThrowsAndPasses) {
  EXPECT_THROW(throw_if_less(2.0, 2.0, "speed", true), InvalidInputException);
  EXPECT_THROW(throw_if_less(1.5, 2.0, "speed", true), InvalidInputException);
  EXPECT_NO_THROW(throw_if_less(2.1, 2.0, "speed", true));

  expect_exception_message<InvalidInputException>(
      []() { throw_if_less(2.0, 2.0, "speed", true); },
      "speed must be strictly larger than " + std::to_string(2.0) +
          ", but is " + std::to_string(2.0) + ".");
}

TEST(CustomExceptions, ThrowIfLessNonInclusiveThrowsAndPasses) {
  EXPECT_THROW(throw_if_less(1.9, 2.0, "speed", false), InvalidInputException);
  EXPECT_NO_THROW(throw_if_less(2.0, 2.0, "speed", false));
  EXPECT_NO_THROW(throw_if_less(2.1, 2.0, "speed", false));

  expect_exception_message<InvalidInputException>(
      []() { throw_if_less(1.9, 2.0, "speed", false); },
      "speed must be at least " + std::to_string(2.0) + ", but is " +
          std::to_string(1.9) + ".");
}

TEST(CustomExceptions, ThrowIfLessThanBoundaryBehavior) {
  EXPECT_THROW(throw_if_less_than(-0.1, 0.0, "x"), InvalidInputException);
  EXPECT_NO_THROW(throw_if_less_than(0.0, 0.0, "x"));
  EXPECT_NO_THROW(throw_if_less_than(0.1, 0.0, "x"));
}

TEST(CustomExceptions, ThrowIfLessThanOrEqualBoundaryBehavior) {
  EXPECT_THROW(throw_if_less_than_or_equal(0.0, 0.0, "x"),
               InvalidInputException);
  EXPECT_THROW(throw_if_less_than_or_equal(-1.0, 0.0, "x"),
               InvalidInputException);
  EXPECT_NO_THROW(throw_if_less_than_or_equal(0.1, 0.0, "x"));
}

TEST(CustomExceptions, ThrowIfNegativeBehaviorAndMessage) {
  EXPECT_THROW(throw_if_negative(-1.0, "distance"), InvalidInputException);
  EXPECT_NO_THROW(throw_if_negative(0.0, "distance"));
  EXPECT_NO_THROW(throw_if_negative(1.0, "distance"));

  expect_exception_message<InvalidInputException>(
      []() { throw_if_negative(-1.0, "distance"); },
      "distance must be at least " + std::to_string(0.0) + ", but is " +
          std::to_string(-1.0) + ".");
}

TEST(CustomExceptions, ThrowIfNonPositiveWithToleranceBehavior) {
  EXPECT_THROW(throw_if_non_positive(1.0, 1.0, "headway"),
               InvalidInputException);
  EXPECT_THROW(throw_if_non_positive(0.9, 1.0, "headway"),
               InvalidInputException);
  EXPECT_NO_THROW(throw_if_non_positive(1.1, 1.0, "headway"));

  expect_exception_message<InvalidInputException>(
      []() { throw_if_non_positive(1.0, 1.0, "headway"); },
      "headway must be strictly larger than " + std::to_string(1.0) +
          ", but is " + std::to_string(1.0) + ".");
}

TEST(CustomExceptions, ThrowIfNonPositiveDoubleBehavior) {
  EXPECT_THROW(throw_if_non_positive(0.0, "time"), InvalidInputException);
  EXPECT_THROW(throw_if_non_positive(-0.1, "time"), InvalidInputException);
  EXPECT_NO_THROW(throw_if_non_positive(0.1, "time"));
}

TEST(CustomExceptions, ThrowIfNonPositiveIntBehavior) {
  EXPECT_THROW(throw_if_non_positive(0, "count"), InvalidInputException);
  EXPECT_THROW(throw_if_non_positive(-1, "count"), InvalidInputException);
  EXPECT_NO_THROW(throw_if_non_positive(1, "count"));
}

TEST(CustomExceptions, ThrowIfInvalidFolderNameAcceptsPortableNames) {
  EXPECT_NO_THROW(throw_if_invalid_folder_name("data"));
  EXPECT_NO_THROW(throw_if_invalid_folder_name("Data Set 2026"));
  EXPECT_NO_THROW(throw_if_invalid_folder_name("run_1-v2.0"));
  EXPECT_NO_THROW(throw_if_invalid_folder_name("COM10"));
  EXPECT_NO_THROW(throw_if_invalid_folder_name("AUXiliary"));
  EXPECT_NO_THROW(throw_if_invalid_folder_name(std::string(255, 'a')));
}

TEST(CustomExceptions, ThrowIfInvalidFolderNameRejectsEmptyAndTooLong) {
  EXPECT_THROW(throw_if_invalid_folder_name(""), InvalidInputException);
  EXPECT_THROW(throw_if_invalid_folder_name(std::string(256, 'a')),
               InvalidInputException);

  expect_exception_message<InvalidInputException>(
      []() { throw_if_invalid_folder_name(""); },
      "Folder name must not be empty.");
  expect_exception_message<InvalidInputException>(
      []() { throw_if_invalid_folder_name(std::string(256, 'a')); },
      "Folder name must not be longer than 255 characters.");
}

TEST(CustomExceptions, ThrowIfInvalidFolderNameRejectsUnsupportedCharacters) {
  EXPECT_THROW(throw_if_invalid_folder_name("bad/name"), InvalidInputException);
  EXPECT_THROW(throw_if_invalid_folder_name("bad:name"), InvalidInputException);

  expect_exception_message<InvalidInputException>(
      []() { throw_if_invalid_folder_name("bad/name"); },
      "Folder name contains unsupported characters. Allowed are A-Z, a-z, "
      "0-9, '_', '-', '.', and space.");
}

TEST(CustomExceptions, ThrowIfInvalidFolderNameRejectsLeadingOrTrailingSpace) {
  EXPECT_THROW(throw_if_invalid_folder_name(" bad"), InvalidInputException);
  EXPECT_THROW(throw_if_invalid_folder_name("bad "), InvalidInputException);

  expect_exception_message<InvalidInputException>(
      []() { throw_if_invalid_folder_name(" bad"); },
      "Folder name must not start with a space.");
  expect_exception_message<InvalidInputException>(
      []() { throw_if_invalid_folder_name("bad "); },
      "Folder name must not end with a space or a dot.");
}

TEST(CustomExceptions, ThrowIfInvalidFolderNameRejectsTrailingDotForms) {
  EXPECT_THROW(throw_if_invalid_folder_name("bad."), InvalidInputException);
  EXPECT_THROW(throw_if_invalid_folder_name("."), InvalidInputException);
  EXPECT_THROW(throw_if_invalid_folder_name(".."), InvalidInputException);
}

TEST(CustomExceptions, ThrowIfInvalidFolderNameRejectsReservedWindowsNames) {
  EXPECT_THROW(throw_if_invalid_folder_name("CON"), InvalidInputException);
  EXPECT_THROW(throw_if_invalid_folder_name("con"), InvalidInputException);
  EXPECT_THROW(throw_if_invalid_folder_name("COM1.txt"), InvalidInputException);
  EXPECT_THROW(throw_if_invalid_folder_name("Lpt9.log"), InvalidInputException);
  EXPECT_THROW(throw_if_invalid_folder_name("Lpt9 .log"),
               InvalidInputException);

  expect_exception_message<InvalidInputException>(
      []() { throw_if_invalid_folder_name("CON"); },
      "Folder name must not be a Windows reserved device name.");
}

TEST(CustomExceptions,
     ThrowIfInvalidFolderNameAcceptsNamesCloseToReservedOnes) {
  EXPECT_NO_THROW(throw_if_invalid_folder_name("NUL1"));
  EXPECT_NO_THROW(throw_if_invalid_folder_name("COM1_port"));
  EXPECT_NO_THROW(throw_if_invalid_folder_name("LPT10"));
}

// NOLINTEND(clang-diagnostic-unused-result)
