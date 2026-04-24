#include "datastructure/Train.hpp"

#include "gtest/gtest.h"
#include <string>
#include <vector>

// Train class

TEST(Train, ConstructorStoresProvidedValuesAndDefaultsTimToTrue) {
  cda_rail::Train const train("ice", 200.0, 55.0, 1.8, 1.2);

  EXPECT_EQ(train.get_name(), "ice");
  EXPECT_DOUBLE_EQ(train.get_length(), 200.0);
  EXPECT_DOUBLE_EQ(train.get_max_speed(), 55.0);
  EXPECT_DOUBLE_EQ(train.get_acceleration(), 1.8);
  EXPECT_DOUBLE_EQ(train.get_deceleration(), 1.2);
  EXPECT_TRUE(train.has_tim());
}

TEST(Train, ConstructorStoresExplicitTimFlag) {
  cda_rail::Train const train("regional", 90.0, 30.0, 1.2, 1.1, false);

  EXPECT_FALSE(train.has_tim());
}

TEST(Train, ConstructorRejectsNegativeLength) {
  EXPECT_THROW((void)cda_rail::Train("ice", -1.0, 55.0, 1.8, 1.2),
               cda_rail::exceptions::InvalidInputException);
}

TEST(Train, ConstructorRejectsMaxSpeedBelowMinimum) {
  EXPECT_THROW((void)cda_rail::Train("ice", 200.0, cda_rail::MIN_NON_ZERO - 0.1,
                                     1.8, 1.2),
               cda_rail::exceptions::InvalidInputException);
}

TEST(Train, ConstructorRejectsAccelerationBelowMinimum) {
  EXPECT_THROW((void)cda_rail::Train("ice", 200.0, 55.0,
                                     cda_rail::MIN_NON_ZERO - 0.1, 1.2),
               cda_rail::exceptions::InvalidInputException);
}

TEST(Train, ConstructorRejectsDecelerationBelowMinimum) {
  EXPECT_THROW((void)cda_rail::Train("ice", 200.0, 55.0, 1.8,
                                     cda_rail::MIN_NON_ZERO - 0.1),
               cda_rail::exceptions::InvalidInputException);
}

TEST(Train, ConstructorAcceptsZeroLength) {
  EXPECT_NO_THROW((void)cda_rail::Train("ice", 0.0, 55.0, 1.8, 1.2));
  cda_rail::Train const train("ice", 0.0, 55.0, 1.8, 1.2);
  EXPECT_DOUBLE_EQ(train.get_length(), 0.0);
}

TEST(Train, ConstructorAcceptsExactlyMinNonZeroForMaxSpeed) {
  EXPECT_NO_THROW(
      (void)cda_rail::Train("ice", 200.0, cda_rail::MIN_NON_ZERO, 1.8, 1.2));
  cda_rail::Train const train("ice", 200.0, cda_rail::MIN_NON_ZERO, 1.8, 1.2);
  EXPECT_DOUBLE_EQ(train.get_max_speed(), cda_rail::MIN_NON_ZERO);
}

TEST(Train, ConstructorAcceptsExactlyMinNonZeroForAcceleration) {
  EXPECT_NO_THROW(
      (void)cda_rail::Train("ice", 200.0, 55.0, cda_rail::MIN_NON_ZERO, 1.2));
  cda_rail::Train const train("ice", 200.0, 55.0, cda_rail::MIN_NON_ZERO, 1.2);
  EXPECT_DOUBLE_EQ(train.get_acceleration(), cda_rail::MIN_NON_ZERO);
}

TEST(Train, ConstructorAcceptsExactlyMinNonZeroForDeceleration) {
  EXPECT_NO_THROW(
      (void)cda_rail::Train("ice", 200.0, 55.0, 1.8, cda_rail::MIN_NON_ZERO));
  cda_rail::Train const train("ice", 200.0, 55.0, 1.8, cda_rail::MIN_NON_ZERO);
  EXPECT_DOUBLE_EQ(train.get_deceleration(), cda_rail::MIN_NON_ZERO);
}

TEST(Train, SetNameUpdatesName) {
  cda_rail::Train train("old-name", 120.0, 45.0, 2.5, 1.5);

  train.set_name("new-name");

  EXPECT_EQ(train.get_name(), "new-name");
}

TEST(Train, SetLengthAcceptsZero) {
  cda_rail::Train train("t", 120.0, 45.0, 2.5, 1.5);

  train.set_length(0.0);

  EXPECT_DOUBLE_EQ(train.get_length(), 0.0);
}

TEST(Train, SetLengthRejectsNegativeValues) {
  cda_rail::Train train("t", 120.0, 45.0, 2.5, 1.5);

  EXPECT_THROW(train.set_length(-0.1),
               cda_rail::exceptions::InvalidInputException);
}

TEST(Train, SetMaxSpeedAcceptsMinimumNonZeroValue) {
  cda_rail::Train train("t", 120.0, 45.0, 2.5, 1.5);

  train.set_max_speed(cda_rail::MIN_NON_ZERO);

  EXPECT_DOUBLE_EQ(train.get_max_speed(), cda_rail::MIN_NON_ZERO);
}

TEST(Train, SetMaxSpeedRejectsValuesBelowMinimum) {
  cda_rail::Train train("t", 120.0, 45.0, 2.5, 1.5);

  EXPECT_THROW(train.set_max_speed(cda_rail::MIN_NON_ZERO - 0.1),
               cda_rail::exceptions::InvalidInputException);
}

TEST(Train, SetAccelerationAcceptsMinimumNonZeroValue) {
  cda_rail::Train train("t", 120.0, 45.0, 2.5, 1.5);

  train.set_acceleration(cda_rail::MIN_NON_ZERO);

  EXPECT_DOUBLE_EQ(train.get_acceleration(), cda_rail::MIN_NON_ZERO);
}

TEST(Train, SetAccelerationRejectsValuesBelowMinimum) {
  cda_rail::Train train("t", 120.0, 45.0, 2.5, 1.5);

  EXPECT_THROW(train.set_acceleration(cda_rail::MIN_NON_ZERO - 0.1),
               cda_rail::exceptions::InvalidInputException);
}

TEST(Train, SetDecelerationAcceptsMinimumNonZeroValue) {
  cda_rail::Train train("t", 120.0, 45.0, 2.5, 1.5);

  train.set_deceleration(cda_rail::MIN_NON_ZERO);

  EXPECT_DOUBLE_EQ(train.get_deceleration(), cda_rail::MIN_NON_ZERO);
}

TEST(Train, SetDecelerationRejectsValuesBelowMinimum) {
  cda_rail::Train train("t", 120.0, 45.0, 2.5, 1.5);

  EXPECT_THROW(train.set_deceleration(cda_rail::MIN_NON_ZERO - 0.1),
               cda_rail::exceptions::InvalidInputException);
}

TEST(Train, SetTimValueCanDisableAndEnableTim) {
  cda_rail::Train train("t", 120.0, 45.0, 2.5, 1.5, true);

  train.set_tim_value(false);
  EXPECT_FALSE(train.has_tim());

  train.set_tim_value(true);
  EXPECT_TRUE(train.has_tim());
}

TEST(Train, SetTimEnablesTim) {
  cda_rail::Train train("t", 120.0, 45.0, 2.5, 1.5, false);

  train.set_tim();

  EXPECT_TRUE(train.has_tim());
}

TEST(Train, SetNoTimDisablesTim) {
  cda_rail::Train train("t", 120.0, 45.0, 2.5, 1.5, true);

  train.set_no_tim();

  EXPECT_FALSE(train.has_tim());
}

// Train list class

TEST(TrainList, DefaultConstructorCreatesEmptyList) {
  cda_rail::TrainList const trains;

  EXPECT_EQ(trains.size(), 0U);
  EXPECT_EQ(trains.get_number_of_trains(), 0U);
  EXPECT_FALSE(trains.has_train("tr1"));
  EXPECT_FALSE(trains.has_train(0));
  EXPECT_EQ(trains.cbegin(), trains.cend());
  EXPECT_EQ(trains.crbegin(), trains.crend());
}

TEST(TrainList, AddTrainFromAttributesReturnsIndexAndStoresTrain) {
  cda_rail::TrainList trains;

  auto const index = trains.add_train("tr1", 100.0, 40.0, 2.0, 1.0);

  EXPECT_EQ(index, 0U);
  EXPECT_EQ(trains.size(), 1U);
  EXPECT_TRUE(trains.has_train("tr1"));
  EXPECT_TRUE(trains.has_train(index));
  EXPECT_EQ(trains.get_train_index("tr1"), index);
  EXPECT_EQ(&trains.get_train(index), &trains.get_train("tr1"));
}

TEST(TrainList, AddTrainObjectReturnsNextIndexAndStoresObjectProperties) {
  cda_rail::TrainList trains;
  trains.add_train("tr1", 100.0, 40.0, 2.0, 1.0);

  auto const index =
      trains.add_train(cda_rail::Train("tr2", 120.0, 45.0, 2.5, 1.5, false));

  EXPECT_EQ(index, 1U);
  EXPECT_EQ(trains.get_number_of_trains(), 2U);
  EXPECT_EQ(trains.get_train(index).get_name(), "tr2");
  EXPECT_DOUBLE_EQ(trains.get_train(index).get_length(), 120.0);
  EXPECT_FALSE(trains.get_train(index).has_tim());
}

TEST(TrainList, AddTrainObjectRejectsDuplicateTrainName) {
  cda_rail::TrainList trains;
  trains.add_train(cda_rail::Train("tr1", 120.0, 45.0, 2.5, 1.5));

  EXPECT_THROW(trains.add_train(cda_rail::Train("tr1", 120.0, 45.0, 2.5, 1.5)),
               cda_rail::exceptions::ConsistencyException);
}

TEST(TrainList, HasTrainByIndexReturnsFalseForOutOfRangeIndex) {
  cda_rail::TrainList trains;
  trains.add_train("tr1", 100.0, 40.0, 2.0, 1.0);
  trains.add_train("tr2", 150.0, 50.0, 3.0, 2.0, false);

  EXPECT_TRUE(trains.has_train(0));
  EXPECT_TRUE(trains.has_train(1));
  EXPECT_FALSE(trains.has_train(2));
}

TEST(TrainList, GetTrainIndexReturnsStoredIndex) {
  cda_rail::TrainList trains;
  trains.add_train("tr1", 100.0, 40.0, 2.0, 1.0);
  trains.add_train("tr2", 150.0, 50.0, 3.0, 2.0, false);

  EXPECT_EQ(trains.get_train_index("tr1"), 0U);
  EXPECT_EQ(trains.get_train_index("tr2"), 1U);
}

TEST(TrainList, GetTrainByIndexReturnsStoredTrain) {
  cda_rail::TrainList trains;
  trains.add_train("tr1", 100.0, 40.0, 2.0, 1.0);
  trains.add_train("tr2", 150.0, 50.0, 3.0, 2.0, false);
  auto const& train = trains.get_train(1);

  EXPECT_EQ(train.get_name(), "tr2");
  EXPECT_DOUBLE_EQ(train.get_length(), 150.0);
  EXPECT_DOUBLE_EQ(train.get_max_speed(), 50.0);
  EXPECT_DOUBLE_EQ(train.get_acceleration(), 3.0);
  EXPECT_DOUBLE_EQ(train.get_deceleration(), 2.0);
  EXPECT_FALSE(train.has_tim());
}

TEST(TrainList, GetTrainByNameReturnsStoredTrain) {
  cda_rail::TrainList trains;
  trains.add_train("tr1", 100.0, 40.0, 2.0, 1.0);
  trains.add_train("tr2", 150.0, 50.0, 3.0, 2.0, false);
  auto const& train = trains.get_train("tr1");

  EXPECT_EQ(train.get_name(), "tr1");
  EXPECT_DOUBLE_EQ(train.get_length(), 100.0);
  EXPECT_DOUBLE_EQ(train.get_max_speed(), 40.0);
  EXPECT_DOUBLE_EQ(train.get_acceleration(), 2.0);
  EXPECT_DOUBLE_EQ(train.get_deceleration(), 1.0);
  EXPECT_TRUE(train.has_tim());
}

TEST(TrainList, EditableTrainByIndexReturnsMutableReference) {
  cda_rail::TrainList trains;
  trains.add_train("tr1", 100.0, 40.0, 2.0, 1.0);
  trains.add_train("tr2", 150.0, 50.0, 3.0, 2.0, false);

  trains.editable_train(0).set_length(180.0);

  EXPECT_DOUBLE_EQ(trains.get_train(0).get_length(), 180.0);
}

TEST(TrainList, EditableTrainByNameReturnsMutableReference) {
  cda_rail::TrainList trains;
  trains.add_train("tr1", 100.0, 40.0, 2.0, 1.0);
  trains.add_train("tr2", 150.0, 50.0, 3.0, 2.0, false);

  trains.editable_train("tr2").set_tim();

  EXPECT_TRUE(trains.get_train("tr2").has_tim());
}

TEST(TrainList, ConstIteratorsTraverseTrainsInInsertionOrder) {
  cda_rail::TrainList trains;
  trains.add_train("tr1", 100.0, 40.0, 2.0, 1.0);
  trains.add_train("tr2", 150.0, 50.0, 3.0, 2.0, false);

  std::vector<std::string> names;
  for (auto it = trains.cbegin(); it != trains.cend(); ++it) {
    names.push_back(it->get_name());
  }

  EXPECT_EQ(names, (std::vector<std::string>{"tr1", "tr2"}));
}

TEST(TrainList, ConstReverseIteratorsTraverseTrainsInReverseOrder) {
  cda_rail::TrainList trains;
  trains.add_train("tr1", 100.0, 40.0, 2.0, 1.0);
  trains.add_train("tr2", 150.0, 50.0, 3.0, 2.0, false);

  std::vector<std::string> names;
  for (auto it = trains.crbegin(); it != trains.crend(); ++it) {
    names.push_back(it->get_name());
  }

  EXPECT_EQ(names, (std::vector<std::string>{"tr2", "tr1"}));
}

TEST(TrainList, GetTrainIndexThrowsForUnknownTrainName) {
  cda_rail::TrainList const trains;

  EXPECT_THROW((void)trains.get_train_index("missing"),
               cda_rail::exceptions::TrainNotExistentException);
}

TEST(TrainList, GetTrainByIndexThrowsForUnknownIndex) {
  cda_rail::TrainList const trains;

  EXPECT_THROW((void)trains.get_train(0),
               cda_rail::exceptions::TrainNotExistentException);
}

TEST(TrainList, GetTrainByNameThrowsForUnknownName) {
  cda_rail::TrainList const trains;

  EXPECT_THROW((void)trains.get_train("missing"),
               cda_rail::exceptions::TrainNotExistentException);
}

TEST(TrainList, EditableTrainByIndexThrowsForUnknownIndex) {
  cda_rail::TrainList trains;

  EXPECT_THROW((void)trains.editable_train(0),
               cda_rail::exceptions::TrainNotExistentException);
}

TEST(TrainList, EditableTrainByNameThrowsForUnknownName) {
  cda_rail::TrainList trains;

  EXPECT_THROW((void)trains.editable_train("missing"),
               cda_rail::exceptions::TrainNotExistentException);
}

TEST(TrainList, ImportFromNonDirectoryPathThrowsImportException) {
  EXPECT_THROW((void)cda_rail::TrainList::import_trains("./nonexistent_path/"),
               cda_rail::exceptions::ImportException);
}

TEST(TrainList, AddTrainFromAttributesDefaultsTimToTrue) {
  cda_rail::TrainList trains;
  trains.add_train("tr1", 100.0, 40.0, 2.0, 1.0);

  EXPECT_TRUE(trains.get_train("tr1").has_tim());
}

// OLD TESTS

TEST(TrainList, ReadTrains) {
  auto trains = cda_rail::TrainList::import_trains(
      "./example-networks/SimpleStation/timetable/");

  // Check if the all trains are imported
  EXPECT_EQ(trains.size(), 3);
  EXPECT_TRUE(trains.has_train("tr1"));
  EXPECT_TRUE(trains.has_train("tr2"));
  EXPECT_TRUE(trains.has_train("tr3"));

  // Check if the train tr1 is imported correctly
  auto tr1 = trains.get_train("tr1");
  EXPECT_EQ(tr1.get_name(), "tr1");
  EXPECT_EQ(tr1.get_length(), 100);
  EXPECT_EQ(tr1.get_max_speed(), 83.33);
  EXPECT_EQ(tr1.get_acceleration(), 2);
  EXPECT_EQ(tr1.get_deceleration(), 1);
  EXPECT_TRUE(tr1.has_tim());

  // Check if the train tr2 is imported correctly
  auto tr2 = trains.get_train("tr2");
  EXPECT_EQ(tr2.get_name(), "tr2");
  EXPECT_EQ(tr2.get_length(), 100);
  EXPECT_EQ(tr2.get_max_speed(), 27.78);
  EXPECT_EQ(tr2.get_acceleration(), 2);
  EXPECT_EQ(tr2.get_deceleration(), 1);
  EXPECT_TRUE(tr2.has_tim());

  // Check if the train tr3 is imported correctly
  auto tr3 = trains.get_train("tr3");
  EXPECT_EQ(tr3.get_name(), "tr3");
  EXPECT_EQ(tr3.get_length(), 250);
  EXPECT_EQ(tr3.get_max_speed(), 20);
  EXPECT_EQ(tr3.get_acceleration(), 2);
  EXPECT_EQ(tr3.get_deceleration(), 1);
  EXPECT_TRUE(tr3.has_tim());
}

TEST(TrainList, WriteTrains) {
  // Create a train list
  auto       trains    = cda_rail::TrainList();
  const auto tr1_index = trains.add_train("tr1", 100, 83.33, 2, 1);
  const auto tr2_index = trains.add_train("tr2", 100, 27.78, 4, 7);
  const auto tr3_index = trains.add_train("tr3", 250, 20, 6, 3);

  // check the train indices
  EXPECT_EQ(trains.get_train_index("tr1"), tr1_index);
  EXPECT_EQ(trains.get_train_index("tr2"), tr2_index);
  EXPECT_EQ(trains.get_train_index("tr3"), tr3_index);

  // Write the train list to a file
  trains.export_trains("./tmp/write_trains_test");

  // Read the train list from the file
  auto trains_read =
      cda_rail::TrainList::import_trains("./tmp/write_trains_test");

  // Delete created directory and everything in it
  std::filesystem::remove_all("./tmp");

  // Check if the all trains are imported
  EXPECT_EQ(trains_read.size(), 3);
  EXPECT_TRUE(trains_read.has_train("tr1"));
  EXPECT_TRUE(trains_read.has_train("tr2"));
  EXPECT_TRUE(trains_read.has_train("tr3"));

  // Check if the train tr1 is imported correctly
  auto tr1 = trains_read.get_train("tr1");
  EXPECT_EQ(tr1.get_name(), "tr1");
  EXPECT_EQ(tr1.get_length(), 100);
  EXPECT_EQ(tr1.get_max_speed(), 83.33);
  EXPECT_EQ(tr1.get_acceleration(), 2);
  EXPECT_EQ(tr1.get_deceleration(), 1);
  EXPECT_TRUE(tr1.has_tim());

  // Check if the train tr2 is imported correctly
  auto tr2 = trains_read.get_train("tr2");
  EXPECT_EQ(tr2.get_name(), "tr2");
  EXPECT_EQ(tr2.get_length(), 100);
  EXPECT_EQ(tr2.get_max_speed(), 27.78);
  EXPECT_EQ(tr2.get_acceleration(), 4);
  EXPECT_EQ(tr2.get_deceleration(), 7);
  EXPECT_TRUE(tr2.has_tim());

  // Check if the train tr3 is imported correctly
  auto tr3 = trains_read.get_train("tr3");
  EXPECT_EQ(tr3.get_name(), "tr3");
  EXPECT_EQ(tr3.get_length(), 250);
  EXPECT_EQ(tr3.get_max_speed(), 20);
  EXPECT_EQ(tr3.get_acceleration(), 6);
  EXPECT_EQ(tr3.get_deceleration(), 3);
  EXPECT_TRUE(tr3.has_tim());
}

TEST(TrainList, EditTrains) {
  // Create a train list
  auto       trains    = cda_rail::TrainList();
  const auto tr1_index = trains.add_train("tr1", 100, 83.33, 2, 1, false);

  EXPECT_EQ(trains.get_train("tr1").get_length(), 100);
  EXPECT_EQ(trains.get_train(tr1_index).get_max_speed(), 83.33);
  EXPECT_EQ(trains.get_train("tr1").get_acceleration(), 2);
  EXPECT_EQ(trains.get_train(tr1_index).get_deceleration(), 1);
  EXPECT_FALSE(trains.get_train("tr1").has_tim());

  trains.editable_train("tr1").set_length(200);
  trains.editable_train(tr1_index).set_max_speed(50);
  trains.editable_train("tr1").set_acceleration(3);
  trains.editable_train(tr1_index).set_deceleration(2);
  trains.editable_train("tr1").set_tim();

  EXPECT_EQ(trains.get_train("tr1").get_length(), 200);
  EXPECT_EQ(trains.get_train(tr1_index).get_max_speed(), 50);
  EXPECT_EQ(trains.get_train("tr1").get_acceleration(), 3);
  EXPECT_EQ(trains.get_train(tr1_index).get_deceleration(), 2);
  EXPECT_TRUE(trains.get_train("tr1").has_tim());
}

TEST(TrainList, TrainExceptions) {
  // Create a train list
  auto trains = cda_rail::TrainList();
  trains.add_train("tr1", 100, 83.33, 2, 1, false);
  EXPECT_THROW((void)trains.add_train("tr1", 100, 83.33, 2, 1, false),
               cda_rail::exceptions::ConsistencyException);
  EXPECT_THROW((void)trains.get_train_index("tr2"),
               cda_rail::exceptions::TrainNotExistentException);
  EXPECT_THROW((void)trains.get_train(10),
               cda_rail::exceptions::TrainNotExistentException);
  EXPECT_THROW((void)trains.editable_train(10),
               cda_rail::exceptions::TrainNotExistentException);
  EXPECT_THROW((void)trains.get_train("tr2"),
               cda_rail::exceptions::TrainNotExistentException);
  EXPECT_THROW((void)trains.editable_train("tr2"),
               cda_rail::exceptions::TrainNotExistentException);
}
