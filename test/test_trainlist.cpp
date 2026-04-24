#include "datastructure/Train.hpp"

#include "gtest/gtest.h"

TEST(Functionality, ReadTrains) {
  auto trains = cda_rail::TrainList::import_trains(
      "./example-networks/SimpleStation/timetable/");

  // Check if the all trains are imported
  EXPECT_EQ(trains.size(), 3);
  EXPECT_TRUE(trains.has_train("tr1"));
  EXPECT_TRUE(trains.has_train("tr2"));
  EXPECT_TRUE(trains.has_train("tr3"));

  // Check if the train tr1 is imported correctly
  auto tr1 = trains.get_train("tr1");
  EXPECT_EQ(tr1.name, "tr1");
  EXPECT_EQ(tr1.length, 100);
  EXPECT_EQ(tr1.max_speed, 83.33);
  EXPECT_EQ(tr1.acceleration, 2);
  EXPECT_EQ(tr1.deceleration, 1);
  EXPECT_TRUE(tr1.tim);

  // Check if the train tr2 is imported correctly
  auto tr2 = trains.get_train("tr2");
  EXPECT_EQ(tr2.name, "tr2");
  EXPECT_EQ(tr2.length, 100);
  EXPECT_EQ(tr2.max_speed, 27.78);
  EXPECT_EQ(tr2.acceleration, 2);
  EXPECT_EQ(tr2.deceleration, 1);
  EXPECT_TRUE(tr2.tim);

  // Check if the train tr3 is imported correctly
  auto tr3 = trains.get_train("tr3");
  EXPECT_EQ(tr3.name, "tr3");
  EXPECT_EQ(tr3.length, 250);
  EXPECT_EQ(tr3.max_speed, 20);
  EXPECT_EQ(tr3.acceleration, 2);
  EXPECT_EQ(tr3.deceleration, 1);
  EXPECT_TRUE(tr3.tim);
}

TEST(Functionality, WriteTrains) {
  // Create a train list
  auto       trains    = cda_rail::TrainList();
  const auto tr1_index = trains.add_train("tr1", 100, 83.33, 2, 1);
  const auto tr2_index = trains.add_train("tr2", 100, 27.78, 2, 1);
  const auto tr3_index = trains.add_train("tr3", 250, 20, 2, 1);

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
  EXPECT_EQ(tr1.name, "tr1");
  EXPECT_EQ(tr1.length, 100);
  EXPECT_EQ(tr1.max_speed, 83.33);
  EXPECT_EQ(tr1.acceleration, 2);
  EXPECT_EQ(tr1.deceleration, 1);
  EXPECT_TRUE(tr1.tim);

  // Check if the train tr2 is imported correctly
  auto tr2 = trains_read.get_train("tr2");
  EXPECT_EQ(tr2.name, "tr2");
  EXPECT_EQ(tr2.length, 100);
  EXPECT_EQ(tr2.max_speed, 27.78);
  EXPECT_EQ(tr2.acceleration, 2);
  EXPECT_EQ(tr2.deceleration, 1);
  EXPECT_TRUE(tr2.tim);

  // Check if the train tr3 is imported correctly
  auto tr3 = trains_read.get_train("tr3");
  EXPECT_EQ(tr3.name, "tr3");
  EXPECT_EQ(tr3.length, 250);
  EXPECT_EQ(tr3.max_speed, 20);
  EXPECT_EQ(tr3.acceleration, 2);
  EXPECT_EQ(tr3.deceleration, 1);
  EXPECT_TRUE(tr3.tim);
}

TEST(Functionality, EditTrains) {
  // Create a train list
  auto       trains    = cda_rail::TrainList();
  const auto tr1_index = trains.add_train("tr1", 100, 83.33, 2, 1, false);

  EXPECT_EQ(trains.get_train("tr1").length, 100);
  EXPECT_EQ(trains.get_train(tr1_index).max_speed, 83.33);
  EXPECT_EQ(trains.get_train("tr1").acceleration, 2);
  EXPECT_EQ(trains.get_train(tr1_index).deceleration, 1);
  EXPECT_FALSE(trains.get_train("tr1").tim);

  trains.editable_train("tr1").length           = 200;
  trains.editable_train(tr1_index).max_speed    = 50;
  trains.editable_train("tr1").acceleration     = 3;
  trains.editable_train(tr1_index).deceleration = 2;
  trains.editable_train("tr1").tim              = true;

  EXPECT_EQ(trains.get_train("tr1").length, 200);
  EXPECT_EQ(trains.get_train(tr1_index).max_speed, 50);
  EXPECT_EQ(trains.get_train("tr1").acceleration, 3);
  EXPECT_EQ(trains.get_train(tr1_index).deceleration, 2);
  EXPECT_TRUE(trains.get_train("tr1").tim);
}

TEST(Functionality, TrainExceptions) {
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
