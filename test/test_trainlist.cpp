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

TEST(Functionality, WriteTrains) {
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

TEST(Functionality, EditTrains) {
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
