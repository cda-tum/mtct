#include "CLI/CLI.hpp"
#include "CommandInterpreter.hpp"
#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "Formatting.hpp"
#include "GeneralHelper.hpp"
#include "Session.hpp"
#include "VSSModel.hpp"
#include "datastructure/RailwayNetwork.hpp"
#include "options_common.hpp"
#include "options_mb_astar.hpp"
#include "options_mb_mip.hpp"
#include "options_vss_mip.hpp"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"
#include "simulator/GreedyHeuristic.hpp"
#include "solver/GeneralSolver.hpp"
#include "solver/astar-based/GenPOMovingBlockAStarSolver.hpp"
#include "solver/mip-based/GenPOMovingBlockMIPSolver.hpp"
#include "solver/mip-based/VSSGenTimetableSolver.hpp"

#include "gtest/gtest.h"
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

// The interpreter is driven directly, without launching a process. Every test
// works in its own directory below the test folder, which is where
// gtest_discover_tests puts the working directory, so that nothing is written
// into test/data.

namespace {
class CliTest : public ::testing::Test {
protected:
  std::filesystem::path directory;

  void SetUp() override {
    directory = std::filesystem::path(
        std::string{"cli-tmp-"} +
        ::testing::UnitTest::GetInstance()->current_test_info()->name());
    std::filesystem::remove_all(directory);
    std::filesystem::create_directories(directory);
  }

  void TearDown() override { std::filesystem::remove_all(directory); }
};

// Runs one line through a fresh interpreter, as the app does.
void run(cda_rail::cli::Session& session, const std::string& line) {
  cda_rail::cli::CommandInterpreter interpreter{session};
  interpreter.run_line(line);
}

void run_all(cda_rail::cli::Session&         session,
             const std::vector<std::string>& lines) {
  for (const auto& line : lines) {
    run(session, line);
  }
}

// Runs one line and returns what it printed.
std::string run_capturing(cda_rail::cli::Session& session,
                          const std::string&      line) {
  const std::ostringstream output;
  auto* const              previous = std::cout.rdbuf(output.rdbuf());
  run(session, line);
  std::cout.rdbuf(previous);
  return output.str();
}

// Feeds a whole script to the read loop, as the app does. Returns false if the
// loop was ended by `exit` and true if it ran out of input.
bool run_script(cda_rail::cli::Session& session, const std::string& script) {
  std::istringstream       input{script};
  const std::ostringstream output;
  auto* const              previous = std::cout.rdbuf(output.rdbuf());
  const bool               keep_going =
      cda_rail::cli::run_stream(input, session, false, false);
  std::cout.rdbuf(previous);
  return keep_going;
}

// The commands building a tiny two-vertex network.
const std::vector<std::string> SIMPLE_NETWORK{
    "network new TestNetwork",
    "network vertex add v0 -t TTD",
    "network vertex add v1 -t NoBorder -w 5",
    "network vertex add v2 -t TTD",
    "network edge add v0 v1 -l 500 -v 27.8",
    "network edge add v1 v2 -l 500 -v 27.8 -u",
    "network successor add v0 v1 v1 v2"};
} // namespace

TEST_F(CliTest, BuildNetwork) {
  cda_rail::cli::Session session{directory};
  run_all(session, SIMPLE_NETWORK);

  ASSERT_TRUE(session.has_network());
  const auto& network = session.const_current_network();
  EXPECT_EQ(network.get_network_name(), "TestNetwork");
  EXPECT_EQ(network.number_of_vertices(), 3);
  EXPECT_EQ(network.number_of_edges(), 2);
  EXPECT_EQ(network.get_vertex({"v0"}).type, cda_rail::VertexType::TTD);
  EXPECT_EQ(network.get_vertex({"v1"}).type, cda_rail::VertexType::NoBorder);
  EXPECT_DOUBLE_EQ(network.get_vertex({"v1"}).headway, 5);
  EXPECT_DOUBLE_EQ(network.get_edge({"v0", "v1"}).length, 500);
  EXPECT_DOUBLE_EQ(network.get_edge({"v0", "v1"}).max_speed, 27.8);
  EXPECT_TRUE(network.get_edge({"v0", "v1"}).breakable);
  EXPECT_FALSE(network.get_edge({"v1", "v2"}).breakable);
  EXPECT_TRUE(network.is_valid_successor({"v0", "v1"}, {"v1", "v2"}));
  EXPECT_TRUE(session.network_modified());
}

TEST_F(CliTest, ChangeNetwork) {
  cda_rail::cli::Session session{directory};
  run_all(session, SIMPLE_NETWORK);
  run(session, "network vertex change v1 -t VSS -w 12 --name middle");
  run(session, "network edge change v0 middle -l 700 -v 30 -m 20 -o 120 -u");

  const auto& network = session.const_current_network();
  EXPECT_FALSE(network.has_vertex("v1"));
  EXPECT_EQ(network.get_vertex({"middle"}).type, cda_rail::VertexType::VSS);
  EXPECT_DOUBLE_EQ(network.get_vertex({"middle"}).headway, 12);
  const auto& edge = network.get_edge({"v0", "middle"});
  EXPECT_DOUBLE_EQ(edge.length, 700);
  EXPECT_DOUBLE_EQ(edge.max_speed, 30);
  EXPECT_DOUBLE_EQ(edge.min_block_length, 20);
  EXPECT_DOUBLE_EQ(edge.min_stop_block_length, 120);
  EXPECT_FALSE(edge.breakable);
}

TEST_F(CliTest, BidirectionalEdge) {
  cda_rail::cli::Session session{directory};
  run(session, "network new TestNetwork");
  run(session, "network vertex add v0 -t TTD");
  run(session, "network vertex add v1 -t TTD");
  run(session, "network edge add-bidirectional v0 v1 -l 100 -v 20");

  const auto& network = session.const_current_network();
  EXPECT_EQ(network.number_of_edges(), 2);
  EXPECT_TRUE(network.has_edge({"v0"}, {"v1"}));
  EXPECT_TRUE(network.has_edge({"v1"}, {"v0"}));
}

TEST_F(CliTest, BuildInstance) {
  cda_rail::cli::Session session{directory};
  run_all(session, SIMPLE_NETWORK);
  // The network of this session is adopted without being saved first.
  run(session, "instance new TestInstance -s cli-test -n TestNetwork");
  run(session, "station add Central");
  run(session, "station add-track Central v0 v1");
  run(session, "train add ICE1 -l 200 -v 83.33 -a 0.5 -d 0.5 -e v0 -t 10 "
               "-x v2 -y 600 -w 2");
  run(session, "train stop add ICE1 Central -t 100 -d 60");
  run(session, "route add ICE1 v0 v1 v2");

  ASSERT_TRUE(session.has_instance());
  const auto& instance = session.const_instance();
  EXPECT_EQ(instance.get_instance_name(), "TestInstance");
  EXPECT_EQ(instance.get_instance_subdirectory(), "cli-test");
  EXPECT_EQ(instance.get_const_network().get_network_name(), "TestNetwork");
  EXPECT_EQ(instance.get_const_train_list().size(), 1);
  EXPECT_EQ(instance.get_const_station_list().size(), 1);
  EXPECT_DOUBLE_EQ(instance.get_train_weight("ICE1"), 2);

  const auto& train = instance.get_const_train_list().get_train("ICE1");
  EXPECT_DOUBLE_EQ(train.get_length(), 200);
  EXPECT_DOUBLE_EQ(train.get_acceleration(), 0.5);

  const auto& schedule = instance.get_const_schedule("ICE1");
  EXPECT_DOUBLE_EQ(schedule.get_entry_time(), 10);
  EXPECT_DOUBLE_EQ(schedule.get_exit_time(), 600);
  EXPECT_EQ(schedule.get_entry_vertex(),
            instance.get_const_network().get_vertex_index("v0"));
  ASSERT_EQ(schedule.get_stops().size(), 1);
  EXPECT_EQ(schedule.get_stops().at(0).get_station().name, "Central");
  EXPECT_DOUBLE_EQ(schedule.get_stops().at(0).get_service_time(), 100);
  EXPECT_DOUBLE_EQ(schedule.get_stops().at(0).get_service_duration(), 60);

  EXPECT_EQ(instance.get_const_routes().get_route("ICE1").size(), 2);
  EXPECT_TRUE(instance.check_consistency(true));
}

TEST_F(CliTest, ChangeTrainAndSchedule) {
  cda_rail::cli::Session session{directory};
  run_all(session, SIMPLE_NETWORK);
  run(session, "instance new TestInstance -s cli-test -n TestNetwork");
  run(session, "train add ICE1 -l 200 -v 83.33 -a 0.5 -d 0.5 -e v0 -x v2 "
               "-y 600");
  run(session, "train change ICE1 -l 100 -v 50 -a 1 -d 2 --no-tim");
  run(session, "train schedule ICE1 -e v2 -t 60 -i 5 -x v0 -y 1200 -z 10");
  run(session, "train weight ICE1 3.5");
  run(session, "instance station-delay-weight 7");

  const auto& instance = session.const_instance();
  const auto& train    = instance.get_const_train_list().get_train("ICE1");
  EXPECT_DOUBLE_EQ(train.get_length(), 100);
  EXPECT_DOUBLE_EQ(train.get_max_speed(), 50);
  EXPECT_DOUBLE_EQ(train.get_acceleration(), 1);
  EXPECT_DOUBLE_EQ(train.get_deceleration(), 2);
  EXPECT_FALSE(train.has_tim());

  const auto& schedule = instance.get_const_schedule("ICE1");
  EXPECT_EQ(schedule.get_entry_vertex(),
            instance.get_const_network().get_vertex_index("v2"));
  EXPECT_EQ(schedule.get_exit_vertex(),
            instance.get_const_network().get_vertex_index("v0"));
  EXPECT_DOUBLE_EQ(schedule.get_entry_time(), 60);
  EXPECT_DOUBLE_EQ(schedule.get_initial_velocity(), 5);
  EXPECT_DOUBLE_EQ(schedule.get_exit_time(), 1200);
  EXPECT_DOUBLE_EQ(schedule.get_exit_velocity(), 10);

  EXPECT_DOUBLE_EQ(instance.get_train_weight("ICE1"), 3.5);
  EXPECT_DOUBLE_EQ(instance.get_station_delay_weight(), 7);
}

TEST_F(CliTest, RemoveStopAndClearRoute) {
  cda_rail::cli::Session session{directory};
  run_all(session, SIMPLE_NETWORK);
  run(session, "instance new TestInstance -s cli-test -n TestNetwork");
  run(session, "station add Central");
  run(session, "station add-track Central v0 v1");
  run(session, "train add ICE1 -l 200 -v 83.33 -a 0.5 -d 0.5 -e v0 -x v2 "
               "-y 600");
  run(session, "train stop add ICE1 Central -t 100 -d 60");
  run(session, "route add ICE1 v0 v1 v2");
  ASSERT_EQ(
      session.const_instance().get_const_schedule("ICE1").get_stops().size(),
      1);

  run(session, "train stop remove ICE1 Central");
  run(session, "route clear ICE1");

  const auto& instance = session.const_instance();
  EXPECT_TRUE(instance.get_const_schedule("ICE1").get_stops().empty());
  EXPECT_TRUE(instance.get_const_routes().get_route("ICE1").empty());
}

TEST_F(CliTest, RoundTrip) {
  {
    cda_rail::cli::Session session{directory};
    run_all(session, SIMPLE_NETWORK);
    run(session, "network save");
    run(session, "instance new TestInstance -s cli-test -n TestNetwork");
    run(session, "station add Central");
    run(session, "station add-track Central v0 v1");
    run(session, "train add ICE1 -l 200 -v 83.33 -a 0.5 -d 0.5 -e v0 -t 10 "
                 "-x v2 -y 600");
    run(session, "route add ICE1 v0 v1 v2");
    run(session, "instance save");
    EXPECT_FALSE(session.has_unsaved_changes());
  }

  EXPECT_TRUE(std::filesystem::exists(directory / "networks" / "TestNetwork" /
                                      "tracks.graphml"));
  EXPECT_TRUE(std::filesystem::exists(directory / "instances" / "cli-test" /
                                      "TestInstance" / "network.json"));

  cda_rail::cli::Session session{directory};
  run(session, "instance load TestInstance -s cli-test");
  ASSERT_TRUE(session.has_instance());
  const auto& instance = session.const_instance();
  EXPECT_EQ(instance.get_const_network().get_network_name(), "TestNetwork");
  EXPECT_EQ(instance.get_const_network().number_of_vertices(), 3);
  EXPECT_EQ(instance.get_const_network().number_of_edges(), 2);
  EXPECT_EQ(instance.get_const_train_list().size(), 1);
  EXPECT_EQ(instance.get_const_station_list().size(), 1);
  EXPECT_EQ(instance.get_const_routes().get_route("ICE1").size(), 2);
  EXPECT_DOUBLE_EQ(instance.get_const_schedule("ICE1").get_entry_time(), 10);
  EXPECT_FALSE(session.has_unsaved_changes());
}

TEST_F(CliTest, InstanceSaveDoesNotRewriteNetwork) {
  cda_rail::cli::Session session{directory};
  run_all(session, SIMPLE_NETWORK);
  run(session, "network save");
  run(session, "instance new TestInstance -s cli-test -n TestNetwork");
  run(session, "instance save");

  // The network was edited afterwards and must not reach the disk on a plain
  // instance save.
  run(session, "network vertex add v3 -t TTD");
  run(session, "instance save");
  EXPECT_TRUE(session.network_modified());
  EXPECT_EQ(cda_rail::Network("TestNetwork", directory).number_of_vertices(),
            3);

  run(session, "instance save --with-network");
  EXPECT_FALSE(session.network_modified());
  EXPECT_EQ(cda_rail::Network("TestNetwork", directory).number_of_vertices(),
            4);
}

TEST_F(CliTest, NetworkSaveIsSeenByOtherInstance) {
  cda_rail::cli::Session session{directory};
  run_all(session, SIMPLE_NETWORK);
  run(session, "network save");
  run(session, "instance new First -s cli-test -n TestNetwork");
  run(session, "instance save");
  run(session, "instance close");
  run(session, "network load TestNetwork");
  run(session, "instance new Second -s cli-test -n TestNetwork");
  run(session, "instance save");

  // Editing the network through the second instance and writing only the
  // network is picked up when the first instance is loaded again.
  run(session, "network vertex add v3 -t TTD");
  run(session, "network save");
  run(session, "instance close");
  run(session, "instance load First -s cli-test");
  EXPECT_EQ(session.const_instance().get_const_network().number_of_vertices(),
            4);
}

TEST_F(CliTest, NetworkRenameWritesNewFolder) {
  cda_rail::cli::Session session{directory};
  run_all(session, SIMPLE_NETWORK);
  run(session, "network save");
  run(session, "network rename OtherNetwork");
  run(session, "network save");

  EXPECT_TRUE(std::filesystem::exists(directory / "networks" / "TestNetwork"));
  EXPECT_TRUE(std::filesystem::exists(directory / "networks" / "OtherNetwork"));
  EXPECT_EQ(session.const_current_network().get_network_name(), "OtherNetwork");
}

TEST_F(CliTest, ReloadDiscardsUnsavedChanges) {
  cda_rail::cli::Session session{directory};
  run_all(session, SIMPLE_NETWORK);
  run(session, "network save");
  run(session, "instance new TestInstance -s cli-test -n TestNetwork");
  run(session, "train add ICE1 -l 200 -v 83.33 -a 0.5 -d 0.5 -e v0 -x v2 "
               "-y 600");
  run(session, "instance save");

  run(session, "train add ICE2 -l 100 -v 50 -a 1 -d 1 -e v0 -x v2 -y 900");
  ASSERT_EQ(session.const_instance().get_const_train_list().size(), 2);
  EXPECT_TRUE(session.instance_modified());

  run(session, "instance reload");
  EXPECT_EQ(session.const_instance().get_const_train_list().size(), 1);
  EXPECT_FALSE(session.instance_modified());

  // The same for a standalone network.
  run(session, "instance close");
  run(session, "network load TestNetwork");
  run(session, "network vertex add v3 -t TTD");
  ASSERT_EQ(session.const_current_network().number_of_vertices(), 4);
  run(session, "network reload");
  EXPECT_EQ(session.const_current_network().number_of_vertices(), 3);
  EXPECT_FALSE(session.network_modified());
}

TEST_F(CliTest, ErrorsLeaveTheSessionUsable) {
  cda_rail::cli::Session session{directory};
  run_all(session, SIMPLE_NETWORK);
  run(session, "instance new TestInstance -s cli-test -n TestNetwork");
  run(session, "train add ICE1 -l 200 -v 83.33 -a 0.5 -d 0.5 -e v0 -x v2 "
               "-y 600");

  // An unknown command.
  run(session, "frobnicate the timetable");
  // A missing required option.
  run(session, "train add ICE2 -l 100");
  // An unknown train.
  run(session, "train change Unknown -l 50");
  // An unknown station.
  run(session, "train stop add ICE1 Nowhere -t 100 -d 60");
  // An unknown vertex.
  run(session, "network edge add v0 nowhere -l 100 -v 20");
  // A network command while an instance is loaded.
  run(session, "network new Another");

  const auto& instance = session.const_instance();
  EXPECT_EQ(instance.get_const_train_list().size(), 1);
  EXPECT_DOUBLE_EQ(
      instance.get_const_train_list().get_train("ICE1").get_length(), 200);
  EXPECT_TRUE(instance.get_const_schedule("ICE1").get_stops().empty());
  EXPECT_EQ(instance.get_const_network().number_of_edges(), 2);
  EXPECT_EQ(instance.get_const_network().get_network_name(), "TestNetwork");

  // The session still works afterwards.
  run(session, "train add ICE2 -l 100 -v 50 -a 1 -d 1 -e v0 -x v2 -y 900");
  EXPECT_EQ(instance.get_const_train_list().size(), 2);
}

TEST_F(CliTest, OptionsOfOneLineDoNotLeakIntoTheNext) {
  cda_rail::cli::Session session{directory};
  run_all(session, SIMPLE_NETWORK);
  run(session, "instance new TestInstance -s cli-test -n TestNetwork");
  run(session, "train add ICE1 -l 200 -v 83.33 -a 0.5 -d 0.5 -e v0 -x v2 "
               "-y 600 --no-tim -w 4");
  run(session, "train add ICE2 -l 100 -v 50 -a 1 -d 1 -e v0 -x v2 -y 900");

  const auto& instance = session.const_instance();
  EXPECT_FALSE(instance.get_const_train_list().get_train("ICE1").has_tim());
  EXPECT_TRUE(instance.get_const_train_list().get_train("ICE2").has_tim());
  EXPECT_DOUBLE_EQ(instance.get_train_weight("ICE1"), 4);
  EXPECT_DOUBLE_EQ(instance.get_train_weight("ICE2"), 1);
}

TEST_F(CliTest, ScriptBuildsInstance) {
  // The same commands one would keep in a file next to the instance.
  const std::vector<std::string> script{
      "# build a small instance",
      "",
      "network new ScriptNetwork",
      "network vertex add l0 -t TTD",
      "network vertex add l1 -t VSS",
      "network vertex add l2 -t TTD",
      "network edge add l0 l1 -l 1000 -v 27.8",
      "network edge add l1 l2 -l 1000 -v 27.8",
      "network successor add l0 l1 l1 l2",
      "network save",
      "instance new ScriptInstance -s cli-test -n ScriptNetwork",
      "station add Halt",
      "station add-track Halt l1 l2",
      "train add RB1 -l 100 -v 27.8 -a 0.5 -d 0.5 -e l0 -t 0 -x l2 -y 900",
      "train stop add RB1 Halt -t 300 -d 60",
      "route add RB1 l0 l1 l2",
      "instance save",
  };

  cda_rail::cli::Session session{directory};
  run_all(session, script);

  ASSERT_TRUE(session.has_instance());
  EXPECT_TRUE(session.const_instance().check_consistency(true));
  EXPECT_FALSE(session.const_instance()
                   .is_obviously_infeasible(false)
                   .is_obviously_infeasible);
  EXPECT_FALSE(session.has_unsaved_changes());
  EXPECT_TRUE(std::filesystem::exists(directory / "instances" / "cli-test" /
                                      "ScriptInstance" / "timetable" /
                                      "trains.json"));
}

TEST_F(CliTest, StatusIsBriefAndTheDetailsAreInInfo) {
  cda_rail::cli::Session session{directory};
  run_all(session, SIMPLE_NETWORK);
  run(session, "instance new TestInstance -s cli-test -n TestNetwork");
  run(session, "station add Central");
  run(session, "train add ICE1 -l 200 -v 83.33 -a 0.5 -d 0.5 -e v0 -x v2 "
               "-y 600");

  // status says where the session is, and nothing else.
  const auto status = run_capturing(session, "status");
  EXPECT_NE(status.find("TestInstance"), std::string::npos);
  EXPECT_NE(status.find("cli-test"), std::string::npos);
  EXPECT_NE(status.find("TestNetwork"), std::string::npos);
  EXPECT_NE(status.find("[modified]"), std::string::npos);
  EXPECT_EQ(status.find("Trains:"), std::string::npos);
  EXPECT_EQ(status.find("Vertices:"), std::string::npos);

  const auto instance_info = run_capturing(session, "instance info");
  EXPECT_NE(instance_info.find("Trains:               1"), std::string::npos);
  EXPECT_NE(instance_info.find("Stations:             1"), std::string::npos);
  EXPECT_NE(instance_info.find("Routes:               0"), std::string::npos);
  EXPECT_NE(instance_info.find("Station delay weight: 1"), std::string::npos);

  const auto network_info = run_capturing(session, "network info");
  EXPECT_NE(network_info.find("TestNetwork"), std::string::npos);
  EXPECT_NE(network_info.find("Vertices:  3"), std::string::npos);
  EXPECT_NE(network_info.find("2 TTD"), std::string::npos);
  EXPECT_NE(network_info.find("1 NoBorder"), std::string::npos);
  EXPECT_NE(network_info.find("Edges:     2"), std::string::npos);
  EXPECT_NE(network_info.find("Breakable: 1"), std::string::npos);
}

TEST_F(CliTest, ExitRequest) {
  cda_rail::cli::Session            session{directory};
  cda_rail::cli::CommandInterpreter interpreter{session};
  EXPECT_TRUE(interpreter.run_line("status"));

  cda_rail::cli::CommandInterpreter exit_interpreter{session};
  EXPECT_FALSE(exit_interpreter.run_line("exit"));

  // Unsaved changes stop the session from ending unless it is forced.
  cda_rail::cli::Session modified_session{directory};
  run(modified_session, "network new TestNetwork");
  cda_rail::cli::CommandInterpreter blocked{modified_session};
  EXPECT_TRUE(blocked.run_line("exit"));
  cda_rail::cli::CommandInterpreter forced{modified_session};
  EXPECT_FALSE(forced.run_line("exit --force"));
}

TEST_F(CliTest, ExitEndsTheReadLoop) {
  cda_rail::cli::Session session{directory};
  EXPECT_FALSE(run_script(session, "status\n"
                                   "exit\n"
                                   "network new NeverCreated\n"));

  // Nothing behind the 'exit' was executed.
  EXPECT_FALSE(session.has_network());
}

TEST_F(CliTest, TheReadLoopEndsWhenTheInputRunsOut) {
  // Without this, a session started on a script or a pipe would never return.
  cda_rail::cli::Session session{directory};
  EXPECT_TRUE(run_script(session, "status\nstatus\n"));
  EXPECT_TRUE(run_script(session, ""));
}

TEST_F(CliTest, ExitIsRefusedWhileChangesAreUnsaved) {
  cda_rail::cli::Session session{directory};
  // The 'exit' in the middle is refused, so the loop runs to the end of the
  // input and the commands behind it still take effect. Only '--force' ends
  // the session while something is unsaved.
  EXPECT_TRUE(run_script(session, "network new TestNetwork\n"
                                  "exit\n"
                                  "network vertex add v0 -t TTD\n"));
  EXPECT_EQ(session.const_current_network().number_of_vertices(), 1);

  EXPECT_FALSE(run_script(session, "exit --force\n"));
}

TEST_F(CliTest, PromptAndAccessorsFollowWhatIsLoaded) {
  cda_rail::cli::Session session{directory};
  EXPECT_EQ(session.prompt(), "rail> ");
  EXPECT_THROW(std::ignore = session.instance(),
               cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW(std::ignore = session.const_instance(),
               cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW(std::ignore = session.current_network(),
               cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW(std::ignore = session.const_current_network(),
               cda_rail::exceptions::InvalidInputException);

  run_all(session, SIMPLE_NETWORK);
  EXPECT_EQ(session.prompt(), "rail [TestNetwork]> ");

  run(session, "instance new TestInstance -s cli-test -n TestNetwork");
  EXPECT_EQ(session.prompt(), "rail (TestInstance)> ");
}

TEST_F(CliTest, NetworkObjectCommands) {
  cda_rail::cli::Session session{directory};
  run_all(session, SIMPLE_NETWORK);

  // Saving under another name renames the network on the way out.
  run(session, "network save --as OtherNetwork");
  EXPECT_EQ(session.const_current_network().get_network_name(), "OtherNetwork");
  EXPECT_TRUE(std::filesystem::exists(directory / "networks" / "OtherNetwork"));
  EXPECT_FALSE(session.network_modified());

  run(session, "network close");
  EXPECT_FALSE(session.has_network());
  // Closing and reloading now have nothing to work on.
  EXPECT_THROW(session.close_network(),
               cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW(session.reload_network(),
               cda_rail::exceptions::InvalidInputException);

  // While an instance is loaded the network belongs to it, so it cannot be
  // reloaded on its own.
  run(session, "instance new TestInstance -s cli-test -n OtherNetwork");
  EXPECT_THROW(session.reload_network(),
               cda_rail::exceptions::InvalidInputException);
}

TEST_F(CliTest, WorkingDirectoryCommand) {
  cda_rail::cli::Session session{directory};
  const auto             printed = run_capturing(session, "working-directory");
  EXPECT_NE(
      printed.find(std::filesystem::absolute(directory).filename().string()),
      std::string::npos);

  const auto other = directory / "elsewhere";
  std::filesystem::create_directories(other);
  run(session, "working-directory " + other.string());
  EXPECT_EQ(session.working_directory(), other);
}

TEST_F(CliTest, ListCommandsDescribeTheSession) {
  cda_rail::cli::Session session{directory};
  run_all(session, SIMPLE_NETWORK);
  run(session, "network save");
  run(session, "instance new TestInstance -s cli-test -n TestNetwork");
  run(session, "station add Central");
  run(session, "station add-track Central v0 v1");
  run(session, "train add ICE1 -l 200 -v 83.33 -a 0.5 -d 0.5 -e v0 -t 10 "
               "-x v2 -y 600");
  run(session, "train stop add ICE1 Central -t 100 -d 60");
  run(session, "route add ICE1 v0 v1 v2");
  run(session, "instance save");

  const auto networks = run_capturing(session, "network list");
  EXPECT_NE(networks.find("TestNetwork"), std::string::npos);

  const auto instances = run_capturing(session, "instance list");
  EXPECT_NE(instances.find("TestInstance"), std::string::npos);
  EXPECT_NE(instances.find("cli-test"), std::string::npos);

  const auto vertices = run_capturing(session, "network vertex list");
  EXPECT_NE(vertices.find("v0"), std::string::npos);
  // The vertex types are printed by name, not as the numbers they are stored
  // as.
  EXPECT_NE(vertices.find("TTD"), std::string::npos);
  EXPECT_NE(vertices.find("NoBorder"), std::string::npos);

  const auto edges = run_capturing(session, "network edge list");
  EXPECT_NE(edges.find("500"), std::string::npos);
  EXPECT_NE(edges.find("27.8"), std::string::npos);

  const auto successors = run_capturing(session, "network successor list");
  EXPECT_NE(successors.find("v0-v1"), std::string::npos);
  EXPECT_NE(successors.find("v1-v2"), std::string::npos);

  const auto stations = run_capturing(session, "station list");
  EXPECT_NE(stations.find("Central"), std::string::npos);
  EXPECT_NE(stations.find("v0-v1"), std::string::npos);

  const auto trains = run_capturing(session, "train list");
  EXPECT_NE(trains.find("ICE1"), std::string::npos);
  EXPECT_NE(trains.find("Central"), std::string::npos);

  const auto routes = run_capturing(session, "route list");
  EXPECT_NE(routes.find("v0-v1"), std::string::npos);
}

TEST_F(CliTest, ListCommandsCopeWithEmptyObjects) {
  cda_rail::cli::Session session{directory};
  // Neither folder exists yet, and the objects are empty.
  EXPECT_NE(run_capturing(session, "network list").find("No networks"),
            std::string::npos);
  EXPECT_NE(run_capturing(session, "instance list").find("No instances"),
            std::string::npos);

  run(session, "network new Empty");
  run(session, "network vertex add v0 -t TTD");
  // An edge without successors, a station without tracks and a train without
  // a route all have their own branch in the list commands.
  run(session, "network vertex add v1 -t TTD");
  run(session, "network edge add v0 v1 -l 100 -v 20");
  run(session, "instance new Empty -s cli-test -n Empty");
  run(session, "station add Nowhere");
  run(session, "train add ICE1 -l 100 -v 20 -a 1 -d 1 -e v0 -x v1 -y 100");

  EXPECT_NE(run_capturing(session, "network successor list").find("(none)"),
            std::string::npos);
  EXPECT_NE(run_capturing(session, "station list").find("(no tracks)"),
            std::string::npos);
  EXPECT_NE(run_capturing(session, "route list").find("(no route)"),
            std::string::npos);
}

TEST_F(CliTest, ClearingARouteThatDoesNotExistYet) {
  cda_rail::cli::Session session{directory};
  run_all(session, SIMPLE_NETWORK);
  run(session, "instance new TestInstance -s cli-test -n TestNetwork");
  run(session, "train add ICE1 -l 200 -v 83.33 -a 0.5 -d 0.5 -e v0 -x v2 "
               "-y 600");

  // The train has no route at all, so one is created before it is emptied.
  run(session, "route clear ICE1");
  const auto& routes = session.const_instance().get_const_routes();
  ASSERT_TRUE(routes.has_route("ICE1"));
  EXPECT_TRUE(routes.get_route("ICE1").empty());
  EXPECT_NE(run_capturing(session, "route list").find("(empty)"),
            std::string::npos);
}

TEST_F(CliTest, EntryTimeMovedBeyondTheCurrentExitTime) {
  cda_rail::cli::Session session{directory};
  run_all(session, SIMPLE_NETWORK);
  run(session, "instance new TestInstance -s cli-test -n TestNetwork");
  run(session, "train add ICE1 -l 200 -v 83.33 -a 0.5 -d 0.5 -e v0 -x v2 "
               "-y 600");

  // The new entry time is past the old exit time, so the exit time has to be
  // written first for the schedule to stay valid in between.
  run(session, "train schedule ICE1 -t 700 -y 1200");
  const auto& schedule = session.const_instance().get_const_schedule("ICE1");
  EXPECT_DOUBLE_EQ(schedule.get_entry_time(), 700);
  EXPECT_DOUBLE_EQ(schedule.get_exit_time(), 1200);
}

TEST_F(CliTest, ThePromptIsPrintedWhileCommandsAreTyped) {
  cda_rail::cli::Session   session{directory};
  std::istringstream       input{"status\n"};
  const std::ostringstream output;
  auto* const              previous = std::cout.rdbuf(output.rdbuf());
  const bool               keep_going =
      cda_rail::cli::run_stream(input, session, true, false);
  std::cout.rdbuf(previous);

  EXPECT_TRUE(keep_going);
  EXPECT_NE(output.str().find("rail> "), std::string::npos);
}

namespace {
// Builds the smallest instance the three solvers accept. Unlike SIMPLE_NETWORK
// it has no NoBorder vertex, which a solver rejects as a breakable edge inside
// an unbreakable section.
void build_solvable_instance(cda_rail::cli::Session& session) {
  run(session, "network new SolveNetwork");
  run(session, "network vertex add v0 -t TTD");
  run(session, "network vertex add v1 -t TTD");
  run(session, "network vertex add v2 -t TTD");
  run(session, "network edge add v0 v1 -l 500 -v 27.8");
  run(session, "network edge add v1 v2 -l 500 -v 27.8");
  run(session, "network successor add v0 v1 v1 v2");
  run(session, "instance new TestInstance -s cli-test -n SolveNetwork");
  run(session, "train add ICE1 -l 50 -v 20 -a 1 -d 1 -e v0 -t 0 -x v2 -y 195");
  run(session, "route add ICE1 v0 v1 v2");
}

// The instance is tiny on purpose: every solver finds it in well under a
// second, so the whole solve path can be checked without a long test.
void expect_solution_exported(const std::filesystem::path& directory) {
  EXPECT_TRUE(std::filesystem::exists(directory / "solutions" /
                                      "cli-test-solutions" / "cli-test" /
                                      "TestInstance" / "solution_data.json"));
}
} // namespace

TEST_F(CliTest, SolveWithTheAStarSearch) {
  cda_rail::cli::Session session{directory};
  build_solvable_instance(session);
  ASSERT_TRUE(session.const_instance().check_consistency(true));

  run(session, "solve mb-astar --dt 10 --export-solution "
               "--solution-export-subdirectory cli-test-solutions");

  expect_solution_exported(directory);
  // Solving must leave the session untouched and usable.
  EXPECT_EQ(session.const_instance().get_const_train_list().size(), 1);
  run(session, "instance check");
}

TEST_F(CliTest, SolveWithTheMovingBlockMip) {
  cda_rail::cli::Session session{directory};
  build_solvable_instance(session);

  run(session, "solve mb-mip -t 60 --export-solution "
               "--solution-export-subdirectory cli-test-solutions");

  expect_solution_exported(directory);
  EXPECT_EQ(session.const_instance().get_const_train_list().size(), 1);
}

TEST_F(CliTest, SolveTheVssGenerationMipOnAMovingBlockSolution) {
  // The two solvers are chained, which is what -m is for: the moving block
  // solution exported by the first run is read back by the second.
  cda_rail::cli::Session session{directory};
  build_solvable_instance(session);

  run(session, "solve mb-mip -t 60 --export-solution "
               "--solution-export-subdirectory mb-solutions");
  ASSERT_TRUE(std::filesystem::exists(directory / "solutions" / "mb-solutions" /
                                      "cli-test" / "TestInstance" /
                                      "solution_data.json"));

  // The exit time of the train is a multiple of the time discretization on
  // purpose: otherwise the VSS model reaches past the end of the moving block
  // solution and asks for a train that has already left.
  run(session, "solve vss-mip -c 15 -t 60 -m mb-solutions --export-solution "
               "--solution-export-subdirectory cli-test-solutions");
  expect_solution_exported(directory);
}

TEST_F(CliTest, SolveWithTheVssGenerationMip) {
  cda_rail::cli::Session session{directory};
  build_solvable_instance(session);

  run(session, "solve vss-mip -c 15 -t 60 --export-solution "
               "--solution-export-subdirectory cli-test-solutions");

  expect_solution_exported(directory);
  EXPECT_EQ(session.const_instance().get_const_train_list().size(), 1);
}

TEST_F(CliTest, HelpListsTheWholeCommandTree) {
  cda_rail::cli::Session session{directory};
  const auto             help = run_capturing(session, "help");
  // Not the help of the 'help' command itself, but every command group.
  EXPECT_NE(help.find("instance"), std::string::npos);
  EXPECT_NE(help.find("network"), std::string::npos);
  EXPECT_NE(help.find("train"), std::string::npos);
  EXPECT_NE(help.find("route"), std::string::npos);
  EXPECT_NE(help.find("solve"), std::string::npos);
}

TEST_F(CliTest, SavingAnInstanceUnderAnotherName) {
  cda_rail::cli::Session session{directory};
  run_all(session, SIMPLE_NETWORK);
  run(session, "network save");
  run(session, "instance new TestInstance -s cli-test -n TestNetwork");
  run(session, "instance save");
  run(session, "instance save --as OtherInstance");

  // The instance is renamed, so both the old and the new one are on disk.
  EXPECT_EQ(session.const_instance().get_instance_name(), "OtherInstance");
  EXPECT_TRUE(std::filesystem::exists(directory / "instances" / "cli-test" /
                                      "TestInstance"));
  EXPECT_TRUE(std::filesystem::exists(directory / "instances" / "cli-test" /
                                      "OtherInstance"));
}

TEST_F(CliTest, InstanceCommandsWithoutAnInstance) {
  cda_rail::cli::Session session{directory};
  // Every one of these has to report that nothing is loaded and leave the
  // session alone.
  EXPECT_THROW(session.reload_instance(),
               cda_rail::exceptions::InvalidInputException);
  EXPECT_THROW(session.close_instance(),
               cda_rail::exceptions::InvalidInputException);
  run(session, "instance reload");
  run(session, "instance close");
  run(session, "instance info");
  run(session, "instance check");
  EXPECT_FALSE(session.has_instance());

  // Loading a network while an instance is loaded would silently replace it.
  run_all(session, SIMPLE_NETWORK);
  run(session, "network save");
  run(session, "instance new TestInstance -s cli-test -n TestNetwork");
  EXPECT_THROW(session.load_network("TestNetwork"),
               cda_rail::exceptions::InvalidInputException);
  EXPECT_EQ(session.const_instance().get_instance_name(), "TestInstance");
}

TEST_F(CliTest, CheckReportsAnObviouslyInfeasibleInstance) {
  cda_rail::cli::Session session{directory};
  build_solvable_instance(session);
  // The network only runs from v0 to v2, so a train going the other way can
  // never reach its exit vertex. The reason is printed alongside the verdict.
  run(session, "train add ICE2 -l 50 -v 20 -a 1 -d 1 -e v2 -x v0 -y 200");

  const auto reported = run_capturing(session, "instance check -l");
  EXPECT_NE(reported.find("Obviously infeasible:            yes"),
            std::string::npos);
  EXPECT_NE(reported.find("ICE2"), std::string::npos);
}

TEST_F(CliTest, ListingSkipsWhatIsNotAnInstance) {
  cda_rail::cli::Session session{directory};
  const auto             instances = directory / "instances";
  std::filesystem::create_directories(instances / "cli-test" /
                                      "not-an-"
                                      "instance");
  // A loose file next to the subdirectories and a directory without a
  // network.json are both passed over.
  std::ofstream(instances / "stray.txt") << "ignored\n";

  const auto listed = run_capturing(session, "instance list");
  EXPECT_EQ(listed.find("not-an-instance"), std::string::npos);
  EXPECT_NE(listed.find("No instances found"), std::string::npos);
}

// ---------------------------------------------------------------------------
// Option parsing of the solver settings, which the three standalone apps and
// the solve subcommands share.
// ---------------------------------------------------------------------------

TEST(CliOptions, MbMipDelays) {
  cda_rail::cli::MbMipSettings settings;
  CLI::App                     app;
  cda_rail::cli::add_mb_mip_options(app, settings);
  app.parse(std::string{"--max-delay 42"});
  cda_rail::cli::finalize_mb_mip_settings(app, settings);

  EXPECT_DOUBLE_EQ(settings.max_exit_delay, 42);
  EXPECT_DOUBLE_EQ(settings.max_station_delay, 42);
}

TEST(CliOptions, MbMipIndividualDelays) {
  cda_rail::cli::MbMipSettings settings;
  CLI::App                     app;
  cda_rail::cli::add_mb_mip_options(app, settings);
  app.parse(std::string{"-x 10 --max-station-delay 20"});
  cda_rail::cli::finalize_mb_mip_settings(app, settings);

  EXPECT_DOUBLE_EQ(settings.max_exit_delay, 10);
  EXPECT_DOUBLE_EQ(settings.max_station_delay, 20);
}

TEST(CliOptions, MbMipMaxDelayExcludesIndividualDelays) {
  cda_rail::cli::MbMipSettings settings;
  CLI::App                     app;
  cda_rail::cli::add_mb_mip_options(app, settings);
  EXPECT_THROW(app.parse(std::string{"--max-delay 42 -x 10"}), CLI::ParseError);
}

TEST(CliOptions, MbMipStrategies) {
  cda_rail::cli::MbMipSettings settings;
  CLI::App                     app;
  cda_rail::cli::add_mb_mip_options(app, settings);
  app.parse(std::string{"-f -j AllChecked -k All -r None -z -a 0.5 -t 60 "
                        "--no-minimum-time-bounds"});

  EXPECT_TRUE(settings.fix_routes);
  EXPECT_EQ(
      settings.lazy_constraint_selection_strategy,
      cda_rail::solver::mip_based::LazyConstraintSelectionStrategy::AllChecked);
  EXPECT_EQ(settings.lazy_train_selection_strategy,
            cda_rail::solver::mip_based::LazyTrainSelectionStrategy::All);
  EXPECT_EQ(settings.velocity_refinement_strategy,
            cda_rail::VelocityRefinementStrategy::None);
  EXPECT_FALSE(settings.use_lazy_constraints);
  EXPECT_FALSE(settings.use_minimum_time_bounds);
  EXPECT_DOUBLE_EQ(settings.abs_mip_gap, 0.5);
  EXPECT_EQ(settings.solving.time_limit, 60);
}

TEST(CliOptions, ExportSubdirectoryIsRequired) {
  cda_rail::cli::MbMipSettings settings;
  CLI::App                     app;
  cda_rail::cli::add_mb_mip_options(app, settings);
  EXPECT_THROW(app.parse(std::string{"-o"}), CLI::ParseError);
}

TEST(CliOptions, ExportOptionsExcludeEachOther) {
  cda_rail::cli::MbMipSettings settings;
  CLI::App                     app;
  cda_rail::cli::add_mb_mip_options(app, settings);
  EXPECT_THROW(app.parse(std::string{"-o -i -e sub"}), CLI::ParseError);
}

TEST(CliOptions, ExportWorkingDirectoryNeedsAnExportOption) {
  cda_rail::cli::MbMipSettings settings;
  CLI::App                     app;
  cda_rail::cli::add_mb_mip_options(app, settings);
  EXPECT_THROW(app.parse(std::string{"-b elsewhere"}), CLI::ParseError);
}

TEST(CliOptions, ExportSettings) {
  cda_rail::cli::MbMipSettings settings;
  CLI::App                     app;
  cda_rail::cli::add_mb_mip_options(app, settings);
  app.parse(std::string{"-i -e my-solutions -b elsewhere -p ident"});

  EXPECT_EQ(settings.exporting.export_option,
            cda_rail::solver::GeneralExportOption::ExportSolutionWithInstance);
  EXPECT_EQ(settings.exporting.solution_subdirectory, "my-solutions");
  ASSERT_TRUE(settings.exporting.export_working_directory.has_value());
  EXPECT_EQ(settings.exporting.export_working_directory.value_or(""),
            "elsewhere");
  ASSERT_TRUE(settings.exporting.parameter_identifier.has_value());
  EXPECT_EQ(settings.exporting.parameter_identifier.value_or(""), "ident");

  const auto solution_settings = cda_rail::cli::general_solution_settings(
      settings.exporting, "the-working-directory");
  EXPECT_EQ(solution_settings.working_directory, "elsewhere");
  EXPECT_EQ(solution_settings.solution_subdirectory, "my-solutions");
}

TEST(CliOptions, ExportWorkingDirectoryDefaultsToTheWorkingDirectory) {
  cda_rail::cli::MbMipSettings settings;
  CLI::App                     app;
  cda_rail::cli::add_mb_mip_options(app, settings);
  app.parse(std::string{"-o -e my-solutions"});

  const auto solution_settings = cda_rail::cli::general_solution_settings(
      settings.exporting, "the-working-directory");
  EXPECT_EQ(solution_settings.working_directory, "the-working-directory");
}

TEST(CliOptions, MbMipGeneratedIdentifier) {
  cda_rail::cli::MbMipSettings settings;
  CLI::App                     app;
  cda_rail::cli::add_mb_mip_options(app, settings);
  app.parse(std::string{"-o -e my-solutions -g"});
  cda_rail::cli::finalize_mb_mip_settings(app, settings);

  ASSERT_TRUE(settings.exporting.parameter_identifier.has_value());
  // The defaults, in the order in which the identifier concatenates them.
  EXPECT_EQ(settings.exporting.parameter_identifier.value_or(""),
            "f_5.55_MinOneStep_f_f_f_t_86400_86400_f_t_f_f_OnlyViolated_"
            "OnlyAdjacent_10_-1");
  // The identifier ends up as a directory name.
  EXPECT_NO_THROW(cda_rail::exceptions::throw_if_invalid_folder_name(
      settings.exporting.parameter_identifier.value_or("")));
}

TEST(CliOptions, GeneratedIdentifierExcludesAnExplicitOne) {
  cda_rail::cli::MbMipSettings settings;
  CLI::App                     app;
  cda_rail::cli::add_mb_mip_options(app, settings);
  EXPECT_THROW(app.parse(std::string{"-o -e sub -g -p ident"}),
               CLI::ParseError);
}

TEST(CliOptions, MbAStarSettings) {
  cda_rail::cli::MbAStarSettings settings;
  CLI::App                       app;
  cda_rail::cli::add_mb_astar_options(app, settings);
  app.parse(std::string{"-c 3 -l -w 1.5 -x NextTTD -r Zero -a -f -y"});

  EXPECT_DOUBLE_EQ(settings.dt, 3);
  EXPECT_TRUE(settings.late_entry_possible);
  EXPECT_DOUBLE_EQ(settings.a_star_weight, 1.5);
  EXPECT_EQ(settings.next_state_strategy,
            cda_rail::solver::astar_based::NextStateStrategy::NextTTD);
  EXPECT_EQ(settings.remaining_time_heuristic_type,
            cda_rail::simulator::RemainingTimeHeuristicType::Zero);
  EXPECT_TRUE(settings.time_aware_state_transitions);
  EXPECT_FALSE(settings.limit_speed_by_leaving_edges);
  EXPECT_FALSE(settings.consider_earliest_exit);
}

TEST(CliOptions, MbAStarHasNoLpModelExport) {
  cda_rail::cli::MbAStarSettings settings;
  CLI::App                       app;
  cda_rail::cli::add_mb_astar_options(app, settings);
  EXPECT_THROW(app.parse(std::string{"--export-lp-model"}), CLI::ParseError);
}

TEST(CliOptions, MbAStarGeneratedIdentifier) {
  cda_rail::cli::MbAStarSettings settings;
  CLI::App                       app;
  cda_rail::cli::add_mb_astar_options(app, settings);
  app.parse(std::string{"-o -e my-solutions -g"});
  cda_rail::cli::finalize_mb_astar_settings(settings);

  ASSERT_TRUE(settings.exporting.parameter_identifier.has_value());
  EXPECT_EQ(settings.exporting.parameter_identifier.value_or(""),
            "6_f_t_t_f_1_SingleEdge_Simple_-1");
}

TEST(CliVssOptions, ModelAndSolverSettings) {
  cda_rail::cli::VssMipSettings settings;
  CLI::App                      app;
  cda_rail::cli::add_vss_mip_options(app, settings);
  app.parse(std::string{"-c 30 -f -a -k -r Inferred -j Uniform -j Chebyshev "
                        "--only-stop-at-vss --use-pwl --no-schedule-cuts "
                        "-x -q Feasible --iterative-update-strategy Relative "
                        "--iterative-initial-value 0.5 "
                        "--iterative-update-value 0.25 --no-iterative-cuts"});

  EXPECT_DOUBLE_EQ(settings.delta_t, 30);
  EXPECT_FALSE(settings.fix_routes);
  EXPECT_FALSE(settings.train_dynamics);
  EXPECT_FALSE(settings.braking_curves);
  EXPECT_EQ(settings.vss_model_type, cda_rail::vss::ModelType::Inferred);
  ASSERT_EQ(settings.separation_functions().size(), 2);
  EXPECT_EQ(settings.separation_functions().at(0).get_name(), "Uniform");
  EXPECT_EQ(settings.separation_functions().at(1).get_name(), "Chebyshev");
  EXPECT_TRUE(settings.only_stop_at_vss);
  EXPECT_TRUE(settings.use_pwl);
  EXPECT_FALSE(settings.use_schedule_cuts);
  EXPECT_TRUE(settings.iterative_approach);
  EXPECT_EQ(settings.optimality_strategy,
            cda_rail::OptimalityStrategy::Feasible);
  EXPECT_EQ(settings.iterative_update_strategy,
            cda_rail::solver::mip_based::UpdateStrategyVSSGen::Relative);
  EXPECT_DOUBLE_EQ(settings.iterative_initial_value, 0.5);
  EXPECT_DOUBLE_EQ(settings.iterative_update_value, 0.25);
  EXPECT_FALSE(settings.iterative_include_cuts);
  EXPECT_FALSE(settings.use_moving_block_information());
}

TEST(CliVssOptions, IterativeSettingsNeedTheIterativeApproach) {
  cda_rail::cli::VssMipSettings settings;
  CLI::App                      app;
  cda_rail::cli::add_vss_mip_options(app, settings);
  EXPECT_THROW(app.parse(std::string{"--iterative-initial-value 2"}),
               CLI::ParseError);
}

TEST(CliVssOptions, MovingBlockInformation) {
  cda_rail::cli::VssMipSettings settings;
  CLI::App                      app;
  cda_rail::cli::add_vss_mip_options(app, settings);
  app.parse(std::string{"-m mb-solutions --moving-block-working-directory "
                        "elsewhere --moving-block-parameter-identifier id -z "
                        "-l -y -u -w"});

  EXPECT_TRUE(settings.use_moving_block_information());
  EXPECT_EQ(settings.moving_block_solution_subdirectory, "mb-solutions");
  ASSERT_TRUE(settings.moving_block_working_directory.has_value());
  EXPECT_EQ(settings.moving_block_working_directory.value_or(""), "elsewhere");
  ASSERT_TRUE(settings.moving_block_parameter_identifier.has_value());
  EXPECT_EQ(settings.moving_block_parameter_identifier.value_or(""), "id");
  EXPECT_FALSE(settings.fix_stop_positions);
  EXPECT_FALSE(settings.fix_exact_positions);
  EXPECT_FALSE(settings.fix_exact_velocities);
  EXPECT_FALSE(settings.hint_approximate_positions);
  EXPECT_FALSE(settings.fix_order_on_edges);
}

TEST(CliVssOptions, MovingBlockSettingsNeedAMovingBlockSolution) {
  cda_rail::cli::VssMipSettings settings;
  CLI::App                      app;
  cda_rail::cli::add_vss_mip_options(app, settings);
  EXPECT_THROW(app.parse(std::string{"--moving-block-working-directory dir"}),
               CLI::ParseError);
}

TEST(CliVssOptions, FreeRoutesExcludeMovingBlockInformation) {
  cda_rail::cli::VssMipSettings settings;
  CLI::App                      app;
  cda_rail::cli::add_vss_mip_options(app, settings);
  EXPECT_THROW(app.parse(std::string{"-m mb-solutions -f"}), CLI::ParseError);
}

TEST(CliVssOptions, GeneratedIdentifier) {
  cda_rail::cli::VssMipSettings settings;
  CLI::App                      app;
  cda_rail::cli::add_vss_mip_options(app, settings);
  app.parse(std::string{"-o -e my-solutions -g"});
  cda_rail::cli::finalize_vss_mip_settings(settings);

  ASSERT_TRUE(settings.exporting.parameter_identifier.has_value());
  // The defaults, in the order in which the identifier concatenates them. The
  // trailing 't' is the fix_routes setting, which only exists without moving
  // block information.
  EXPECT_EQ(settings.exporting.parameter_identifier.value_or(""),
            "15_t_t_Continuous_f_f_t_f_Optimal_Fixed_1_2_t_f_-1_t");
}

TEST(CliVssOptions, GeneratedIdentifierWithMovingBlockInformation) {
  cda_rail::cli::VssMipSettings settings;
  CLI::App                      app;
  cda_rail::cli::add_vss_mip_options(app, settings);
  app.parse(std::string{"-o -e my-solutions -g -m mb-solutions"});
  cda_rail::cli::finalize_vss_mip_settings(settings);

  ASSERT_TRUE(settings.exporting.parameter_identifier.has_value());
  EXPECT_EQ(settings.exporting.parameter_identifier.value_or(""),
            "15_t_t_Continuous_f_f_t_f_Optimal_Fixed_1_2_t_f_-1_t_t_t_t_t");
}

TEST(CliVssOptions, PostprocessIsPartOfTheExportOptions) {
  cda_rail::cli::VssMipSettings settings;
  CLI::App                      app;
  cda_rail::cli::add_vss_mip_options(app, settings);
  app.parse(std::string{"-o -e my-solutions --postprocess --export-lp-model "
                        "--model-name my-model"});

  EXPECT_TRUE(settings.exporting.postprocess);
  EXPECT_TRUE(settings.exporting.export_lp_model);
  EXPECT_EQ(settings.exporting.model_name, "my-model");

  const auto solution_settings =
      cda_rail::cli::vss_mip_solution_settings(settings, "working");
  EXPECT_TRUE(solution_settings.postprocess);
  EXPECT_TRUE(solution_settings.export_lp_model);
  EXPECT_EQ(solution_settings.model_name, "my-model");
  EXPECT_EQ(solution_settings.working_directory, "working");
}

// ---------------------------------------------------------------------------
// The settings are handed to the solvers and logged. Both the standalone apps
// and the solve subcommands go through these functions, so they are checked
// here rather than through a solver run.
// ---------------------------------------------------------------------------

namespace {
// Raises the log severity so that the bodies of the logging statements run,
// and puts it back afterwards to keep the rest of the suite quiet.
void with_debug_logging(const std::function<void()>& what) {
  cda_rail::initialize_plog(true, true);
  what();
  cda_rail::initialize_plog(false, true);
}
} // namespace

TEST(CliOptions, MbMipSettingsReachTheSolver) {
  cda_rail::cli::MbMipSettings settings;
  CLI::App                     app;
  cda_rail::cli::add_mb_mip_options(app, settings);
  app.parse(std::string{"-f -m 3 -y -q -l -c -w -u -a 0.25 -x 10 "
                        "--max-station-delay 20 -t 30 -v -i -e sub "
                        "--export-lp-model --model-name my-model"});
  cda_rail::cli::finalize_mb_mip_settings(app, settings);

  const auto detail = cda_rail::cli::mb_mip_model_detail(settings);
  EXPECT_TRUE(detail.fix_routes);
  EXPECT_DOUBLE_EQ(detail.max_velocity_delta, 3);
  EXPECT_TRUE(detail.simplify_headway_constraints);
  EXPECT_TRUE(detail.strengthen_vertex_headway_constraints);
  EXPECT_TRUE(detail.allow_late_entry);
  EXPECT_DOUBLE_EQ(detail.max_exit_delay, 10);
  EXPECT_DOUBLE_EQ(detail.max_station_delay, 20);

  const auto strategy = cda_rail::cli::mb_mip_solver_strategy(settings);
  EXPECT_TRUE(strategy.use_indicator_constraints);
  EXPECT_TRUE(strategy.include_reverse_headways);
  EXPECT_TRUE(strategy.include_higher_velocities_in_edge_expr);
  EXPECT_DOUBLE_EQ(strategy.abs_mip_gap, 0.25);

  const auto solution =
      cda_rail::cli::mb_mip_solution_settings(settings, "the-directory");
  EXPECT_EQ(solution.export_option,
            cda_rail::solver::GeneralExportOption::ExportSolutionWithInstance);
  EXPECT_EQ(solution.working_directory, "the-directory");
  EXPECT_EQ(solution.solution_subdirectory, "sub");
  EXPECT_TRUE(solution.export_lp_model);
  EXPECT_EQ(solution.model_name, "my-model");

  EXPECT_EQ(settings.solving.time_limit, 30);
  EXPECT_TRUE(settings.solving.debug_output);

  with_debug_logging([&settings]() {
    cda_rail::cli::log_mb_mip_settings(settings, "the-directory");
  });
}

TEST(CliOptions, MbAStarSettingsReachTheSolver) {
  cda_rail::cli::MbAStarSettings settings;
  CLI::App                       app;
  cda_rail::cli::add_mb_astar_options(app, settings);
  app.parse(std::string{"-c 3 -l -f -y -a -w 1.5 -x NextTTD -r Zero"});
  cda_rail::cli::finalize_mb_astar_settings(settings);

  const auto detail = cda_rail::cli::mb_astar_model_detail(settings);
  EXPECT_DOUBLE_EQ(detail.dt, 3);
  EXPECT_TRUE(detail.late_entry_possible);
  EXPECT_FALSE(detail.limit_speed_by_leaving_edges);

  const auto strategy = cda_rail::cli::mb_astar_solver_strategy(settings);
  EXPECT_EQ(strategy.remaining_time_heuristic_type,
            cda_rail::simulator::RemainingTimeHeuristicType::Zero);
  EXPECT_EQ(strategy.next_state_strategy,
            cda_rail::solver::astar_based::NextStateStrategy::NextTTD);
  EXPECT_FALSE(strategy.consider_earliest_exit);
  EXPECT_TRUE(strategy.time_aware_state_transitions);
  EXPECT_DOUBLE_EQ(strategy.a_star_weight, 1.5);

  with_debug_logging([&settings]() {
    cda_rail::cli::log_mb_astar_settings(settings, "the-directory");
  });
}

TEST(CliOptions, InstanceExportAndSolutionAreLogged) {
  cda_rail::cli::InstanceSettings instance_settings;
  cda_rail::cli::MbMipSettings    settings;
  CLI::App                        app;
  cda_rail::cli::add_instance_options(app, instance_settings);
  cda_rail::cli::add_mb_mip_options(app, settings);
  app.parse(std::string{"-n SimpleStation -s atmos2023 -d data -o -e sub"});

  EXPECT_EQ(instance_settings.instance_name, "SimpleStation");
  EXPECT_EQ(instance_settings.instance_subdirectory, "atmos2023");
  EXPECT_EQ(instance_settings.working_directory, "data");

  with_debug_logging([&instance_settings, &settings]() {
    cda_rail::cli::log_instance_settings(instance_settings);
    cda_rail::cli::log_solving_settings(settings.solving);
    cda_rail::cli::log_export_settings(settings.exporting, "data");
    // Every status the solvers can report has its own line.
    for (const auto status :
         {cda_rail::SolutionStatus::Optimal, cda_rail::SolutionStatus::Feasible,
          cda_rail::SolutionStatus::Infeasible,
          cda_rail::SolutionStatus::Timeout,
          cda_rail::SolutionStatus::Unknown}) {
      cda_rail::cli::log_solution_status(status, 42);
    }
  });
}

TEST(CliOptions, AnUnknownEnumValueIsPrintedAsUnknown) {
  // The listings look a value up in the option map to print its name; a value
  // that is not in the map must not go unnoticed.
  EXPECT_EQ(cda_rail::cli::vertex_type_to_string(cda_rail::VertexType::VSS),
            "VSS");
  // The enum has a fixed underlying type, so a value without an enumerator is
  // well-defined; it is exactly what the lookup has to survive here.
  // NOLINTNEXTLINE(clang-analyzer-optin.core.EnumCastOutOfRange)
  const auto unknown_type = static_cast<cda_rail::VertexType>(42);
  EXPECT_EQ(cda_rail::cli::key_by_value(cda_rail::cli::vertex_type_map(),
                                        unknown_type),
            "unknown");
}

TEST(CliVssOptions, SettingsReachTheSolver) {
  cda_rail::cli::VssMipSettings settings;
  CLI::App                      app;
  cda_rail::cli::add_vss_mip_options(app, settings);
  app.parse(std::string{"-c 30 -f -a -k -r Inferred -j Uniform --use-pwl "
                        "--only-stop-at-vss --no-schedule-cuts -x "
                        "-q Feasible --iterative-update-strategy Relative "
                        "--iterative-initial-value 0.5 "
                        "--iterative-update-value 0.25 --no-iterative-cuts "
                        "-o -e sub"});
  cda_rail::cli::finalize_vss_mip_settings(settings);

  const auto detail = cda_rail::cli::vss_mip_model_detail(settings);
  EXPECT_DOUBLE_EQ(detail.delta_t, 30);
  EXPECT_FALSE(detail.fix_routes);
  EXPECT_FALSE(detail.train_dynamics);
  EXPECT_FALSE(detail.braking_curves);

  const auto model = cda_rail::cli::vss_mip_model_settings(settings);
  EXPECT_TRUE(model.use_pwl);
  EXPECT_FALSE(model.use_schedule_cuts);

  const auto strategy = cda_rail::cli::vss_mip_solver_strategy(settings);
  EXPECT_TRUE(strategy.iterative_approach);
  EXPECT_EQ(strategy.optimality_strategy,
            cda_rail::OptimalityStrategy::Feasible);
  EXPECT_EQ(strategy.update_strategy,
            cda_rail::solver::mip_based::UpdateStrategyVSSGen::Relative);
  EXPECT_DOUBLE_EQ(strategy.initial_value, 0.5);
  EXPECT_DOUBLE_EQ(strategy.update_value, 0.25);
  EXPECT_FALSE(strategy.include_cuts);

  with_debug_logging([&settings]() {
    cda_rail::cli::log_vss_mip_settings(settings, "the-directory");
  });
}

TEST(CliVssOptions, SettingsReachTheSolverUsingMovingBlockInformation) {
  cda_rail::cli::VssMipSettings settings;
  CLI::App                      app;
  cda_rail::cli::add_vss_mip_options(app, settings);
  app.parse(std::string{"-m mb-solutions -z -l -y -o -e sub"});
  cda_rail::cli::finalize_vss_mip_settings(settings);

  const auto detail = cda_rail::cli::vss_mip_model_detail_mb(settings);
  EXPECT_DOUBLE_EQ(detail.delta_t, 15);
  EXPECT_FALSE(detail.fix_stop_positions);
  EXPECT_FALSE(detail.fix_exact_positions);
  EXPECT_FALSE(detail.fix_exact_velocities);
  EXPECT_TRUE(detail.hint_approximate_positions);
  EXPECT_TRUE(detail.fix_order_on_edges);

  // The moving block branch of the logging prints the extra settings.
  with_debug_logging([&settings]() {
    cda_rail::cli::log_vss_mip_settings(settings, "the-directory");
  });
}

TEST(CliVssOptions, GeneratedIdentifierListsTheSeparationFunctions) {
  cda_rail::cli::VssMipSettings settings;
  CLI::App                      app;
  cda_rail::cli::add_vss_mip_options(app, settings);
  app.parse(std::string{"-o -e sub -g -r Inferred -j Uniform -j Chebyshev"});
  cda_rail::cli::finalize_vss_mip_settings(settings);

  ASSERT_TRUE(settings.exporting.parameter_identifier.has_value());
  const auto identifier = settings.exporting.parameter_identifier.value_or("");
  EXPECT_NE(identifier.find("Inferred_Uniform_Chebyshev"), std::string::npos);
  EXPECT_NO_THROW(
      cda_rail::exceptions::throw_if_invalid_folder_name(identifier));
}
