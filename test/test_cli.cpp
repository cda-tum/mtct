#include "CLI/CLI.hpp"
#include "CommandInterpreter.hpp"
#include "CustomExceptions.hpp"
#include "Definitions.hpp"
#include "Session.hpp"
#include "VSSModel.hpp"
#include "datastructure/RailwayNetwork.hpp"
#include "options_common.hpp"
#include "options_mb_astar.hpp"
#include "options_mb_mip.hpp"
#include "options_vss_mip.hpp"
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"

#include "gtest/gtest.h"
#include <filesystem>
#include <iostream>
#include <istream>
#include <sstream>
#include <string>
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

/** @brief Runs one line through a fresh interpreter, as the app does. */
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

/** @brief Runs one line and returns what it printed. */
std::string run_capturing(cda_rail::cli::Session& session,
                          const std::string&      line) {
  std::ostringstream output;
  auto* const        previous = std::cout.rdbuf(output.rdbuf());
  run(session, line);
  std::cout.rdbuf(previous);
  return output.str();
}

/**
 * @brief Feeds a whole script to the read loop, as the app does.
 *
 * @return `false` if the loop was ended by `exit`, `true` if it ran out of
 *         input.
 */
bool run_script(cda_rail::cli::Session& session, const std::string& script) {
  std::istringstream input{script};
  std::ostringstream output;
  auto* const        previous = std::cout.rdbuf(output.rdbuf());
  const bool         keep_going =
      cda_rail::cli::run_stream(input, session, false, false);
  std::cout.rdbuf(previous);
  return keep_going;
}

/** @brief The commands building a tiny two-vertex network. */
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
  EXPECT_EQ(settings.exporting.export_working_directory.value(), "elsewhere");
  ASSERT_TRUE(settings.exporting.parameter_identifier.has_value());
  EXPECT_EQ(settings.exporting.parameter_identifier.value(), "ident");

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
  EXPECT_EQ(settings.exporting.parameter_identifier.value(),
            "f_5.55_MinOneStep_f_f_f_t_86400_86400_f_t_f_f_OnlyViolated_"
            "OnlyAdjacent_10_-1");
  // The identifier ends up as a directory name.
  EXPECT_NO_THROW(cda_rail::exceptions::throw_if_invalid_folder_name(
      settings.exporting.parameter_identifier.value()));
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
  EXPECT_EQ(settings.exporting.parameter_identifier.value(),
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
  EXPECT_EQ(settings.moving_block_working_directory.value(), "elsewhere");
  ASSERT_TRUE(settings.moving_block_parameter_identifier.has_value());
  EXPECT_EQ(settings.moving_block_parameter_identifier.value(), "id");
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
  EXPECT_EQ(settings.exporting.parameter_identifier.value(),
            "15_t_t_Continuous_f_f_t_f_Optimal_Fixed_1_2_t_f_-1_t");
}

TEST(CliVssOptions, GeneratedIdentifierWithMovingBlockInformation) {
  cda_rail::cli::VssMipSettings settings;
  CLI::App                      app;
  cda_rail::cli::add_vss_mip_options(app, settings);
  app.parse(std::string{"-o -e my-solutions -g -m mb-solutions"});
  cda_rail::cli::finalize_vss_mip_settings(settings);

  ASSERT_TRUE(settings.exporting.parameter_identifier.has_value());
  EXPECT_EQ(settings.exporting.parameter_identifier.value(),
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
