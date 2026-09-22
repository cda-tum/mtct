# The command logic lives in a library so that the tests can drive the interpreter directly instead of launching a
# subprocess.
add_library(
  ${PROJECT_NAME}_cli_lib
  ${CMAKE_CURRENT_SOURCE_DIR}/Session.hpp
  Session.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/CommandInterpreter.hpp
  CommandInterpreter.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Formatting.hpp
  Formatting.cpp
  commands_session.cpp
  commands_instance.cpp
  commands_network.cpp
  commands_timetable.cpp
  commands_route.cpp
  commands_solve.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/options_common.hpp
  options_common.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/options_mb_mip.hpp
  options_mb_mip.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/options_mb_astar.hpp
  options_mb_astar.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/options_vss_mip.hpp
  options_vss_mip.cpp)

target_include_directories(${PROJECT_NAME}_cli_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(${PROJECT_NAME}_cli_lib PUBLIC ${PROJECT_NAME} CLI11::CLI11)

add_executable(${PROJECT_NAME}_cli ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)
target_link_libraries(${PROJECT_NAME}_cli PRIVATE ${PROJECT_NAME}_cli_lib)
set_static_link_options_on_mingw(${PROJECT_NAME}_cli)
