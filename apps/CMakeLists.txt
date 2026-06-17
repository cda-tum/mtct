# macro to add a executable with the given libraries
macro(ADD_SIM_EXECUTABLE appname)
  add_executable(${PROJECT_NAME}_${appname} ${CMAKE_CURRENT_SOURCE_DIR}/${appname}.cpp)
  target_link_libraries(${PROJECT_NAME}_${appname} PRIVATE ${PROJECT_NAME} ${ARGN})

  # Statically link libgcc and libstdc++ on MinGW to avoid DLL problems on Windows.
  if(MINGW)
    target_link_options(${PROJECT_NAME}_${appname} PRIVATE "-static" "-static-libgcc" "-static-libstdc++")
    if(CMAKE_CXX_COMPILER_ID MATCHES ".*Clang")
      target_compile_definitions(${PROJECT_NAME}_${appname} PRIVATE GXX_TYPEINFO_EQUALITY_INLINE=0)
    else()
      target_link_options(${PROJECT_NAME}_${appname} PRIVATE "-Wl,--allow-multiple-definition")
    endif()
  endif()
endmacro()

add_sim_executable(gen_po_moving_block_astar_testing CLI11::CLI11)
