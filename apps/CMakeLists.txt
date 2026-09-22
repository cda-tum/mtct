# macro applying the link options a Windows executable needs to run without the MinGW runtime DLLs
macro(SET_STATIC_LINK_OPTIONS_ON_MINGW target)
  # Statically link libgcc and libstdc++ on MinGW to avoid DLL problems on Windows.
  if(MINGW)
    target_link_options(${target} PRIVATE "-static" "-static-libgcc" "-static-libstdc++")
    if(CMAKE_CXX_COMPILER_ID MATCHES ".*Clang")
      target_compile_definitions(${target} PRIVATE GXX_TYPEINFO_EQUALITY_INLINE=0)
    else()
      target_link_options(${target} PRIVATE "-Wl,--allow-multiple-definition")
    endif()
    # MinGW GCC 15+ updated libwinpthread/libmsvcrt with new symbol dependencies: libpthread.a needs
    # __intrinsic_setjmpex  (alias to _setjmpex in libmsvcrt.a) libmsvcrt.a  needs __ms_vsnprintf        (defined in
    # libmingwex.a) Both providing libs are already linked by MinGW but scanned before the libs that need them. Use
    # --defsym to alias __intrinsic_setjmpex to _setjmpex (same signature/semantics, already in msvcrt.dll) and
    # --undefined to force libmingwex to pull in the __ms_vsnprintf object early. GCC below 15 has neither symbol, and
    # --defsym then fails the link, so the workaround is skipped there.
    if(NOT (CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS "15.0"))
      target_link_options(${target} PRIVATE "-Wl,--defsym=__intrinsic_setjmpex=_setjmpex"
                          "-Wl,--undefined=__ms_vsnprintf")
    endif()
  endif()
endmacro()

# macro to add a executable with the given libraries
macro(ADD_SIM_EXECUTABLE appname)
  add_executable(${PROJECT_NAME}_${appname} ${CMAKE_CURRENT_SOURCE_DIR}/${appname}.cpp)
  target_link_libraries(${PROJECT_NAME}_${appname} PRIVATE ${PROJECT_NAME} ${ARGN})
  set_static_link_options_on_mingw(${PROJECT_NAME}_${appname})
endmacro()

# The interactive app; its library also holds the option blocks that the three apps below share with it.
add_subdirectory(cli)

add_sim_executable(gen_po_moving_block_astar_testing ${PROJECT_NAME}_cli_lib)
add_sim_executable(gen_po_moving_block_mip_testing ${PROJECT_NAME}_cli_lib)
add_sim_executable(vss_generation_timetable_mip_testing ${PROJECT_NAME}_cli_lib)
