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
    # MinGW GCC 15+ updated libwinpthread/libmsvcrt with new symbol dependencies: libpthread.a needs
    # __intrinsic_setjmpex  (alias to _setjmpex in libmsvcrt.a) libmsvcrt.a  needs __ms_vsnprintf        (defined in
    # libmingwex.a) Both providing libs are already linked by MinGW but scanned before the libs that need them. Use
    # --defsym to alias __intrinsic_setjmpex to _setjmpex (same signature/semantics, already in msvcrt.dll) and
    # --undefined to force libmingwex to pull in the __ms_vsnprintf object early.
    target_link_options(${PROJECT_NAME}_${appname} PRIVATE "-Wl,--defsym=__intrinsic_setjmpex=_setjmpex"
                        "-Wl,--undefined=__ms_vsnprintf")
  endif()
endmacro()

# add_sim_executable(vss_generation_timetable_mip_testing)
# add_sim_executable(vss_generation_timetable_mip_iterative_vss_testing)
# add_sim_executable(vss_generation_timetable_iterative_parameter_testing)
# add_sim_executable(vss_generation_timetable_using_mb_information_testing)
# add_sim_executable(gen_po_moving_block_lazy_vss_gen_testing) add_sim_executable(gen_po_moving_block_lazy_testing)
# add_sim_executable(gen_po_moving_block_simplified_vss_gen_testing)
# add_sim_executable(gen_po_moving_block_simplified_testing)
add_sim_executable(gen_po_moving_block_astar_testing CLI11::CLI11)
