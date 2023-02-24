macro(add_rail_executable APPNAME)
    add_executable(${PROJECT_NAME}_${APPNAME} ${CMAKE_CURRENT_SOURCE_DIR}/${APPNAME}.cpp)
    target_link_libraries(${PROJECT_NAME}_${APPNAME} PUBLIC ${PROJECT_NAME})
    add_custom_command(TARGET ${PROJECT_NAME}_${APPNAME}
            POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E create_symlink $<TARGET_FILE_DIR:${PROJECT_NAME}_${APPNAME}>/${PROJECT_NAME}_${APPNAME} ${CMAKE_BINARY_DIR}/${PROJECT_NAME}_${APPNAME}
            COMMENT "Creating symlink for ${PROJECT_NAME}_${APPNAME}"
            VERBATIM)

    set_target_properties(${PROJECT_NAME}_${APPNAME} PROPERTIES EXPORT_NAME ${PROJECT_NAME}_${APPNAME})
endmacro()

add_rail_executable(main)