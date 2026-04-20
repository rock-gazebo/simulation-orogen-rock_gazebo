# Generated from orogen/lib/orogen/templates/tasks/CMakeLists.txt

find_package(gz-sim-physics-system REQUIRED)

include(rock_gazeboTaskLib)
ADD_LIBRARY(${ROCK_GAZEBO_TASKLIB_NAME} SHARED
    ${ROCK_GAZEBO_TASKLIB_SOURCES})
add_dependencies(${ROCK_GAZEBO_TASKLIB_NAME}
    regen-typekit)

TARGET_LINK_LIBRARIES(${ROCK_GAZEBO_TASKLIB_NAME}
    ${OrocosRTT_LIBRARIES}
    ${QT_LIBRARIES}
    ${ROCK_GAZEBO_TASKLIB_DEPENDENT_LIBRARIES}
    gz-sim::physics-system
)
SET_TARGET_PROPERTIES(${ROCK_GAZEBO_TASKLIB_NAME}
    PROPERTIES LINK_INTERFACE_LIBRARIES "${ROCK_GAZEBO_TASKLIB_INTERFACE_LIBRARIES}")
SET_TARGET_PROPERTIES(${ROCK_GAZEBO_TASKLIB_NAME}
    PROPERTIES INTERFACE_LINK_LIBRARIES "${ROCK_GAZEBO_TASKLIB_INTERFACE_LIBRARIES}")

orogen_apply_task_library_options(${ROCK_GAZEBO_TASKLIB_NAME})

INSTALL(TARGETS ${ROCK_GAZEBO_TASKLIB_NAME}
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib/orocos)

# If you are wondering. Yes. This really sucks badly
execute_process(
    COMMAND ${CMAKE_COMMAND} -E create_symlink
    ${CMAKE_CURRENT_SOURCE_DIR}/PluginTaskI.hpp
    ${CMAKE_SOURCE_DIR}/.orogen/rock_gazebo/PluginTaskI.hpp
)

INSTALL(FILES ${ROCK_GAZEBO_TASKLIB_HEADERS}
    PluginTaskI.hpp Helpers.hpp
    DESTINATION include/orocos/rock_gazebo)

