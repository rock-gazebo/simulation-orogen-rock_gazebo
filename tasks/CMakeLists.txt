# Generated from orogen/lib/orogen/templates/tasks/CMakeLists.txt

find_package(gz-sim-physics-system REQUIRED)

include(gz_rockTaskLib)
ADD_LIBRARY(${GZ_ROCK_TASKLIB_NAME} SHARED
    ${GZ_ROCK_TASKLIB_SOURCES})
add_dependencies(${GZ_ROCK_TASKLIB_NAME}
    regen-typekit)

TARGET_LINK_LIBRARIES(${GZ_ROCK_TASKLIB_NAME}
    ${OrocosRTT_LIBRARIES}
    ${QT_LIBRARIES}
    ${GZ_ROCK_TASKLIB_DEPENDENT_LIBRARIES}
    gz-sim::physics-system
)
SET_TARGET_PROPERTIES(${GZ_ROCK_TASKLIB_NAME}
    PROPERTIES LINK_INTERFACE_LIBRARIES "${GZ_ROCK_TASKLIB_INTERFACE_LIBRARIES}")
SET_TARGET_PROPERTIES(${GZ_ROCK_TASKLIB_NAME}
    PROPERTIES INTERFACE_LINK_LIBRARIES "${GZ_ROCK_TASKLIB_INTERFACE_LIBRARIES}")

TARGET_COMPILE_FEATURES(${GZ_ROCK_TASKLIB_NAME} PUBLIC cxx_std_17)
target_compile_options(${GZ_ROCK_TASKLIB_NAME} PUBLIC "-std=c++17")

INSTALL(TARGETS ${GZ_ROCK_TASKLIB_NAME}
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib/orocos)

# If you are wondering. Yes. This really sucks badly
execute_process(
    COMMAND ${CMAKE_COMMAND} -E create_symlink
    ${CMAKE_CURRENT_SOURCE_DIR}/PluginTaskI.hpp
    ${CMAKE_SOURCE_DIR}/.orogen/gz_rock/PluginTaskI.hpp
)

INSTALL(FILES ${GZ_ROCK_TASKLIB_HEADERS}
    PluginTaskI.hpp
    DESTINATION include/orocos/gz_rock)

