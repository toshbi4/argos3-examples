link_directories(${CMAKE_BINARY_DIR}/myepuck)

set(custom_loop_functions_SOURCES
  custom_functions.h
  custom_functions.cpp)

add_library(custom_loop_functions MODULE
  ${custom_loop_functions_SOURCES})
target_link_libraries(custom_loop_functions
  argos3core_simulator
  argos3plugin_simulator_entities)
# argos3plugin_simulator_myepack
