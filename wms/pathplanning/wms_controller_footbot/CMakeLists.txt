add_library(wms_controller_pathplanning SHARED wms_controller.h wms_controller.cpp)
add_library(pathplanning SHARED pathplanning.h pathplanning.cpp)
target_link_libraries(
  pathplanning
  wms_controller_pathplanning
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot)
