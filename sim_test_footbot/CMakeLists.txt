# compile test loop functions
add_library(footbot_drive_forward_dynamics2d_loop_functions MODULE
  loop_functions.h
  loop_functions.cpp)
target_link_libraries(footbot_drive_forward_dynamics2d_loop_functions
    argos3core_${ARGOS_BUILD_FOR}
    argos3plugin_${ARGOS_BUILD_FOR}_footbot)
# compile test controller
add_library(footbot_drive_forward_dynamics2d_controller MODULE
  controller.h
  controller.cpp)
target_link_libraries(footbot_drive_forward_dynamics2d_controller
    argos3core_${ARGOS_BUILD_FOR}
    argos3plugin_${ARGOS_BUILD_FOR}_footbot)
# configure experiment
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/conf_text.argos.in
  ${CMAKE_CURRENT_BINARY_DIR}/conf_text.argos)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/conf_visual.argos.in
  ${CMAKE_CURRENT_BINARY_DIR}/conf_visual.argos)
# define test
add_test(
   NAME footbot_drive_forward_dynamics2d
   WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
   COMMAND argos3 -zc configuration.argos)
set_tests_properties(footbot_drive_forward_dynamics2d
  PROPERTIES ENVIRONMENT "ARGOS_PLUGIN_PATH=${ARGOS_PLUGIN_PATH}")

