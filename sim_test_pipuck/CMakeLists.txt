# compile test loop functions
add_library(pipuck_drive_forward_dynamics_loop_functions MODULE
  loop_functions.h
  loop_functions.cpp)
target_link_libraries(pipuck_drive_forward_dynamics_loop_functions
    argos3core_${ARGOS_BUILD_FOR}
    argos3plugin_${ARGOS_BUILD_FOR}_footbot)
# compile test controller
add_library(pipuck_drive_forward_dynamics_controller MODULE
  controller.h
  controller.cpp)
target_link_libraries(pipuck_drive_forward_dynamics_controller
    argos3core_${ARGOS_BUILD_FOR}
    argos3plugin_${ARGOS_BUILD_FOR}_footbot)
# configure experiment
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/conf_text2D.argos.in
  ${CMAKE_CURRENT_BINARY_DIR}/conf_text2D.argos)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/conf_visual2D.argos.in
  ${CMAKE_CURRENT_BINARY_DIR}/conf_visual2D.argos)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/conf_text3D.argos.in
  ${CMAKE_CURRENT_BINARY_DIR}/conf_text3D.argos)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/conf_visual3D.argos.in
  ${CMAKE_CURRENT_BINARY_DIR}/conf_visual3D.argos)

