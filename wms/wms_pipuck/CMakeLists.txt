# compile test loop functions
add_library(wms_pipuck_loop_functions MODULE
  loop_functions.h
  loop_functions.cpp)
target_link_libraries(wms_pipuck_loop_functions
    argos3core_${ARGOS_BUILD_FOR}
    argos3plugin_${ARGOS_BUILD_FOR}_footbot)
# compile test controller
add_library(wms_pipuck_controller MODULE
  controller.h
  controller.cpp)
target_link_libraries(wms_pipuck_controller
    argos3core_${ARGOS_BUILD_FOR}
    argos3plugin_${ARGOS_BUILD_FOR}_footbot)

