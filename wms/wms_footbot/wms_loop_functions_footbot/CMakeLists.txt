link_directories(${CMAKE_BINARY_DIR}/wms/wms_controllers/wms_controller)
set(wms_loop_functions_SOURCES wms_loop_functions.cpp)

if(ARGOS_QTOPENGL_FOUND)
  include_directories(${ARGOS_QTOPENGL_INCLUDE_DIRS})
  set(wms_loop_functions_SOURCES
    ${wms_loop_functions_SOURCES}
    wms_qt_user_functions.cpp)
endif(ARGOS_QTOPENGL_FOUND)

add_library(wms_loop_functions MODULE ${wms_loop_functions_SOURCES})
target_link_libraries(wms_loop_functions
  wms_controller
  argos3core_simulator
  argos3plugin_simulator_dynamics2d
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_media)

if(ARGOS_QTOPENGL_FOUND)
  target_link_libraries(wms_loop_functions argos3plugin_simulator_qtopengl)
endif(ARGOS_QTOPENGL_FOUND)
