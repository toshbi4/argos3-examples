add_library(argos3plugin_simulator_myepuck SHARED
  simulator/dynamics2d_myepuck_model.cpp
  simulator/dynamics2d_myepuck_model.h
  simulator/myepuck_entity.cpp
  simulator/myepuck_entity.h
  simulator/qtopengl_myepuck.cpp
  simulator/qtopengl_myepuck.h)
target_link_libraries(argos3plugin_simulator_myepuck
    argos3core_simulator
    argos3plugin_simulator_dynamics2d
    argos3plugin_simulator_entities
    argos3plugin_simulator_genericrobot
    argos3plugin_simulator_media)
if(ARGOS_COMPILE_QTOPENGL)
  target_link_libraries(argos3plugin_simulator_myepuck
    argos3plugin_simulator_qtopengl
    ${QT_LIBRARIES} ${GLUT_LIBRARY} ${OPENGL_LIBRARY})
endif(ARGOS_COMPILE_QTOPENGL)
