add_library(marching_qt_user_functions MODULE 
  marching_qt_user_functions.h
  marching_qt_user_functions.cpp)

target_link_libraries(marching_qt_user_functions
  argos3core_simulator
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot
  argos3plugin_simulator_qtopengl
  ${ARGOS_QTOPENGL_LIBRARIES})
