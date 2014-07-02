cmake_minimum_required(VERSION 2.8.3)
project(beliefstate)

find_package(catkin REQUIRED COMPONENTS roscpp roslib designator_integration designator_integration_msgs sensor_msgs cv_bridge roslib interactive_markers visualization_msgs)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bs_plugin_ros bs_plugin_gazebo bs_plugin_console bs_plugin_experiment_knowledge bs_plugin_experiment_context bs_plugin_interactive bs_plugin_symboliclog bs_plugin_supervisor bs_plugin_owlexporter bs_plugin_dotexporter
  CATKIN_DEPENDS roscpp roslib designator_integration designator_integration_msgs sensor_msgs cv_bridge interactive_markers visualization_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include)

add_library(bs_plugin_ros SHARED
  src/GlobalFunctions.cpp
  src/Plugin.cpp
  src/plugins/ros/PluginROS.cpp)

add_library(bs_plugin_prediction SHARED
  src/GlobalFunctions.cpp
  src/Plugin.cpp
  src/plugins/prediction/PluginPrediction.cpp
  src/plugins/owlexporter/CExporterOwl.cpp
  src/CExporter.cpp
  src/CExporterFileoutput.cpp
  src/Node.cpp
  src/JSON.cpp
  src/Property.cpp)

add_library(bs_plugin_gazebo SHARED
  src/GlobalFunctions.cpp
  src/Plugin.cpp
  src/plugins/gazebo/PluginGazebo.cpp)

add_library(bs_plugin_console SHARED
  src/GlobalFunctions.cpp
  src/Plugin.cpp
  src/plugins/console/PluginConsole.cpp)

add_library(bs_plugin_experiment_knowledge SHARED
  src/GlobalFunctions.cpp
  src/Plugin.cpp
  src/plugins/experiment-knowledge/PluginExperimentKnowledge.cpp)

add_library(bs_plugin_experiment_context SHARED
  src/GlobalFunctions.cpp
  src/CExporter.cpp
  src/CExporterFileoutput.cpp
  src/Plugin.cpp
  src/plugins/experiment-context/PluginExperimentContext.cpp)

add_library(bs_plugin_interactive SHARED
  src/GlobalFunctions.cpp
  src/Plugin.cpp
  src/plugins/interactive/PluginInteractive.cpp
  src/plugins/interactive/InteractiveObject.cpp)

add_library(bs_plugin_symboliclog SHARED
  src/GlobalFunctions.cpp
  src/Plugin.cpp
  src/Node.cpp
  src/plugins/symboliclog/PluginSymbolicLog.cpp)

add_library(bs_plugin_supervisor SHARED
  src/GlobalFunctions.cpp
  src/Plugin.cpp
  src/plugins/supervisor/PluginSupervisor.cpp)

add_library(bs_plugin_imagecapturer SHARED
  src/GlobalFunctions.cpp
  src/Plugin.cpp
  src/plugins/imagecapturer/CImageCapturer.cpp
  src/plugins/imagecapturer/PluginImageCapturer.cpp)

add_library(bs_plugin_owlexporter SHARED
  src/GlobalFunctions.cpp
  src/Plugin.cpp
  src/Node.cpp
  src/CExporter.cpp
  src/CExporterFileoutput.cpp
  src/plugins/owlexporter/CExporterOwl.cpp
  src/plugins/owlexporter/PluginOWLExporter.cpp)

add_library(bs_plugin_dotexporter SHARED
  src/GlobalFunctions.cpp
  src/Plugin.cpp
  src/Node.cpp
  src/CExporter.cpp
  src/CExporterFileoutput.cpp
  src/plugins/dotexporter/CExporterDot.cpp
  src/plugins/dotexporter/PluginDOTExporter.cpp)

add_executable(beliefstate
  src/UtilityBase.cpp
  src/main.cpp
  src/GlobalFunctions.cpp
  src/PluginSystem.cpp
  src/PluginInstance.cpp
  src/Plugin.cpp
  src/Node.cpp
  src/Beliefstate.cpp
  src/BeliefstateROS.cpp)

add_dependencies(bs_plugin_ros designator_integration_generate_messages_cpp)
add_dependencies(beliefstate designator_integration_generate_messages_cpp)

target_link_libraries(beliefstate
  ${catkin_LIBRARIES}
  dl
  config++
)

target_link_libraries(bs_plugin_ros
  ${catkin_LIBRARIES}
)

target_link_libraries(bs_plugin_prediction
  ${catkin_LIBRARIES}
  json
)

target_link_libraries(bs_plugin_gazebo
  ${catkin_LIBRARIES}
)

target_link_libraries(bs_plugin_console
  ${catkin_LIBRARIES}
  ncurses
)

target_link_libraries(bs_plugin_experiment_knowledge
  ${catkin_LIBRARIES}
)

target_link_libraries(bs_plugin_experiment_context
  ${catkin_LIBRARIES}
)

target_link_libraries(bs_plugin_interactive
  ${catkin_LIBRARIES}
)

target_link_libraries(bs_plugin_owlexporter
  ${catkin_LIBRARIES}
  config++
)

target_link_libraries(bs_plugin_dotexporter
  ${catkin_LIBRARIES}
)

target_link_libraries(bs_plugin_imagecapturer
  ${catkin_LIBRARIES}
)

target_link_libraries(bs_plugin_supervisor
  ${catkin_LIBRARIES}
)

install(TARGETS beliefstate
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
