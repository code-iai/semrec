cmake_minimum_required(VERSION 2.8.3)
project(beliefstate)

find_package(catkin REQUIRED COMPONENTS roscpp roslib rospack designator_integration designator_integration_msgs sensor_msgs cv_bridge interactive_markers visualization_msgs)

add_subdirectory(3rdparty)

set(COMPILE_DEBUG 0)

if(CMAKE_COMPILER_IS_GNUCXX)
  if(COMPILE_DEBUG)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -pg")
  else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -O2")
  endif()
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bs_plugin_ros bs_plugin_gazebo bs_plugin_console bs_plugin_experiment_context bs_plugin_interactive bs_plugin_symboliclog bs_plugin_supervisor bs_plugin_owlexporter bs_plugin_dotexporter
  CATKIN_DEPENDS roscpp roslib designator_integration designator_integration_msgs sensor_msgs cv_bridge interactive_markers visualization_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  3rdparty)

add_library(bs_base_plugins)
set_target_properties(bs_base_plugins PROPERTIES LINKER_LANGUAGE C)


target_link_libraries(bs_base_plugins
  ${catkin_LIBRARIES}
  config++
)

add_library(bs_plugin_prediction SHARED
  src/GlobalFunctions.cpp
  src/Plugin.cpp
  src/plugins/prediction/PluginPrediction.cpp
  src/plugins/prediction/DecisionTree.cpp
  src/plugins/owlexporter/CExporterOwl.cpp
  src/CExporter.cpp
  src/CExporterFileoutput.cpp
  src/Node.cpp
  src/JSON.cpp
  src/Property.cpp)

add_dependencies(bs_plugin_prediction
  git-clone-json-c)

add_library(bs_plugin_ros SHARED
  src/GlobalFunctions.cpp
  src/Plugin.cpp
  src/plugins/ros/PluginROS.cpp)

add_library(bs_plugin_gazebo SHARED
  src/GlobalFunctions.cpp
  src/Plugin.cpp
  src/plugins/gazebo/PluginGazebo.cpp)

add_library(bs_plugin_console SHARED
  src/GlobalFunctions.cpp
  src/Plugin.cpp
  src/plugins/console/PluginConsole.cpp)

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
  src/ArbitraryMappingsHolder.cpp
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
  bs_base_plugins
  dl
)

target_link_libraries(bs_plugin_ros
  bs_base_plugins
)

target_link_libraries(bs_plugin_prediction
  bs_base_plugins
  ${PROJECT_SOURCE_DIR}/3rdparty/json-c/.libs/libjson-c.so
)

target_link_libraries(bs_plugin_gazebo
  bs_base_plugins
)

target_link_libraries(bs_plugin_console
  bs_base_plugins
  ncurses
)

target_link_libraries(bs_plugin_experiment_context
  bs_base_plugins
)

target_link_libraries(bs_plugin_interactive
  bs_base_plugins
)

target_link_libraries(bs_plugin_owlexporter
  bs_base_plugins
)

target_link_libraries(bs_plugin_dotexporter
  bs_base_plugins
)

target_link_libraries(bs_plugin_imagecapturer
  bs_base_plugins
)

target_link_libraries(bs_plugin_supervisor
  bs_base_plugins
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
