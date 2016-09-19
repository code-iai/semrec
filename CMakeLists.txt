cmake_minimum_required(VERSION 2.8.3)
project(semrec)

find_package(catkin REQUIRED COMPONENTS roscpp roslib rospack designator_integration_cpp designator_integration_msgs sensor_msgs cv_bridge interactive_markers visualization_msgs)

set(COMPILE_DEBUG 0)

if(CMAKE_COMPILER_IS_GNUCXX)
  if(COMPILE_DEBUG)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pg")
  else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2")
  endif()
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sr_plugin_ros sr_plugin_symboliclog sr_plugin_owlexporter sr_plugin_dotexporter
  CATKIN_DEPENDS roscpp roslib designator_integration_cpp designator_integration_msgs sensor_msgs cv_bridge interactive_markers visualization_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include)

add_library(sr_base_plugin
  src/GlobalFunctions.cpp
  src/Plugin.cpp)

add_library(sr_exporter_plugin
  src/CExporter.cpp
  src/CExporterFileoutput.cpp)


set_target_properties(sr_base_plugin PROPERTIES LINKER_LANGUAGE C)
set_target_properties(sr_exporter_plugin PROPERTIES LINKER_LANGUAGE C)


target_link_libraries(sr_base_plugin
  ${catkin_LIBRARIES}
  config++
)

add_library(sr_plugin_prediction SHARED
  src/plugins/prediction/PluginPrediction.cpp
  src/plugins/prediction/DecisionTree.cpp
  src/plugins/owlexporter/CExporterOwl.cpp
  src/plugins/owlexporter/OwlIndividual.cpp
  src/CExporter.cpp
  src/CExporterFileoutput.cpp
  src/Node.cpp
  src/JSON.cpp
  src/Property.cpp)

add_library(sr_plugin_ros SHARED
  src/plugins/ros/PluginROS.cpp)

add_library(sr_plugin_symboliclog SHARED
  src/Node.cpp
  src/plugins/symboliclog/PluginSymbolicLog.cpp)

add_library(sr_plugin_owlexporter SHARED
  src/Node.cpp
  src/CExporter.cpp
  src/CExporterFileoutput.cpp
  src/plugins/owlexporter/CExporterOwl.cpp
  src/plugins/owlexporter/OwlIndividual.cpp
  src/plugins/owlexporter/PluginOWLExporter.cpp)

add_library(sr_plugin_dotexporter SHARED
  src/Node.cpp
  src/CExporter.cpp
  src/CExporterFileoutput.cpp
  src/plugins/dotexporter/CExporterDot.cpp
  src/plugins/dotexporter/PluginDOTExporter.cpp)

add_executable(semrec
  src/UtilityBase.cpp
  src/ArbitraryMappingsHolder.cpp
  src/main.cpp
  src/PluginSystem.cpp
  src/PluginInstance.cpp
  src/Node.cpp
  src/SemanticHierarchyRecorder.cpp
  src/SemanticHierarchyRecorderROS.cpp)

target_link_libraries(semrec
  sr_base_plugin
  dl)

target_link_libraries(sr_exporter_plugin
  sr_base_plugin)

target_link_libraries(sr_plugin_ros
  sr_base_plugin)

target_link_libraries(sr_plugin_prediction
  sr_base_plugin)

target_link_libraries(sr_plugin_owlexporter
  sr_exporter_plugin)

target_link_libraries(sr_plugin_dotexporter
  sr_exporter_plugin)

install(TARGETS semrec
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)
