cmake_minimum_required(VERSION 2.8.3)
project(semrec)

find_package(catkin REQUIRED COMPONENTS roscpp roslib rospack designator_integration_cpp designator_integration_msgs sensor_msgs cv_bridge interactive_markers visualization_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS roscpp roslib designator_integration_cpp designator_integration_msgs sensor_msgs cv_bridge interactive_markers visualization_msgs)

set(COMPILE_DEBUG 0)

if(CMAKE_COMPILER_IS_GNUCXX)
  if(COMPILE_DEBUG)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pg")
  else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2")
  endif()
endif()

include_directories(
  ${catkin_INCLUDE_DIRS}
  include)

set(PLUGINS_SOURCE_DIRECTORY "${PROJECT_SOURCE_DIR}/src/plugins/")


macro(add_semrec_plugin)
  set(PLUGIN_NAME "sr_plugin_${ARGN}")
  
  message(STATUS "Adding semrec plugin '${ARGN}', identifier '${PLUGIN_NAME}'")
  
  file(GLOB_RECURSE LIB_SOURCE "${PLUGINS_SOURCE_DIRECTORY}/${ARGN}/*.cpp")
  
  add_library(${PLUGIN_NAME} SHARED
    ${LIB_SOURCE})
  
  set_target_properties(${PLUGIN_NAME} PROPERTIES LINKER_LANGUAGE C)
  
  target_link_libraries(${PLUGIN_NAME}
    sr_exporter_plugin)
endmacro(add_semrec_plugin)

add_library(sr_base_plugin
  src/GlobalFunctions.cpp
  src/Plugin.cpp
  src/Node.cpp)

add_library(sr_exporter_plugin
  src/CExporter.cpp
  src/CExporterFileoutput.cpp)


set_target_properties(sr_base_plugin PROPERTIES LINKER_LANGUAGE C)
set_target_properties(sr_exporter_plugin PROPERTIES LINKER_LANGUAGE C)


target_link_libraries(sr_base_plugin
  ${catkin_LIBRARIES}
  config++)

target_link_libraries(sr_exporter_plugin
  sr_base_plugin)

add_semrec_plugin(ros)
add_semrec_plugin(owlexporter)
add_semrec_plugin(dotexporter)
add_semrec_plugin(symboliclog)

add_library(sr_plugin_prediction SHARED
  src/plugins/prediction/PluginPrediction.cpp
  src/plugins/prediction/DecisionTree.cpp
  src/plugins/owlexporter/CExporterOwl.cpp
  src/plugins/owlexporter/OwlIndividual.cpp
  src/JSON.cpp
  src/Property.cpp)

add_executable(semrec
  src/UtilityBase.cpp
  src/ArbitraryMappingsHolder.cpp
  src/main.cpp
  src/Node.cpp
  src/PluginSystem.cpp
  src/PluginInstance.cpp
  src/SemanticHierarchyRecorder.cpp
  src/SemanticHierarchyRecorderROS.cpp)

target_link_libraries(semrec
  sr_exporter_plugin
  dl)

target_link_libraries(sr_plugin_prediction
  sr_exporter_plugin)

install(TARGETS semrec
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)
