cmake_minimum_required(VERSION 2.8.3)
project(beliefstate)

find_package(catkin REQUIRED COMPONENTS roscpp roslib designator_integration designator_integration_msgs sensor_msgs cv_bridge roslib interactive_markers visualization_msgs)

if(CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -O2 -pg")
endif()

catkin_package(
  INCLUDE_DIRS include 3rdparty
  LIBRARIES bs_plugin_ros bs_plugin_gazebo bs_plugin_console bs_plugin_experiment_context bs_plugin_interactive bs_plugin_symboliclog bs_plugin_supervisor bs_plugin_owlexporter bs_plugin_dotexporter
  CATKIN_DEPENDS roscpp roslib designator_integration designator_integration_msgs sensor_msgs cv_bridge interactive_markers visualization_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  3rdparty)

add_library(bs_aux_c50
  3rdparty/C50/attwinnow.c
  3rdparty/C50/classify.c
  3rdparty/C50/confmat.c
  3rdparty/C50/construct.c
  3rdparty/C50/contin.c
  3rdparty/C50/discr.c
  3rdparty/C50/formrules.c
  3rdparty/C50/formtree.c
  3rdparty/C50/getdata.c
  3rdparty/C50/mcost.c
  3rdparty/C50/modelfiles.c
  3rdparty/C50/prune.c
  3rdparty/C50/p-thresh.c
#  3rdparty/C50/report.c
  3rdparty/C50/rules.c
  3rdparty/C50/ruletree.c
  3rdparty/C50/siftrules.c
  3rdparty/C50/sort.c
  3rdparty/C50/subset.c
  3rdparty/C50/trees.c
  3rdparty/C50/update.c
#  3rdparty/C50/xval.c
  3rdparty/C50/utility.c)

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
  src/plugins/owlexporter/CExporterOwl.cpp
  src/CExporter.cpp
  src/CExporterFileoutput.cpp
  src/Node.cpp
  src/JSON.cpp
  src/Property.cpp)

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
  json
  bs_aux_c50
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
