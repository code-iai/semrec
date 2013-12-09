#ifndef __PLUGIN_DOTEXPORTER_H__
#define __PLUGIN_DOTEXPORTER_H__


// System
#include <cstdlib>
#include <iostream>

// ROS
#include <ros/ros.h>

// Designators
#include <designators/CDesignator.h>
#include <designator_integration_msgs/DesignatorCommunication.h>

// Private
#include <Types.h>
#include <ForwardDeclarations.h>
#include <Plugin.h>
#include <plugins/dotexporter/CExporterDot.h>

using namespace std;


namespace beliefstate {
  namespace plugins {
    class PluginDOTExporter : public Plugin {
    private:
    public:
      PluginDOTExporter();
      ~PluginDOTExporter();
      
      virtual Result init(int argc, char** argv);
      virtual Result deinit();
      
      virtual Result cycle();
      
      virtual void consumeEvent(Event evEvent);
      Event consumeServiceEvent(ServiceEvent seServiceEvent);
    };
  }
  
  extern "C" plugins::PluginDOTExporter* createInstance();
  extern "C" void destroyInstance(plugins::PluginDOTExporter* icDestroy);
}


#endif /* __PLUGIN_DOTEXPORTER_H__ */
