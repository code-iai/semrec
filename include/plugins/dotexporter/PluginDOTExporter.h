#ifndef __PLUGIN_DOTEXPORTER_H__
#define __PLUGIN_DOTEXPORTER_H__


#define PLUGIN_CLASS PluginDotExporter


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
    class PLUGIN_CLASS : public Plugin {
    private:
    public:
      PLUGIN_CLASS();
      ~PLUGIN_CLASS();
      
      virtual Result init(int argc, char** argv);
      virtual Result deinit();
      
      virtual Result cycle();
      
      virtual void consumeEvent(Event evEvent);
      Event consumeServiceEvent(ServiceEvent seServiceEvent);
    };
  }
  
  extern "C" plugins::PLUGIN_CLASS* createInstance();
  extern "C" void destroyInstance(plugins::PLUGIN_CLASS* icDestroy);
}


#endif /* __PLUGIN_DOTEXPORTER_H__ */
