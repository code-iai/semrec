#ifndef __PLUGIN_OWLEXPORTER_H__
#define __PLUGIN_OWLEXPORTER_H__


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
#include <plugins/owlexporter/CExporterOwl.h>

using namespace std;


namespace beliefstate {
  namespace plugins {
    class PluginOWLExporter : public Plugin {
    private:
    public:
      PluginOWLExporter();
      ~PluginOWLExporter();
      
      virtual Result init(int argc, char** argv);
      virtual Result deinit();
      
      virtual Result cycle();
      
      virtual void consumeEvent(Event evEvent);
      Event consumeServiceEvent(ServiceEvent seServiceEvent);
    };
  }
  
  extern "C" plugins::PluginOWLExporter* createInstance();
  extern "C" void destroyInstance(plugins::PluginOWLExporter* icDestroy);
}


#endif /* __PLUGIN_OWLEXPORTER_H__ */
