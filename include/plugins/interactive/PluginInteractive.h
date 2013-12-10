#ifndef __PLUGIN_INTERACTIVE_H__
#define __PLUGIN_INTERACTIVE_H__


// System
#include <cstdlib>
#include <iostream>

// ROS
#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

// Designators
#include <designators/CDesignator.h>
#include <designator_integration_msgs/DesignatorCommunication.h>

// Private
#include <Types.h>
#include <ForwardDeclarations.h>
#include <Plugin.h>
#include <plugins/interactive/InteractiveObject.h>

using namespace std;
using namespace visualization_msgs;
using namespace interactive_markers;


namespace beliefstate {
  namespace plugins {
    class PluginInteractive : public Plugin {
    private:
      InteractiveMarkerServer* m_imsServer;
      list<InteractiveObject*> m_lstInteractiveObjects;
      
    public:
      PluginInteractive();
      ~PluginInteractive();
      
      virtual Result init(int argc, char** argv);
      virtual Result deinit();
      
      virtual Result cycle();
      
      virtual void consumeEvent(Event evEvent);
      
      InteractiveObject* addInteractiveObject(InteractiveObject* ioAdd);
    };
  }
  
  extern "C" plugins::PluginInteractive* createInstance();
  extern "C" void destroyInstance(plugins::PluginInteractive* icDestroy);
}


#endif /* __PLUGIN_INTERACTIVE_H__ */
