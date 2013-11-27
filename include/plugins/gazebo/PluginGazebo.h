#ifndef __PLUGIN_GAZEBO_H__
#define __PLUGIN_GAZEBO_H__


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

using namespace std;


namespace beliefstate {
  namespace plugins {
    class PluginGazebo : public Plugin {
    private:
      ros::ServiceServer m_srvTest;
      
    public:
      PluginGazebo();
      ~PluginGazebo();
      
      virtual Result init(int argc, char** argv);
      virtual Result deinit();
      
      virtual Result cycle();
      
      virtual void consumeEvent(Event evEvent);
      virtual Event consumeServiceEvent(ServiceEvent seServiceEvent);
      
      bool serviceCallbackTest(designator_integration_msgs::DesignatorCommunication::Request &req, designator_integration_msgs::DesignatorCommunication::Response &res);
    };
  }
  
  extern "C" plugins::PluginGazebo* createInstance();
  extern "C" void destroyInstance(plugins::PluginGazebo* icDestroy);
}


#endif /* __PLUGIN_GAZEBO_H__ */
