#ifndef __PLUGIN_ROS_H__
#define __PLUGIN_ROS_H__


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
    class PluginROS : public Plugin{
    private:
      ros::NodeHandle* m_nhHandle;
      ros::ServiceServer m_srvBeginContext;
      ros::ServiceServer m_srvEndContext;
      ros::ServiceServer m_srvAlterContext;
      
    public:
      PluginROS();
      ~PluginROS();
      
      virtual Result init(int argc, char** argv);
      virtual Result deinit();
      
      virtual Result cycle();
      
      bool serviceCallbackBeginContext(designator_integration_msgs::DesignatorCommunication::Request &req, designator_integration_msgs::DesignatorCommunication::Response &res);
      bool serviceCallbackEndContext(designator_integration_msgs::DesignatorCommunication::Request &req, designator_integration_msgs::DesignatorCommunication::Response &res);
      bool serviceCallbackAlterContext(designator_integration_msgs::DesignatorCommunication::Request &req, designator_integration_msgs::DesignatorCommunication::Response &res);
      
      virtual void consumeEvent(Event evEvent);
    };
    
  }
  
  extern "C" plugins::PluginROS* createInstance();
  extern "C" void destroyInstance(plugins::PluginROS* icDestroy);
}


#endif /* __PLUGIN_ROS_H__ */
