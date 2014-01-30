#ifndef __PLUGIN_ROS_H__
#define __PLUGIN_ROS_H__


#define PLUGIN_CLASS PluginROS


// System
#include <cstdlib>
#include <iostream>
#include <algorithm>

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
    class PLUGIN_CLASS : public Plugin {
    private:
      ros::NodeHandle* m_nhHandle;
      ros::ServiceServer m_srvBeginContext;
      ros::ServiceServer m_srvEndContext;
      ros::ServiceServer m_srvAlterContext;
      ros::Publisher m_pubLoggedDesignators;
      ros::Publisher m_pubInteractiveCallback;
      bool m_bStartedSpinning;
      ros::AsyncSpinner* m_aspnAsyncSpinner;
      
    public:
      PLUGIN_CLASS();
      ~PLUGIN_CLASS();
      
      virtual Result init(int argc, char** argv);
      virtual Result deinit();
      
      virtual Result cycle();
      
      bool serviceCallbackBeginContext(designator_integration_msgs::DesignatorCommunication::Request &req, designator_integration_msgs::DesignatorCommunication::Response &res);
      bool serviceCallbackEndContext(designator_integration_msgs::DesignatorCommunication::Request &req, designator_integration_msgs::DesignatorCommunication::Response &res);
      bool serviceCallbackAlterContext(designator_integration_msgs::DesignatorCommunication::Request &req, designator_integration_msgs::DesignatorCommunication::Response &res);
      
      virtual void consumeEvent(Event evEvent);
      virtual Event consumeServiceEvent(ServiceEvent seServiceEvent);
      
      string getDesignatorTypeString(CDesignator* desigDesignator);
    };
  }
  
  extern "C" plugins::PLUGIN_CLASS* createInstance();
  extern "C" void destroyInstance(plugins::PLUGIN_CLASS* icDestroy);
}


#endif /* __PLUGIN_ROS_H__ */
