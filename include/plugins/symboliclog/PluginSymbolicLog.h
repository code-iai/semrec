#ifndef __PLUGIN_SYMBOLIC_LOG_H__
#define __PLUGIN_SYMBOLIC_LOG_H__


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
#include <Node.h>

using namespace std;


namespace beliefstate {
  namespace plugins {
    class PluginSymbolicLog : public Plugin {
    private:
      list<Node*> m_lstNodes;
      Node* m_ndActive;
      list< pair<string, string> > m_lstDesignatorIDs;
      list< pair<string, string> > m_lstDesignatorEquations;
      list< pair<string, string> > m_lstDesignatorEquationTimes;
      
    public:
      PluginSymbolicLog();
      ~PluginSymbolicLog();
      
      virtual Result init(int argc, char** argv);
      virtual Result deinit();
      
      virtual Result cycle();
      
      virtual void consumeEvent(Event evEvent);
      virtual Event consumeServiceEvent(ServiceEvent seServiceEvent);
      
      Node* addNode(string strName, int nContextID);
      void setNodeAsActive(Node* ndActive);
      Node* activeNode();
      
      string getDesignatorID(string strMemoryAddress);
      string getUniqueDesignatorID(string strMemoryAddress);
      string generateRandomIdentifier(string strPrefix, unsigned int unLength);
      void equateDesignators(string strMAChild, string strMAParent);
    };
  }
  
  extern "C" plugins::PluginSymbolicLog* createInstance();
  extern "C" void destroyInstance(plugins::PluginSymbolicLog* icDestroy);
}


#endif /* __PLUGIN_SYMBOLIC_LOG_H__ */
