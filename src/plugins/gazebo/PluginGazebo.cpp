#include <plugins/gazebo/PluginGazebo.h>


namespace beliefstate {
  namespace plugins {
    PluginGazebo::PluginGazebo() {
    }
    
    PluginGazebo::~PluginGazebo() {
    }
    
    Result PluginGazebo::init(int argc, char** argv) {
      Result resInit = defaultResult();
      
      ros::NodeHandle nh("~");
      m_srvTest = nh.advertiseService<PluginGazebo>("test", &PluginGazebo::serviceCallbackTest, this);
      
      return resInit;
    }
    
    Result PluginGazebo::deinit() {
      return defaultResult();
    }
    
    bool PluginGazebo::serviceCallbackTest(designator_integration_msgs::DesignatorCommunication::Request &req, designator_integration_msgs::DesignatorCommunication::Response &res) {
      cout << "!" << endl;
      return true;
    }
    
    Result PluginGazebo::cycle() {
      Result resCycle = defaultResult();
      
      m_mtxEventsStore.lock();
      resCycle.lstEvents = m_lstEvents;
      m_lstEvents.clear();
      m_mtxEventsStore.unlock();
      
      return resCycle;
    }
    
    void PluginGazebo::consumeEvent(Event evEvent) {
      cout << "PluginGazebo: Consume event!" << endl;
    }
  }
  
  extern "C" plugins::PluginGazebo* createInstance() {
    return new plugins::PluginGazebo();
  }
  
  extern "C" void destroyInstance(plugins::PluginGazebo* icDestroy) {
    delete icDestroy;
  }
}
