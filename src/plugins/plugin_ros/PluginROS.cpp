#include <plugins/plugin_ros/PluginROS.h>


namespace beliefstate {
  namespace plugins {
    PluginROS::PluginROS() {
      m_nhHandle = NULL;
    }
    
    PluginROS::~PluginROS() {
      if(m_nhHandle) {
	delete m_nhHandle;
      }
    }
    
    Result PluginROS::init(int argc, char** argv) {
      Result resInit = defaultResult();
      
      if(!ros::ok()) {
	cout << "Starting ROS node." << endl;
	
	ros::init(argc, argv, "beliefstate_ros");
	m_nhHandle = new ros::NodeHandle("~");
	
	if(ros::ok()) {
	  m_srvBeginContext = m_nhHandle->advertiseService<PluginROS>("begin_context", &PluginROS::serviceCallbackBeginContext, this);
	  m_srvEndContext = m_nhHandle->advertiseService<PluginROS>("end_context", &PluginROS::serviceCallbackEndContext, this);
	  m_srvAlterContext = m_nhHandle->advertiseService<PluginROS>("alter_context", &PluginROS::serviceCallbackAlterContext, this);
	  
	  cout << "ROS node started." << endl;
	} else {
	  resInit.bSuccess = false;
	  resInit.strErrorMessage = "Failed to start ROS node.";
	}
      } else {
	cout << "ROS node already started." << endl;
      }
      
      return resInit;
    }
    
    Result PluginROS::deinit() {
    }
    
    Result PluginROS::cycle() {
      Result resCycle = defaultResult();
      
      if(ros::ok()) {
	ros::spinOnce();
	
	m_mtxEventsStore.lock();
	resCycle.lstEvents = m_lstEvents;
	m_lstEvents.clear();
	m_mtxEventsStore.unlock();
      } else {
	resCycle.bSuccess = false;
      }
      
      return resCycle;
    }
    
    bool PluginROS::serviceCallbackBeginContext(designator_integration_msgs::DesignatorCommunication::Request &req, designator_integration_msgs::DesignatorCommunication::Response &res) {
      m_mtxEventsStore.lock();
      
      Event evBeginContext;
      evBeginContext.eiEventIdentifier = EI_BEGIN_CONTEXT;
      evBeginContext.nContextID = createContextID();
      
      // TODO: Add designator information
      
      m_lstEvents.push_back(evBeginContext);
      
      m_mtxEventsStore.unlock();
      
      return true;
    }

    bool PluginROS::serviceCallbackEndContext(designator_integration_msgs::DesignatorCommunication::Request &req, designator_integration_msgs::DesignatorCommunication::Response &res) {
      m_mtxEventsStore.lock();

      Event evEndContext;
      evEndContext.eiEventIdentifier = EI_END_CONTEXT;
      
      // TODO: Add designator information, like the context id and so
      // forth
      
      m_lstEvents.push_back(evEndContext);

      m_mtxEventsStore.unlock();
      
      return true;
    }

    bool PluginROS::serviceCallbackAlterContext(designator_integration_msgs::DesignatorCommunication::Request &req, designator_integration_msgs::DesignatorCommunication::Response &res) {
      m_mtxEventsStore.lock();

      Event evAlterContext;
      
      // evAlterContext.eiEventIdentifier = EI_END_CONTEXT;
      
      // TODO: Add designator information, like the context id and so
      // forth
      
      m_lstEvents.push_back(evAlterContext);
      
      m_mtxEventsStore.unlock();
      
      return true;
    }
    
    void PluginROS::consumeEvent(Event evEvent) {
      cout << "PluginROS: Consume event!" << endl;
    }
  }
  
  extern "C" plugins::PluginROS* createInstance() {
    return new plugins::PluginROS();
  }
  
  extern "C" void destroyInstance(plugins::PluginROS* icDestroy) {
    delete icDestroy;
  }
}
