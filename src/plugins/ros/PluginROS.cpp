#include <plugins/ros/PluginROS.h>


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
      ros::shutdown();
      
      return defaultResult();
    }
    
    Result PluginROS::cycle() {
      Result resCycle = defaultResult();
      this->deployCycleData(resCycle);
      
      if(ros::ok()) {
	ros::spinOnce();
      } else {
	resCycle.bSuccess = false;
      }
      
      return resCycle;
    }
    
    bool PluginROS::serviceCallbackBeginContext(designator_integration_msgs::DesignatorCommunication::Request &req, designator_integration_msgs::DesignatorCommunication::Response &res) {
      m_mtxEventsStore.lock();
      
      Event evBeginContext = defaultEvent();
      evBeginContext.eiEventIdentifier = EI_BEGIN_CONTEXT;
      evBeginContext.nContextID = createContextID();
      evBeginContext.cdDesignator = new CDesignator(req.request.designator);
      
      // TODO: Add designator information
      
      m_lstEvents.push_back(evBeginContext);
      
      m_mtxEventsStore.unlock();
      
      return true;
    }

    bool PluginROS::serviceCallbackEndContext(designator_integration_msgs::DesignatorCommunication::Request &req, designator_integration_msgs::DesignatorCommunication::Response &res) {
      m_mtxEventsStore.lock();

      Event evEndContext = defaultEvent();
      evEndContext.eiEventIdentifier = EI_END_CONTEXT;
      evEndContext.cdDesignator = new CDesignator(req.request.designator);
      
      // TODO: Add designator information, like the context id and so
      // forth
      
      m_lstEvents.push_back(evEndContext);

      m_mtxEventsStore.unlock();
      
      return true;
    }

    bool PluginROS::serviceCallbackAlterContext(designator_integration_msgs::DesignatorCommunication::Request &req, designator_integration_msgs::DesignatorCommunication::Response &res) {
      m_mtxEventsStore.lock();
      
      Event evAlterContext = defaultEvent();
      evAlterContext.cdDesignator = new CDesignator(req.request.designator);
      
      string strCommand = evAlterContext.cdDesignator->stringValue("command");
      if(strCommand == "add-image") {
	evAlterContext.eiEventIdentifier = EI_ADD_IMAGE_FROM_TOPIC;
      } else if(strCommand == "add-failure") {
	evAlterContext.eiEventIdentifier = EI_ADD_FAILURE;
      } else if(strCommand == "add-designator") {
	evAlterContext.eiEventIdentifier = EI_ADD_DESIGNATOR;
      } else if(strCommand == "equate-designators") {
	evAlterContext.eiEventIdentifier = EI_EQUATE_DESIGNATORS;
      } else if(strCommand == "add-object") {
	evAlterContext.eiEventIdentifier = EI_ADD_OBJECT;
      } else if(strCommand == "extract-planlog") {
	evAlterContext.eiEventIdentifier = EI_EXTRACT_PLANLOG;
      }
      //
      evAlterContext.eiEventIdentifier = EI_ADD_IMAGE_FROM_TOPIC;
      //
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
