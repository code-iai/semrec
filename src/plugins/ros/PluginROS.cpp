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
	string strROSNodeName = "beliefstate_ros";
	this->info("Starting ROS node '" + strROSNodeName + "'.");
	
	ros::init(argc, argv, strROSNodeName);
	m_nhHandle = new ros::NodeHandle("~");
	
	if(ros::ok()) {
	  m_srvBeginContext = m_nhHandle->advertiseService<PluginROS>("begin_context", &PluginROS::serviceCallbackBeginContext, this);
	  m_srvEndContext = m_nhHandle->advertiseService<PluginROS>("end_context", &PluginROS::serviceCallbackEndContext, this);
	  m_srvAlterContext = m_nhHandle->advertiseService<PluginROS>("alter_context", &PluginROS::serviceCallbackAlterContext, this);
	  
	  this->info("ROS node started.");
	} else {
	  resInit.bSuccess = false;
	  resInit.strErrorMessage = "Failed to start ROS node.";
	}
      } else {
      	this->warn("ROS node already started.");
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
      Event evBeginContext = defaultEvent("begin-context");
      evBeginContext.nContextID = createContextID();
      evBeginContext.cdDesignator = new CDesignator(req.request.designator);
      
      this->info("When beginning context, received " + this->getDesignatorTypeString(evBeginContext.cdDesignator) + " designator");
      this->deployEvent(evBeginContext);
      
      CDesignator *desigResponse = new CDesignator();
      desigResponse->setType(ACTION);
      desigResponse->setValue(string("_id"), evBeginContext.nContextID);
      
      res.response.designators.push_back(desigResponse->serializeToMessage());
      delete desigResponse;
      
      return true;
    }

    bool PluginROS::serviceCallbackEndContext(designator_integration_msgs::DesignatorCommunication::Request &req, designator_integration_msgs::DesignatorCommunication::Response &res) {
      Event evEndContext = defaultEvent("end-context");
      evEndContext.cdDesignator = new CDesignator(req.request.designator);
      
      this->info("When ending context, received " + this->getDesignatorTypeString(evEndContext.cdDesignator) + " designator");
      this->deployEvent(evEndContext);
      
      return true;
    }

    bool PluginROS::serviceCallbackAlterContext(designator_integration_msgs::DesignatorCommunication::Request &req, designator_integration_msgs::DesignatorCommunication::Response &res) {
      Event evAlterContext = defaultEvent();
      evAlterContext.cdDesignator = new CDesignator(req.request.designator);
      
      this->info("When altering context, received " + this->getDesignatorTypeString(evAlterContext.cdDesignator) + " designator");
      
      string strCommand = evAlterContext.cdDesignator->stringValue("command");
      transform(strCommand.begin(), strCommand.end(), strCommand.begin(), ::tolower);
      
      if(strCommand == "add-image") {
	evAlterContext.strEventName = "add-image-from-topic";
      } else if(strCommand == "add-failure") {
	evAlterContext.strEventName = "add-failure";
      } else if(strCommand == "add-designator") {
	evAlterContext.strEventName = "add-designator";
      } else if(strCommand == "equate-designators") {
	evAlterContext.strEventName = "equate-designators";
      } else if(strCommand == "add-object") {
	evAlterContext.strEventName = "add-object";
      } else if(strCommand == "export-planlog") {
	evAlterContext.strEventName = "export-planlog";
      } else {
	this->warn("Unknown command when altering context: '" + strCommand + "'");
      }
      
      this->deployEvent(evAlterContext);
      
      return true;
    }
    
    void PluginROS::consumeEvent(Event evEvent) {
      this->info("Consume event!");
    }
    
    Event PluginROS::consumeServiceEvent(ServiceEvent seServiceEvent) {
      Event evService = defaultEvent();
      
      this->info("Consume service event of type '" + seServiceEvent.strServiceName + "'!");
      
      return evService;
    }
    
    string PluginROS::getDesignatorTypeString(CDesignator* desigDesignator) {
      string strDesigType = "UNKNOWN";
      switch(desigDesignator->type()) {
      case ACTION:
	strDesigType = "ACTION";
	break;
	
      case OBJECT:
	strDesigType = "OBJECT";
	break;
	
      case LOCATION:
	strDesigType = "LOCATION";
	break;
	
      default:
	break;
      }
      
      return strDesigType;
    }
  }
  
  extern "C" plugins::PluginROS* createInstance() {
    return new plugins::PluginROS();
  }
  
  extern "C" void destroyInstance(plugins::PluginROS* icDestroy) {
    delete icDestroy;
  }
}
