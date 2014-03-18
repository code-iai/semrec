/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Institute for Artificial Intelligence,
 *  Universität Bremen.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Institute for Artificial Intelligence,
 *     Universität Bremen, nor the names of its contributors may be
 *     used to endorse or promote products derived from this software
 *     without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/** \author Jan Winkler */


#include <plugins/ros/PluginROS.h>


namespace beliefstate {
  namespace plugins {
    PLUGIN_CLASS::PLUGIN_CLASS() {
      m_nhHandle = NULL;
      m_aspnAsyncSpinner = NULL;
      m_bRoslogMessages = false;
      
      this->setPluginVersion("0.9");
    }
    
    PLUGIN_CLASS::~PLUGIN_CLASS() {
      if(m_nhHandle) {
	delete m_nhHandle;
      }
      
      if(m_aspnAsyncSpinner) {
	delete m_aspnAsyncSpinner;
      }
    }
    
    Result PLUGIN_CLASS::init(int argc, char** argv) {
      Result resInit = defaultResult();
      
      CDesignator* cdConfig = this->getIndividualConfig();
      
      this->setSubscribedToEvent("add-image-from-file", true);
      this->setSubscribedToEvent("add-image-from-topic", true);
      this->setSubscribedToEvent("symbolic-create-designator", true);
      this->setSubscribedToEvent("interactive-callback", true);
      this->setSubscribedToEvent("status-message", true);
      
      if(!ros::ok()) {
	string strROSNodeName = cdConfig->stringValue("node-name");
	
	if(strROSNodeName == "") {
	  strROSNodeName = "beliefstate_ros";
	}
	
	this->info("Starting ROS node '" + strROSNodeName + "'.");
	
	ros::init(argc, argv, strROSNodeName);
	m_nhHandle = new ros::NodeHandle("~");
	
	if(ros::ok()) {
	  m_srvBeginContext = m_nhHandle->advertiseService<PLUGIN_CLASS>("begin_context", &PLUGIN_CLASS::serviceCallbackBeginContext, this);
	  m_srvEndContext = m_nhHandle->advertiseService<PLUGIN_CLASS>("end_context", &PLUGIN_CLASS::serviceCallbackEndContext, this);
	  m_srvAlterContext = m_nhHandle->advertiseService<PLUGIN_CLASS>("alter_context", &PLUGIN_CLASS::serviceCallbackAlterContext, this);
	  m_pubLoggedDesignators = m_nhHandle->advertise<designator_integration_msgs::Designator>("/logged_designators", 1);
	  m_pubInteractiveCallback = m_nhHandle->advertise<designator_integration_msgs::Designator>("/interactive_callback", 1);
	  
	  if(cdConfig->floatValue("roslog-messages") != 0) {
	    string strTopic = cdConfig->stringValue("roslog-topic");
	    
	    if(strTopic != "") {
	      m_pubStatusMessages = m_nhHandle->advertise<rosgraph_msgs::Log>(strTopic, 1);
	      m_bRoslogMessages = true;
	    } else {
	      this->warn("You requested the status messages to be roslog'ged, but didn't specify a roslog topic. Ignoring the whole thing.");
	    }
	  }
	  
	  int nThreads = 4;
	  if(cdConfig->childForKey("async-threads")) {
	    nThreads = cdConfig->floatValue("async-threads");
	  }
	  
	  stringstream sts;
	  sts << nThreads;
	  
	  this->info("ROS node started. Starting to spin (" + sts.str() + " threads).");
	  m_aspnAsyncSpinner = new ros::AsyncSpinner(nThreads);
	  m_aspnAsyncSpinner->start();
	} else {
	  resInit.bSuccess = false;
	  resInit.strErrorMessage = "Failed to start ROS node.";
	}
      } else {
      	this->warn("ROS node already started.");
      }
      
      return resInit;
    }
    
    Result PLUGIN_CLASS::deinit() {
      ros::shutdown();
      
      return defaultResult();
    }
    
    Result PLUGIN_CLASS::cycle() {
      Result resCycle = defaultResult();
      this->deployCycleData(resCycle);
      
      return resCycle;
    }
    
    bool PLUGIN_CLASS::serviceCallbackBeginContext(designator_integration_msgs::DesignatorCommunication::Request &req, designator_integration_msgs::DesignatorCommunication::Response &res) {
      Event evBeginContext = defaultEvent("begin-context");
      evBeginContext.nContextID = createContextID();
      evBeginContext.cdDesignator = new CDesignator(req.request.designator);
      
      stringstream sts;
      sts << evBeginContext.nContextID;
      
      this->info("When beginning context (ID = " + sts.str() + "), received " + this->getDesignatorTypeString(evBeginContext.cdDesignator) + " designator");
      this->deployEvent(evBeginContext);
      
      CDesignator *desigResponse = new CDesignator();
      desigResponse->setType(ACTION);
      desigResponse->setValue(string("_id"), evBeginContext.nContextID);
      
      res.response.designators.push_back(desigResponse->serializeToMessage());
      delete desigResponse;
      
      return true;
    }
    
    bool PLUGIN_CLASS::serviceCallbackEndContext(designator_integration_msgs::DesignatorCommunication::Request &req, designator_integration_msgs::DesignatorCommunication::Response &res) {
      Event evEndContext = defaultEvent("end-context");
      evEndContext.cdDesignator = new CDesignator(req.request.designator);
      
      int nContextID = (int)evEndContext.cdDesignator->floatValue("_id");
      stringstream sts;
      sts << nContextID;
      
      this->info("When ending context (ID = " + sts.str() + "), received " + this->getDesignatorTypeString(evEndContext.cdDesignator) + " designator");
      this->deployEvent(evEndContext);
      
      freeContextID(nContextID);
      
      return true;
    }

    bool PLUGIN_CLASS::serviceCallbackAlterContext(designator_integration_msgs::DesignatorCommunication::Request &req, designator_integration_msgs::DesignatorCommunication::Response &res) {
      Event evAlterContext = defaultEvent();
      evAlterContext.cdDesignator = new CDesignator(req.request.designator);
      
      this->info("When altering context, received " + this->getDesignatorTypeString(evAlterContext.cdDesignator) + " designator");
      
      string strCommand = evAlterContext.cdDesignator->stringValue("command");
      transform(strCommand.begin(), strCommand.end(), strCommand.begin(), ::tolower);
      
      if(strCommand == "add-image") {
	evAlterContext.strEventName = "add-image-from-topic";
	evAlterContext.nOpenRequestID = this->openNewRequestID();
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
      } else if(strCommand == "start-new-experiment") {
	evAlterContext.strEventName = "start-new-experiment";
      } else if(strCommand == "set-experiment-meta-data") {
	evAlterContext.strEventName = "set-experiment-meta-data";
      } else if(strCommand == "register-interactive-object") {
	evAlterContext.strEventName = "symbolic-add-object";
      } else if(strCommand == "unregister-interactive-object") {
	evAlterContext.strEventName = "symbolic-remove-object";
      } else if(strCommand == "set-interactive-object-menu") {
	evAlterContext.strEventName = "symbolic-set-interactive-object-menu";
      } else if(strCommand == "update-interactive-object-pose") {
	evAlterContext.strEventName = "symbolic-update-object-pose";
      } else {
	this->warn("Unknown command when altering context: '" + strCommand + "'");
      }
      
      this->deployEvent(evAlterContext, true);
      
      return true;
    }
    
    void PLUGIN_CLASS::consumeEvent(Event evEvent) {
      if(evEvent.bRequest == false) {
	this->closeRequestID(evEvent.nOpenRequestID);
      } else {
	if(evEvent.strEventName == "symbolic-create-designator") {
	  if(evEvent.cdDesignator) {
	    m_pubLoggedDesignators.publish(evEvent.cdDesignator->serializeToMessage());
	  }
	} else if(evEvent.strEventName == "interactive-callback") {
	  if(evEvent.cdDesignator) {
	    m_pubInteractiveCallback.publish(evEvent.cdDesignator->serializeToMessage());
	  }
	} else if(evEvent.strEventName == "status-message") {
	  if(m_bRoslogMessages) {
	    StatusMessage msgStatus = evEvent.msgStatusMessage;
	    rosgraph_msgs::Log rgmLogMessage;
	    
	    if(msgStatus.bBold == true) {
	      rgmLogMessage.level = rosgraph_msgs::Log::WARN;
	    } else {
	      rgmLogMessage.level = rosgraph_msgs::Log::INFO;
	    }
	    
	    rgmLogMessage.name = msgStatus.strPrefix;
	    rgmLogMessage.msg = msgStatus.strMessage;
	    
	    m_pubStatusMessages.publish(rgmLogMessage);
	  }
	}
      }
    }
    
    Event PLUGIN_CLASS::consumeServiceEvent(ServiceEvent seServiceEvent) {
      Event evService = defaultEvent();
      
      this->info("Consume service event of type '" + seServiceEvent.strServiceName + "'!");
      
      return evService;
    }
    
    string PLUGIN_CLASS::getDesignatorTypeString(CDesignator* desigDesignator) {
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
  
  extern "C" plugins::PLUGIN_CLASS* createInstance() {
    return new plugins::PLUGIN_CLASS();
  }
  
  extern "C" void destroyInstance(plugins::PLUGIN_CLASS* icDestroy) {
    delete icDestroy;
  }
}
