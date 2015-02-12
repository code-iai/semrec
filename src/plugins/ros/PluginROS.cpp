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


namespace semrec {
  namespace plugins {
    PLUGIN_CLASS::PLUGIN_CLASS() {
      m_nhHandle = NULL;
      m_aspnAsyncSpinner = NULL;
      m_bRoslogMessages = false;
      m_bFirstContextReceived = false;
      m_bKeepSpinning = true;
      m_bSpinWorkerRunning = false;
      m_thrdSpinWorker = NULL;
      
      this->setPluginVersion("0.93");
    }
    
    PLUGIN_CLASS::~PLUGIN_CLASS() {
      this->shutdownSpinWorker();
      
      if(m_nhHandle) {
	delete m_nhHandle;
      }
      
      if(m_thrdSpinWorker) {
	delete m_thrdSpinWorker;
      }
    }
    
    Result PLUGIN_CLASS::init(int argc, char** argv) {
      Result resInit = defaultResult();
      
      Designator* cdConfig = this->getIndividualConfig();
      
      this->setSubscribedToEvent("symbolic-create-designator", true);
      this->setSubscribedToEvent("interactive-callback", true);
      this->setSubscribedToEvent("symbolic-add-image", true);
      this->setSubscribedToEvent("cancel-open-request", true);
      
      if(cdConfig->floatValue("roslog-messages") != 0) {
	this->setSubscribedToEvent("status-message", false);
      }
      
      if(!ros::ok()) {
	std::string strROSNodeName = cdConfig->stringValue("node-name");
	
	if(strROSNodeName == "") {
	  strROSNodeName = "semrec_ros";
	}
	
	this->info("Starting ROS node '" + strROSNodeName + "'.");
	
	ros::init(argc, argv, strROSNodeName);
	m_nhHandle = new ros::NodeHandle("~");
	
	if(ros::ok()) {
	  m_srvCallback = m_nhHandle->advertiseService<PLUGIN_CLASS>("operate", &PLUGIN_CLASS::serviceCallback, this);
	  
	  m_pubLoggedDesignators = m_nhHandle->advertise<designator_integration_msgs::Designator>("/logged_designators", 1);
	  m_pubInteractiveCallback = m_nhHandle->advertise<designator_integration_msgs::Designator>("/interactive_callback", 1);
	  
	  if(cdConfig->floatValue("roslog-messages") != 0) {
	    std::string strTopic = cdConfig->stringValue("roslog-topic");
	    
	    if(strTopic != "") {
	      m_pubStatusMessages = m_nhHandle->advertise<rosgraph_msgs::Log>(strTopic, 1);
	      m_bRoslogMessages = true;
	    } else {
	      this->warn("You requested the status messages to be roslog'ged, but didn't specify a roslog topic. Ignoring the whole thing.");
	    }
	  }
	  
	  this->info("ROS node started. Starting to spin.");
	  m_thrdSpinWorker = new boost::thread(&PLUGIN_CLASS::spinWorker, this);
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
    
    void PLUGIN_CLASS::setKeepSpinning(bool bKeepSpinning) {
      m_mtxSpinWorker.lock();
      m_bKeepSpinning = bKeepSpinning;
      m_mtxSpinWorker.unlock();
    }
    
    bool PLUGIN_CLASS::keepSpinning() {
      m_mtxSpinWorker.lock();
      bool bReturn = m_bKeepSpinning;
      m_mtxSpinWorker.unlock();
      
      return bReturn;
    }
    
    void PLUGIN_CLASS::spinWorker() {
      ros::Rate rSpin(10000);
      
      this->setKeepSpinning(true);
      this->setSpinWorkerRunning(true);
      
      while(this->keepSpinning()) {
	ros::spinOnce();
	//rSpin.sleep();
      }
      
      this->setSpinWorkerRunning(false);
    }
    
    void PLUGIN_CLASS::setSpinWorkerRunning(bool bSpinWorkerRunning) {
      m_mtxSpinWorkerRunning.lock();
      m_bSpinWorkerRunning = bSpinWorkerRunning;
      m_mtxSpinWorkerRunning.unlock();
    }
    
    bool PLUGIN_CLASS::spinWorkerRunning() {
      m_mtxSpinWorkerRunning.lock();
      bool bReturn = m_bSpinWorkerRunning;
      m_mtxSpinWorkerRunning.unlock();
      
      return bReturn;
    }
    
    void PLUGIN_CLASS::shutdownSpinWorker() {
      this->setKeepSpinning(false);
      
      while(this->spinWorkerRunning()) {
	usleep(10000);
      }
      
      m_thrdSpinWorker->join();
    }
    
    bool PLUGIN_CLASS::serviceCallback(designator_integration_msgs::DesignatorCommunication::Request &req,
				       designator_integration_msgs::DesignatorCommunication::Response &res) {
      bool bReturn = true;
      
      m_mtxGlobalInputLock.lock();
      
      // TODO(winkler): Maybe use this designator instance for the
      // events below, to not deserialize it twice.
      Designator* cdDesig = new Designator(req.request.designator);
      std::string strCBType = cdDesig->stringValue("_cb_type");
      delete cdDesig;
      
      transform(strCBType.begin(), strCBType.end(), strCBType.begin(), ::tolower);
      
      if(strCBType == "begin") {
	Event evBeginContext = defaultEvent("begin-context");
	evBeginContext.nContextID = createContextID();
	evBeginContext.cdDesignator = new Designator(req.request.designator);
	
	std::stringstream sts;
	sts << evBeginContext.nContextID;
	
	this->info("Beginning context (ID = " + sts.str() + "): '" + evBeginContext.cdDesignator->stringValue("_name") + "'");
	this->deployEvent(evBeginContext);
	
	Designator *desigResponse = new Designator();
	desigResponse->setType(Designator::DesignatorType::ACTION);
	desigResponse->setValue(std::string("_id"), evBeginContext.nContextID);
	
	res.response.designators.push_back(desigResponse->serializeToMessage());
	delete desigResponse;
	
	if(!m_bFirstContextReceived) {
	  this->info("First context received - logging is active.", true);
	  m_bFirstContextReceived = true;
	}
      } else if(strCBType == "end") {
	Event evEndContext = defaultEvent("end-context");
	evEndContext.cdDesignator = new Designator(req.request.designator);
	
	int nContextID = (int)evEndContext.cdDesignator->floatValue("_id");
	std::stringstream sts;
	sts << nContextID;
	
	this->info("When ending context (ID = " + sts.str() + "), received " + this->getDesignatorTypeString(evEndContext.cdDesignator) + " designator");
	this->deployEvent(evEndContext);
	
	freeContextID(nContextID);
      } else if(strCBType == "alter") {
	Event evAlterContext = defaultEvent();
	evAlterContext.cdDesignator = new Designator(req.request.designator);
	
	if(evAlterContext.cdDesignator->stringValue("_type") == "service") {
	  ServiceEvent seService = defaultServiceEvent();
	  seService.smResultModifier = SM_IGNORE_RESULTS;
	  
	  seService.cdDesignator = evAlterContext.cdDesignator;
	  seService.bPreserve = true;
	  
	  std::string strCommand = seService.cdDesignator->stringValue("command");
	  transform(strCommand.begin(), strCommand.end(), strCommand.begin(), ::tolower);
	  seService.strServiceName = strCommand;
	  
	  ServiceEvent seResult = this->deployServiceEvent(seService, true);
	  
	  this->waitForAssuranceToken(evAlterContext.cdDesignator->stringValue("_assurance_token"));
	  
	  if(seResult.cdDesignator) {
	    res.response.designators.push_back(seResult.cdDesignator->serializeToMessage());
	    
	    if(seResult.bPreserve) {
	      delete seResult.cdDesignator;
	    }
	  }
	  
	  delete seService.cdDesignator;
	} else if(evAlterContext.cdDesignator->stringValue("_type") == "alter") {
	  this->info("When altering context, received " + this->getDesignatorTypeString(evAlterContext.cdDesignator) + " designator");
	  
	  std::string strCommand = evAlterContext.cdDesignator->stringValue("command");
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
	  } else if(strCommand == "catch-failure") {
	    evAlterContext.strEventName = "catch-failure";
	  } else if(strCommand == "rethrow-failure") {
	    evAlterContext.strEventName = "rethrow-failure";
	  } else {
	    this->info("Forwarding alter command: '" + strCommand + "'");
	    evAlterContext.strEventName = strCommand;
	  }
	  
	  this->waitForAssuranceToken(evAlterContext.cdDesignator->stringValue("_assurance_token"));
	  
	  this->deployEvent(evAlterContext, true);
	}
      } else {
	this->fail("Unknown callback operation: '" + strCBType + "'");
      }
      
      m_mtxGlobalInputLock.unlock();
      
      return bReturn;
    }
    
    void PLUGIN_CLASS::consumeEvent(Event evEvent) {
      if(evEvent.bRequest == false) {
	this->closeRequestID(evEvent.nOpenRequestID);
      } else {
	if(evEvent.strEventName == "symbolic-add-image") {
	  this->closeRequestID(evEvent.nOpenRequestID);
	} else if(evEvent.strEventName == "cancel-open-request") {
	  this->closeRequestID(evEvent.nOpenRequestID);
	} else if(evEvent.strEventName == "symbolic-create-designator") {
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
      Event evReturn = Plugin::consumeServiceEvent(seServiceEvent);
      
      //this->info("Consume service event of type '" + seServiceEvent.strServiceName + "'!");
      
      return evReturn;
    }
    
    std::string PLUGIN_CLASS::getDesignatorTypeString(Designator* desigDesignator) {
      std::string strDesigType = "UNKNOWN";
      
      switch(desigDesignator->type()) {
      case Designator::DesignatorType::ACTION:
	strDesigType = "ACTION";
	break;
	
      case Designator::DesignatorType::OBJECT:
	strDesigType = "OBJECT";
	break;
	
      case Designator::DesignatorType::LOCATION:
	strDesigType = "LOCATION";
	break;

      case Designator::DesignatorType::HUMAN:
	strDesigType = "HUMAN";
	break;
	
      default:
	break;
      }
      
      return strDesigType;
    }
    
    void PLUGIN_CLASS::waitForAssuranceToken(std::string strToken) {
      if(strToken != "") {
	// Wait for issueance of assurance token.
	if(waitForGlobalToken(strToken, 2.0)) { // Timeout of 2.0 seconds
	  revokeGlobalToken(strToken);
	} else {
	  this->warn("Failed to deliver assurance token '" + strToken + "'. Waiting for it was a waste of time. Check your model, something's off.");
	}
      }
	  
    }
  }
  
  extern "C" plugins::PLUGIN_CLASS* createInstance() {
    return new plugins::PLUGIN_CLASS();
  }
  
  extern "C" void destroyInstance(plugins::PLUGIN_CLASS* icDestroy) {
    delete icDestroy;
  }
}
