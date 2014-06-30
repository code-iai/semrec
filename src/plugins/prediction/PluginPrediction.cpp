/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Institute for Artificial Intelligence,
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


#include <plugins/prediction/PluginPrediction.h>


namespace beliefstate {
  namespace plugins {
    PLUGIN_CLASS::PLUGIN_CLASS() {
      m_nhHandle = NULL;
      m_jsnModel = NULL;

      this->addDependency("ros");
      this->setPluginVersion("0.1");
    }

    PLUGIN_CLASS::~PLUGIN_CLASS() {
      if(m_nhHandle) {
	delete m_nhHandle;
      }
      
      if(m_jsnModel) {
	delete m_jsnModel;
      }
      
      m_lstPredictionStack.clear();
    }

    Result PLUGIN_CLASS::init(int argc, char** argv) {
      Result resInit = defaultResult();
      
      // Plan node control events
      this->setSubscribedToEvent("symbolic-begin-context", true);
      this->setSubscribedToEvent("symbolic-end-context", true);
      
      // Prepare the JSON prediction model parser
      m_jsnModel = new JSON();
      
      // ROS-related initialization
      m_nhHandle = new ros::NodeHandle("~");

      m_srvPredict = m_nhHandle->advertiseService<PLUGIN_CLASS>("predict", &PLUGIN_CLASS::serviceCallbackPredict, this);
      m_srvLoad = m_nhHandle->advertiseService<PLUGIN_CLASS>("load", &PLUGIN_CLASS::serviceCallbackLoad, this);

      return resInit;
    }

    Result PLUGIN_CLASS::deinit() {
      return defaultResult();
    }

    Result PLUGIN_CLASS::cycle() {
      Result resCycle = defaultResult();
      this->deployCycleData(resCycle);

      return resCycle;
    }

    void PLUGIN_CLASS::consumeEvent(Event evEvent) {
      if(evEvent.strEventName == "symbolic-begin-context") {
	if(evEvent.lstNodes.size() > 0) {
	  string strTitle = evEvent.lstNodes.front()->title();
	  
	  //m_lstPredictionStack
	  this->info("Prediction refined: Begin context '" + strTitle + "'");
	} else {
	  this->warn("Consuming 'symbolic-begin-context' event without nodes!");
	}
      } else if(evEvent.strEventName == "symbolic-end-context") {
	if(evEvent.lstNodes.size() > 0) {
	  string strTitle = evEvent.lstNodes.front()->title();
	  
	  this->info("Prediction refined: End context '" + strTitle + "'"); 
	} else {
	  this->warn("Consuming 'symbolic-end-context' event without nodes!");
	}
      }
    }

    bool PLUGIN_CLASS::serviceCallbackLoad(designator_integration_msgs::DesignatorCommunication::Request &req, designator_integration_msgs::DesignatorCommunication::Response &res) {
      bool bSuccess = false;

      CDesignator* desigRequest = new CDesignator(req.request.designator);
      CDesignator *desigResponse = new CDesignator();
      desigResponse->setType(ACTION);

      if(desigRequest->stringValue("load") == "model") {
        this->info("Received Model Load Request.");

        string strFile = desigRequest->stringValue("file");
        if(strFile != "") {
          this->info(" - Load model: '" + strFile + "'");

          bSuccess = this->load(strFile);
        } else {
          this->fail(" - No model file specified!");
        }
      }

      res.response.designators.push_back(desigResponse->serializeToMessage());

      delete desigRequest;
      delete desigResponse;

      return bSuccess;
    }

    bool PLUGIN_CLASS::serviceCallbackPredict(designator_integration_msgs::DesignatorCommunication::Request &req, designator_integration_msgs::DesignatorCommunication::Response &res) {
      CDesignator* desigRequest = new CDesignator(req.request.designator);
      CDesignator *desigResponse = new CDesignator();
      desigResponse->setType(ACTION);

      this->info("Received Prediction Request.");

      bool bSuccess = false;
      if(this->predict(desigRequest, desigResponse)) {
        res.response.designators.push_back(desigResponse->serializeToMessage());

        bSuccess = true;
      } else {
        this->fail("Failed to predict!");
      }

      delete desigRequest;
      delete desigResponse;

      return bSuccess;
    }

    bool PLUGIN_CLASS::load(string strFile) {
      bool bReturn = false;
      
      ifstream ifFile(strFile.c_str());
      if(ifFile.good()) {
	string strJSON((istreambuf_iterator<char>(ifFile)), (istreambuf_iterator<char>()));
	
	m_jsnModel->parse(strJSON);
	
	if(m_jsnModel->rootProperty()) {
	  this->info("Successfully loaded and parsed model '" + strFile + "'.");
	  bReturn = true;
	  
	  //m_jsnModel->rootProperty()->print();
	  
	  this->info("Okay, descending to toplevel.");
	  this->descend(m_jsnModel->rootProperty()->namedSubProperty("tree")->subProperties().front());
	} else {
	  this->fail("Failed to load model '" + strFile + "', unable to parse.");
	}
      } else {
	this->fail("Failed to load model '" + strFile + "', file does not exist.");
	bReturn = false;
      }
      
      ifFile.close();
      
      return bReturn;
    }
    
    bool PLUGIN_CLASS::descend(Property* prDescend) {
      // Go down one level in the property stack (this does away from
      // TopLevel, so DEEPER INTO THE STACK).
      bool bReturn = false;
      
      if(prDescend) {
	cout << "Descend to: " << endl;
	prDescend->print();
      }
      
      return bReturn;
    }
    
    bool PLUGIN_CLASS::ascend(Property* prAscend) {
      // Go up one level in the property stack (this goes in the
      // direction of TopLevel, so OUT OF THE STACK).
      bool bReturn = false;
      
      if(prAscend) {
	if(m_lstPredictionStack.size() > 0) {
	  cout << "Ascend from: " << endl;
	  prAscend->print();
	}
      }
      
      return bReturn;
    }
    
    bool PLUGIN_CLASS::predict(CDesignator* desigRequest, CDesignator* desigResponse) {
      return true;
    }
  }

  extern "C" plugins::PLUGIN_CLASS* createInstance() {
    return new plugins::PLUGIN_CLASS();
  }

  extern "C" void destroyInstance(plugins::PLUGIN_CLASS* icDestroy) {
    delete icDestroy;
  }
}
