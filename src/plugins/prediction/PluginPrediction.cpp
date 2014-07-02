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
      m_expOwl = NULL;
      m_bInsidePredictionModel = false;
      
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
      
      if(m_expOwl) {
	delete m_expOwl;
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
      
      // OWL Exporter instance for class ontology
      m_expOwl = new CExporterOwl();
      
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
	  this->descend(m_expOwl->owlClassForNode(evEvent.lstNodes.front(), true));
	} else {
	  this->warn("Consuming 'symbolic-begin-context' event without nodes!");
	}
      } else if(evEvent.strEventName == "symbolic-end-context") {
	if(evEvent.lstNodes.size() > 0) {
	  this->ascend(m_expOwl->owlClassForNode(evEvent.lstNodes.front(), true));
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
      CDesignator* desigResponse = new CDesignator();
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
	  m_bInsidePredictionModel = true;
	  
	  this->info("Okay, descending to toplevel.");
	  this->descend("Toplevel");
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
    
    bool PLUGIN_CLASS::descend(string strClass) {
      bool bResult = false;
      
      Property* prTopmost = NULL;
      Property* prSubs = NULL;

      m_mtxStackProtect.lock();
      if(m_lstPredictionStack.size() > 0) {
	prTopmost = m_lstPredictionStack.back().prLevel;
	
	if(prTopmost) {
	  prSubs = prTopmost->namedSubProperty("subs");
	}
      } else {
	prTopmost = m_jsnModel->rootProperty();
	
	if(prTopmost) {
	  prSubs = prTopmost->namedSubProperty("tree");
	}
      }
      
      list<string> lstSubClasses;
      
      if(prSubs) {
	for(Property* prSub : prSubs->subProperties()) {
	  Property* prClasses = prSub->namedSubProperty("class");
	  
	  if(prClasses) {
	    bool bFound = false;
	    bool bWildcardPresent = false;
	    
	    for(Property* prClass : prClasses->subProperties()) {
	      string strSubClass = prClass->getString();
	      
	      if(strSubClass == "*") {
		bWildcardPresent = true;
		this->info("Found a wildcard, using it.");
	      }
	      
	      lstSubClasses.push_back(strSubClass);
	    }
	    
	    for(Property* prClass : prClasses->subProperties()) {
	      string strSubClass = prClass->getString();
	      
	      if(strSubClass == strClass || strSubClass == "*") {
		// This is the sub property we're looking for.
		PredictionTrack ptTrack;
		ptTrack.prLevel = prSub;
		ptTrack.strClass = (bWildcardPresent ? "*" : strSubClass);
		m_lstPredictionStack.push_back(ptTrack);
		
		bFound = true;
		break;
	      }
	    }
	    
	    if(bFound) {
	      bResult = true;
	      break;
	    }
	  }
	}
      }
      
      if(bResult) {
	this->info("Descending by class: '" + strClass + "'");
      } else {
	this->warn("Couldn't descend by class: '" + strClass + "'");
	
	string strClasses = "";
	for(string strClass : lstSubClasses) {
	  strClasses += " " + strClass;
	}
	this->warn("Available classes:" + strClasses);
	
	if(m_bInsidePredictionModel) {
	  this->warn("Leaving prediction model, entering unknown sub-tree.");
	  m_bInsidePredictionModel = false;
	} else {
	  this->info("Continuing descent into unknown sub-tree.");
	}
	
	PredictionTrack ptTrack;
	ptTrack.prLevel = NULL;
	ptTrack.strClass = strClass;
	m_lstPredictionStack.push_back(ptTrack);
      }
      
      m_mtxStackProtect.unlock();
      
      return bResult;
    }
    
    bool PLUGIN_CLASS::ascend(string strClass) {
      bool bResult = false;
      
      m_mtxStackProtect.lock();
      if(m_lstPredictionStack.size() > 0) {
	bool bGoon = true;
	
	while(bGoon) {
	  PredictionTrack ptTrack = m_lstPredictionStack.back();
	  m_lstPredictionStack.pop_back();
	  
	  if(strClass == ptTrack.strClass || ptTrack.strClass == "*") {
	    if(m_lstPredictionStack.size() > 0) {
	      PredictionTrack ptTrackNew = m_lstPredictionStack.back();
	    
	      if(m_bInsidePredictionModel) {
		this->info("Ascending by class: '" + strClass + "'");
	      } else {
		if(ptTrackNew.prLevel) {
		  this->info("Ascending back into prediction model. Predictions possible again.");
		  m_bInsidePredictionModel = true;
		} else {
		  this->info("Ascending out of part of the unknown sub-tree.");
		}
	      }
	    } else {
	      this->fail("Careful: Ascending into empty prediction stack. This shouldn't happen, as at least a 'Toplevel' stack entry is expected.");
	    }
	    
	    bResult = true;
	  } else {
	    this->fail("Tried to ascend from '" + strClass + "', but we're in '" + ptTrack.strClass + "'. Moving up the chain.");
	  }
	  
	  if(m_lstPredictionStack.size() == 0 || bResult) {
	    bGoon = false;
	  }
	}
      } else {
	this->warn("Ascending although we have no prediction tree loaded. Did you load a model?");
      }
      
      m_mtxStackProtect.unlock();
      
      return bResult;
    }
    
    map<string, int> PLUGIN_CLASS::failuresForNode(string strNode) {
      map<string, int> mapFailures;
      Property* prRoot = m_jsnModel->rootProperty();
      
      if(prRoot) {
	Property* prFailures = prRoot->namedSubProperty("failures");
	
	if(prFailures) {
	  for(Property* prFailure : prFailures->subProperties()) {
	    string strEmitter = prFailure->namedSubProperty("emitter")->getString();
	    
	    if(strEmitter == strNode) {
	      string strFailure = prFailure->namedSubProperty("type")->getString();
	      
	      if(mapFailures[strFailure]) {
		mapFailures[strFailure]++;
	      } else {
		mapFailures[strFailure] = 1;
	      }
	    }
	  }
	}
      }
      
      return mapFailures;
    }
    
    pair<map<string, float>, float> PLUGIN_CLASS::predictBranch(Property* prBranch) {
      map<string, float> mapResult;
      float fSuccess = 1.0;
      
      // TODO(winkler): Do the actual prediction here.
      cout << "Predicting branch" << endl;
      
      map<string, int> mapCombinedFailures;
      Property* prNames = prBranch->namedSubProperty("names");
      
      if(prNames) {
	for(Property* prName : prNames->subProperties()) {
	  string strName = prName->getString();
	  map<string, int> mapFailures = this->failuresForNode(strName);
	  
	  for(auto itPair = mapFailures.begin(); itPair != mapFailures.end(); itPair++) {
	    pair<string, int> prPair = *itPair;
	    
	    if(mapCombinedFailures[prPair.first]) {
	      mapCombinedFailures[prPair.first] += prPair.second;
	    } else {
	      mapCombinedFailures[prPair.first] = prPair.second;
	    }
	  }
	}
      }
      
      int nAccumFailures = 0;
      for(auto itPair = mapCombinedFailures.begin(); itPair != mapCombinedFailures.end(); itPair++) {
	pair<string, int> prPair = *itPair;
	nAccumFailures += prPair.second;
      }
      
      for(auto itPair = mapCombinedFailures.begin(); itPair != mapCombinedFailures.end(); itPair++) {
	pair<string, int> prPair = *itPair;
	mapResult[prPair.first] = (float)prPair.second / (float)(nAccumFailures + 1);
      }
      
      Property* prSubs = prBranch->namedSubProperty("subs");
      
      if(prSubs) {
	for(Property* prSub : prSubs->subProperties()) {
	  pair<map<string, float>, float> prSubFailures = this->predictBranch(prSub);
	  
	  for(auto itPair = prSubFailures.first.begin(); itPair != prSubFailures.first.end(); itPair++) {
	    pair<string, float> prSubFailure = *itPair;
	    
	    if(mapResult[prSubFailure.first]) {
	      mapResult[prSubFailure.first] += prSubFailure.second * prSubFailures.second;
	    } else {
	      mapResult[prSubFailure.first] = prSubFailure.second * prSubFailures.second;
	    }
	  }
	}
      }
      
      fSuccess = 1.0f;
      for(auto itPair = mapResult.begin(); itPair != mapResult.end(); itPair++) {
	pair<string, int> prPair = *itPair;
	cout << prPair.first << "," << prPair.second << endl;
	fSuccess -= prPair.second;
      }
      
      return make_pair(mapResult, fSuccess);
    }
    
    bool PLUGIN_CLASS::predict(CDesignator* desigRequest, CDesignator* desigResponse) {
      bool bResult = false;
      
      m_mtxStackProtect.lock();
      if(m_bInsidePredictionModel) {
	Property* prRoot = m_jsnModel->rootProperty();
	
	if(prRoot) {
	  Property* prFailures = prRoot->namedSubProperty("failures");
	  map<string, float> mapPossibleFailures;
	  float fSuccess = 1.0f;
	  
	  if(prFailures) {
	    if(m_lstPredictionStack.size() > 0) {
	      Property* prCurrent = m_lstPredictionStack.back().prLevel;
	      pair<map<string, float>, float> prFailures = this->predictBranch(prCurrent);
	      
	      mapPossibleFailures = prFailures.first;
	      fSuccess = prFailures.second;
	      
	      bResult = true;
	    } else {
	      this->fail("There is no prediction stack. Returning a success of 100%. Maybe you forgot to load a model?");
	    }
	  } else {
	    this->warn("No failure instances present. This projects 100% success. Is this what you wanted?");
	    
	    bResult = true;
	  }
	  
	  desigResponse->setValue("success", fSuccess);
	  CKeyValuePair* ckvpFailures = desigResponse->addChild("failures");
	  ckvpFailures->setType(LIST);
	  
	  for(auto itPair = mapPossibleFailures.begin(); itPair != mapPossibleFailures.end(); itPair++) {
	    pair<string, float> prFailure = *itPair;
	    ckvpFailures->setValue(prFailure.first, prFailure.second);
	  }
	}
      } else {
	this->fail("Outside of the prediction model, no predictions are possible.");
      }
      m_mtxStackProtect.unlock();
      
      return bResult;
    }
  }

  extern "C" plugins::PLUGIN_CLASS* createInstance() {
    return new plugins::PLUGIN_CLASS();
  }

  extern "C" void destroyInstance(plugins::PLUGIN_CLASS* icDestroy) {
    delete icDestroy;
  }
}
