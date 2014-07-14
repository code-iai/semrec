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
      m_ndActive = NULL;
      
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
      this->setSubscribedToEvent("symbolic-node-active", true);
      
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
	  this->descend(evEvent.lstNodes.front()->title());
	} else {
	  this->warn("Consuming 'symbolic-begin-context' event without nodes!");
	}
      } else if(evEvent.strEventName == "symbolic-end-context") {
	if(evEvent.lstNodes.size() > 0) {
	  this->ascend(evEvent.lstNodes.front()->title());
	} else {
	  this->warn("Consuming 'symbolic-end-context' event without nodes!");
	}
      } else if(evEvent.strEventName == "symbolic-node-active") {
	if(evEvent.lstNodes.size() > 0) {
	  Node* ndNode = evEvent.lstNodes.front();
	  
	  this->info("Prediction context is now node '" + ndNode->title() + "'.");
	  m_ndActive = ndNode;
	} else {
	  this->info("Prediction context is now top-level (so not predicting).");
	  m_ndActive = NULL;
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
	  
	  this->info("Optimizing: Mapping node failures and parameters");
	  this->mapNodeFailuresParameters();
	  this->info("Optimized");
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
    
    void PLUGIN_CLASS::mapNodeFailuresParameters() {
      Property* prToplevel = NULL;
      list<Property*> lstLinearModel;
      
      m_mapNodeFailures.clear();
      m_mapNodeParameters.clear();
      
      if(m_jsnModel->rootProperty()->namedSubProperty("tree")->subProperties().size() > 0) {
	prToplevel = m_jsnModel->rootProperty()->namedSubProperty("tree")->subProperties().front();
      }
      
      if(prToplevel) {
	lstLinearModel = this->linearizeTree(prToplevel);
      }
      
      int nNodes = 0;
      int nFailures = 0;
      int nParameters = 0;
      
      for(Property* prNode : lstLinearModel) {
	nNodes++;
	list<string> lstNames;
	
	for(Property* prName : prNode->namedSubProperty("names")->subProperties()) {
	  lstNames.push_back(prName->getString());
	}
	
	list<Property*> lstFailures;
	list<Property*> lstParameters;
	for(Property* prFailureSet : m_jsnModel->rootProperty()->namedSubProperty("failures")->subProperties()) {
	  for(Property* prFailure : prFailureSet->namedSubProperty("failures")->subProperties()) {
	    if(find(lstNames.begin(), lstNames.end(), prFailure->namedSubProperty("emitter")->getString()) != lstNames.end()) {
	      // The emitter name is on our node's 'names' list
	      nFailures++;
	      lstFailures.push_back(prFailure);
	    }
	  }
	  
	  for(Property* prParameters : prFailureSet->namedSubProperty("parameters")->subProperties()) {
	    for(string strName : lstNames) {
	      if(prParameters->key() == strName) {
		nParameters++;
		lstParameters.push_back(prParameters->namedSubProperty(strName));
	      }
	    }
	  }
	}
	
	m_mapNodeFailures[prNode] = lstFailures;
	m_mapNodeParameters[prNode] = lstParameters;
      }
      
      stringstream sts;
      sts << "Mapped " << nFailures << " failures and " << nParameters << " parameter sets on " << nNodes << " nodes.";
      this->info(sts.str());
    }
    
    bool PLUGIN_CLASS::predict(CDesignator* desigRequest, CDesignator* desigResponse) {
      bool bResult = false;
      
      m_mtxStackProtect.lock();
      
      if(m_bInsidePredictionModel) {
	Property* prRoot = m_jsnModel->rootProperty();
	
	if(prRoot) {
	  if(m_lstPredictionStack.size() > 0) {
	    PredictionTrack ptCurrent = m_lstPredictionStack.back();
	    list<Property*> lstParameters;
	    PredictionResult presResult;
	    map<string, double> mapEffectiveFailureRates;
	    Property* prParameters = new Property();
	    prParameters->set(Property::Array);
	    
	    for(string strKey : desigRequest->keys()) {
	      Property* prParameter = new Property();
	      
	      prParameter->set(Property::Double);
	      prParameter->setKey(strKey);
	      prParameter->set((double)desigRequest->floatValue(strKey));
	      
	      lstParameters.push_back(prParameter);
	      prParameters->addSubProperty(prParameter);
	    }
	    
	    // Automatic tree walking
	    list<Property*> lstLinearTree = this->linearizeTree(ptCurrent.prLevel);
	    list<Property*> lstRunTree;
	    double dLeftOverSuccess = 1.0f;
	    
	    for(Property* prCurrent : lstLinearTree) {
	      lstRunTree.push_back(prCurrent);
	      
	      presResult = this->probability(lstRunTree, prParameters, lstParameters);
	      for(Failure flFailure : presResult.lstFailureProbabilities) {
	    	if(mapEffectiveFailureRates.find(flFailure.strClass) == mapEffectiveFailureRates.end()) {
		  mapEffectiveFailureRates[flFailure.strClass] = 0.0f;
		}
		
		mapEffectiveFailureRates[flFailure.strClass] += (flFailure.dProbability * dLeftOverSuccess);
		dLeftOverSuccess -= (flFailure.dProbability * dLeftOverSuccess);
	      }
	    }
	    
	    CKeyValuePair* ckvpFailures = desigResponse->addChild("failures");
	    presResult.dSuccessRate = 1.0f;
	    for(auto itMap = mapEffectiveFailureRates.begin(); itMap != mapEffectiveFailureRates.end(); itMap++) {
	      pair<string, double> prFailure = *itMap;
	      ckvpFailures->setValue(prFailure.first, prFailure.second);
	      
	      presResult.dSuccessRate -= prFailure.second;
	    }

	    desigResponse->setValue("success", presResult.dSuccessRate);
	    
	    delete prParameters;
	    lstParameters.clear();
	    
	    bResult = true;
	  } else {
	    this->fail("Nothing on the prediction stack. Toplevel not loaded. This is bad.");
	  }
	} else {
	  this->fail("No root element loaded. Did you load a prediction model?");
	}
      } else {
	this->fail("Outside of prediction model, no predictions possible.");
      }
      
      m_mtxStackProtect.unlock();
      
      return bResult;
    }
    
    list<Property*> PLUGIN_CLASS::linearizeTree(Property* prTop) {
      list<Property*> lstProps;
      lstProps.push_back(prTop);
      
      Property* prSubs = prTop->namedSubProperty("subs");
      
      if(prSubs) {
	for(Property* prSub : prSubs->subProperties()) {
	  list<Property*> lstSubProps = this->linearizeTree(prSub);
	  
	  for(Property* prSubSub : lstSubProps) {
	    lstProps.push_back(prSubSub);
	  }
	}
      }
      
      return lstProps;
    }
    
    map<string, double> PLUGIN_CLASS::relativeFailureOccurrences(list<Property*> lstFailures, int nTracks) {
      map<string, int> mapFailureOcc;
      
      for(Property* prFailure : lstFailures) {
	string strType = prFailure->namedSubProperty("type")->getString();
	
	if(mapFailureOcc[strType] == 0) {
	  mapFailureOcc[strType] = 1;
	} else {
	  mapFailureOcc[strType]++;
	}
      }
      
      map<string, double> mapFailureRelOcc;
      for(auto itMap = mapFailureOcc.begin(); itMap != mapFailureOcc.end(); itMap++) {
	pair<string, int> prFailOcc = *itMap;
	
	mapFailureRelOcc[prFailOcc.first] = (double)prFailOcc.second / (double)nTracks;
      }
      
      return mapFailureRelOcc;
    }
    
    list<Property*> PLUGIN_CLASS::failuresForTreeNode(Property* prNode) {
      if(m_mapNodeFailures.find(prNode) != m_mapNodeFailures.end()) {
	return m_mapNodeFailures[prNode];
      }
      
      list<Property*> lstEmpty;
      return lstEmpty;
    }
    
    list<Property*> PLUGIN_CLASS::parametersForTreeNode(Property* prNode) {
      if(m_mapNodeParameters.find(prNode) != m_mapNodeParameters.end()) {
	return m_mapNodeParameters[prNode];
      }
      
      list<Property*> lstEmpty;
      return lstEmpty;
    }
    
    map<string, double> PLUGIN_CLASS::relativeFailuresForNode(Property* prNode, Property* prParameters) {
      map<string, double> mapRelFail;
      
      int nBranches = prNode->namedSubProperty("names")->subProperties().size();
      
      list<Property*> lstFailures = this->failuresForTreeNode(prNode);
      map<string, double> mapFailureRelOcc = this->relativeFailureOccurrences(lstFailures, nBranches);
      
      if(prNode) {
	Property* prClass = prNode->namedSubProperty("class");
	string strTaskContext = "";
	
	if(prClass) {
	  if(prClass->subProperties().size() > 0) {
	    strTaskContext = prClass->subProperties().front()->getString();
	  }
	}
	
	if(strTaskContext == "") {
	  this->fail("No task context found for node. Prediction is likely going to fail badly. Expect wrong behaviour.");
	}
	
      	bool bNavX = prParameters->namedSubProperty("NAVIGATE-TO-X");
      	bool bNavY = prParameters->namedSubProperty("NAVIGATE-TO-Y");
      	bool bObjDist = prParameters->namedSubProperty("OBJ-DIST");
	
      	double dNavX = (bNavX ? prParameters->namedSubProperty("NAVIGATE-TO-X")->getDouble() : 0);
      	double dNavY = (bNavY ? prParameters->namedSubProperty("NAVIGATE-TO-Y")->getDouble() : 0);
      	double dObjDist = (bObjDist ? prParameters->namedSubProperty("OBJ-DIST")->getDouble() : 0);
	
      	if(bNavY && dNavY <= 5) { // nav y <= 5
	  if(strTaskContext == "MODEL-PLAN") {
	    // Success
	  } else if(strTaskContext == "WITH-FAILURE-HANDLING") {
	    if(bNavX && dNavX > 7) { // nav x > 7
	      mapRelFail["LOCATION-NOT-REACHED-FAILURE"] = 1.0f;
	    }
	  } else if(strTaskContext == "REPLACEABLE-FUNCTION-MODEL-PLAN") {
	    // Success
	  } else if(strTaskContext == "WITH-DESIGNATORS") {
	    // Success
	  } else if(strTaskContext == "REPLACEABLE-FUNCTION-PERFORM-PLAN-TASKS") {
	    // Success
	  } else if(strTaskContext == "REPLACEABLE-FUNCTION-GO-TO") {
	    // Success
	  } else if(strTaskContext == "REPLACEABLE-FUNCTION-GRASP") {
	    if(bObjDist && dObjDist <= 3) {
	      // Success
	    } else {
	      mapRelFail["MANIPULATION-FAILURE"] = 1.0f;
	    }
	  } else if(strTaskContext == "REPLACEABLE-FUNCTION-FIND-OBJ") {
	    if(bObjDist && dObjDist <= 4.47214) {
	      // Success
	    } else {
	      mapRelFail["OBJECT-NOT-FOUND"] = 1.0f;
	    }
	  }
	} else { // nav y > 5
	  if(strTaskContext == "MODEL-PLAN") {
	    // Success
	  } else if(strTaskContext == "WITH-FAILURE-HANDLING") {
	    mapRelFail["LOCATION-NOT-REACHED-FAILURE"] = 1.0f;
	  } else if(strTaskContext == "REPLACEABLE-FUNCTION-MODEL-PLAN") {
	    // Success
	  } else if(strTaskContext == "WITH-DESIGNATORS") {
	    // Success
	  } else if(strTaskContext == "REPLACEABLE-FUNCTION-PERFORM-PLAN-TASKS") {
	    // Success
	  } else if(strTaskContext == "REPLACEABLE-FUNCTION-GO-TO") {
	    // Success
	  } else if(strTaskContext == "REPLACEABLE-FUNCTION-GRASP") {
	    // Success
	  } else if(strTaskContext == "REPLACEABLE-FUNCTION-FIND-OBJ") {
	    // Success
	  }
	}
      }

      return mapRelFail;
    }
    
    PLUGIN_CLASS::PredictionResult PLUGIN_CLASS::probability(list<Property*> lstSequence, Property* prParameters, list<Property*> lstParameters) {
      PredictionResult presResult;
      
      if(lstSequence.size() > 0) {
	Property* prCurrentNode = lstSequence.back();
	
	// Relative occurrences
	map<string, double> mapFailureRelOcc = this->relativeFailuresForNode(prCurrentNode, prParameters);
	
	double dLocalSuccess = 1.0f;
	for(auto itMap = mapFailureRelOcc.begin(); itMap != mapFailureRelOcc.end(); itMap++) {
	  pair<string, double> prPair = *itMap;
	  dLocalSuccess -= prPair.second;
	}
	
	list<Failure> lstSubFailures;
	if(dLocalSuccess > 0.0f) { // Recursion only makes sense if we can actually reach the sub-node.
	  // Prepare list for recursion
	  list<Property*> lstAllButLast = lstSequence;
	  lstAllButLast.pop_back();
	  
	  // Recurse
	  PredictionResult presSub = this->probability(lstAllButLast, prParameters, lstParameters);
	  lstSubFailures = presSub.lstFailureProbabilities;
	} else {
	  lstSequence.clear();
	}
	
	// Add the failures from THIS node
	for(auto itMap = mapFailureRelOcc.begin(); itMap != mapFailureRelOcc.end(); itMap++) {
	  pair<string, double> prMap = *itMap;
	  
	  Failure flFailure;
	  flFailure.strClass = prMap.first;
	  flFailure.dProbability = prMap.second;
	  presResult.lstFailureProbabilities.push_back(flFailure);
	}
	
	double dLocalSuccessDelta = 0.0f;
	
	// Add the failures from CHILD node
	for(Failure flFailure : lstSubFailures) {
	  Failure flFailureScaled;
	  flFailureScaled = flFailure;
	  flFailureScaled.dProbability *= dLocalSuccess;
	  presResult.lstFailureProbabilities.push_back(flFailureScaled);
	  
	  dLocalSuccessDelta += flFailureScaled.dProbability;
	}
	
	presResult.dSuccessRate = dLocalSuccess - dLocalSuccessDelta;
      } else {
	// This is the node we already passed. This is S0, which has
	// 100% success rate per definition.
	presResult.dSuccessRate = 1.0f;
      }
      
      return presResult;
    }
  }
  
  extern "C" plugins::PLUGIN_CLASS* createInstance() {
    return new plugins::PLUGIN_CLASS();
  }
  
  extern "C" void destroyInstance(plugins::PLUGIN_CLASS* icDestroy) {
    delete icDestroy;
  }
}
