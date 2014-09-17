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
      m_jsnModel = NULL;
      m_expOwl = NULL;
      m_bInsidePredictionModel = false;
      m_ndActive = NULL;
      m_bModelLoaded = false;
      m_nClassFlexibility = 6;
      
      this->setPluginVersion("0.1");
    }

    PLUGIN_CLASS::~PLUGIN_CLASS() {
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
      
      this->setOffersService("predict", true);
      this->setOffersService("load-model", true);
      
      // Prepare the JSON prediction model parser
      m_jsnModel = new JSON();
      
      // OWL Exporter instance for class ontology
      m_expOwl = new CExporterOwl();
      
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
    
    Event PLUGIN_CLASS::consumeServiceEvent(ServiceEvent seEvent) {
      Event evReturn = this->Plugin::consumeServiceEvent(seEvent);
      
      if(seEvent.siServiceIdentifier == SI_REQUEST) {
	if(seEvent.strServiceName == "predict") {
	  ServiceEvent seResponse = eventInResponseTo(seEvent);
	  CDesignator* cdRequest = seEvent.cdDesignator;
	  
	  seResponse.bPreserve = true;
	  CDesignator* cdResponse = new CDesignator();
	  cdResponse->setType(ACTION);
	  
	  bool bSuccess = false;
	  if(m_bModelLoaded) {
	    this->info("Received Prediction Request.");
	    
	    if(!this->predict(cdRequest, cdResponse)) {
	      this->fail("Failed to predict!");
	      
	      cdResponse->setValue("success", false);
	      cdResponse->setValue("message", "Failed to predict.");
	    } else {
	      cdResponse->setValue("success", true);
	    }
	  } else {
	    this->warn("Received Prediction Request without loaded prediction model. Ignoring.");
	    
	    cdResponse->setValue("success", false);
	    cdResponse->setValue("message", "No model loaded.");
	  }
	  
	  seResponse.cdDesignator = cdResponse;
	  this->deployServiceEvent(seResponse);
	} else if(seEvent.strServiceName == "load-model") {
	  CDesignator* cdRequest = seEvent.cdDesignator;
	  
	  if(cdRequest) {
	    if(cdRequest->stringValue("type") == "task-tree") {
	      std::string strPath = cdRequest->stringValue("file");
	      
	      if(strPath != "") {
		this->info("Load task tree model: '" + strPath + "'");
		
		if(this->load(strPath)) {
		  this->fail("Failed to load task tree model.");
		}
	      } else {
		this->fail("No file specified for loading task tree model.");
	      }
	    } else if(cdRequest->stringValue("type") == "decision-tree") {
	      std::string strPath = cdRequest->stringValue("file");
	      this->info("Load decision tree model: '" + strPath + "'");
	      
	      if(strPath != "") {
		if(this->loadDecisionTree(strPath)) {
		  this->info("Finished loading decision tree model.");
		} else {
		  this->fail("Failed to load decision tree model.");
		}
	      } else {
		this->fail("No file specified for loading decision tree model.");
	      }
	    }
	  }
	}
      }
      
      return evReturn;
    }
    
    void PLUGIN_CLASS::consumeEvent(Event evEvent) {
      if(evEvent.strEventName == "symbolic-begin-context") {
	if(m_bInsidePredictionModel) {
	  if(evEvent.lstNodes.size() > 0) {
	    this->descend(evEvent.lstNodes.front());
	    issueGlobalToken("symbolic-context-began");
	  } else {
	    this->warn("Consuming 'symbolic-begin-context' event without nodes!");
	  }
	} else {
	  this->fail("No prediction model loaded, won't descend.");
	}
      } else if(evEvent.strEventName == "symbolic-end-context") {
	if(evEvent.lstNodes.size() > 0) {
	  this->ascend(evEvent.lstNodes.front());
	} else {
	  this->warn("Consuming 'symbolic-end-context' event without nodes!");
	}
      } else if(evEvent.strEventName == "symbolic-node-active") {
	if(evEvent.lstNodes.size() > 0) {
	  Node* ndNode = evEvent.lstNodes.front();
	  
	  this->info("Prediction context is now node '" + ndNode->title() + "', of class '" + m_expOwl->owlClassForNode(ndNode, true) + "'.");
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
      CDesignator* desigResponse = new CDesignator();
      desigResponse->setType(ACTION);
      
      if(desigRequest->stringValue("load") == "model") {
        this->info("Received Model Load Request.");
	
        std::string strFile = desigRequest->stringValue("file");
        if(strFile != "") {
          this->info(" - Load model: '" + strFile + "'");
	  
          bSuccess = this->load(strFile);
	  
	  if(bSuccess) {
	    m_bModelLoaded = true;
	  }
        } else {
          this->fail(" - No model file specified!");
        }
      }
      
      res.response.designators.push_back(desigResponse->serializeToMessage());
      
      delete desigRequest;
      delete desigResponse;
      
      return bSuccess;
    }
    
    bool PLUGIN_CLASS::load(string strFile) {
      bool bReturn = false;
      
      std::ifstream ifFile(strFile.c_str());
      if(ifFile.good()) {
	std::string strJSON((istreambuf_iterator<char>(ifFile)), (istreambuf_iterator<char>()));
	
	m_jsnModel->parse(strJSON);
	
	if(m_jsnModel->rootProperty()) {
	  this->info("Successfully loaded and parsed model '" + strFile + "'.");
	  bReturn = true;
	  m_bInsidePredictionModel = true;
	  
	  this->info("Okay, descending to 'Toplevel'.");
	  this->descend("Toplevel");
	  
	  // this->info("Optimizing: Mapping node failures and parameters");
	  // this->mapNodeFailuresParameters();
	  // this->info("Optimizing: Timestamps");
	  // this->mapTimeStamps();
	  // this->info("Optimized");
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
    
    bool PLUGIN_CLASS::descend(Node* ndDescend) {
      return this->descend(m_expOwl->owlClassForNode(ndDescend, true));
    }
    
    bool PLUGIN_CLASS::descend(std::string strClass, bool bForceClass) {
      bool bResult = false;
      bool bFlexible = false;
      
      m_mtxStackProtect.lock();
      Property* prDescendTo = NULL;
      
      if(m_lstPredictionStack.size() == 0) {
	// We are on top-level and must look for the class to ascend
	// into in the current JSON model keys.
	prDescendTo = m_jsnModel->rootProperty()->namedSubProperty(strClass);
      } else {
	// We are already somewhere in the prediction tree and must
	// find a fitting subType in the currently active state.
	if(m_bInsidePredictionModel) {
	  Property* prActive = m_lstPredictionStack.back().prLevel;
	  
	  if(prActive->namedSubProperty("subTypes")) {
	    prDescendTo = prActive->namedSubProperty("subTypes")->namedSubProperty(strClass);
	  }
	  
	  if(!prDescendTo) {
	    // There was no subType to descend to. Check if we are
	    // still flexible for other types. If that is the case,
	    // use the first type that comes up.
	    std::string strClasses = "";
	    for(Property* prType : prActive->namedSubProperty("subTypes")->subProperties()) {
	      if(strClasses != "") {
		strClasses += " ";
	      }
	      
	      strClasses += prType->key();
	    }
	    
	    this->warn("No match for descend. Available classes: " + strClasses);
	    
	    if(m_nClassFlexibility > 0) {
	      // Yes, we are flexible. Check if there is at least one subType.
	      if(prActive->namedSubProperty("subTypes")->subProperties().size() > 0) {
		// There are subTypes. Use the first one.
		this->warn("Being flexible: Accept '" + prActive->namedSubProperty("subTypes")->subProperties().front()->key() + "' for '" + strClass + "'");
		prDescendTo = prActive->namedSubProperty("subTypes")->subProperties().front();
		bFlexible = true;
		m_nClassFlexibility--;
	      }
	    }
	  }
	}
      }
      
      if(prDescendTo) {
	m_bInsidePredictionModel = true;
	
       	this->info("Descending by class: '" + strClass + "'");
	
	PredictionTrack ptTrack;
	ptTrack.prLevel = prDescendTo;
	ptTrack.strClass = (bFlexible ? "*" : strClass);
	m_lstPredictionStack.push_back(ptTrack);
	
	bResult = true;
      } else {
      	this->warn("Couldn't descend by class: '" + strClass + "'");
	
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
    
    bool PLUGIN_CLASS::ascend(Node* ndAscend) {
      bool bResult = false;
      
      std::string strClass = m_expOwl->owlClassForNode(ndAscend, true);
      
      m_mtxStackProtect.lock();
      if(m_lstPredictionStack.size() > 0) {
	bool bGoon = true;
	
	while(bGoon) {
	  PredictionTrack ptTrack = m_lstPredictionStack.back();
	  m_lstPredictionStack.pop_back();
	  
	  if(strClass == ptTrack.strClass || ptTrack.strClass == "*") {
	    if(ptTrack.strClass == "*") {
	      m_nClassFlexibility++;
	    }
	    
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
	    bResult = true;
	  }
	  
	  if(m_lstPredictionStack.size() == 0 || bResult) {
	    bGoon = false;
	  }
	}
      } else {
	this->warn("Ascending although we have no prediction tree loaded. Did you load a model? (Hint: The answer is 'No'.)");
      }
      
      m_mtxStackProtect.unlock();
      
      return bResult;
    }
    
    void PLUGIN_CLASS::mapNodeFailuresParameters() {
      // Property* prToplevel = NULL;
      // std::list< pair<Property*, int> > lstLinearModel;
      
      // m_mapNodeFailures.clear();
      // m_mapNodeParameters.clear();
      
      // if(m_jsnModel->rootProperty()->namedSubProperty("tree")->subProperties().size() > 0) {
      // 	prToplevel = m_jsnModel->rootProperty()->namedSubProperty("tree")->subProperties().front();
      // }
      
      // if(prToplevel) {
      // 	lstLinearModel = this->linearizeTree(prToplevel, 0);
      // }
      
      // int nNodes = 0;
      // int nFailures = 0;
      // int nParameters = 0;
      
      // for(pair<Property*, int> prNode : lstLinearModel) {
      // 	nNodes++;
      // 	std::list<std::string> lstNames;
	
      // 	for(Property* prName : prNode.first->namedSubProperty("names")->subProperties()) {
      // 	  lstNames.push_back(prName->getString());
      // 	}
	
      // 	std::list<Property*> lstFailures;
      // 	std::list<Property*> lstParameters;
      // 	for(Property* prFailureSet : m_jsnModel->rootProperty()->namedSubProperty("failures")->subProperties()) {
      // 	  for(Property* prFailure : prFailureSet->namedSubProperty("failures")->subProperties()) {
      // 	    if(find(lstNames.begin(), lstNames.end(), prFailure->namedSubProperty("emitter")->getString()) != lstNames.end()) {
      // 	      // The emitter name is on our node's 'names' list
      // 	      nFailures++;
      // 	      lstFailures.push_back(prFailure);
      // 	    }
      // 	  }
	  
      // 	  for(Property* prParameters : prFailureSet->namedSubProperty("parameters")->subProperties()) {
      // 	    for(string strName : lstNames) {
      // 	      if(prParameters->key() == strName) {
      // 		nParameters++;
      // 		lstParameters.push_back(prParameters->namedSubProperty(strName));
      // 	      }
      // 	    }
      // 	  }
      // 	}
	
      // 	m_mapNodeFailures[prNode.first] = lstFailures;
      // 	m_mapNodeParameters[prNode.first] = lstParameters;
      // }
      
      // std::stringstream sts;
      // sts << "Mapped " << nFailures << " failures and " << nParameters << " parameter sets on " << nNodes << " nodes.";
      // this->info(sts.str());
    }
    
    void PLUGIN_CLASS::mapTimeStamps() {
      // Property* prTimestamps = NULL;
      // m_mapTimeStamps.clear();
      
      // if(m_jsnModel->rootProperty()->namedSubProperty("timestamps")->subProperties().size() > 0) {
      // 	prTimestamps = m_jsnModel->rootProperty()->namedSubProperty("timestamps");
      // }
      
      // if(prTimestamps) {
      // 	for(Property* prTimestamp : prTimestamps->subProperties()) {
      // 	  m_mapTimeStamps[prTimestamp->key()] = make_pair(prTimestamp->namedSubProperty("start")->getInteger(),
      // 							  prTimestamp->namedSubProperty("end")->getInteger());
      // 	}
      // }
    }
    
    PLUGIN_CLASS::PredictionResult PLUGIN_CLASS::evaluatePredictionRequest(Property* prActive, CKeyValuePair* ckvpFeatures, CKeyValuePair* ckvpRequested) {
      PredictionResult presResult;
      
      // Here, the actual prediction takes place.
      std::map<std::string, double> mapEffectiveFailureRates;
      
      // Automatic tree walking
      std::list< pair<Property*, int> > lstLinearTree = this->linearizeTree(prActive, m_lstPredictionStack.size());
      std::list< pair<Property*, int> > lstRunTree;
      double dLeftOverSuccess = 1.0f;
      
      for(pair<Property*, int> prCurrent : lstLinearTree) {
	lstRunTree.push_back(prCurrent);
	
	presResult = this->probability(lstRunTree, ckvpFeatures, ckvpRequested);
	
	for(pair<std::string, double> prFailure : presResult.mapFailureProbabilities) {
	  if(mapEffectiveFailureRates.find(prFailure.first) == mapEffectiveFailureRates.end()) {
	    mapEffectiveFailureRates[prFailure.first] = 0.0f;
	  }
	  
	  mapEffectiveFailureRates[prFailure.first] += (prFailure.second * dLeftOverSuccess);
	  dLeftOverSuccess -= (prFailure.second * dLeftOverSuccess);
	}
      }
      
      presResult.dSuccessRate = 1.0f;
      
      for(std::pair<std::string, double> prFailure : mapEffectiveFailureRates) {
	presResult.mapFailureProbabilities[prFailure.first] = prFailure.second;
	presResult.dSuccessRate -= prFailure.second;
      }
      
      return presResult;
    }
    
    bool PLUGIN_CLASS::predict(CDesignator* cdRequest, CDesignator* cdResponse) {
      bool bResult = false;
      
      m_mtxStackProtect.lock();
      
      if(m_bInsidePredictionModel) {
	// Make sure there is a model present before diving into the
	// tree.
	if(m_jsnModel->rootProperty()) {
	  // Make sure we are actually in a valid prediction state.
	  if(m_lstPredictionStack.size() > 0) {
	    PredictionTrack ptCurrent = m_lstPredictionStack.back();
	    this->info("Predicting for class: '" + ptCurrent.strClass + "' at level " + this->str((int)m_lstPredictionStack.size()));
	    
	    CKeyValuePair* ckvpActiveFeatures = cdRequest->childForKey("active-features");
	    CKeyValuePair* ckvpRequestedFeatures = cdRequest->childForKey("requested-features");
	    
	    if(ckvpActiveFeatures) {
	      std::string strActiveFeatures = "";
	      
	      for(std::string strKey : ckvpActiveFeatures->keys()) {
		if(strActiveFeatures != "") {
		  strActiveFeatures += ", ";
		}
		
		strActiveFeatures += strKey;
	      }
	      
	      this->info("Active features: " + strActiveFeatures);
	    }
	    
	    if(ckvpRequestedFeatures) {
	      std::string strRequestedFeatures = "";
	      
	      for(std::string strKey : ckvpRequestedFeatures->keys()) {
		if(strRequestedFeatures != "") {
		  strRequestedFeatures += ", ";
		}
		
		strRequestedFeatures += strKey;
	      }
	      
	      this->info("Requested features: " + strRequestedFeatures);
	    }
	    
	    // Here, the actual prediction is triggered.
	    Property* prActive = m_lstPredictionStack.back().prLevel;
	    PredictionResult presResult = this->evaluatePredictionRequest(prActive, ckvpActiveFeatures, ckvpRequestedFeatures);
	    
	    // Prepare the results for returning them.
	    // Failure probabilities
	    CKeyValuePair* ckvpFailureProbabilities = cdResponse->addChild("failures");
	    for(pair<std::string, double> prFailure : presResult.mapFailureProbabilities) {
	      ckvpFailureProbabilities->setValue(prFailure.first, prFailure.second);
	    }
	    
	    // Requested feature values
	    CKeyValuePair* ckvpRequestedFeatureValues = cdResponse->addChild("requested");
	    for(pair<std::string, Property*> prReq : presResult.mapRequestedFeatureValues) {
	      ckvpRequestedFeatureValues->addChild(prReq.first, prReq.second);
	    }
	    
	    // Overall success rate
	    cdResponse->setValue("success", presResult.dSuccessRate);
	    
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
    
    std::list< pair<Property*, int> > PLUGIN_CLASS::linearizeTree(Property* prTop, int nLevel) {
      std::list< pair<Property*, int> > lstProps;
      lstProps.push_back(make_pair(prTop, nLevel));
      
      Property* prSubs = prTop->namedSubProperty("subTypes");
      
      if(prSubs) {
	for(Property* prSub : prSubs->subProperties()) {
	  std::list< pair<Property*, int> > lstSubProps = this->linearizeTree(prSub, nLevel + 1);
	  
	  for(pair<Property*, int> prSubSub : lstSubProps) {
	    lstProps.push_back(make_pair(prSubSub.first, prSubSub.second));
	  }
	}
      }
      
      return lstProps;
    }
    
    std::map<std::string, double> PLUGIN_CLASS::relativeFailureOccurrences(std::list<Property*> lstFailures, int nTracks) {
      std::map<std::string, int> mapFailureOcc;
      
      for(Property* prFailure : lstFailures) {
	std::string strType = prFailure->namedSubProperty("type")->getString();
	
	if(mapFailureOcc[strType] == 0) {
	  mapFailureOcc[strType] = 1;
	} else {
	  mapFailureOcc[strType]++;
	}
      }
      
      std::map<std::string, double> mapFailureRelOcc;
      for(std::pair<std::string, int> prFailOcc : mapFailureOcc) {
	mapFailureRelOcc[prFailOcc.first] = (double)prFailOcc.second / (double)nTracks;
      }
      
      return mapFailureRelOcc;
    }
    
    std::list<Property*> PLUGIN_CLASS::failuresForTreeNode(Property* prNode) {
      if(m_mapNodeFailures.find(prNode) != m_mapNodeFailures.end()) {
	return m_mapNodeFailures[prNode];
      }
      
      std::list<Property*> lstEmpty;
      return lstEmpty;
    }
    
    std::list<Property*> PLUGIN_CLASS::parametersForTreeNode(Property* prNode) {
      if(m_mapNodeParameters.find(prNode) != m_mapNodeParameters.end()) {
	return m_mapNodeParameters[prNode];
      }
      
      std::list<Property*> lstEmpty;
      return lstEmpty;
    }
    
    std::map<std::string, double> PLUGIN_CLASS::evaluateDecisionTree(std::string strClass, int nLevel, CKeyValuePair* ckvpFeatures) {
      std::map<std::string, double> mapRelFail;
      
      this->info("Evaluate decision tree for class '" + strClass + "' on level '" + this->str(nLevel) + "'.");
      
      bool bNavX = (ckvpFeatures->childForKey("nav-x") != NULL);
      bool bNavY = (ckvpFeatures->childForKey("nav-y") != NULL);
      bool bDistance = (ckvpFeatures->childForKey("distance") != NULL);
      
      double dNavX = 0.0f;
      if(bNavX) {
	dNavX = ckvpFeatures->floatValue("nav-x");
      }
      
      double dNavY = 0.0f;
      if(bNavY) {
	dNavY = ckvpFeatures->floatValue("nav-y");
      }

      double dDistance = 0.0f;
      if(bDistance) {
	dDistance = ckvpFeatures->floatValue("distance");
      }
      
      std::cout << dNavX << ", " << dNavY << ", " << dDistance << std::endl;
      
      if(nLevel == 9) {
	if(bNavX && bNavY) {
	  if(dNavY <= 3) {
	    if(dNavX <= 2 || dNavX > 8) {
	      mapRelFail["LocationNotReached"] = 0.981;
	    }
	  } else {
	    if(dNavY > 8.9) {
	      if(dNavX <= 1.9 || dNavX > 8) {
		mapRelFail["LocationNotReached"] = 0.981;
	      }
	    }
	  }
	}
      } else if(nLevel == 10) {
	if(bDistance) {
	  if(dDistance > 4.9679) {
	    mapRelFail["ObjectNotFound"] = 0.999;
	  }
	}
      }
      
      return mapRelFail;
    }
    
    std::map<std::string, double> PLUGIN_CLASS::relativeFailuresForNode(Property* prNode, int nLevel, CKeyValuePair* ckvpFeatures) {
      std::map<std::string, double> mapRelFail;
      
      if(prNode) {
	std::string strClass = prNode->key();
	
	mapRelFail = this->evaluateDecisionTree(strClass, nLevel, ckvpFeatures);
      }
      
      // 	bool bNavX = prParameters->namedSubProperty("NAVIGATE-TO-X");
      // 	bool bNavY = prParameters->namedSubProperty("NAVIGATE-TO-Y");
      // 	bool bObjDist = prParameters->namedSubProperty("OBJ-DIST");
	
      // 	double dNavX = (bNavX ? prParameters->namedSubProperty("NAVIGATE-TO-X")->getDouble() : 0);
      // 	double dNavY = (bNavY ? prParameters->namedSubProperty("NAVIGATE-TO-Y")->getDouble() : 0);
      // 	double dObjDist = (bObjDist ? prParameters->namedSubProperty("OBJ-DIST")->getDouble() : 0);
	
      // 	if(bNavY && dNavY <= 5) { // nav y <= 5
      // 	  if(strTaskContext == "MODEL-PLAN") {
      // 	    // Success
      // 	  } else if(strTaskContext == "WITH-FAILURE-HANDLING") {
      // 	    if(bNavX && dNavX > 7) { // nav x > 7
      // 	      mapRelFail["LOCATION-NOT-REACHED-FAILURE"] = 1.0f;
      // 	    }
      // 	  } else if(strTaskContext == "REPLACEABLE-FUNCTION-MODEL-PLAN") {
      // 	    // Success
      // 	  } else if(strTaskContext == "WITH-DESIGNATORS") {
      // 	    // Success
      // 	  } else if(strTaskContext == "REPLACEABLE-FUNCTION-PERFORM-PLAN-TASKS") {
      // 	    // Success
      // 	  } else if(strTaskContext == "REPLACEABLE-FUNCTION-GO-TO") {
      // 	    // Success
      // 	  } else if(strTaskContext == "REPLACEABLE-FUNCTION-GRASP") {
      // 	    if(bObjDist && dObjDist <= 3) {
      // 	      // Success
      // 	    } else {
      // 	      if(bObjDist) {
      // 		mapRelFail["MANIPULATION-FAILURE"] = 1.0f;
      // 	      }
      // 	    }
      // 	  } else if(strTaskContext == "REPLACEABLE-FUNCTION-FIND-OBJ") {
      // 	    if(bObjDist && dObjDist <= 4.47214) {
      // 	      // Success
      // 	    } else {
      // 	      if(bObjDist) {
      // 		mapRelFail["OBJECT-NOT-FOUND"] = 1.0f;
      // 	      }
      // 	    }
      // 	  }
      // 	} else { // nav y > 5
      // 	  if(strTaskContext == "MODEL-PLAN") {
      // 	    // Success
      // 	  } else if(strTaskContext == "WITH-FAILURE-HANDLING") {
      // 	    if(bNavY) {
      // 	      mapRelFail["LOCATION-NOT-REACHED-FAILURE"] = 1.0f;
      // 	    }
      // 	  } else if(strTaskContext == "REPLACEABLE-FUNCTION-MODEL-PLAN") {
      // 	    // Success
      // 	  } else if(strTaskContext == "WITH-DESIGNATORS") {
      // 	    // Success
      // 	  } else if(strTaskContext == "REPLACEABLE-FUNCTION-PERFORM-PLAN-TASKS") {
      // 	    // Success
      // 	  } else if(strTaskContext == "REPLACEABLE-FUNCTION-GO-TO") {
      // 	    // Success
      // 	  } else if(strTaskContext == "REPLACEABLE-FUNCTION-GRASP") {
      // 	    // Success
      // 	  } else if(strTaskContext == "REPLACEABLE-FUNCTION-FIND-OBJ") {
      // 	    // Success
      // 	  }
      // 	}
      
      return mapRelFail;
    }
    
    PLUGIN_CLASS::PredictionResult PLUGIN_CLASS::probability(std::list< pair<Property*, int> > lstSequence, CKeyValuePair* ckvpFeatures, CKeyValuePair* ckvpRequested) {
      PredictionResult presResult;
      
      // TODO(winkler): Right now, the requested features
      // `ckvpRequested' are not evaluated. Implement this.
      
      if(lstSequence.size() > 0) {
	Property* prCurrentNode = lstSequence.back().first;
	
	// Relative occurrences
	std::map<std::string, double> mapFailureRelOcc = this->relativeFailuresForNode(prCurrentNode, lstSequence.back().second, ckvpFeatures);
	
	double dLocalSuccess = 1.0f;
	for(std::pair<std::string, double> prPair : mapFailureRelOcc) {
	  dLocalSuccess -= prPair.second;
	}
	
	std::map<std::string, double> mapSubFailures;
	if(dLocalSuccess > 0.0f) { // Recursion only makes sense if we can actually reach the sub-node.
	  // Prepare list for recursion
	  std::list< pair<Property*, int> > lstAllButLast = lstSequence;
	  lstAllButLast.pop_back();
	  
	  // Recurse
	  PredictionResult presSub = this->probability(lstAllButLast, ckvpFeatures, ckvpRequested);
	  mapSubFailures = presSub.mapFailureProbabilities;
	} else {
	  lstSequence.clear();
	}
	
	// Add the failures from THIS node
	for(std::pair<std::string, double> prMap : mapFailureRelOcc) {
	  presResult.mapFailureProbabilities[prMap.first] = prMap.second;
	}
	
	double dLocalSuccessDelta = 0.0f;
	
	// Add the failures from CHILD node
	for(pair<std::string, double> prFailure : mapSubFailures) {
	  presResult.mapFailureProbabilities[prFailure.first] = prFailure.second * dLocalSuccess;
	  dLocalSuccessDelta += presResult.mapFailureProbabilities[prFailure.first];
	}
	
	presResult.dSuccessRate = dLocalSuccess - dLocalSuccessDelta;
      } else {
	// This is the node we already passed. This is S0, which has
	// 100% success rate per definition.
	presResult.dSuccessRate = 1.0f;
      }
      
      return presResult;
    }
    
    bool PLUGIN_CLASS::loadDecisionTree(std::string strPath) {
      bool bResult = false;
      
      // ...
      
      return bResult;
    }
    
    Property* PLUGIN_CLASS::evaluateDecisionTree(Property* prTree, Property* prFeatures) {
      Property* prResult = NULL;
      
      // ...
      
      return prResult;
    }
  }
  
  extern "C" plugins::PLUGIN_CLASS* createInstance() {
    return new plugins::PLUGIN_CLASS();
  }
  
  extern "C" void destroyInstance(plugins::PLUGIN_CLASS* icDestroy) {
    delete icDestroy;
  }
}
