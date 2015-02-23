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


namespace semrec {
  namespace plugins {
    PLUGIN_CLASS::PLUGIN_CLASS() {
      m_jsnModel = NULL;
      m_dtDecisionTree = NULL;
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

      if(m_dtDecisionTree) {
	delete m_dtDecisionTree;
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
      this->setOffersService("invert-decision-tree", true);
      
      // Prepare the JSON prediction model parsers
      m_jsnModel = new JSON();
      m_dtDecisionTree = new DecisionTree();
      
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
	  Designator* cdRequest = seEvent.cdDesignator;
	  
	  seResponse.bPreserve = true;
	  Designator* cdResponse = new Designator();
	  cdResponse->setType(Designator::DesignatorType::ACTION);
	  
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
	  Designator* cdRequest = seEvent.cdDesignator;
	  
	  if(cdRequest) {
	    if(cdRequest->stringValue("type") == "task-tree") {
	      std::string strPath = cdRequest->stringValue("file");
	      
	      if(strPath != "") {
		this->info("Load task tree model: '" + strPath + "'");
		
		if(this->load(strPath)) {
		  this->info("Finished loading task tree model.");
		  m_bModelLoaded = true;
		} else {
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
	  
	  ServiceEvent seResponse = eventInResponseTo(seEvent);
	  this->deployServiceEvent(seResponse);
	} else if(seEvent.strServiceName == "invert-decision-tree") {
	  if(seEvent.cdDesignator) {
	    std::string strTargetResult = seEvent.cdDesignator->stringValue("target-result");
	    
	    if(strTargetResult != "") {
	      KeyValuePair* ckvpFeatures = seEvent.cdDesignator->childForKey("features");
	      
	      Property* prTargetResult = new Property();
	      prTargetResult->set(strTargetResult);
	      
	      std::vector<KeyValuePair*> vecSolutions = m_dtDecisionTree->invert(prTargetResult, ckvpFeatures);
	      delete prTargetResult;
	      
	      ServiceEvent seResponse = eventInResponseTo(seEvent);
	      seResponse.bPreserve = true;
	      seResponse.cdDesignator = new Designator();
	      seResponse.cdDesignator->setType(Designator::DesignatorType::ACTION);
	      
	      KeyValuePair* ckvpSolutions = seResponse.cdDesignator->addChild("solutions");
	      
	      for(int nI = 0; nI < vecSolutions.size(); nI++) {
		KeyValuePair* ckvpSolution = vecSolutions.at(nI);
		
		ckvpSolution->setKey("solution_" + this->str(nI));
		ckvpSolutions->addChild(ckvpSolution);
	      }
	      
	      this->deployServiceEvent(seResponse);
	    } else {
	      this->warn("Cannot invert decision tree model without target result.");
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
	  
	  this->info("Prediction context is now node '" + ndNode->title() + "'.");//, of class '" + m_expOwl->owlClassForNode(ndNode, true) + "'.");
	  m_ndActive = ndNode;
	} else {
	  this->info("Prediction context is now top-level (so not predicting).");
	  m_ndActive = NULL;
	}
      }
    }
    
    bool PLUGIN_CLASS::serviceCallbackLoad(designator_integration_msgs::DesignatorCommunication::Request &req, designator_integration_msgs::DesignatorCommunication::Response &res) {
      bool bSuccess = false;
      
      Designator* desigRequest = new Designator(req.request.designator);
      Designator* desigResponse = new Designator();
      desigResponse->setType(Designator::DesignatorType::ACTION);
      
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
    
    bool PLUGIN_CLASS::load(std::string strFile) {
      bool bReturn = false;
      
      std::ifstream ifFile(strFile.c_str());
      if(ifFile.good()) {
	std::string strJSON((std::istreambuf_iterator<char>(ifFile)), (std::istreambuf_iterator<char>()));
	
	m_jsnModel->parse(strJSON);
	
	if(m_jsnModel->rootProperty()) {
	  this->info("Successfully loaded and parsed model '" + strFile + "'.");
	  bReturn = true;
	  m_bInsidePredictionModel = true;
	  
	  this->info("Okay, descending to 'Toplevel'.");
	  this->descend("Toplevel");
	  
	  bReturn = true;
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
      return this->descend(ndDescend->title());
      //return this->descend(m_expOwl->owlClassForNode(ndDescend, true));
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
      
      std::string strClass = ndAscend->title();//m_expOwl->owlClassForNode(ndAscend, true);
      
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
    
    PLUGIN_CLASS::PredictionResult PLUGIN_CLASS::evaluatePredictionRequest(Property* prActive, KeyValuePair* ckvpFeatures, KeyValuePair* ckvpRequested) {
      PredictionResult presResult;
      
      // Here, the actual prediction takes place.
      std::map<std::string, double> mapEffectiveFailureRates;
      
      // Automatic tree walking
      std::list< std::pair<Property*, int> > lstLinearTree = this->linearizeTree(prActive, m_lstPredictionStack.size());
      std::list< std::pair<Property*, int> > lstRunTree;
      double dLeftOverSuccess = 1.0f;
      
      for(std::pair<Property*, int> prCurrent : lstLinearTree) {
	lstRunTree.push_back(prCurrent);
	
	presResult = this->probability(lstRunTree, ckvpFeatures, ckvpRequested);
	
	for(std::pair<std::string, double> prFailure : presResult.mapFailureProbabilities) {
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
    
    bool PLUGIN_CLASS::predict(Designator* cdRequest, Designator* cdResponse) {
      bool bResult = false;
      
      m_mtxStackProtect.lock();
      
      if(m_bInsidePredictionModel) {
	// Make sure there is a model present before diving into the
	// tree.
	if(m_jsnModel->rootProperty()) {
	  // Make sure we actually are in a valid prediction state.
	  if(m_lstPredictionStack.size() > 0) {
	    PredictionTrack ptCurrent = m_lstPredictionStack.back();
	    this->info("Predicting for class: '" + ptCurrent.strClass + "' at level " + this->str((int)m_lstPredictionStack.size()));
	    
	    KeyValuePair* ckvpActiveFeatures = cdRequest->childForKey("active-features");
	    KeyValuePair* ckvpRequestedFeatures = cdRequest->childForKey("requested-features");
	    
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
	    KeyValuePair* ckvpFailureProbabilities = cdResponse->addChild("failures");
	    for(std::pair<std::string, double> prFailure : presResult.mapFailureProbabilities) {
	      ckvpFailureProbabilities->setValue(prFailure.first, prFailure.second);
	    }
	    
	    // Requested feature values
	    KeyValuePair* ckvpRequestedFeatureValues = cdResponse->addChild("requested");
	    for(std::pair<std::string, Property*> prReq : presResult.mapRequestedFeatureValues) {
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
    
    std::list< std::pair<Property*, int> > PLUGIN_CLASS::linearizeTree(Property* prTop, int nLevel) {
      std::list< std::pair<Property*, int> > lstProps;
      lstProps.push_back(std::make_pair(prTop, nLevel));
      
      Property* prSubs = prTop->namedSubProperty("subTypes");
      
      if(prSubs) {
	for(Property* prSub : prSubs->subProperties()) {
	  std::list< std::pair<Property*, int> > lstSubProps = this->linearizeTree(prSub, nLevel + 1);
	  
	  for(std::pair<Property*, int> prSubSub : lstSubProps) {
	    lstProps.push_back(std::make_pair(prSubSub.first, prSubSub.second));
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
    
    std::map<std::string, double> PLUGIN_CLASS::relativeFailuresForNode(Property* prNode, int nLevel, KeyValuePair* ckvpFeatures) {
      std::map<std::string, double> mapRelFail;
      
      if(prNode) {
	ckvpFeatures->setValue("Level", nLevel);
	ckvpFeatures->setValue("Task", prNode->key());
	
	// Find TAGNAME if this is a tag
	std::string strTagName = "";
	
	KeyValuePair* ckvpDesignators = m_ndActive->metaInformation()->childForKey("designators");
	if(ckvpDesignators) {
	  for(KeyValuePair* ckvpDesignator : ckvpDesignators->children()) {
	    KeyValuePair* ckvpDescription = ckvpDesignator->childForKey("description");
	    
	    if(ckvpDescription) {
	      strTagName = ckvpDescription->stringValue("tagname");
	      
	      if(strTagName != "") {
		break;
	      }
	    }
	  }
	}
	
	if(strTagName != "") {
	  ckvpFeatures->setValue("TAGNAME", strTagName);
	}
	
	Property* prResult = this->evaluateDecisionTree(ckvpFeatures);
	
	if(prResult) {
	  // TODO(winkler): The confidence in the calculated failure
	  // is not always 1.0f (although mostly it is very close to
	  // it). This information is coming from Weka as well and
	  // should be contained in the decision tree model.
	  mapRelFail[prResult->getString()] = 1.0f;
	}
      }
      
      return mapRelFail;
    }
    
    PLUGIN_CLASS::PredictionResult PLUGIN_CLASS::probability(std::list< std::pair<Property*, int> > lstSequence, KeyValuePair* ckvpFeatures, KeyValuePair* ckvpRequested) {
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
	  std::list< std::pair<Property*, int> > lstAllButLast = lstSequence;
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
	for(std::pair<std::string, double> prFailure : mapSubFailures) {
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
      return m_dtDecisionTree->load(strPath);
    }
    
    Property* PLUGIN_CLASS::evaluateDecisionTree(KeyValuePair* ckvpFeatures) {
      return m_dtDecisionTree->evaluate(ckvpFeatures);
    }
  }
  
  extern "C" plugins::PLUGIN_CLASS* createInstance() {
    return new plugins::PLUGIN_CLASS();
  }
  
  extern "C" void destroyInstance(plugins::PLUGIN_CLASS* icDestroy) {
    delete icDestroy;
  }
}
