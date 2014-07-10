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
      } else if(evEvent.strEventName == "symbolic-node-active") {
	if(evEvent.lstNodes.size() > 0) {
	  Node* ndNode = evEvent.lstNodes.front();
	  
	  this->info("Prediction context is now node '" + ndNode->title() + "'.");
	} else {
	  this->info("Prediction context is now top-level (so not predicting).");
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
	  
	  this->info("Optimizing: Mapping node failures");
	  this->mapNodeFailures();
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
    
    void PLUGIN_CLASS::mapNodeFailures() {
      Property* prToplevel = NULL;
      list<Property*> lstLinearModel;
      
      m_mapNodeFailures.clear();
      
      if(m_jsnModel->rootProperty()->namedSubProperty("tree")->subProperties().size() > 0) {
	prToplevel = m_jsnModel->rootProperty()->namedSubProperty("tree")->subProperties().front();
      }
      
      if(prToplevel) {
	lstLinearModel = this->linearizeTree(prToplevel);
      }
      
      int nNodes = 0;
      int nFailures = 0;
      
      for(Property* prNode : lstLinearModel) {
	nNodes++;
	list<string> lstNames;
	
	for(Property* prName : prNode->namedSubProperty("names")->subProperties()) {
	  lstNames.push_back(prName->getString());
	}
	
	list<Property*> lstFailures;
	for(Property* prFailureSet : m_jsnModel->rootProperty()->namedSubProperty("failures")->subProperties()) {
	  for(Property* prFailure : prFailureSet->namedSubProperty("failures")->subProperties()) {
	    if(find(lstNames.begin(), lstNames.end(), prFailure->namedSubProperty("emitter")->getString()) != lstNames.end()) {
	      // The emitter name is on our node's 'names' list
	      nFailures++;
	      lstFailures.push_back(prFailure);
	    }
	  }
	}
	
	m_mapNodeFailures[prNode] = lstFailures;
      }
      
      stringstream sts;
      sts << "Mapped " << nFailures << " failures on " << nNodes << " nodes.";
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
	    
	    for(string strKey : desigRequest->keys()) {
	      Property* prParameter = new Property();
	      
	      prParameter->set(Property::Double);
	      prParameter->setKey(strKey);
	      prParameter->set((double)desigRequest->floatValue(strKey));
	      
	      lstParameters.push_back(prParameter);
	    }
	    
	    list<Property*> lstLinearTree = this->linearizeTree(ptCurrent.prLevel);
	    lstLinearTree.reverse();
	    PredictionResult presResult;
	    int nTracks = m_jsnModel->rootProperty()->namedSubProperty("tracks")->subProperties().size();
	    
	    map<string, double> mapEffectiveFailureRates;
	    
	    while(lstLinearTree.size() > 0) {
	      presResult = this->probability(lstLinearTree, nTracks, lstParameters);
	      
	      for(Failure flFailure : presResult.lstFailureProbabilities) {
		if(mapEffectiveFailureRates[flFailure.strClass] == 0) {
		  mapEffectiveFailureRates[flFailure.strClass] = flFailure.dProbability;
		} else {
		  mapEffectiveFailureRates[flFailure.strClass] += flFailure.dProbability;
		}
	      }
	      
	      lstLinearTree.pop_front();
	    }
	    
	    CKeyValuePair* ckvpFailures = desigResponse->addChild("failures");
	    presResult.dSuccessRate = 1.0f;
	    for(auto itMap = mapEffectiveFailureRates.begin(); itMap != mapEffectiveFailureRates.end(); itMap++) {
	      pair<string, double> prFailure = *itMap;
	      ckvpFailures->setValue(prFailure.first, prFailure.second);
	      
	      presResult.dSuccessRate -= prFailure.second;
	    }

	    desigResponse->setValue("success", presResult.dSuccessRate);
	    
	    for(Property* prDelete : lstParameters) {
	      delete prDelete;
	    }
	    
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
      // list<string> lstNames;
      
      // // Decision tree
      // //InitialiseTreeData();
      
      // for(Property* prName : prNode->namedSubProperty("names")->subProperties()) {
      // 	lstNames.push_back(prName->getString());
      // }
      
      // list<Property*> lstFailures;
      // for(Property* prFailureSet : m_jsnModel->rootProperty()->namedSubProperty("failures")->subProperties()) {
      // 	for(Property* prFailure : prFailureSet->namedSubProperty("failures")->subProperties()) {
      // 	  if(find(lstNames.begin(), lstNames.end(), prFailure->namedSubProperty("emitter")->getString()) != lstNames.end()) {
      // 	    // The emitter name is on our node's 'names' list
      // 	    lstFailures.push_back(prFailure);
      // 	  }
      // 	}
      // }
      
      // // Clean decision tree
      // //Cleanup();
      
      // return lstFailures;
      
      if(m_mapNodeFailures.find(prNode) != m_mapNodeFailures.end()) {
	return m_mapNodeFailures[prNode];
      }
      
      list<Property*> lstEmpty;
      return lstEmpty;
    }
    
    PLUGIN_CLASS::PredictionResult PLUGIN_CLASS::probability(list<Property*> lstSequence, int nTracks, list<Property*> lstParameters) {
      PredictionResult presResult;
      
      for(Property* propParam : lstParameters) {
	propParam->print();
      }
      
      if(lstSequence.size() > 0) {
	// Relative occurrences
	int nBranches = lstSequence.back()->namedSubProperty("names")->subProperties().size();
	
	list<Property*> lstFailures = this->failuresForTreeNode(lstSequence.back());
	map<string, double> mapFailureRelOcc = this->relativeFailureOccurrences(lstFailures, nBranches);
	
	double dLocalSuccess = 1.0f;
	
	for(auto itMap = mapFailureRelOcc.begin(); itMap != mapFailureRelOcc.end(); itMap++) {
	  pair<string, double> prPair = *itMap;
	  dLocalSuccess -= prPair.second;
	}
	
	// Prepare list for recursion
	list<Property*> lstAllButLast = lstSequence;
	lstAllButLast.pop_back();
	
	// Recurse
	PredictionResult presSub = this->probability(lstAllButLast, nBranches, lstParameters);
	list<Failure> lstSubFailures = presSub.lstFailureProbabilities;
	
	if(lstSequence.size() > 1) {
	  presResult.dSuccessRate = dLocalSuccess * presSub.dSuccessRate;
	  
	  for(Failure flFailure : presSub.lstFailureProbabilities) {
	    flFailure.dProbability *= dLocalSuccess;
	    presResult.lstFailureProbabilities.push_back(flFailure);
	  }
	} else {
	  presResult.dSuccessRate = dLocalSuccess;
	  
	  for(auto itMap = mapFailureRelOcc.begin(); itMap != mapFailureRelOcc.end(); itMap++) {
	    pair<string, double> prMap = *itMap;
	    
	    Failure flFailure;
	    flFailure.strClass = prMap.first;
	    flFailure.dProbability = prMap.second;
	    
	    presResult.lstFailureProbabilities.push_back(flFailure);
	  }
	}
      } else {
	// This is the node we already passed. This is S0.
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
