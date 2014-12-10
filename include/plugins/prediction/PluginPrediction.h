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


#ifndef __PLUGIN_PREDICTION_H__
#define __PLUGIN_PREDICTION_H__


#define PLUGIN_CLASS PluginPrediction


// System
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <mutex>
#include <map>
#include <sstream>

// Designators
#include <designators/Designator.h>
#include <designator_integration_msgs/DesignatorCommunication.h>

// OWL Exporter for ontology
#include <plugins/owlexporter/CExporterOwl.h>

// Private
#include <Types.h>
#include <ForwardDeclarations.h>
#include <Plugin.h>
#include <Property.h>
#include <JSON.h>
#include <Node.h>
#include <plugins/prediction/DecisionTree.h>


namespace semrec {
  namespace plugins {
    class PLUGIN_CLASS : public Plugin {
    private:
      typedef struct {
	Property* prLevel;
	std::string strClass;
      } PredictionTrack;
      
      typedef struct {
	std::map<std::string, double> mapFailureProbabilities;
	std::map<std::string, Property*> mapRequestedFeatureValues;
	double dSuccessRate;
      } PredictionResult;
      
      JSON* m_jsnModel;
      DecisionTree* m_dtDecisionTree;
      std::list<PredictionTrack> m_lstPredictionStack;
      CExporterOwl* m_expOwl;
      bool m_bInsidePredictionModel;
      std::mutex m_mtxStackProtect;
      Node* m_ndActive;
      std::map< Property*, std::list<Property*> > m_mapNodeFailures;
      std::map< Property*, std::list<Property*> > m_mapNodeParameters;
      std::map< std::string, std::pair<int, int> > m_mapTimeStamps;
      bool m_bModelLoaded;
      int m_nClassFlexibility;
      
    public:
      PLUGIN_CLASS();
      ~PLUGIN_CLASS();
      
      virtual Result init(int argc, char** argv);
      virtual Result deinit();
      
      virtual Result cycle();
      
      virtual Event consumeServiceEvent(ServiceEvent seEvent);
      virtual void consumeEvent(Event evEvent);
      
      bool serviceCallbackLoad(designator_integration_msgs::DesignatorCommunication::Request &req, designator_integration_msgs::DesignatorCommunication::Response &res);
      
      bool load(std::string strFile);
      
      bool descend(std::string strClass, bool bForceClass = false);
      bool descend(Node* ndDescend);
      bool ascend(Node* ndAscend);
      
      bool predict(Designator* desigRequest, Designator* desigResponse);
      PredictionResult evaluatePredictionRequest(Property* prActive, KeyValuePair* ckvpFeatures, KeyValuePair* ckvpRequested);
      
      std::list< std::pair<Property*, int> > linearizeTree(Property* prTop, int nLevel);
      PredictionResult probability(std::list< std::pair<Property*, int> > lstSequence, KeyValuePair* ckvpFeatures, KeyValuePair* ckvpRequested);
      std::map<std::string, double> relativeFailureOccurrences(std::list<Property*> lstFailures, int nTracks);
      std::list<Property*> failuresForTreeNode(Property* prNode);
      std::map<std::string, double> relativeFailuresForNode(Property* prNode, int nLevel, KeyValuePair* ckvpFeatures);
      std::list<Property*> parametersForTreeNode(Property* prNode);
      
      bool loadDecisionTree(std::string strPath);
      Property* evaluateDecisionTree(KeyValuePair* ckvpFeatures);
    };
  }
  
  extern "C" plugins::PLUGIN_CLASS* createInstance();
  extern "C" void destroyInstance(plugins::PLUGIN_CLASS* icDestroy);
}


#endif /* __PLUGIN_PREDICTION_H__ */
