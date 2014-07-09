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

// Designators
#include <designators/CDesignator.h>
#include <designator_integration_msgs/DesignatorCommunication.h>

// OWL Exporter for ontology
#include <plugins/owlexporter/CExporterOwl.h>

// Private
#include <Types.h>
#include <ForwardDeclarations.h>
#include <Plugin.h>
#include <Property.h>
#include <JSON.h>

using namespace std;


namespace beliefstate {
  namespace plugins {
    class PLUGIN_CLASS : public Plugin {
    private:
      typedef struct {
	Property* prLevel;
	string strClass;
      } PredictionTrack;
      
      typedef struct {
	string strClass;
	double dProbability;
      } Failure;
      
      typedef struct {
	list<Failure> lstFailureProbabilities;
	double dSuccessRate;
      } PredictionResult;
      
      ros::NodeHandle* m_nhHandle;
      ros::ServiceServer m_srvPredict;
      ros::ServiceServer m_srvLoad;
      JSON* m_jsnModel;
      list<PredictionTrack> m_lstPredictionStack;
      CExporterOwl* m_expOwl;
      bool m_bInsidePredictionModel;
      mutex m_mtxStackProtect;

    public:
      PLUGIN_CLASS();
      ~PLUGIN_CLASS();
      
      virtual Result init(int argc, char** argv);
      virtual Result deinit();
      
      virtual Result cycle();
      
      virtual void consumeEvent(Event evEvent);
      
      bool serviceCallbackLoad(designator_integration_msgs::DesignatorCommunication::Request &req, designator_integration_msgs::DesignatorCommunication::Response &res);
      bool serviceCallbackPredict(designator_integration_msgs::DesignatorCommunication::Request &req, designator_integration_msgs::DesignatorCommunication::Response &res);
      
      bool load(string strFile);
      
      bool descend(string strClass);
      bool ascend(string strClass);
      
      bool predict(CDesignator* desigRequest, CDesignator* desigResponse);
      list<Property*> linearizeTree(Property* prTop);
      PredictionResult probability(list<Property*> lstSequence, int nTracks, list<Property*> lstParameters);
      map<string, double> relativeFailureOccurrences(list<Property*> lstFailures, int nTracks);
      list<Property*> failuresForTreeNode(Property* prNode);
    };
  }
  
  extern "C" plugins::PLUGIN_CLASS* createInstance();
  extern "C" void destroyInstance(plugins::PLUGIN_CLASS* icDestroy);
}


#endif /* __PLUGIN_PREDICTION_H__ */