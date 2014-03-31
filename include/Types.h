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


#ifndef __TYPES_H__
#define __TYPES_H__


// System
#include <string>
#include <list>
#include <designators/CDesignator.h>
#include <vector>
#include <map>

// Private
#include <Node.h>

using namespace std;


namespace beliefstate {
  typedef enum {
    SI_REQUEST,
    SI_RESPONSE
  } ServiceIdentifier;
  
  typedef enum {
    SM_FIRST_RESULT,
    SM_AGGREGATE_RESULTS,
    SM_IGNORE_RESULTS
  } ServiceModifier;
  
  typedef enum {
    RI_NONE,
    RI_PLUGIN_LOADING_FAILED,
    RI_CONFIG_FILE_NOT_FOUND,
    RI_FILE_NOT_FOUND,
    RI_PLUGIN_DEPENDENCY_NOT_MET,
    RI_PLUGIN_DEVELOPMENT_NOT_LOADING
  } ResultIdentifier;
  
  typedef struct {
    string strColorCode;
    bool bBold;
    string strPrefix;
    string strMessage;
  } StatusMessage;
  
  typedef struct {
    string strEventName;
    int nContextID;
    CDesignator* cdDesignator;
    string strSupplementary;
    string strAnnotation;
    int nOriginID;
    int nOpenRequestID;
    bool bRequest;
    bool bPreempt;
    StatusMessage msgStatusMessage;
    list<Node*> lstNodes;
    list< pair<string, string> > lstDesignatorIDs;
    list< pair<string, string> > lstEquations;
    list< pair<string, string> > lstEquationTimes;
  } Event;
  
  typedef struct {
    ServiceIdentifier siServiceIdentifier;
    ServiceModifier smResultModifier;
    string strServiceName;
    int nServiceEventID;
    int nRequesterID;
    CDesignator* cdDesignator;
    list<Event> lstResultEvents;
  } ServiceEvent;
  
  typedef struct {
    // Generic fields
    bool bSuccess;
    ResultIdentifier riResultIdentifier;
    list<Event> lstEvents;
    list<ServiceEvent> lstServiceEvents;
    
    // In case of failure
    string strErrorMessage;
    
    // Status message that were collected
    list<StatusMessage> lstStatusMessages;
  } Result;
  
  typedef struct {
    // Basics
    string strBaseDataDirectory;
    string strExperimentNameMask;
    
    // Current Experiment Instance
    string strExperimentDirectory;
    string strSymlinkName;
    
    // MongoDB
    bool bUseMongoDB;
    string strMongoDBHost;
    int nMongoDBPort;
    string strMongoDBDatabase;
    
    // Plugin loading
    bool bLoadDevelopmentPlugins;
    bool bFailedPluginsInvalidateStartup;
    
    // Plugin output
    vector<string> vecPluginOutputColors;
    
    // Miscellaneous
    bool bDisplayUnhandledEvents;
    bool bDisplayUnhandledServiceEvents;
  } ConfigSettings;
}


#endif /* __TYPES_H__ */
