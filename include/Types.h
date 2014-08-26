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


namespace beliefstate {
  class PluginInstance;
  
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
    std::string strColorCode;
    bool bBold;
    std::string strPrefix;
    std::string strMessage;
  } StatusMessage;
  
  typedef struct {
    std::string strEventName;
    int nContextID;
    CDesignator* cdDesignator;
    std::string strSupplementary;
    std::string strAnnotation;
    int nOriginID;
    int nOpenRequestID;
    bool bRequest;
    bool bPreempt;
    StatusMessage msgStatusMessage;
    std::list<Node*> lstNodes;
    std::list<Node*> lstRootNodes;
    std::list< std::pair<std::string, std::string> > lstDesignatorIDs;
    std::list< std::pair<std::string, std::string> > lstEquations;
    std::list< std::pair<std::string, std::string> > lstEquationTimes;
  } Event;
  
  typedef struct {
    ServiceIdentifier siServiceIdentifier;
    ServiceModifier smResultModifier;
    std::string strServiceName;
    int nServiceEventID;
    int nRequesterID;
    CDesignator* cdDesignator;
    std::list<Event> lstResultEvents;
  } ServiceEvent;
  
  typedef struct {
    // Generic fields
    bool bSuccess;
    ResultIdentifier riResultIdentifier;
    std::list<Event> lstEvents;
    std::list<ServiceEvent> lstServiceEvents;
    
    // In case of failure
    std::string strErrorMessage;
    
    // Status message that were collected
    std::list<StatusMessage> lstStatusMessages;
    
    PluginInstance* piPlugin;
  } Result;
  
  typedef struct {
    // Basics
    std::string strBaseDataDirectory;
    std::string strExperimentNameMask;
    
    // Current Experiment Instance
    std::string strExperimentDirectory;
    std::string strSymlinkName;
    
    // MongoDB
    bool bUseMongoDB;
    std::string strMongoDBHost;
    int nMongoDBPort;
    std::string strMongoDBDatabase;
    
    // Plugin loading
    bool bLoadDevelopmentPlugins;
    bool bFailedPluginsInvalidateStartup;
    
    // Plugin output
    std::vector<std::string> vecPluginOutputColors;
    
    // Miscellaneous
    bool bDisplayUnhandledEvents;
    bool bDisplayUnhandledServiceEvents;
    bool bOnlyDisplayImportant;
  } ConfigSettings;
}


#endif /* __TYPES_H__ */
