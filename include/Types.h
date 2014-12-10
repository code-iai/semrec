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
#include <designators/Designator.h>
#include <vector>
#include <map>

// Private
#include <Node.h>


using namespace designator_integration;


namespace semrec {
  class PluginInstance;
  
  /*! \brief Enumeration of possible ServiceEvent types */
  typedef enum {
    /*! \brief This is a service request */
    SI_REQUEST,
    /*! \brief This is a service response */
    SI_RESPONSE
  } ServiceIdentifier;
  
  /*! \brief Enumeration of possible ServiceEvent modificator flags */
  typedef enum {
    /*! \brief Only return the first response for this service request */
    SM_FIRST_RESULT,
    /*! \brief Return all responses for this service request */
    SM_AGGREGATE_RESULTS,
    /*! \brief Don't collect results for this service request */
    SM_IGNORE_RESULTS
  } ServiceModifier;
  
  /*! \brief Enumeration of fixed result types
   
   The fixed result types are mostly used internally, for plugin
   loading. */
  typedef enum {
    /*! \brief No specific fixed result identifier
     
     This is the default value for most cases. */
    RI_NONE,
    /*! \brief Loading a plugin failed */
    RI_PLUGIN_LOADING_FAILED,
    /*! \brief A configuration file was not found */
    RI_CONFIG_FILE_NOT_FOUND,
    /*! \brief A file to load was not found */
    RI_FILE_NOT_FOUND,
    /*! \brief Dependencies for loading a plugin were not met */
    RI_PLUGIN_DEPENDENCY_NOT_MET,
    /*! \brief A plugin is not loading because it is a development plugin
     
     Plugins can be specifically declared as development plugins. When
     the central system is configured to not load development plugins,
     these are not loaded, and don't signal a failure. Is the system
     is configured to load development plugins, this plugin setting is
     ignored. A message will be displayed in both cases, informing the
     user about the development plugin and how it is handled. */
    RI_PLUGIN_DEVELOPMENT_NOT_LOADING
  } ResultIdentifier;
  
  /*! \brief Structure describing a status message to be printed to the screen */
  typedef struct {
    /*! \brief The color code (two digit integer) to use for printing this message */
    std::string strColorCode;
    /*! \brief Signals whether this message is printed in bold letters */
    bool bBold;
    /*! \brief The [prefix] to print for this message
     
     This has purely esthetical purposes, but helps to distinguish
     output on the command line. */
    std::string strPrefix;
    /*! \brief The message to print */
    std::string strMessage;
  } StatusMessage;
  
  /*! \brief Central Event structure, allowing information flow between components */
  typedef struct {
    /*! \brief The event's identifier
      
      Every event needs to have an identifier, allowing components
      such as the main system and receiving plugins to distinguish on
      how to handle the event. Plugins can also subscribe to specific
      types of events, only receiving the types they actually
      subscribed to. */
    std::string strEventName;
    int nContextID;
    /*! \brief An optional designator to accompany the event
      
      Designators can hold arbitrary information for an event and helps
      receiving components to process them. */
    Designator* cdDesignator;
    std::string strSupplementary;
    /*! \brief Optional extra annotation for this event */
    std::string strAnnotation;
    /*! \brief Internal ID of origin plugin for reference */
    int nOriginID;
    int nOpenRequestID;
    bool bRequest;
    bool bPreempt;
    StatusMessage msgStatusMessage;
    /*! \brief Node references accompanying this event */
    std::list<Node*> lstNodes;
    std::list<Node*> lstRootNodes;
    std::list< std::pair<std::string, std::string> > lstDesignatorIDs;
    std::list< std::pair<std::string, std::string> > lstEquations;
    std::list< std::pair<std::string, std::string> > lstEquationTimes;
    int nSequenceNumber;
  } Event;
  
  /*! \brief Central ServiceEvent structure, allowing asynchronous services between components */
  typedef struct {
    ServiceIdentifier siServiceIdentifier;
    ServiceModifier smResultModifier;
    std::string strServiceName;
    int nServiceEventID;
    bool bPreserve;
    int nRequesterID;
    Designator* cdDesignator;
    std::list<Event> lstResultEvents;
    int nSequenceNumber;
  } ServiceEvent;
  
  /*! \brief Central Result container for requests of all types
   
    This structure handles primarily success and failure flags inside
    of the system, also allowing additional information. */
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
  
  /*! \brief Configuration settings container
    
    Used to store configurations for the main system, and for individual plugins. */
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
