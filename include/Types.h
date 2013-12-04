#ifndef __TYPES_H__
#define __TYPES_H__


// System
#include <string>
#include <list>
#include <designators/CDesignator.h>

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
    RI_PLUGIN_DEPENDENCY_NOT_MET
  } ResultIdentifier;
  
  typedef struct {
    string strEventName;
    int nContextID;
    CDesignator* cdDesignator;
    string strSupplementary;
    int nOriginID;
    int nOpenRequestID;
    bool bRequest;
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
  } ConfigSettings;
}


#endif /* __TYPES_H__ */
