#ifndef __TYPES_H__
#define __TYPES_H__


// System
#include <string>
#include <list>
#include <designators/CDesignator.h>

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
  
  typedef enum {
    EI_UNDEFINED,
    EI_BEGIN_CONTEXT,
    EI_END_CONTEXT,
    EI_ADD_DESIGNATOR,
    EI_ADD_IMAGE_FROM_FILE,
    EI_ADD_IMAGE_FROM_TOPIC,
    EI_ADD_OBJECT,
    EI_ADD_FAILURE,
    EI_EQUATE_DESIGNATORS,
    EI_EXTRACT_PLANLOG
  } EventIdentifier;
  
  typedef struct {
    EventIdentifier eiEventIdentifier;
    int nContextID;
    CDesignator* cdDesignator;
    string strSupplementary;
    int nOriginID;
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
}


#endif /* __TYPES_H__ */
