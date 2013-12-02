#include <ForwardDeclarations.h>


namespace beliefstate {
  static list<int> g_lstContextIDs;
  static list<int> g_lstPluginIDs;
  
  
  int createContextID() {
    int nID = 0;
    
    while(contextIDTaken(nID)) {
      nID++;
    }
    
    g_lstContextIDs.push_back(nID);
    
    return nID;
  }
  
  bool contextIDTaken(int nID) {
    for(list<int>::iterator itID = g_lstContextIDs.begin();
	itID != g_lstContextIDs.end();
	itID++) {
      if(*itID == nID) {
	return true;
      }
    }
    
    return false;
  }
  
  void freeContextID(int nID) {
    g_lstContextIDs.remove(nID);
  }
  
  int createPluginID() {
    int nID = 0;
    
    while(pluginIDTaken(nID)) {
      nID++;
    }
    
    g_lstPluginIDs.push_back(nID);
    
    return nID;
  }
  
  bool pluginIDTaken(int nID) {
    for(list<int>::iterator itID = g_lstPluginIDs.begin();
	itID != g_lstPluginIDs.end();
	itID++) {
      if(*itID == nID) {
	return true;
      }
    }
    
    return false;
  }
  
  void freePluginID(int nID) {
    g_lstPluginIDs.remove(nID);
  }
  
  Result defaultResult() {
    Result resDefault;
    resDefault.bSuccess = true;
    resDefault.riResultIdentifier = RI_NONE;
    resDefault.strErrorMessage = "";
    
    return resDefault;
  }
  
  ServiceEvent defaultServiceEvent(string strServiceName) {
    ServiceEvent seDefault;
    seDefault.strServiceName = strServiceName;
    seDefault.siServiceIdentifier = SI_REQUEST;
    seDefault.smResultModifier = SM_AGGREGATE_RESULTS;
    seDefault.cdDesignator = NULL;
    
    return seDefault;
  }
  
  Event defaultEvent(string strEventName) {
    Event evDefault;
    evDefault.strEventName = strEventName;
    evDefault.cdDesignator = NULL;
    evDefault.nOriginID = -1;
    
    return evDefault;
  }
  
  string colorSpecifierForID(int nID, bool bBold) {
    int nLength = 7;
    string strColorCodes[] = {"31", "32", "33", "34", "35", "36", "37"};
    
    int nUseIndex = (nID + 3) % nLength;
    return "\033[" + string(bBold ? "1" : "0") + ";" + strColorCodes[nUseIndex] + "m";
  }
  
  string normalColorSpecifier() {
    return "\033[0m";
  }
}
