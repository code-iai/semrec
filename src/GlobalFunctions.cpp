#include <ForwardDeclarations.h>


namespace beliefstate {
  list<int> g_lstContextIDs;
  
  int createContextID() {
    int nID = 0;
    
    while(contextIDTaken(nID)) {
      nID++;
    }
    
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
  
  Result defaultResult() {
    Result resDefault;
    resDefault.bSuccess = true;
    resDefault.riResultIdentifier = RI_NONE;
    resDefault.strErrorMessage = "";
    
    return resDefault;
  }
}
