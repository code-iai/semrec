#ifndef __FORWARD_DECLARATIONS_H__
#define __FORWARD_DECLARATIONS_H__


// Private
#include <Types.h>


namespace beliefstate {
  // Context specific functions
  int createContextID();
  bool contextIDTaken(int nID);
  void freeContextID(int nID);
  
  // Result container specific functions
  Result defaultResult();
  
  // Event container specific functions
  Event defaultEvent();
  
  // Service event container specific functions
  ServiceEvent defaultServiceEvent(string strServiceName = "");
  
  // Plugin specific functions
  int createPluginID();
  bool pluginIDTaken(int nID);
  void freePluginID(int nID);
  
  // Output specific functions
  string colorSpecifierForID(int nID, bool bBold);
  string normalColorSpecifier();
}


#endif /* __FORWARD_DECLARATIONS_H__ */
