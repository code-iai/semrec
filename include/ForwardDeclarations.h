#ifndef __FORWARD_DECLARATIONS_H__
#define __FORWARD_DECLARATIONS_H__


// Private
#include <Types.h>
#include <mutex>


namespace beliefstate {
  // Context specific functions
  int createContextID();
  bool contextIDTaken(int nID);
  void freeContextID(int nID);
  
  // Result container specific functions
  Result defaultResult();
  
  // Event container specific functions
  Event defaultEvent(string strEventName = "");
  Event eventInResponseTo(Event evRequest, string strEventName = "");
  
  // Service event container specific functions
  ServiceEvent defaultServiceEvent(string strServiceName = "");
  
  // Plugin specific functions
  int createPluginID();
  bool pluginIDTaken(int nID);
  void freePluginID(int nID);
  
  // Output specific functions
  string colorSpecifierForID(int nID, bool bBold);
  string normalColorSpecifier();
  
  // Global config settings specific functions
  ConfigSettings configSettings();
  void setConfigSettings(ConfigSettings cfgsetSettings);
  
  // Per-Plugin configuration space accessor functions
  string getPluginConfigString(string strPluginName, string strKey);
  void setPluginConfigValue(string strPluginName, string strKey, string strValue);
}


#endif /* __FORWARD_DECLARATIONS_H__ */
