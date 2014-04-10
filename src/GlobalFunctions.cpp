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


#include <ForwardDeclarations.h>


namespace beliefstate {
  static list<int> g_lstContextIDs;
  static list<int> g_lstPluginIDs;
  static ConfigSettings g_cfgsetSettings;
  static mutex g_mtxGlobalSettings;
  static map<string, CDesignator*> g_mapPluginSettings;
  static mutex g_mtxStatusMessages;
  static list<StatusMessage> g_lstStatusMessages;
  
  
  int rmfile(const char *path, const struct stat *sb, int flag, struct FTW *ftwbuf) {
    return ::remove(path);
  }
  
  void deleteDirectory(string strPath, bool bEvenIfNonEmpty) {
    if(bEvenIfNonEmpty) {
      ::nftw(strPath.c_str(), rmfile, 64, FTW_DEPTH | FTW_PHYS);
    } else {
      ::rmdir(strPath.c_str());
    }
  }
  
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
    evDefault.nOpenRequestID = -1;
    evDefault.bRequest = true;
    evDefault.bPreempt = true;
    
    return evDefault;
  }
  
  Event eventInResponseTo(Event evRequest, string strEventName) {
    if(strEventName == "") {
      strEventName = evRequest.strEventName;
    }
    
    Event evDefault = defaultEvent(strEventName);
    evDefault.nOpenRequestID = evRequest.nOpenRequestID;
    evDefault.bRequest = false;
    
    return evDefault;
  }
  
  string colorSpecifierForID(int nID, bool bBold) {
    ConfigSettings cfgSet = configSettings();
    int nLength = cfgSet.vecPluginOutputColors.size();
    int nUseIndex = (nID + 3) % nLength;
    
    return cfgSet.vecPluginOutputColors[nUseIndex];
  }
  
  string normalColorSpecifier() {
    return "\033[0m";
  }
  
  ConfigSettings configSettings() {
    g_mtxGlobalSettings.lock();
    ConfigSettings cfgsetReturn = g_cfgsetSettings;
    g_mtxGlobalSettings.unlock();
    
    return cfgsetReturn;
  }
  
  void setConfigSettings(ConfigSettings cfgsetSettings) {
    g_mtxGlobalSettings.lock();
    g_cfgsetSettings = cfgsetSettings;
    g_mtxGlobalSettings.unlock();
  }
  
  CDesignator* getPluginConfig(string strPluginName) {
    CDesignator* cdReturn = NULL;
    map<string, CDesignator*>::iterator itPlugin = g_mapPluginSettings.find(strPluginName);
    
    if(itPlugin != g_mapPluginSettings.end()) {
      cdReturn = (*itPlugin).second;
    } else {
      cdReturn = new CDesignator();
      g_mapPluginSettings[strPluginName] = cdReturn;
    }
    
    return cdReturn;
  }
  
  void queueMessage(StatusMessage msgQueue) {
    g_mtxStatusMessages.lock();
    g_lstStatusMessages.push_back(msgQueue);
    g_mtxStatusMessages.unlock();
  }
  
  StatusMessage queueMessage(string strColorCode, bool bBold, string strPrefix, string strMessage) {
    StatusMessage msgQueue;
    msgQueue.strColorCode = strColorCode;
    msgQueue.bBold = bBold;
    msgQueue.strPrefix = strPrefix;
    msgQueue.strMessage = strMessage;
    
    queueMessage(msgQueue);
    
    return msgQueue;
  }
  
  list<StatusMessage> queuedMessages() {
    g_mtxStatusMessages.lock();
    list<StatusMessage> lstMessages = g_lstStatusMessages;
    g_lstStatusMessages.clear();
    g_mtxStatusMessages.unlock();
    
    return lstMessages;
  }
}
