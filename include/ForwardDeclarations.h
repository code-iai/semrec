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


#ifndef __FORWARD_DECLARATIONS_H__
#define __FORWARD_DECLARATIONS_H__


// System
#include <ftw.h>

// Private
#include <Types.h>
#include <mutex>


namespace semrec {
  // File system specific functions
  void deleteDirectory(std::string strPath, bool bEvenIfNonEmpty = true);
  
  // Context specific functions
  int createContextID();
  bool contextIDTaken(int nID);
  void freeContextID(int nID);
  
  // Result container specific functions
  Result defaultResult();
  
  // Event container specific functions
  Event defaultEvent(std::string strEventName = "");
  Event eventInResponseTo(Event evRequest, std::string strEventName = "");
  
  // Service event container specific functions
  ServiceEvent defaultServiceEvent(std::string strServiceName = "");
  ServiceEvent eventInResponseTo(ServiceEvent seRequest, std::string strServiceName = "");
  
  // Plugin specific functions
  int createPluginID();
  bool pluginIDTaken(int nID);
  void freePluginID(int nID);
  
  // Output specific functions
  std::string colorSpecifierForID(int nID, bool bBold = false);
  std::string normalColorSpecifier();
  
  // Global config settings specific functions
  ConfigSettings configSettings();
  void setConfigSettings(ConfigSettings cfgsetSettings);
  
  // Per-Plugin configuration space accessor function
  Designator* getPluginConfig(std::string strPluginName);
  
  void queueMessage(StatusMessage msgQueue);
  StatusMessage queueMessage(std::string strColorCode, bool bBold, std::string strPrefix, std::string strMessage);
  std::list<StatusMessage> queuedMessages();
  
  int nextSequenceNumber();
  void resetSequenceNumbers();
  
  void revokeGlobalToken(std::string strToken);
  bool waitForGlobalToken(std::string strToken, float fTimeout = 2.0);
  void issueGlobalToken(std::string strToken);
  bool wasGlobalTokenIssued(std::string strToken);
}


#endif /* __FORWARD_DECLARATIONS_H__ */
