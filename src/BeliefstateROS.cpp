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


#include <BeliefstateROS.h>


namespace beliefstate {
  BeliefstateROS::BeliefstateROS(int argc, char** argv) : Beliefstate(argc, argv) {
    m_lstConfigFileLocations.push_back(ros::package::getPath("beliefstate"));
  }
  
  BeliefstateROS::~BeliefstateROS() {
  }
  
  string BeliefstateROS::findTokenReplacement(string strToken) {
    string strReplacement = "";
    
    if(strToken.size() > 8) {
      if(strToken.substr(0, 8) == "PACKAGE ") {
	string strPackage = strToken.substr(8);
	  
	strReplacement = ros::package::getPath(strPackage);
      } else if(strToken == "WORKSPACE") {
	strReplacement = this->workspaceDirectory();
      }
    }
    
    if(strReplacement == "") {
      strReplacement = this->Beliefstate::findTokenReplacement(strToken);
    }
    
    return strReplacement;
  }
  
  string BeliefstateROS::workspaceDirectory() {
    string strWorkspaceReplacement = this->Beliefstate::workspaceDirectory();
    
    if(strWorkspaceReplacement == "") {
      const char* cROSWorkspace = getenv("ROS_WORKSPACE");
      const char* cCMAKEPrefixPath = getenv("CMAKE_PREFIX_PATH");
      const char* cROSPackagePath = getenv("ROS_PACKAGE_PATH");
      
      string strROSWorkspace = "";
      string strCMAKEPrefixPath = "";
      string strROSPackagePath = "";
      
      if(cROSWorkspace) {
	strROSWorkspace = cROSWorkspace;
      }
      
      if(cCMAKEPrefixPath) {
	strCMAKEPrefixPath = cCMAKEPrefixPath;
      }
      
      if(cROSPackagePath) {
	strROSPackagePath = cROSPackagePath;
      }
      
      // ROS_WORKSPACE takes precedence over CMAKE_PREFIX_PATH
      if(strROSWorkspace != "") {
	strWorkspaceReplacement = strROSWorkspace;
      } else {
	// CMAKE_PREFIX_PATH takes precedence over ROS_PACKAGE_PATH
	if(strCMAKEPrefixPath != "") {
	  strWorkspaceReplacement = this->findPrefixPath(strCMAKEPrefixPath, "/devel");
	} else {
	  strWorkspaceReplacement = this->findPrefixPath(strROSPackagePath, "/src");
	}
      }
    }
    
    return strWorkspaceReplacement;
  }
  
  string BeliefstateROS::findPrefixPath(string strPathList, string strMatchingSuffix, string strDelimiter) {
    string strPathReturn = "";
    size_t szLastPos = 0;
    size_t szCurrentPos = 0;
    
    if(strPathList != "") {
      while(szLastPos != string::npos) {
	szCurrentPos = strPathList.find(strDelimiter, szLastPos + strDelimiter.length());
	
	if(szCurrentPos != string::npos) {
	  string strPath = strPathList.substr(szLastPos + (szLastPos != 0 ? strDelimiter.length() : 0), szCurrentPos - (szLastPos + (szLastPos == 0 ? 0 : strDelimiter.length())));
	  
	  if(strPath.length() >= strMatchingSuffix.length()) {
	    if(strPath.substr(strPath.length() - strMatchingSuffix.length()) == strMatchingSuffix) {
	      strPathReturn = this->stripPostfix(strPath, strMatchingSuffix);
	      break;
	    }
	  }
	}
	
	szLastPos = szCurrentPos;
      }
    }
    
    return strPathReturn;
  }
}
