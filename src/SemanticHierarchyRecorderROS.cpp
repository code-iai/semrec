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


#include <SemanticHierarchyRecorderROS.h>


namespace semrec {
  SemanticHierarchyRecorderROS::SemanticHierarchyRecorderROS(int argc, char** argv) : SemanticHierarchyRecorder(argc, argv) {
    this->crawlROS();
    
    m_lstConfigFileLocations.push_back(ros::package::getPath("semrec"));
  }
  
  SemanticHierarchyRecorderROS::~SemanticHierarchyRecorderROS() {
  }
  
  std::list<std::string> SemanticHierarchyRecorderROS::findTokenReplacements(std::string strToken) {
    std::list<std::string> lstReplacements = this->SemanticHierarchyRecorder::findTokenReplacements(strToken);
    
    if(strToken.size() > 8) {
      if(strToken.substr(0, 8) == "PACKAGE ") {
	std::string strPackage = strToken.substr(8);
	std::string strPackageResolved = this->getROSPackagePath(strPackage);
	
	if(strPackageResolved != "") {
	  lstReplacements.push_back(strPackageResolved);
	}
      } else if(strToken == "WORKSPACE") {
	for(std::string strWorkspaceDirectory : this->workspaceDirectories()) {
	  lstReplacements.push_back(strWorkspaceDirectory);
	}
      }
    }
    
    return lstReplacements;
  }
  
  std::string SemanticHierarchyRecorderROS::getROSPackagePath(std::string strPackageName, bool bQuiet) {
    std::string strPath = "";
    m_rstRospack.setQuiet(bQuiet);
    m_rstRospack.find(strPackageName, strPath);
    
    // Scrape any newlines out of it
    for(size_t szNewline = strPath.find('\n'); szNewline != std::string::npos; szNewline = strPath.find('\n')) {
      strPath.erase(szNewline, 1);
    }
    
    return strPath;
  }
  
  void SemanticHierarchyRecorderROS::crawlROS(bool bForce) {
    std::vector<std::string> vecSearchPaths;
    m_rstRospack.getSearchPathFromEnv(vecSearchPaths);
    
    m_rstRospack.crawl(vecSearchPaths, bForce);
  }
  
  std::list<std::string> SemanticHierarchyRecorderROS::workspaceDirectories() {
    std::list<std::string> lstWorkspaceDirectories = this->SemanticHierarchyRecorder::workspaceDirectories();
    
    const char* cROSWorkspace = getenv("ROS_WORKSPACE");
    const char* cCMAKEPrefixPath = getenv("CMAKE_PREFIX_PATH");
    const char* cROSPackagePath = getenv("ROS_PACKAGE_PATH");
    
    if(cROSWorkspace) {
      lstWorkspaceDirectories.push_back(std::string(cROSWorkspace));
    }
    
    std::vector<std::string> vecSearchPaths;
    m_rstRospack.getSearchPathFromEnv(vecSearchPaths);
    
    for(std::string strPath : vecSearchPaths) {
      bool bWasAdded = false;
      
      if(strPath.length() >= 6) {
	if(strPath.substr(strPath.length() - 6) == "/devel") {
	  lstWorkspaceDirectories.push_back(strPath.substr(0, strPath.length() - 6));
	  bWasAdded = true;
	}
      }
      
      if(strPath.length() >= 4) {
	if(strPath.substr(strPath.length() - 4) == "/src") {
	  lstWorkspaceDirectories.push_back(strPath.substr(0, strPath.length() - 4));
	  bWasAdded = true;
	}
      }
      
      if(!bWasAdded) {
	lstWorkspaceDirectories.push_back(strPath);
      }
    }
    
    return lstWorkspaceDirectories;
  }
  
  std::vector<std::string> SemanticHierarchyRecorderROS::getAdditionalSearchPaths() {
    std::vector<std::string> vecPaths = this->SemanticHierarchyRecorder::getAdditionalSearchPaths();
    
    ros::package::getPlugins("semrec", "plugin_path", vecPaths);
    
    return vecPaths;
  }
}
