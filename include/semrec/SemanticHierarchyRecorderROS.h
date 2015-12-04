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


#ifndef __SEMANTIC_HIERARCHY_RECORDER_ROS_H__
#define __SEMANTIC_HIERARCHY_RECORDER_ROS_H__


// System
#include <mutex>

// ROS
#include <ros/package.h>
#include <rospack/rospack.h>

// Private
#include <SemanticHierarchyRecorder.h>


namespace semrec {
  class SemanticHierarchyRecorderROS : public SemanticHierarchyRecorder {
  private:
    std::mutex m_mtxRospackLock;
    rospack::Rospack m_rstRospack;
    
  public:
    SemanticHierarchyRecorderROS(int argc, char** argv);
    ~SemanticHierarchyRecorderROS();
    
    virtual std::list<std::string> findTokenReplacements(std::string strToken);
    
    /*! \brief Returns the current ROS workspace directory
      
      Searches the environmental variables ROS_WORKSPACE,
      CMAKE_PREFIX_PATH, and ROS_PACKAGE_PATH (in that order) for the
      current ROS workspace. The semantics are as follows:

       - If ROS_WORKSPACE is not empty, use it's value.
         Else:
       - If CMAKE_PREFIX_PATH is not empty, use it's first token
         separated by ':', without it's possibly trailing '/devel'
         postfix.
	 Else:
       - If ROS_PACKAGE_PATH is not empty, use it's first token
         separated by ':', without it's possibly trailing '/src'
         postfix.
      
      If all of this fails, return an empty string. During
      initialization of the belief state system, this value will be
      checked. If it is empty at that time, a warning will be
      printed. The configuration file allows for manual override of
      this in case the workspace directory cannot be detected
      properly. */
    virtual std::list<std::string> workspaceDirectories();
    
    std::string getROSPackagePath(std::string strPackageName, bool bQuiet = true);
    std::string rospackCommand(std::string strCmd, bool bQuiet = true);
    void crawlROS(bool bForce = false);
    
    virtual std::vector<std::string> getAdditionalSearchPaths();
  };
}


#endif /* __SEMANTIC_HIERARCHY_RECORDER_ROS_H__ */
