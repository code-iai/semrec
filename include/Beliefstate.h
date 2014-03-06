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


#ifndef __BELIEFSTATE_H__
#define __BELIEFSTATE_H__


// System
#include <iostream>
#include <cstdlib>
#include <list>
#include <string>
#include <libconfig.h++>

// ROS
#include <ros/package.h>

// Private
#include <ForwardDeclarations.h>
#include <Types.h>
#include <PluginSystem.h>
#include <UtilityBase.h>

using namespace std;
using namespace libconfig;


namespace beliefstate {
  class Beliefstate : public UtilityBase {
  private:
    PluginSystem* m_psPlugins;
    bool m_bRun;
    int m_argc;
    char** m_argv;
    list<string> m_lstPluginsToLoad;
    list<Event> m_lstGlobalEvents;
    string m_strWorkspaceDirectory;
    
  public:
    Beliefstate(int argc, char** argv);
    ~Beliefstate();
    
    Result init(string strConfigFile = "");
    Result deinit();
    
    bool loadConfigFile(string strConfigFile);
    bool loadIndividualPluginConfigurationBranch(Setting &sBranch, CKeyValuePair* ckvpInto, string strConfigPath = "", bool bIgnorePluginField = false);
    void replaceStringInPlace(string& subject, const string& search, const string& replace);
    
    void spreadEvent(Event evEvent);
    void spreadServiceEvent(ServiceEvent seServiceEvent);
    bool cycle();
    
    void triggerShutdown();
    
    void setBaseDataDirectory(string strBaseDataDirectory);
    string baseDataDirectory();
    string resolveDirectoryTokens(string strPath);
    
    string workspaceDirectory();
    string homeDirectory();
  };
}


#endif /* __BELIEFSTATE_H__ */
