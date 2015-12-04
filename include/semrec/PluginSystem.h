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


#ifndef __PLUGIN_SYSTEM_H__
#define __PLUGIN_SYSTEM_H__


// System
#include <iostream>
#include <cstdlib>
#include <list>
#include <string>

// Private
#include <ForwardDeclarations.h>
#include <Types.h>
#include <PluginInstance.h>
#include <UtilityBase.h>


namespace semrec {
  class PluginSystem : public UtilityBase {
  private:
    std::list<PluginInstance*> m_lstLoadedPlugins;
    std::list<PluginInstance*> m_lstUnloadPlugins;
    std::list<std::string> m_lstLoadFailedPlugins;
    std::list<std::string> m_lstPluginSearchPaths;
    int m_argc;
    char** m_argv;
    bool m_bLoadDevelopmentPlugins;
    
  public:
    PluginSystem(int argc, char** argv);
    ~PluginSystem();
    
    void setLoadDevelopmentPlugins(bool bLoadDevelopmentPlugins);
    bool loadDevelopmentPlugins();
    
    std::string pluginNameFromPath(std::string strPath);
    bool pluginLoaded(std::string strPluginName);
    Result loadPluginLibrary(std::string strFilepath, bool bIsNameOnly = false);
    void queueUnloadPluginInstance(PluginInstance* icUnload);
    
    int spreadEvent(Event evEvent);
    int spreadServiceEvent(ServiceEvent seServiceEvent);
    Result cycle();
    
    void addPluginSearchPaths(std::list<std::string> lstPaths);
    void addPluginSearchPath(std::string strPath);
    
    PluginInstance* pluginInstanceByID(int nID);
    bool pluginFailedToLoadBefore(std::string strName);
  };
}


#endif /* __PLUGIN_SYSTEM_H__ */
