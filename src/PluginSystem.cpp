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


#include <PluginSystem.h>


namespace semrec {
  PluginSystem::PluginSystem(int argc, char** argv) {
    m_argc = argc;
    m_argv = argv;
    
    this->setMessagePrefixLabel("plugins");
  }
  
  PluginSystem::~PluginSystem() {
    m_lstLoadedPlugins.reverse();
    
    // Trigger kill signals
    for(PluginInstance* icCurrent : m_lstLoadedPlugins) {
      icCurrent->setRunning(false);
    }
    
    // Join all threads
    for(PluginInstance* icCurrent : m_lstLoadedPlugins) {
      icCurrent->waitForJoin();
    }
    
    // Delete all structures
    for(PluginInstance* icCurrent : m_lstLoadedPlugins) {
      icCurrent->unload();
      delete icCurrent;
    }
    
    m_lstLoadedPlugins.clear();
  }
  
  std::string PluginSystem::pluginNameFromPath(std::string strPath) {
    // Remove path
    const size_t last_slash_idx = strPath.find_last_of("\\/");
    if(std::string::npos != last_slash_idx) {
      strPath.erase(0, last_slash_idx + 1);
    }
    
    // Remove extension
    const size_t period_idx = strPath.rfind('.');
    if(std::string::npos != period_idx) {
      strPath.erase(period_idx);
    }
    
    // Remove plugin prefix
    std::string strPrefix = "libsr_plugin_";
    
    if(strPath.substr(0, strPrefix.size()) == strPrefix) {
      strPath = strPath.substr(strPrefix.size());
    }
    
    return strPath;
  }
  
  bool PluginSystem::pluginLoaded(std::string strPluginName) {
    for(PluginInstance* icCurrent : m_lstLoadedPlugins) {
      if(icCurrent->name() == strPluginName) {
	return true;
      }
    }
    
    return false;
  }
  
  void PluginSystem::setLoadDevelopmentPlugins(bool bLoadDevelopmentPlugins) {
    m_bLoadDevelopmentPlugins = bLoadDevelopmentPlugins;
  }
  
  bool PluginSystem::loadDevelopmentPlugins() {
    return m_bLoadDevelopmentPlugins;
  }
  
  bool PluginSystem::pluginFailedToLoadBefore(std::string strName) {
    for(std::string strPluginName : m_lstLoadFailedPlugins) {
      if(strPluginName == strName) {
	return true;
      }
    }
    
    return false;
  }
  
  Result PluginSystem::loadPluginLibrary(std::string strFilepath, bool bIsNameOnly) {
    PluginInstance* icLoad = NULL;
    std::string strPrefix = "libsr_plugin_";
    std::string strSuffix = ".so";
    
    if(bIsNameOnly) {
      strFilepath = strPrefix + strFilepath + strSuffix;
    }
    
    std::list<std::string> lstSearchPaths = m_lstPluginSearchPaths;
    lstSearchPaths.push_front("./"); // Add local path as search path
    
    Result resLoad = defaultResult();
    resLoad.bSuccess = false;
    
    if(this->pluginLoaded(this->pluginNameFromPath(strFilepath))) {
      this->info("Plugin '" + this->pluginNameFromPath(strFilepath) + "' already loaded.");
      resLoad.bSuccess = true;
    } else {
      if(!this->pluginFailedToLoadBefore(strFilepath)) {
	for(std::string strSP : lstSearchPaths) {
	  std::string strSearchFilepath = strSP + (strSP[strSP.size() - 1] != '/' && strFilepath[0] != '/' && strSP.size() > 0 ? "/" : "") + strFilepath;
	  
	  icLoad = new PluginInstance();
	  resLoad = icLoad->loadPluginLibrary(strSearchFilepath);
	  resLoad.piPlugin = icLoad;
	  
	  if(resLoad.bSuccess) {
	    // Check if this is a development plugin and if we're supposed to load it.
	    if((icLoad->developmentPlugin() && m_bLoadDevelopmentPlugins) || !icLoad->developmentPlugin()) {
	      if(icLoad->developmentPlugin()) {
		this->info("This is a development plugin: '" + strFilepath + "'");
	      }
	      
	      // Check and meet dependencies
	      std::list<std::string> lstDeps = icLoad->dependencies();
	      for(std::string strDep : lstDeps) {
		if(this->pluginLoaded(strDep) == false) {
		  Result resLoadDep = this->loadPluginLibrary(strPrefix + strDep + strSuffix);
		  
		  if(resLoadDep.bSuccess == false) {
		    this->fail("Unable to meet dependency of '" + strSearchFilepath + "': '" + strDep + "'");
		    
		    resLoad.bSuccess = false;
		    resLoad.riResultIdentifier = RI_PLUGIN_DEPENDENCY_NOT_MET;
		    resLoad.strErrorMessage = strDep;
		    
		    break;
		  }
		}
	      }
	    } else {
	      this->info("Not loading development plugin: '" + strFilepath + "'");
	      
	      resLoad.bSuccess = false;
	      resLoad.riResultIdentifier = RI_PLUGIN_DEVELOPMENT_NOT_LOADING;
	    }
	    
	    if(resLoad.bSuccess) {
	      // Initialize the plugin
	      Result rsResult = icLoad->init(m_argc, m_argv);
	      
	      if(rsResult.bSuccess) {
		m_lstLoadedPlugins.push_back(icLoad);
	      } else {
		resLoad.bSuccess = false;
		resLoad.riResultIdentifier = RI_PLUGIN_LOADING_FAILED;
	      }
	      
	      break;
	    }
	  } else {
	    resLoad.bSuccess = false;
	    resLoad.riResultIdentifier = RI_PLUGIN_LOADING_FAILED;
	  }
	  
	  if(resLoad.bSuccess == false) {
	    icLoad->unload();
	    m_lstLoadFailedPlugins.push_back(strFilepath);
	    
	    delete icLoad;
	  } else {
	    break;
	  }
	}
      } else {
	this->warn("This plugin failed to load before. Skipping it.");
      }
    }
    
    if(resLoad.bSuccess == false) {
      this->fail("Failed to load plugin '" + strFilepath + "'");
    } else {
      resLoad.piPlugin = icLoad;
    }
    
    return resLoad;
  }
  
  void PluginSystem::queueUnloadPluginInstance(PluginInstance* icUnload) {
    m_lstUnloadPlugins.push_back(icUnload);
  }
  
  int PluginSystem::spreadEvent(Event evEvent) {
    int nReceivers = 0;
    
    for(PluginInstance* piPlugin : m_lstLoadedPlugins) {
      if(piPlugin->subscribedToEvent(evEvent.strEventName)) {
	piPlugin->consumeEvent(evEvent);
	nReceivers++;
      }
    }
    
    return nReceivers;
  }
  
  int PluginSystem::spreadServiceEvent(ServiceEvent seServiceEvent) {
    std::list<Event> lstResultEvents;
    int nReceivers = 0;
    
    for(PluginInstance* piPlugin : m_lstLoadedPlugins) {
      if(piPlugin->offersService(seServiceEvent.strServiceName) ||
	 seServiceEvent.siServiceIdentifier == SI_RESPONSE) {
	Event evResult = piPlugin->consumeServiceEvent(seServiceEvent);
	nReceivers++;
	
	if(seServiceEvent.smResultModifier != SM_IGNORE_RESULTS) {
	  evResult.nOriginID = piPlugin->pluginID();
	  lstResultEvents.push_back(evResult);
	  
	  if(seServiceEvent.smResultModifier == SM_FIRST_RESULT) {
	    break;
	  } else {
	    // Aggregate results
	  }
	}
      }
    }
    
    PluginInstance* piRequester = this->pluginInstanceByID(seServiceEvent.nRequesterID);
    if(piRequester && seServiceEvent.smResultModifier != SM_IGNORE_RESULTS) {
      ServiceEvent seResponses = seServiceEvent;
      seResponses.lstResultEvents = lstResultEvents;
      seResponses.siServiceIdentifier = SI_RESPONSE;
      
      piRequester->consumeServiceEvent(seResponses);
    }
    
    return nReceivers;
  }
  
  Result PluginSystem::cycle() {
    Result resCycle = defaultResult();
    
    for(PluginInstance* icCurrent : m_lstLoadedPlugins) {
      Result resCurrent = icCurrent->cycle();
      
      for(StatusMessage smCurrent : resCurrent.lstStatusMessages) {
	resCycle.lstStatusMessages.push_back(smCurrent);
      }
      
      if(resCurrent.bSuccess == false) {
	// NOTE(winkler): This might also be a good place to implement
	// a recovery mechanism in case a plugin actually fails during
	// its cycle. Reload plugins and notify all "depending"
	// plugins (in order of dependency) to "recover".
	this->queueUnloadPluginInstance(icCurrent);
      } else {
	for(Event evtCurrent : resCurrent.lstEvents) {
	  resCycle.lstEvents.push_back(evtCurrent);
	}
	
	for(ServiceEvent seCurrent : resCurrent.lstServiceEvents) {
	  resCycle.lstServiceEvents.push_back(seCurrent);
	}
      }
    }
    
    for(PluginInstance* icCurrent : m_lstUnloadPlugins) {
      icCurrent->unload();
      m_lstLoadedPlugins.remove(icCurrent);
      delete icCurrent;
    }
    
    return resCycle;
  }
  
  void PluginSystem::addPluginSearchPaths(std::list<std::string> lstPaths) {
    for(std::string strPath : lstPaths) {
      this->addPluginSearchPath(strPath);
    }
  }
  
  void PluginSystem::addPluginSearchPath(std::string strPath) {
    // Make sure it is in there only once.
    m_lstPluginSearchPaths.remove(strPath);
    m_lstPluginSearchPaths.push_back(strPath);
  }
  
  PluginInstance* PluginSystem::pluginInstanceByID(int nID) {
    for(PluginInstance* icCurrent : m_lstLoadedPlugins) {
      if(icCurrent->pluginID() == nID) {
	return icCurrent;
      }
    }
    
    return NULL;
  }
}
