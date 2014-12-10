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


#include <PluginInstance.h>


namespace semrec {
  PluginInstance::PluginInstance() {
    m_strName = "";
    m_vdLibHandle = NULL;
    m_piInstance = NULL;
    m_thrdPluginCycle = NULL;
    m_bRunCycle = true;
    m_resCycleResult = defaultResult();
    
    this->setMessagePrefixLabel("plugin-instance");
  }
  
  PluginInstance::~PluginInstance() {
  }
  
  Result PluginInstance::loadPluginLibrary(std::string strFilepath) {
    Result resLoad = defaultResult();
    
    std::fstream fsFile;
    fsFile.open(strFilepath.c_str(), std::fstream::in);
    
    if(fsFile.is_open()) {
      fsFile.close();
      
      m_vdLibHandle = dlopen(strFilepath.c_str(), RTLD_LAZY);
      if(m_vdLibHandle) {
	plugins::Plugin* (*createInstance)();
	createInstance = (plugins::Plugin* (*)())dlsym(m_vdLibHandle, "createInstance");
	m_piInstance = (plugins::Plugin*)createInstance();
	
	m_strName = strFilepath;
	
	// Remove path
	const size_t last_slash_idx = m_strName.find_last_of("\\/");
	if(std::string::npos != last_slash_idx) {
	  m_strName.erase(0, last_slash_idx + 1);
	}
	
	// Remove extension
	const size_t period_idx = m_strName.rfind('.');
	if(std::string::npos != period_idx) {
	  m_strName.erase(period_idx);
	}
	
	// Remove plugin prefix
	std::string strPrefix = "libbs_plugin_";
	m_strName = m_strName.substr(strPrefix.size());
	
	std::string strVersionString = m_piInstance->pluginVersion();
	
	this->info("Loaded plugin '" + m_strName + "'" + (strVersionString == "" ? "" : " (version: " + strVersionString + ")"));
	m_piInstance->setPluginName(m_strName);
      } else {
	resLoad.riResultIdentifier = RI_PLUGIN_LOADING_FAILED;
	resLoad.bSuccess = false;
	resLoad.strErrorMessage = "Failed to load library `" + strFilepath + "'.";
	this->fail("Could not open shared library: " + std::string(dlerror()));
      }
    } else {
      resLoad.riResultIdentifier = RI_FILE_NOT_FOUND;
      resLoad.bSuccess = false;
      resLoad.strErrorMessage = "File `" + strFilepath + "' could not be found.";
    }
    
    return resLoad;
  }
  
  Result PluginInstance::init(int argc, char** argv) {
    Result resInit = m_piInstance->init(argc, argv);
    
    if(resInit.bSuccess) {
      this->success("Initialized plugin '" + m_strName + "'");
    } else {
      this->fail("Failed to initialize plugin '" + m_strName + "'.");
    }
    
    return resInit;
  }
  
  void PluginInstance::setDevelopmentPlugin(bool bDevelopmentPlugin) {
    if(m_vdLibHandle) {
      m_piInstance->setDevelopmentPlugin(bDevelopmentPlugin);
    }
  }
  
  bool PluginInstance::developmentPlugin() {
    if(m_vdLibHandle) {
      return m_piInstance->developmentPlugin();
    }
    
    return false;
  }
  
  void PluginInstance::unload() {
    if(m_vdLibHandle) {
      m_piInstance->deinit();
      
      void (*destroyInstance)(plugins::Plugin*);
      destroyInstance = (void (*)(plugins::Plugin*))dlsym(m_vdLibHandle, "destroyInstance");
      destroyInstance(m_piInstance);
      dlclose(m_vdLibHandle);
      
      this->info("Unloaded plugin '" + m_strName + "'");
    }
  }
  
  int PluginInstance::pluginID() {
    return m_piInstance->pluginID();
  }
  
  Result PluginInstance::cycle() {
    if(m_thrdPluginCycle == NULL) {
      m_thrdPluginCycle = new std::thread(&PluginInstance::spinCycle, this);
    }
    
    return this->currentResult();
  }
  
  void PluginInstance::spinCycle() {
    while(m_bRunCycle) {
      Result resCycle = m_piInstance->cycle();
      
      m_mtxCycleResults.lock();
      for(Event evCurrent : resCycle.lstEvents) {
	m_resCycleResult.lstEvents.push_back(evCurrent);
      }
      resCycle.lstEvents.clear();
      
      for(ServiceEvent seCurrent : resCycle.lstServiceEvents) {
	m_resCycleResult.lstServiceEvents.push_back(seCurrent);
      }
      resCycle.lstServiceEvents.clear();
      
      for(StatusMessage smCurrent : resCycle.lstStatusMessages) {
	m_resCycleResult.lstStatusMessages.push_back(smCurrent);
      }
      // TODO(winkler): Maybe there is a `clear` missing here. Check this.
      
      m_mtxCycleResults.unlock();
      
      usleep(10);
    }
  }
  
  std::list<std::string> PluginInstance::dependencies() {
    return m_piInstance->dependencies();
  }
  
  bool PluginInstance::subscribedToEvent(std::string strEventName) {
    return m_piInstance->subscribedToEvent(strEventName);
  }
  
  void PluginInstance::consumeEvent(Event evEvent) {
    m_piInstance->consumeEvent(evEvent);
  }
  
  bool PluginInstance::offersService(std::string strServiceName) {
    return m_piInstance->offersService(strServiceName);
  }
  
  Event PluginInstance::consumeServiceEvent(ServiceEvent seServiceEvent) {
    return m_piInstance->consumeServiceEvent(seServiceEvent);
  }
  
  std::string PluginInstance::name() {
    return m_strName;
  }
  
  Result PluginInstance::currentResult() {
    Result resReturn = defaultResult();
    
    if(m_mtxCycleResults.try_lock()) {
      resReturn = m_resCycleResult;
      m_resCycleResult = defaultResult();
      m_mtxCycleResults.unlock();
    }
    
    return resReturn;
  }
  
  void PluginInstance::setRunning(bool bRunCycle) {
    m_bRunCycle = bRunCycle;
    
    m_piInstance->setRunning(bRunCycle);
  }
  
  void PluginInstance::waitForJoin() {
    m_thrdPluginCycle->join();
    delete m_thrdPluginCycle;
  }
}
