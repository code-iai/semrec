#include <PluginSystem.h>


namespace beliefstate {
  PluginSystem::PluginSystem(int argc, char** argv) {
    m_argc = argc;
    m_argv = argv;
  }
  
  PluginSystem::~PluginSystem() {
    m_lstLoadedPlugins.reverse();
    
    for(list<PluginInstance*>::iterator itPlugin = m_lstLoadedPlugins.begin();
	itPlugin != m_lstLoadedPlugins.end();
	itPlugin++) {
      PluginInstance* icCurrent = *itPlugin;
      
      icCurrent->unload();
      delete icCurrent;
    }
  }
  
  bool PluginSystem::pluginLoaded(string strPluginName) {
    for(list<PluginInstance*>::iterator itPlugin = m_lstLoadedPlugins.begin();
	itPlugin != m_lstLoadedPlugins.end();
	itPlugin++) {
      PluginInstance* icCurrent = *itPlugin;
      
      if(icCurrent->name() == strPluginName) {
	return true;
      }
    }
    
    return false;
  }
  
  Result PluginSystem::loadPluginLibrary(string strFilepath) {
    PluginInstance* icLoad = NULL;
    
    list<string> lstSearchPaths = m_lstPluginSearchPaths;
    lstSearchPaths.push_front(""); // Add local path as search path
    
    Result resLoad = defaultResult();
    resLoad.bSuccess = false;
    
    for(list<string>::iterator itSP = lstSearchPaths.begin();
	itSP != lstSearchPaths.end();
	itSP++) {
      string strSP = *itSP;
      string strSearchFilepath = strSP + (strSP[strSP.size() - 1] != '/' && strFilepath[0] != '/' ? "/" : "") + strFilepath;
      
      icLoad = new PluginInstance();
      resLoad = icLoad->loadPluginLibrary(strSearchFilepath);
      if(resLoad.bSuccess) {
	// Check and meet dependencies
	list<string> lstDeps = icLoad->dependencies();
	for(list<string>::iterator itDep = lstDeps.begin();
	    itDep != lstDeps.end();
	    itDep++) {
	  string strDep = *itDep;
	  
	  if(this->pluginLoaded(strDep) == false) {
	    Result resLoadDep = this->loadPluginLibrary("libbs_plugin_" + strDep + ".so");
	    
	    if(resLoadDep.bSuccess == false) {
	      cerr << "Unable to meet dependency of '" << strSearchFilepath << "': '" << strDep << "'" << endl;
	      
	      resLoad.bSuccess = false;
	      resLoad.riResultIdentifier = RI_PLUGIN_DEPENDENCY_NOT_MET;
	      resLoad.strErrorMessage = strDep;
	      
	      break;
	    }
	  }
	}
	
	if(resLoad.bSuccess) {
	  // Initialize the plugin
	  icLoad->init(m_argc, m_argv);
	  m_lstLoadedPlugins.push_back(icLoad);
	  break;
	}
      } else {
	resLoad.bSuccess = false;
	resLoad.riResultIdentifier = RI_PLUGIN_LOADING_FAILED;
      }
      
      if(resLoad.bSuccess == false) {
	icLoad->unload();
	delete icLoad;
      } else {
	break;
      }
    }
    
    return resLoad;
  }
  
  void PluginSystem::queueUnloadPluginInstance(PluginInstance* icUnload) {
    m_lstUnloadPlugins.push_back(icUnload);
  }
  
  void PluginSystem::spreadEvent(Event evEvent) {
    for(list<PluginInstance*>::iterator itPlugin = m_lstLoadedPlugins.begin();
	itPlugin != m_lstLoadedPlugins.end();
	itPlugin++) {
      PluginInstance* piPlugin = *itPlugin;
      
      if(piPlugin->subscribedToEvent(evEvent.eiEventIdentifier)) {
	piPlugin->consumeEvent(evEvent);
      }
    }
  }
  
  Result PluginSystem::cycle() {
    Result resCycle = defaultResult();
    
    for(list<PluginInstance*>::iterator itPlugin = m_lstLoadedPlugins.begin();
	itPlugin != m_lstLoadedPlugins.end();
	itPlugin++) {
      PluginInstance* icCurrent = *itPlugin;
      Result resCurrent = icCurrent->cycle();
      
      if(resCurrent.bSuccess == false) {
	this->queueUnloadPluginInstance(icCurrent);
      } else {
	for(list<Event>::iterator itEvent = resCurrent.lstEvents.begin();
	    itEvent != resCurrent.lstEvents.end();
	    itEvent++) {
	  resCycle.lstEvents.push_back(*itEvent);
	}
      }
    }
    
    for(list<PluginInstance*>::iterator itPlugin = m_lstUnloadPlugins.begin();
	itPlugin != m_lstUnloadPlugins.end();
	itPlugin++) {
      PluginInstance* icCurrent = *itPlugin;
      
      icCurrent->unload();
      m_lstLoadedPlugins.remove(icCurrent);
      delete icCurrent;
    }
    
    return resCycle;
  }
  
  void PluginSystem::addPluginSearchPath(string strPath) {
    // Make sure it is in there only once.
    m_lstPluginSearchPaths.remove(strPath);
    
    m_lstPluginSearchPaths.push_back(strPath);
  }
}
