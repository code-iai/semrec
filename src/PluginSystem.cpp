#include <PluginSystem.h>


namespace beliefstate {
  PluginSystem::PluginSystem(int argc, char** argv) {
    m_argc = argc;
    m_argv = argv;
  }
  
  PluginSystem::~PluginSystem() {
    for(list<PluginInstance*>::iterator itPlugin = m_lstLoadedPlugins.begin();
	itPlugin != m_lstLoadedPlugins.end();
	itPlugin++) {
      PluginInstance* icCurrent = *itPlugin;
      
      icCurrent->unload();
      delete icCurrent;
    }
  }
  
  Result PluginSystem::loadPluginLibrary(string strFilepath) {
    PluginInstance* icLoad = new PluginInstance();
    
    Result resLoad = icLoad->loadPluginLibrary(strFilepath, m_argc, m_argv);
    if(resLoad.bSuccess) {
      m_lstLoadedPlugins.push_back(icLoad);
    } else {
      delete icLoad;
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
}
