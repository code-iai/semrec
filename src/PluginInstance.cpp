#include <PluginInstance.h>


namespace beliefstate {
  PluginInstance::PluginInstance() {
    m_strName = "";
    m_vdLibHandle = NULL;
    m_piInstance = NULL;
    m_thrdPluginCycle = NULL;
    m_bRunCycle = true;
    m_resCycleResult = defaultResult();
  }
  
  PluginInstance::~PluginInstance() {
  }
  
  Result PluginInstance::loadPluginLibrary(string strFilepath) {
    Result resLoad = defaultResult();
    
    fstream fsFile;
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
	string strPrefix = "libbs_plugin_";
	m_strName = m_strName.substr(strPrefix.size());
	
	cout << "Loaded plugin '" << m_strName << "'" << endl;
	m_piInstance->setPluginName(m_strName);
      } else {
	resLoad.riResultIdentifier = RI_PLUGIN_LOADING_FAILED;
	resLoad.bSuccess = false;
	resLoad.strErrorMessage = "Failed to load library `" + strFilepath + "'.";
	cout << "Could not open shared library: " << dlerror() << endl;
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
      cout << "Initialized plugin '" << m_strName << "'" << endl;
    } else {
      cerr << "Failed to initialize plugin '" << m_strName << "'." << endl;
    }
    
    return resInit;
  }
  
  void PluginInstance::unload() {
    if(m_vdLibHandle) {
      m_piInstance->deinit();
      
      void (*destroyInstance)(plugins::Plugin*);
      destroyInstance = (void (*)(plugins::Plugin*))dlsym(m_vdLibHandle, "destroyInstance");
      destroyInstance(m_piInstance);
      dlclose(m_vdLibHandle);
      
      cout << "Unloaded plugin '" << m_strName << "'" << endl;
    }
  }
  
  int PluginInstance::pluginID() {
    return m_piInstance->pluginID();
  }
  
  Result PluginInstance::cycle() {
    if(m_thrdPluginCycle == NULL) {
      m_thrdPluginCycle = new thread(&PluginInstance::spinCycle, this);
    }
    
    return this->currentResult();
  }
  
  void PluginInstance::spinCycle() {
    while(m_bRunCycle) {
      Result resCycle = m_piInstance->cycle();
      
      m_mtxCycleResults.lock();
      for(list<Event>::iterator itEvt = resCycle.lstEvents.begin();
	  itEvt != resCycle.lstEvents.end();
	  itEvt++) {
	m_resCycleResult.lstEvents.push_back(*itEvt);
      }
      resCycle.lstEvents.clear();
      
      for(list<ServiceEvent>::iterator itEvt = resCycle.lstServiceEvents.begin();
	  itEvt != resCycle.lstServiceEvents.end();
	  itEvt++) {
	m_resCycleResult.lstServiceEvents.push_back(*itEvt);
      }
      resCycle.lstServiceEvents.clear();
      m_mtxCycleResults.unlock();
    }
  }
  
  list<string> PluginInstance::dependencies() {
    return m_piInstance->dependencies();
  }
  
  bool PluginInstance::subscribedToEvent(string strEventName) {
    return m_piInstance->subscribedToEvent(strEventName);
  }
  
  void PluginInstance::consumeEvent(Event evEvent) {
    m_piInstance->consumeEvent(evEvent);
  }
  
  bool PluginInstance::offersService(string strServiceName) {
    return m_piInstance->offersService(strServiceName);
  }
  
  Event PluginInstance::consumeServiceEvent(ServiceEvent seServiceEvent) {
    return m_piInstance->consumeServiceEvent(seServiceEvent);
  }
  
  string PluginInstance::name() {
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
