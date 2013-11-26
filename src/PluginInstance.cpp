#include <PluginInstance.h>


namespace beliefstate {
  PluginInstance::PluginInstance() {
    m_strName = "";
    m_vdLibHandle = NULL;
    m_piInstance = NULL;
  }
  
  PluginInstance::~PluginInstance() {
  }
  
  Result PluginInstance::loadPluginLibrary(string strFilepath, int argc, char** argv) {
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
	Result resInit = m_piInstance->init(argc, argv);
	
	if(resInit.bSuccess) {
	  m_strName = strFilepath; // TODO: Replace this by just the
				   // filename w/o extension!
	  cout << "Loaded plugin '" << m_strName << "'" << endl;
	} else {
	  resLoad = resInit;
	  dlclose(m_vdLibHandle);
	}
      } else {
	resLoad.riResultIdentifier = RI_PLUGIN_LOADING_FAILED;
	resLoad.bSuccess = false;
	resLoad.strErrorMessage = "Failed to load library `" + strFilepath + "'.";
      }
    } else {
      resLoad.riResultIdentifier = RI_FILE_NOT_FOUND;
      resLoad.bSuccess = false;
      resLoad.strErrorMessage = "File `" + strFilepath + "' could not be found.";
    }
    
    return resLoad;
  }
  
  void PluginInstance::unload() {
    m_piInstance->deinit();
    
    void (*destroyInstance)(plugins::Plugin*);
    destroyInstance = (void (*)(plugins::Plugin*))dlsym(m_vdLibHandle, "destroyInstance");
    destroyInstance(m_piInstance);
    dlclose(m_vdLibHandle);
    
    cout << "Unloaded plugin '" << m_strName << "'" << endl;
  }
  
  Result PluginInstance::cycle() {
    return m_piInstance->cycle();
  }
  
  bool PluginInstance::subscribedToEvent(EventIdentifier eiEventIdentifier) {
    return m_piInstance->subscribedToEvent(eiEventIdentifier);
  }
  
  void PluginInstance::consumeEvent(Event evEvent) {
    m_piInstance->consumeEvent(evEvent);
  }
}
