#include <plugins/supervisor/PluginSupervisor.h>


namespace beliefstate {
  namespace plugins {
    PLUGIN_CLASS::PLUGIN_CLASS() {
      m_bFirstExperiment = true;
    }
    
    PLUGIN_CLASS::~PLUGIN_CLASS() {
    }
    
    Result PLUGIN_CLASS::init(int argc, char** argv) {
      Result resInit = defaultResult();
      
      this->setSubscribedToEvent("startup-complete", true);
      this->setSubscribedToEvent("start-new-experiment", true);
      
      return resInit;
    }
    
    Result PLUGIN_CLASS::deinit() {
      return defaultResult();
    }
    
    Result PLUGIN_CLASS::cycle() {
      Result resCycle = defaultResult();
      this->deployCycleData(resCycle);
      
      return resCycle;
    }
    
    void PLUGIN_CLASS::consumeEvent(Event evEvent) {
      if(evEvent.strEventName == "start-new-experiment") {
	// Signal global experiment shut down.
	if(m_bFirstExperiment) {
	  // Only signal an experiment shutdown if this is not the
	  // first experiment.
	  m_bFirstExperiment = false;
	} else {
	  this->deployEvent(defaultEvent("experiment-shutdown"));
	}
	
	// Wait 1 sec for all plugins to realize what to do when
	// shutting down an experiment.
	sleep(1);
	
	ConfigSettings cfgsetCurrent = configSettings();
	// Create base data directory
	mkdir(cfgsetCurrent.strBaseDataDirectory.c_str(), 0777);
	
	int nIndex = 0;
	bool bExists;
	string strNewName;
	string strNewExp;
	
	do {
	  // This is a hardcoded length. If the length of the numeric
	  // identifier in the string actually exceeds this, this will
	  // probably result in a segfault. This should very rarely be
	  // the case, though. We're assuming no problem here.
	  char cName[cfgsetCurrent.strExperimentNameMask.size() + 10];
	  sprintf(cName, (const char*)(cfgsetCurrent.strExperimentNameMask.c_str()), nIndex);
	  strNewName = cfgsetCurrent.strBaseDataDirectory + "/" + cName + "/";
	  strNewExp = cName;
	  nIndex++;
	  
	  struct stat sb;
	  int nReturnStat = stat(strNewName.c_str(), &sb);
	  bExists = (nReturnStat == 0);
	} while(bExists);
	
	mkdir(strNewName.c_str(), 0777);
	cfgsetCurrent.strExperimentDirectory = strNewName;
	this->info("Created new experiment space: '" + strNewExp + "'.");
	setConfigSettings(cfgsetCurrent);
	
	string strSymlink = cfgsetCurrent.strBaseDataDirectory + "/" + cfgsetCurrent.strSymlinkName;
	remove(strSymlink.c_str());
	symlink(strNewName.c_str(), strSymlink.c_str());
	this->info("Symlink set accordingly.");
	
	// Signal global experiment start.
	this->deployEvent(defaultEvent("experiment-start"));
      } else if(evEvent.strEventName == "startup-complete") {
	this->deployEvent(defaultEvent("start-new-experiment"));
      }
    }
  }
  
  extern "C" plugins::PLUGIN_CLASS* createInstance() {
    return new plugins::PLUGIN_CLASS();
  }
  
  extern "C" void destroyInstance(plugins::PLUGIN_CLASS* icDestroy) {
    delete icDestroy;
  }
}
