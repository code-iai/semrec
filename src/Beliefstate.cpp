#include <Beliefstate.h>


namespace beliefstate {
  Beliefstate::Beliefstate(int argc, char** argv) {
    m_psPlugins = NULL;
    m_bRun = true;
    m_argc = argc;
    m_argv = argv;
    m_strBaseDataDirectory = "";
  }
  
  Beliefstate::~Beliefstate() {
    if(m_psPlugins) {
      delete m_psPlugins;
    }
  }
  
  Result Beliefstate::init(string strConfigFile) {
    Result resInit = defaultResult();
    
    list<string> lstConfigFileLocations;
    lstConfigFileLocations.push_back("./");
    lstConfigFileLocations.push_back("~/.beliefstate/");
    lstConfigFileLocations.push_back(ros::package::getPath("beliefstate"));
    
    // Do the actual init here.
    m_psPlugins = new PluginSystem(m_argc, m_argv);
    
    bool bConfigLoaded = false;
    if(strConfigFile != "") {
      bConfigLoaded = this->loadConfigFile(strConfigFile);
    }
    
    if(!bConfigLoaded) {
      for(list<string>::iterator itPath = lstConfigFileLocations.begin();
	  itPath != lstConfigFileLocations.end();
	  itPath++) {
	string strPath = *itPath;
	
	if(this->loadConfigFile(strPath + "/config.cfg")) {
	  bConfigLoaded = true;
	  break;
	}
      }
    }
    
    if(bConfigLoaded) {
      // Set the settings concerning MongoDB, and experiment name mask
      // for each plugin here (through PluginSystem).
      
      for(list<string>::iterator itPluginName = m_lstPluginsToLoad.begin();
	  itPluginName != m_lstPluginsToLoad.end();
	  itPluginName++) {
	m_psPlugins->loadPluginLibrary(*itPluginName, true);
      }
    } else {
      cerr << "Failed to load a valid config file." << endl;
      resInit.bSuccess = false;
    }
    
    return resInit;
  }
  
  Result Beliefstate::deinit() {
    Result resInit = defaultResult();
    
    // Do the actual deinit here.
    
    return resInit;
  }
  
  bool Beliefstate::loadConfigFile(string strConfigFile) {
    if(this->fileExists(strConfigFile)) {
      Config cfgConfig;
      
      try {
	cfgConfig.readFile(strConfigFile.c_str());
	
	// Section: Persistent data storage
	Setting &sPersistentDataStorage = cfgConfig.lookup("persistent-data-storage");
	sPersistentDataStorage.lookupValue("base-data-directory", m_strBaseDataDirectory);
	
	bool bUseMongoDB;
	string strMongoDBHost;
	int nMongoDBPort;
	string strMongoDBDatabase;
	sPersistentDataStorage.lookupValue("use-mongodb", bUseMongoDB);
	
	if(bUseMongoDB) {
	  Setting &sMongoDB = cfgConfig.lookup("persistent-data-storage.mongodb");
	  sMongoDB.lookupValue("host", strMongoDBHost);
	  sMongoDB.lookupValue("port", nMongoDBPort);
	  sMongoDB.lookupValue("database", strMongoDBDatabase);
	}
	
	// Section: Experiment data
	string strExperimentNameMask;
	Setting &sExperimentData = cfgConfig.lookup("experiment-data");
	sExperimentData.lookupValue("experiment-name-mask", strExperimentNameMask);
	
	// -> Set the global settings
	ConfigSettings cfgsetCurrent = configSettings();
	cfgsetCurrent.bUseMongoDB = bUseMongoDB;
	cfgsetCurrent.strMongoDBHost = strMongoDBHost;
	cfgsetCurrent.nMongoDBPort = nMongoDBPort;
	cfgsetCurrent.strMongoDBDatabase = strMongoDBDatabase;
	cfgsetCurrent.strExperimentNameMask = strExperimentNameMask;
	setConfigSettings(cfgsetCurrent);
	
	// Section: Plugins
	Setting &sPluginsLoad = cfgConfig.lookup("plugins.load");
	m_lstPluginsToLoad.clear();
	for(int nI = 0; nI < sPluginsLoad.getLength(); nI++) {
	  string strLoad = sPluginsLoad[nI];
	  
	  m_lstPluginsToLoad.remove(strLoad);
	  m_lstPluginsToLoad.push_back(strLoad);
	}
	
	Setting &sPluginsPaths = cfgConfig.lookup("plugins.search-paths");
	for(int nI = 0; nI < sPluginsPaths.getLength(); nI++) {
	  string strPath = sPluginsPaths[nI];
	  
	  // Replace environment variables
	  const char* cROSWorkspace = getenv("ROS_WORKSPACE");
	  if(cROSWorkspace) {
	    string strROSWorkspace = cROSWorkspace;
	    this->replaceStringInPlace(strPath, "$ROS_WORKSPACE", strROSWorkspace);
	  }
	  
	  m_psPlugins->addPluginSearchPath(strPath);
	}
	
	return true;
      } catch(ParseException e) {
	cerr << "Error while parsing config file '" << strConfigFile << "': " << e.getError() << endl;
      } catch(...) {
	cerr << "Undefined error while parsing config file '" << strConfigFile << "'" << endl;
      }
    }
    
    return false;
  }
  
  bool Beliefstate::fileExists(string strFileName) {
    ifstream ifile(strFileName.c_str());
  
    if(ifile) {
      ifile.close();
      
      return true;
    }
  
    return false;
  }
  
  void Beliefstate::replaceStringInPlace(string& subject, const string& search, const string& replace) {
    size_t pos = 0;
    
    while((pos = subject.find(search, pos)) != string::npos) {
      subject.replace(pos, search.length(), replace);
      pos += replace.length();
    }
  }
  
  void Beliefstate::spreadEvent(Event evEvent) {
    if(m_psPlugins->spreadEvent(evEvent) == 0) {
      cerr << "[beliefstate] Unhandled event dropped." << endl;
      
      if(evEvent.cdDesignator) {
	cerr << "[beliefstate] Content was:" << endl;
	evEvent.cdDesignator->printDesignator();
      } else {
	cerr << "[beliefstate] No content given." << endl;
      }
    }
  }
  
  void Beliefstate::spreadServiceEvent(ServiceEvent seServiceEvent) {
    if(m_psPlugins->spreadServiceEvent(seServiceEvent) == 0) {
      cerr << "[beliefstate] Unhandled service event ('" << seServiceEvent.strServiceName << "') dropped." << endl;
      
      if(seServiceEvent.cdDesignator) {
	cerr << "[beliefstate] Content was:" << endl;
	seServiceEvent.cdDesignator->printDesignator();
      } else {
	cerr << "[beliefstate] No content given." << endl;
      }
    }
  }
  
  bool Beliefstate::cycle() {
    bool bContinue = true;
    
    if(m_bRun) {
      Result resCycle = m_psPlugins->cycle();
      
      if(resCycle.bSuccess) {
	// Events
	for(list<Event>::iterator itEvent = resCycle.lstEvents.begin();
	    itEvent != resCycle.lstEvents.end();
	    itEvent++) {
	  Event evEvent = *itEvent;
	  
	  // Distribute the event
	  this->spreadEvent(evEvent);
	  
	  // Clean up
	  if(evEvent.cdDesignator) {
	    delete evEvent.cdDesignator;
	  }
	}
	
	// Services
	for(list<ServiceEvent>::iterator itEvent = resCycle.lstServiceEvents.begin();
	    itEvent != resCycle.lstServiceEvents.end();
	    itEvent++) {
	  ServiceEvent seServiceEvent = *itEvent;
	  
	  // Distribute the event
	  this->spreadServiceEvent(seServiceEvent);
	  
	  // Clean up
	  if(seServiceEvent.cdDesignator) {
	    delete seServiceEvent.cdDesignator;
	  }
	}
      }
    } else {
      bContinue = false;
    }
    
    return bContinue;
  }
  
  void Beliefstate::triggerShutdown() {
    m_bRun = false;
  }
  
  void Beliefstate::setBaseDataDirectory(string strBaseDataDirectory) {
    ConfigSettings cfgsetCurrent = configSettings();
    cfgsetCurrent.strBaseDataDirectory = strBaseDataDirectory;
    setConfigSettings(cfgsetCurrent);
  }
  
  string Beliefstate::baseDataDirectory() {
    ConfigSettings cfgsetCurrent = configSettings();
    return cfgsetCurrent.strBaseDataDirectory;
  }
}
