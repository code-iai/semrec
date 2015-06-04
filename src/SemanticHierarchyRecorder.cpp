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


#include <SemanticHierarchyRecorder.h>


namespace semrec {
  SemanticHierarchyRecorder::SemanticHierarchyRecorder(int argc, char** argv) {
    m_psPlugins = NULL;
    m_bRun = true;
    m_argc = argc;
    m_argv = argv;
    m_bTerminalWindowResize = false;
    m_bCommandLineOutput = true;
    m_bDisplayConfigurationDetails = true;
    m_strVersion = SR_VERSION_STRING;
    
    this->setRedirectOutput(false);
    
    this->setMessagePrefixLabel("core");
    
    m_lstConfigFileLocations.push_back(""); // Current directory
    m_lstConfigFileLocations.merge(this->resolveDirectoryTokens("${HOME}/.semrec/")); // Home directory
  }
  
  SemanticHierarchyRecorder::~SemanticHierarchyRecorder() {
  }
  
  std::string SemanticHierarchyRecorder::version() {
    return m_strVersion;
  }
  
  Result SemanticHierarchyRecorder::init(std::string strConfigFile) {
    Result resInit = defaultResult();
    
    // Do the actual init here.
    m_psPlugins = new PluginSystem(m_argc, m_argv);
    
    std::list<std::string> lstConfigFiles;
    for(std::string strLocation : m_lstConfigFileLocations) {
      std::string strLocationCleaned = strLocation;
      if(strLocationCleaned[strLocationCleaned.length() - 1] != '/') {
	strLocationCleaned += "/";
      }
      
      strLocationCleaned += "config.cfg";
      lstConfigFiles.push_back(strLocationCleaned);
    }
    
    if(strConfigFile != "") {
      lstConfigFiles.push_front(strConfigFile);
    }
    
    bool bConfigLoaded = false;
    for(std::string strConfigFileCurrent : lstConfigFiles) {
      if(this->loadConfigFile(strConfigFileCurrent)) {
	this->info("Loaded config file '" + strConfigFileCurrent + "'.");
	bConfigLoaded = true;
	
	break;
      }
    }
    
    if(bConfigLoaded) {
      // Messages are distributed through the event system from now
      // on.
      this->setRedirectOutput(true);
      
      // Set the global PluginSystem settings.
      ConfigSettings cfgsetCurrent = configSettings();
      m_psPlugins->setLoadDevelopmentPlugins(cfgsetCurrent.bLoadDevelopmentPlugins);
      
      // Set the settings concerning MongoDB, and experiment name mask
      // for each plugin here (through PluginSystem).
      for(std::string strPluginName : m_lstPluginsToLoad) {
	Result rsResult = m_psPlugins->loadPluginLibrary(strPluginName, true);
	
	if(!rsResult.bSuccess) {
	  if(cfgsetCurrent.bFailedPluginsInvalidateStartup) {
	    this->fail("Failed plugin load invalidates startup. Cancelling.");
	    resInit.bSuccess = false;
	    break;
	  }
	} else {
	  if(rsResult.piPlugin) {
	    rsResult.piPlugin->setOnlyDisplayImportant(m_bOnlyDisplayImportant);
	  }
	}
      }
    } else {
      this->fail("Failed to load a valid config file.");
      resInit.bSuccess = false;
    }
    
    // Additional checks to make the user aware of potential problems
    if(this->workspaceDirectories().size() == 0) {
      this->warn("The workspace directory could not be resolved. This might cause problems, especially when trying to load plugins. Please ensure that your environment is set up properly. If everything seems alright, consider to override the workspace-dependent paths in a custom file (i.e. ~/.semrec/config.cfg).");
    }
    
    if(resInit.bSuccess) {
      m_lstGlobalEvents.push_back(defaultEvent("startup-complete"));
    }
    
    return resInit;
  }
  
  Result SemanticHierarchyRecorder::deinit() {
    Result resInit = defaultResult();
    
    if(m_psPlugins) {
      delete m_psPlugins;
    }
    
    return resInit;
  }
  
  bool SemanticHierarchyRecorder::loadConfigFile(std::string strConfigFile) {
    if(this->fileExists(strConfigFile)) {
      libconfig::Config cfgConfig;
      
      try {
	cfgConfig.readFile(strConfigFile.c_str());
	
	// Section: Miscellaneous -- IMPORTANT: Check first. This
	// section sets the 'workspace-directory' variable if
	// available. This affects all directory resolutions
	// afterwards that might use the ${WORKSPACE} token.
	bool bDisplayUnhandledEvents = true;
	bool bDisplayUnhandledServiceEvents = true;
	m_bOnlyDisplayImportant = false;
	
	if(cfgConfig.exists("miscellaneous")) {
	  libconfig::Setting &sMiscellaneous = cfgConfig.lookup("miscellaneous");
	  
	  if(cfgConfig.exists("miscellaneous.workspace-directories")) {
	    libconfig::Setting &sWSDirs = cfgConfig.lookup("miscellaneous.workspace-directories");
	    
	    for(int nI = 0; nI < sWSDirs.getLength(); nI++) {
	      std::string strWSDir = sWSDirs[nI];
	      m_lstWorkspaceDirectories.merge(this->resolveDirectoryTokens(strWSDir));
	    }
	  }
	  
	  sMiscellaneous.lookupValue("display-unhandled-events", bDisplayUnhandledEvents);
	  sMiscellaneous.lookupValue("display-unhandled-service-events", bDisplayUnhandledServiceEvents);
	  sMiscellaneous.lookupValue("command-line-output", m_bCommandLineOutput);
	  sMiscellaneous.lookupValue("only-display-important-messages", m_bOnlyDisplayImportant);
	  sMiscellaneous.lookupValue("display-configuration-details", m_bDisplayConfigurationDetails);
	  
	  if(!m_bCommandLineOutput) {
	    this->setRedirectOutput(true);
	  }
	}
	
	// Section: Persistent data storage
	std::string strBaseDataDirectory = "";
	std::string strMongoDBHost = "";
	std::string strMongoDBDatabase = "";
	int nMongoDBPort = 27017;
	bool bUseMongoDB = false;
	
	if(cfgConfig.exists("persistent-data-storage")) {
	  libconfig::Setting &sPersistentDataStorage = cfgConfig.lookup("persistent-data-storage");
	  sPersistentDataStorage.lookupValue("base-data-directory", strBaseDataDirectory);
	  
	  // NOTE(winkler): This selects the first (and mostly only)
	  // solution for the base data directory. If this produces
	  // multiple solutions, we have to reconsider what to do
	  // here.
	  strBaseDataDirectory = this->resolveDirectoryTokens(strBaseDataDirectory).front();
	  
	  sPersistentDataStorage.lookupValue("use-mongodb", bUseMongoDB);
	  
	  if(bUseMongoDB) {
	    if(cfgConfig.exists("persistent-data-storage.mongodb")) {
	      libconfig::Setting &sMongoDB = cfgConfig.lookup("persistent-data-storage.mongodb");
	      sMongoDB.lookupValue("host", strMongoDBHost);
	      sMongoDB.lookupValue("port", nMongoDBPort);
	      sMongoDB.lookupValue("database", strMongoDBDatabase);
	    }
	  }
	}
	
	// Section: Experiment data
	std::string strExperimentNameMask = "";
	std::string strSymlinkName = "";
	
	if(cfgConfig.exists("experiment-data")) {
	  libconfig::Setting &sExperimentData = cfgConfig.lookup("experiment-data");
	  sExperimentData.lookupValue("experiment-name-mask", strExperimentNameMask);
	  sExperimentData.lookupValue("symlink-name", strSymlinkName);
	}
	
	// Check to see if the data given so far is valid. If it is
	// not, abort loading and notify the user.
	bool bSettingsOK = true;
	
	if((bUseMongoDB == true && (strMongoDBHost == "" ||
				    strMongoDBDatabase == "" ||
				    nMongoDBPort == 0))) {
	  this->fail("Error while loading MongoDB settings:");
	  if(strMongoDBHost == "") {
	    this->fail(" - MongoDB Host is empty");
	  }
	  
	  if(strMongoDBHost == "") {
	    this->fail(" - MongoDB Database is empty");
	  }
	  
	  if(nMongoDBPort == 0) {
	    this->fail(" - MongoDB Port is not set or '0' (which is invalid)");
	  }
	  
	  bSettingsOK = false;
	}
	
	if(bSettingsOK == false) {
	  return false;
	}
	
	if(strSymlinkName == "" || strExperimentNameMask == "" || (strExperimentNameMask.find("%d") == std::string::npos && strExperimentNameMask.find("%s") == std::string::npos) || strBaseDataDirectory == "") {
	  if(strBaseDataDirectory == "") {
	    this->warn("The base data directory path is empty.");
	    strBaseDataDirectory = this->resolveDirectoryTokens("${HOME}/sr_experimental_data").front();
	    this->warn("Defaulting to: '" + strBaseDataDirectory + "'");
	  }
	  
	  if(strSymlinkName == "") {
	    this->warn("The symlink name for experiments is empty.");
	    strSymlinkName = "current-experiment";
	    this->warn("Defaulting to: '" + strSymlinkName + "'");
	  }
	  
	  if(strExperimentNameMask == "") {
	    this->warn("The experiment name mask is empty.");
	    strExperimentNameMask = "exp-%d";
	    this->warn("Defaulting to: '" + strExperimentNameMask + "'");
	  } else if(strExperimentNameMask.find("%d") == std::string::npos && strExperimentNameMask.find("%s") == std::string::npos) {
	    this->warn("The experiment name mask does not include the '\%d' or '\%s' escape sequence.");
	    this->warn("It is currently: '" + strExperimentNameMask + "'");
	    this->warn("This will cause your experiments to be overwritten. Be careful.");
	  }
	}
	
	// Section: Plugins
	bool bLoadDevelopmentPlugins = false;
	bool bFailedPluginsInvalidateStartup = true;
	std::vector<std::string> vecPluginOutputColors;
	bool bSearchPathsSet = false;
	
	if(cfgConfig.exists("plugins")) {
	  libconfig::Setting &sPlugins = cfgConfig.lookup("plugins");
	  sPlugins.lookupValue("load-development-plugins", bLoadDevelopmentPlugins);
	  sPlugins.lookupValue("failed-plugins-invalidate-startup", bFailedPluginsInvalidateStartup);
	  
	  if(cfgConfig.exists("plugins.load")) {
	    libconfig::Setting &sPluginsLoad = cfgConfig.lookup("plugins.load");
	    m_lstPluginsToLoad.clear();
	    
	    for(int nI = 0; nI < sPluginsLoad.getLength(); nI++) {
	      std::string strLoad = sPluginsLoad[nI];
	      
	      m_lstPluginsToLoad.remove(strLoad);
	      m_lstPluginsToLoad.push_back(strLoad);
	    }
	  }
	  
	  if(cfgConfig.exists("plugins.search-paths")) {
	    libconfig::Setting &sPluginsPaths = cfgConfig.lookup("plugins.search-paths");
	    
	    for(int nI = 0; nI < sPluginsPaths.getLength(); nI++) {
	      std::string strPath = sPluginsPaths[nI];
	      
	      std::list<std::string> lstPaths = this->resolveDirectoryTokens(strPath);
	      m_psPlugins->addPluginSearchPaths(lstPaths);
	      
	      bSearchPathsSet = true;
	    }
	  }
	  
	  if(cfgConfig.exists("plugins.colors")) {
	    libconfig::Setting &sPluginsColors = cfgConfig.lookup("plugins.colors");
	    
	    for(int nI = 0; nI < sPluginsColors.getLength(); nI++) {
	      std::string strColor = sPluginsColors[nI];
	      vecPluginOutputColors.push_back(strColor);
	    }
	  }
	  
	  if(cfgConfig.exists("plugins.individual-configurations")) {
	    libconfig::Setting &sPluginsIndividualConfigurations = cfgConfig.lookup("plugins.individual-configurations");
	    
	    for(int nI = 0; nI < sPluginsIndividualConfigurations.getLength(); nI++) {
	      std::string strPluginName;
	      
	      if(sPluginsIndividualConfigurations[nI].lookupValue("plugin", strPluginName)) {
		if(m_bDisplayConfigurationDetails) {
		  this->info("Loading per-plugin configuration for plugin '" + strPluginName + "'");
		}
		
		Designator* cdConfig = getPluginConfig(strPluginName);
		
		if(this->loadIndividualPluginConfigurationBranch(sPluginsIndividualConfigurations[nI], cdConfig, "", true) == false) {
		  this->warn("Failed to load configuration for plugin '" + strPluginName + "'.");
		}
	      } else {
		this->warn("No 'plugin'-field specified for individual plugin configuration.");
	      }
	    }
	  }
	}
	
	std::vector<std::string> vecAdditionalSearchPaths = this->getAdditionalSearchPaths();
	for(std::string strPath : vecAdditionalSearchPaths) {
	  m_psPlugins->addPluginSearchPath(strPath);
	  
	  bSearchPathsSet = true;
	}
	
	// Check if any search paths were set
	if(bSearchPathsSet == false) {
	  this->warn("You didn't specify any search paths. This will prevent the system");
	  this->warn("from finding any plugins. A default will be assumed.");
	  std::list<std::string> lstPaths = this->resolveDirectoryTokens("$WORKSPACE/lib/");
	  std::string strPaths = "";
	  for(std::string strPath : lstPaths) {
	    if(strPaths != "") {
	      strPaths += ", ";
	    }
	    
	    strPaths += strPath;
	  }
	  
	  this->warn("Defaulting to: '" + strPaths + "'");
	  
	  m_psPlugins->addPluginSearchPaths(lstPaths);
	}
	
	if(vecPluginOutputColors.size() == 0) {
	  this->warn("The plugin output colors are empty. This might cause display issues.");
	  
	  vecPluginOutputColors.push_back("31");
	  vecPluginOutputColors.push_back("32");
	  vecPluginOutputColors.push_back("33");
	  vecPluginOutputColors.push_back("34");
	  vecPluginOutputColors.push_back("35");
	  vecPluginOutputColors.push_back("36");
	  vecPluginOutputColors.push_back("37");
	  
	  std::string strColors = "";
	  for(std::string strC : vecPluginOutputColors) {
	    strColors += (strColors != "" ? ", " : "") + std::string("\033[0;") + strC + "m" + strC + "\033[0m";
	  }
	  
	  this->warn("Defaulting to: " + strColors);
	}
	
	// -> Set the global settings
	ConfigSettings cfgsetCurrent = configSettings();
	cfgsetCurrent.bLoadDevelopmentPlugins = bLoadDevelopmentPlugins;
	cfgsetCurrent.bFailedPluginsInvalidateStartup = bFailedPluginsInvalidateStartup;
	cfgsetCurrent.bUseMongoDB = bUseMongoDB;
	cfgsetCurrent.strMongoDBHost = strMongoDBHost;
	cfgsetCurrent.nMongoDBPort = nMongoDBPort;
	cfgsetCurrent.strMongoDBDatabase = strMongoDBDatabase;
	cfgsetCurrent.strExperimentNameMask = strExperimentNameMask;
	cfgsetCurrent.strBaseDataDirectory = strBaseDataDirectory;
	cfgsetCurrent.strSymlinkName = strSymlinkName;
	cfgsetCurrent.bDisplayUnhandledEvents = bDisplayUnhandledEvents;
	cfgsetCurrent.bDisplayUnhandledServiceEvents = bDisplayUnhandledServiceEvents;
	cfgsetCurrent.vecPluginOutputColors = vecPluginOutputColors;
	cfgsetCurrent.bOnlyDisplayImportant = m_bOnlyDisplayImportant;
	setConfigSettings(cfgsetCurrent);
	
	return true;
      } catch(libconfig::ParseException e) {
	std::stringstream sts;
        sts << e.getLine();
	
	this->fail("Error while parsing config file '" + strConfigFile + "': " + e.getError() + ", on line " + sts.str());
      } catch(...) {
	this->fail("Undefined error while parsing config file '" + strConfigFile + "'");
      }
    }
    
    return false;
  }
  
  bool SemanticHierarchyRecorder::loadIndividualPluginConfigurationBranch(libconfig::Setting &sBranch, KeyValuePair* ckvpInto, std::string strConfigPath, bool bIgnorePluginField) {
    for(int nJ = 0; nJ < sBranch.getLength(); nJ++) {
      if(sBranch.getType() != libconfig::Setting::TypeGroup) {
	for(int nI = 0; nI < sBranch.getLength(); nI++) {
	  std::stringstream sts;
	  sts << nI;
	  
	  std::string strConfigKeyPath = strConfigPath + (strConfigPath == "" ? "" : "/") + sts.str();
	  if(m_bDisplayConfigurationDetails) {
	    this->info(" - " + strConfigKeyPath);
	  }
	  
	  KeyValuePair* ckvpChild = ckvpInto->addChild(sts.str());
	  
	  switch(sBranch[nI].getType()) {
	  case libconfig::Setting::TypeString: {
	    std::string strContent = sBranch[nI];
	    // NOTE(winkler): Using the first generated path
	    // here. Additional considerations might be necessary -
	    // and maybe it is wiser to allow the individually
	    // configured plugins access to all solutions instead of
	    // just the first. This will require some refactoring in
	    // the code of the respective plugins as well, though.
	    strContent = this->resolveDirectoryTokens(strContent).front();
	    ckvpInto->setValue(sts.str(), strContent);
	  } break;
	    
	  case libconfig::Setting::TypeInt:
	  case libconfig::Setting::TypeInt64: {
	    int nContent = sBranch[nI];
	    ckvpInto->setValue(sts.str(), nContent);
	  } break;
	    
	  case libconfig::Setting::TypeFloat: {
	    int fContent = sBranch[nI];
	    ckvpInto->setValue(sts.str(), fContent);
	  } break;
	    
	  case libconfig::Setting::TypeBoolean: {
	    bool bContent = sBranch[nI];
	    ckvpInto->setValue(sts.str(), bContent);
	  } break;
	    
	  default:
	    this->warn("Unknown type in config file for key path '" + strConfigKeyPath + "'!");
	  }
	}
      } else {
	std::string strConfigDetailName = sBranch[nJ].getName();
	
	if(strConfigDetailName != "plugin" || !bIgnorePluginField) {
	  if(m_bDisplayConfigurationDetails) {
	    this->info(" - " + strConfigPath + (strConfigPath == "" ? "" : "/") + strConfigDetailName);
	  }
	  
	  switch(sBranch[strConfigDetailName].getType()) {
	  case libconfig::Setting::TypeString: {
	    std::string strContent;
	    sBranch.lookupValue(strConfigDetailName, strContent);
	    strContent = this->resolveDirectoryTokens(strContent).front();
	    ckvpInto->setValue(strConfigDetailName, strContent);
	  } break;
		      
	  case libconfig::Setting::TypeInt:
	  case libconfig::Setting::TypeInt64: {
	    int nContent;
	    sBranch.lookupValue(strConfigDetailName, nContent);
	    ckvpInto->setValue(strConfigDetailName, nContent);
	  } break;
	    
	  case libconfig::Setting::TypeFloat: {
	    float fContent;
	    sBranch.lookupValue(strConfigDetailName, fContent);
	    ckvpInto->setValue(strConfigDetailName, fContent);
	  } break;
		      
	  case libconfig::Setting::TypeBoolean: {
	    bool bContent;
	    sBranch.lookupValue(strConfigDetailName, bContent);
	    ckvpInto->setValue(strConfigDetailName, bContent);
	  } break;
	  
	  case libconfig::Setting::TypeGroup: {
	    KeyValuePair* ckvpChild = ckvpInto->addChild(strConfigDetailName);
	    libconfig::Setting& sBranchChild = sBranch[strConfigDetailName];
	    
	    if(this->loadIndividualPluginConfigurationBranch(sBranchChild, ckvpChild, strConfigPath + (strConfigPath == "" ? "" : "/") + strConfigDetailName) == false) {
	      return false;
	    }
	  } break;
	    
	  case libconfig::Setting::TypeArray: {
	    KeyValuePair* ckvpChild = ckvpInto->addChild(strConfigDetailName);
	    libconfig::Setting& sBranchChild = sBranch[strConfigDetailName];
	    
	    if(this->loadIndividualPluginConfigurationBranch(sBranchChild, ckvpChild, strConfigPath + (strConfigPath == "" ? "" : "/") + strConfigDetailName) == false) {
	      return false;
	    }
	  } break;
	  }
	}
      }
    }
    
    return true;
  }
  
  bool SemanticHierarchyRecorder::spreadEvent(Event evEvent) {
    if(m_psPlugins->spreadEvent(evEvent) == 0) {
      ConfigSettings cfgSet = configSettings();
      if(cfgSet.bDisplayUnhandledEvents) {
	this->warn("Unhandled event dropped: '" + evEvent.strEventName + "'");
	
	if(evEvent.cdDesignator) {
	  //this->warn("Content was:");
	  //evEvent.cdDesignator->printDesignator();
	}
      }
      
      if(!this->handleUnhandledEvent(evEvent)) {
	if(!evEvent.bPreempt) {
	  //m_lstGlobalEvents.push_back(evEvent);
	}
      }
      
      return false; // Event was not received
    }
    
    return true; // Event was received by some entity (e.g. plugin)
  }
  
  void SemanticHierarchyRecorder::spreadServiceEvent(ServiceEvent seServiceEvent) {
    if(m_psPlugins->spreadServiceEvent(seServiceEvent) == 0) {
      // The service event wasn't handled (i.e. there was no valid
      // receiver for it).
      ConfigSettings cfgSet = configSettings();
      
      if(cfgSet.bDisplayUnhandledServiceEvents) {
	this->warn("Unhandled service event ('" + seServiceEvent.strServiceName + "') dropped.");
	
	if(seServiceEvent.cdDesignator) {
	  //this->warn("Content was:");
	  //seServiceEvent.cdDesignator->printDesignator();
	}
      }
      
      // Send back an answer to the initial caller, stating that there
      // was no response (this will help avoiding deadlocks on plugins
      // that actually wait for the reply).
      
      // TODO(winkler): Implement automatic reply to non-answered
      // service event requests.
    }
  }
  
  bool SemanticHierarchyRecorder::cycle() {
    bool bContinue = true;
    
    if(m_bRun) {
      Result resCycle = m_psPlugins->cycle();
      
      // Forward all status messages collected from the plugins and
      // the core into the event system
      std::list<StatusMessage> lstCoreMessages = queuedMessages();
      for(StatusMessage smCurrent : lstCoreMessages) {
	resCycle.lstStatusMessages.push_back(smCurrent);
      }
      
      for(StatusMessage smCurrent : resCycle.lstStatusMessages) {
	Event evEvent = defaultEvent("status-message");
	evEvent.msgStatusMessage = smCurrent;
	evEvent.bPreempt = false;
	
	if(this->spreadEvent(evEvent)) {
	  this->setRedirectOutput(true);
	}
      }
      
      for(Event evtCurrent : m_lstGlobalEvents) {
	resCycle.lstEvents.push_back(evtCurrent);
      }
      
      if(m_lstGlobalEvents.size() > 0) {
	m_lstGlobalEvents.clear();
      }
      
      if(resCycle.bSuccess) {
	// NOTE(winkler): (Service)Event distribution was done in a
	// linear manner before. This was changed, as the incoming
	// events in the feeder plugins (for example, ROS) might
	// appear in a different order than they are actually
	// processed in the message queue. The order in the feeder
	// plugins is __CORRECT__. Therefore, all events in them are
	// (automatically) equipped with a sequence number. This
	// number is evaluated here, distributing (i.e. spreading) the
	// next event on the line, starting with the lowest. This
	// measure was taken to prevent race conditions, which came up
	// due to fast, but ordered messages from outside.
	
	// While there are events in either list, look for the lowest
	// one.
	while(resCycle.lstEvents.size() > 0 || resCycle.lstServiceEvents.size() > 0) {
	  // Identify the next highest (i.e. at the moment lowest)
	  // sequence number
	  int nSequenceNumber = 100000; // Dirty: Don't put more than
					// 100000 msgs in the queue at
					// the same time, or the
					// system will deadlock.
	  
	  // In Events
	  for(Event evEvent : resCycle.lstEvents) {
	    if(evEvent.nSequenceNumber < nSequenceNumber) {
	      nSequenceNumber = evEvent.nSequenceNumber;
	    }
	  }
	  
	  // In ServiceEvents
	  for(ServiceEvent seEvent : resCycle.lstServiceEvents) {
	    if(seEvent.nSequenceNumber < nSequenceNumber) {
	      nSequenceNumber = seEvent.nSequenceNumber;
	    }
	  }
	  
	  // Spread the respective (Service)Event
	  bool bWasSpread = false;
	  
	  for(std::list<Event>::iterator itEvent = resCycle.lstEvents.begin();
	      itEvent != resCycle.lstEvents.end(); itEvent++) {
	    Event evEvent = *itEvent;
	    
	    if(evEvent.nSequenceNumber == nSequenceNumber) {
	      // Distribute the event
	      this->spreadEvent(evEvent);
	      
	      // Clean up
	      if(evEvent.cdDesignator) {
		delete evEvent.cdDesignator;
	      }
	      
	      resCycle.lstEvents.erase(itEvent);
	      
	      nSequenceNumber = evEvent.nSequenceNumber;
	      bWasSpread = true;
	      
	      break;
	    }
	  }
	  
	  if(!bWasSpread) {
	    for(std::list<ServiceEvent>::iterator itEvent = resCycle.lstServiceEvents.begin();
		itEvent != resCycle.lstServiceEvents.end(); itEvent++) {
	      ServiceEvent seEvent = *itEvent;
	      
	      if(seEvent.nSequenceNumber == nSequenceNumber) {
		// Distribute the event
		this->spreadServiceEvent(seEvent);
		
		// Clean up
		if(seEvent.cdDesignator) {
		  if(!seEvent.bPreserve) {
		    delete seEvent.cdDesignator;
		  }
		}
		
		resCycle.lstServiceEvents.erase(itEvent);
		
		nSequenceNumber = seEvent.nSequenceNumber;
		bWasSpread = true;
		
		break;
	      }
	    }
	  }
	  
	  if(!bWasSpread) {
	    // This should __never_ever__ happen. The sequence number
	    // we found before wasn't found again. This means either
	    // binary or memory corruption.
	    this->fail("Sequence number hazard! Restart and possibly recompile.");
	  }
	}
	
	resetSequenceNumbers();
	
	// Special events
	m_mtxTerminalResize.lock();
	bool bTerminalWindowResize = m_bTerminalWindowResize;
	m_bTerminalWindowResize = false;
	m_mtxTerminalResize.unlock();
	
	if(bTerminalWindowResize) {
	  Event evResize = defaultEvent("resize-terminal-window");
	  this->spreadEvent(evResize);
	}
      }
    } else {
      bContinue = false;
      
      // Last shot for the unhandled messages
      std::list<StatusMessage> lstCoreMessages = queuedMessages();
      for(StatusMessage smCurrent : lstCoreMessages) {
	Event evMessage = defaultEvent("status-message");
	evMessage.msgStatusMessage = smCurrent;
	
	this->handleUnhandledEvent(evMessage);
      }
      
      // Send the shutdown message
      m_lstGlobalEvents.push_back(defaultEvent("shutdown"));
    }
    
    return bContinue;
  }
  
  void SemanticHierarchyRecorder::triggerShutdown() {
    m_bRun = false;
  }
  
  void SemanticHierarchyRecorder::triggerTerminalResize() {
    m_mtxTerminalResize.lock();
    m_bTerminalWindowResize = true;
    m_mtxTerminalResize.unlock();
  }
  
  void SemanticHierarchyRecorder::setBaseDataDirectory(std::string strBaseDataDirectory) {
    ConfigSettings cfgsetCurrent = configSettings();
    cfgsetCurrent.strBaseDataDirectory = strBaseDataDirectory;
    setConfigSettings(cfgsetCurrent);
  }
  
  std::string SemanticHierarchyRecorder::baseDataDirectory() {
    ConfigSettings cfgsetCurrent = configSettings();
    return cfgsetCurrent.strBaseDataDirectory;
  }
  
  std::list<std::string> SemanticHierarchyRecorder::workspaceDirectories() {
    return m_lstWorkspaceDirectories;
  }
  
  std::string SemanticHierarchyRecorder::homeDirectory() {
    std::string strHome = "";
    
    char* cHome = getenv("HOME");
    if(cHome) {
      strHome = cHome;
    }
    
    return strHome;
  }
  
  std::list<std::string> SemanticHierarchyRecorder::resolveDirectoryTokens(std::string strSubject) {
    // First, make list of things to replace
    std::list< std::pair<std::string, bool> > lstTokens;
    
    size_t pos = 0;
    while((pos = strSubject.find("$", pos)) != std::string::npos) {
      size_t offset = 1;
      
      if(pos < strSubject.size() - 1) {
	if(strSubject.at(pos + 1) != ' ') {
	  std::string strToken = "";
	  bool bInBrackets = false;
	  
	  if(strSubject.at(pos + 1) == '{') {
	    offset++;
	    size_t pos_endbracket = strSubject.find("}", pos + 1);
	    
	    if(pos_endbracket != std::string::npos) {
	      strToken = strSubject.substr(pos + 2, pos_endbracket - (pos + 2));
	      offset += 1 + pos_endbracket - (pos + 2);
	      bInBrackets = true;
	    } else {
	      // This token is invalid (no ending bracket).
	    }
	  } else {
	    size_t pos_space_or_end = strSubject.find_first_of(" /.~_-\\$", pos + 1);
	    if(pos_space_or_end == std::string::npos) {
	      // The rest of the string is part of the token
	      strToken = strSubject.substr(pos + 1);
	      offset += pos + 1;
	    } else {
	      // The token is delimited by a space
	      strToken = strSubject.substr(pos + 1, pos_space_or_end - (pos + 1));
	      offset += pos_space_or_end - (pos + 1);
	    }
	  }
	  
	  if(strToken != "") {
	    lstTokens.push_back(std::make_pair(strToken, bInBrackets));
	  }
	}
      }
      
      pos += offset;
    }
    
    std::list<std::string> lstResolved;
    
    for(std::pair<std::string, bool> prToken : lstTokens) {
      std::string strToken = prToken.first;
      bool bInBrackets = prToken.second;
      std::string strToReplace = (bInBrackets ? "${" + strToken + "}" : "$" + strToken);
      std::list<std::string> lstReplacements = this->findTokenReplacements(strToken);
      
      if(lstReplacements.size() == 0) {
	this->warn("Failed to resolve token '" + strToken + "', asserting empty string.");
	
	lstResolved.push_back("");
      } else {
	for(std::string strReplacement : lstReplacements) {
	  std::string strSubjectTemp = strSubject;
	  this->replaceStringInPlace(strSubjectTemp, strToReplace, strReplacement);
	  
	  lstResolved.push_back(strSubjectTemp);
	}
      }
    }
    
    if(lstResolved.size() == 0) {
      lstResolved.push_back(strSubject);
    }
    
    return lstResolved;
  }
  
  std::list<std::string> SemanticHierarchyRecorder::findTokenReplacements(std::string strToken) {
    std::list<std::string> lstReplacements;
    
    if(strToken == "HOME") {
      lstReplacements.push_back(this->homeDirectory());
    }
    
    return lstReplacements;
  }
  
  bool SemanticHierarchyRecorder::handleUnhandledEvent(Event evEvent) {
    if(evEvent.strEventName == "status-message") {
      StatusMessage msgStatus = evEvent.msgStatusMessage;
      
      if(m_bCommandLineOutput) {
	this->setRedirectOutput(false);
      } else {
	this->setRedirectOutput(true);
      }
      
      return true;
    }
    
    return false;
  }
  
  std::list<std::string> SemanticHierarchyRecorder::findPrefixPaths(std::string strPathList, std::string strMatchingSuffix, std::string strDelimiter) {
    std::list<std::string> lstPathsReturn;
    
    while(strPathList.length() > 0) {
      size_t szDelimiterPos = strPathList.find(strDelimiter);
      std::string strCheckPart = "";
      
      if(szDelimiterPos != std::string::npos) {
	strCheckPart = strPathList.substr(0, szDelimiterPos);
      } else {
	strCheckPart = strPathList;
      }
      
      if(strCheckPart.length() >= strMatchingSuffix.length()) {
	if(strCheckPart.substr(strCheckPart.length() - strMatchingSuffix.length()) == strMatchingSuffix) {
	  lstPathsReturn.push_back(strCheckPart.substr(0, strCheckPart.length() - strMatchingSuffix.length()));
	}
      }
      
      if(szDelimiterPos != std::string::npos) {
	strPathList = strPathList.substr(strCheckPart.length() + 1);
      } else {
	strPathList = "";
      }
    }
    
    return lstPathsReturn;
  }
  
  std::vector<std::string> SemanticHierarchyRecorder::getAdditionalSearchPaths() {
    std::vector<std::string> vecEmpty;
    
    return vecEmpty;
  }
}
