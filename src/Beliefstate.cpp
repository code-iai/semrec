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


#include <Beliefstate.h>


namespace beliefstate {
  Beliefstate::Beliefstate(int argc, char** argv) {
    m_psPlugins = NULL;
    m_bRun = true;
    m_argc = argc;
    m_argv = argv;
    m_strWorkspaceDirectory = "";
    m_bTerminalWindowResize = false;
    this->setRedirectOutput(false);
    
    this->setMessagePrefixLabel("core");
    
    m_lstConfigFileLocations.push_back("./");
    m_lstConfigFileLocations.push_back("~/.beliefstate/");
  }
  
  Beliefstate::~Beliefstate() {
  }
  
  Result Beliefstate::init(string strConfigFile) {
    Result resInit = defaultResult();
    
    // Do the actual init here.
    m_psPlugins = new PluginSystem(m_argc, m_argv);
    
    bool bConfigLoaded = false;
    if(strConfigFile != "") {
      bConfigLoaded = this->loadConfigFile(strConfigFile);
    }
    
    if(!bConfigLoaded) {
      for(list<string>::iterator itPath = m_lstConfigFileLocations.begin();
	  itPath != m_lstConfigFileLocations.end();
	  itPath++) {
	string strPath = *itPath;
	if(strPath.length() > 0) {
	  if(strPath[strPath.length() - 1] != '/') {
	    strPath += "/";
	  }
	}
	
	if(this->loadConfigFile(strPath + "config.cfg")) {
	  bConfigLoaded = true;
	  this->info("Loaded config file '" + strPath + "config.cfg'.");
	  
	  break;
	}
      }
    } else {
      this->info("Loaded config file '" + strConfigFile + "'.");
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
      for(list<string>::iterator itPluginName = m_lstPluginsToLoad.begin();
	  itPluginName != m_lstPluginsToLoad.end();
	  itPluginName++) {
	m_psPlugins->loadPluginLibrary(*itPluginName, true);
      }
    } else {
      this->fail("Failed to load a valid config file.");
      resInit.bSuccess = false;
    }
    
    // Additional checks to make the user aware of potential problems
    if(this->workspaceDirectory() == "") {
      this->warn("The workspace directory could not be resolved. This might cause problems, especially when trying to load plugins. Please ensure that your environment is set up properly. If everything seems alright, consider to override the workspace-dependent paths in a custom file (i.e. ~/.beliefstate/config.cfg).");
    }
    
    m_lstGlobalEvents.push_back(defaultEvent("startup-complete"));
    
    return resInit;
  }
  
  Result Beliefstate::deinit() {
    Result resInit = defaultResult();
    
    if(m_psPlugins) {
      delete m_psPlugins;
    }
    
    return resInit;
  }
  
  bool Beliefstate::loadConfigFile(string strConfigFile) {
    if(this->fileExists(strConfigFile)) {
      Config cfgConfig;
      
      try {
	cfgConfig.readFile(strConfigFile.c_str());
	
	// Section: Miscellaneous -- IMPORTANT: Check first. This
	// section sets the 'workspace-directory' variable if
	// available. This affects all directory resolutions
	// afterwards that might use the ${WORKSPACE} token.
	bool bDisplayUnhandledEvents = true;
	bool bDisplayUnhandledServiceEvents = true;
	
	if(cfgConfig.exists("miscellaneous")) {
	  Setting &sMiscellaneous = cfgConfig.lookup("miscellaneous");
	  sMiscellaneous.lookupValue("display-unhandled-events", bDisplayUnhandledEvents);
	  sMiscellaneous.lookupValue("display-unhandled-service-events", bDisplayUnhandledServiceEvents);
	  sMiscellaneous.lookupValue("workspace-directory", m_strWorkspaceDirectory);
	}
	
	// Section: Persistent data storage
	string strBaseDataDirectory = "";
	bool bUseMongoDB = false;
	string strMongoDBHost = "";
	int nMongoDBPort = 27017;
	string strMongoDBDatabase = "";
	
	if(cfgConfig.exists("persistent-data-storage")) {
	  Setting &sPersistentDataStorage = cfgConfig.lookup("persistent-data-storage");
	  sPersistentDataStorage.lookupValue("base-data-directory", strBaseDataDirectory);
	  strBaseDataDirectory = this->resolveDirectoryTokens(strBaseDataDirectory);
	  
	  sPersistentDataStorage.lookupValue("use-mongodb", bUseMongoDB);
	  
	  if(bUseMongoDB) {
	    if(cfgConfig.exists("persistent-data-storage.mongodb")) {
	      Setting &sMongoDB = cfgConfig.lookup("persistent-data-storage.mongodb");
	      sMongoDB.lookupValue("host", strMongoDBHost);
	      sMongoDB.lookupValue("port", nMongoDBPort);
	      sMongoDB.lookupValue("database", strMongoDBDatabase);
	    }
	  }
	}
	
	// Section: Experiment data
	string strExperimentNameMask = "";
	string strSymlinkName = "";
	
	if(cfgConfig.exists("experiment-data")) {
	  Setting &sExperimentData = cfgConfig.lookup("experiment-data");
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
	
	if(strSymlinkName == "" || strExperimentNameMask == "" || (strExperimentNameMask.find("%d") == string::npos && strExperimentNameMask.find("%s") == string::npos) || strBaseDataDirectory == "") {
	  if(strBaseDataDirectory == "") {
	    this->warn("The base data directory path is empty.");
	    strBaseDataDirectory = this->resolveDirectoryTokens("${HOME}/bs_experimental_data");
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
	  } else if(strExperimentNameMask.find("%d") == string::npos && strExperimentNameMask.find("%s") == string::npos) {
	    this->warn("The experiment name mask does not include the '\%d' or '\%s' escape sequence.");
	    this->warn("It is currently: '" + strExperimentNameMask + "'");
	    this->warn("This will cause your experiments to be overwritten. Be careful.");
	  }
	}
	
	// Section: Plugins
	bool bLoadDevelopmentPlugins = false;
	vector<string> vecPluginOutputColors;
	bool bSearchPathsSet = false;
	
	if(cfgConfig.exists("plugins")) {
	  Setting &sPlugins = cfgConfig.lookup("plugins");
	  sPlugins.lookupValue("load-development-plugins", bLoadDevelopmentPlugins);
	  
	  if(cfgConfig.exists("plugins.load")) {
	    Setting &sPluginsLoad = cfgConfig.lookup("plugins.load");
	    m_lstPluginsToLoad.clear();
	    for(int nI = 0; nI < sPluginsLoad.getLength(); nI++) {
	      string strLoad = sPluginsLoad[nI];
	      
	      m_lstPluginsToLoad.remove(strLoad);
	      m_lstPluginsToLoad.push_back(strLoad);
	    }
	  }
	  
	  if(cfgConfig.exists("plugins.search-paths")) {
	    Setting &sPluginsPaths = cfgConfig.lookup("plugins.search-paths");
	    for(int nI = 0; nI < sPluginsPaths.getLength(); nI++) {
	      string strPath = sPluginsPaths[nI];
	      
	      strPath = this->resolveDirectoryTokens(strPath);
	      m_psPlugins->addPluginSearchPath(strPath);
	      
	      bSearchPathsSet = true;
	    }
	  }
	  
	  if(cfgConfig.exists("plugins.colors")) {
	    Setting &sPluginsColors = cfgConfig.lookup("plugins.colors");
	    for(int nI = 0; nI < sPluginsColors.getLength(); nI++) {
	      string strColor = sPluginsColors[nI];
	      vecPluginOutputColors.push_back(strColor);
	    }
	  }
	  
	  if(cfgConfig.exists("plugins.individual-configurations")) {
	    Setting &sPluginsIndividualConfigurations = cfgConfig.lookup("plugins.individual-configurations");
	    
	    for(int nI = 0; nI < sPluginsIndividualConfigurations.getLength(); nI++) {
	      string strPluginName;
	      
	      if(sPluginsIndividualConfigurations[nI].lookupValue("plugin", strPluginName)) {
		this->info("Loading per-plugin configuration for plugin '" + strPluginName + "'");
		CDesignator* cdConfig = getPluginConfig(strPluginName);
		
		if(this->loadIndividualPluginConfigurationBranch(sPluginsIndividualConfigurations[nI], cdConfig, "", true) == false) {
		  this->warn("Failed to load configuration for plugin '" + strPluginName + "'.");
		}
	      } else {
		this->warn("No 'plugin'-field specified for individual plugin configuration.");
	      }
	    }
	  }
	}
	
	// Check if any search paths were set
	if(bSearchPathsSet == false) {
	  this->warn("You didn't specify any search paths. This will prevent the system");
	  this->warn("from finding any plugins. A default will be assumed.");
	  string strSP = this->resolveDirectoryTokens("$WORKSPACE/lib/");
	  this->warn("Defaulting to: '" + strSP + "'");
	  
	  m_psPlugins->addPluginSearchPath(strSP);
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
	  
	  string strColors = "";
	  for(vector<string>::iterator itC = vecPluginOutputColors.begin();
	      itC != vecPluginOutputColors.end();
	      itC++) {
	    string strC = *itC;
	    
	    strColors += (strColors != "" ? ", " : "") + string("\033[0;") + strC + "m" + strC + "\033[0m";
	  }
	  
	  this->warn("Defaulting to: " + strColors);
	}
	
	// -> Set the global settings
	ConfigSettings cfgsetCurrent = configSettings();
	cfgsetCurrent.bLoadDevelopmentPlugins = bLoadDevelopmentPlugins;
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
	setConfigSettings(cfgsetCurrent);
	
	return true;
      } catch(ParseException e) {
        stringstream sts;
        sts << e.getLine();
	
	this->fail("Error while parsing config file '" + strConfigFile + "': " + e.getError() + ", on line " + sts.str());
      } catch(...) {
	this->fail("Undefined error while parsing config file '" + strConfigFile + "'");
      }
    }
    
    return false;
  }
  
  bool Beliefstate::loadIndividualPluginConfigurationBranch(Setting &sBranch, CKeyValuePair* ckvpInto, string strConfigPath, bool bIgnorePluginField) {
    for(int nJ = 0; nJ < sBranch.getLength(); nJ++) {
      string strConfigDetailName = sBranch[nJ].getName();
      
      if(strConfigDetailName != "plugin" || !bIgnorePluginField) {
	this->info(" - " + strConfigPath + (strConfigPath == "" ? "" : "/") + strConfigDetailName);
	
	switch(sBranch[strConfigDetailName].getType()) {
	case libconfig::Setting::TypeString: {
	  string strContent;
	  sBranch.lookupValue(strConfigDetailName, strContent);
	  strContent = this->resolveDirectoryTokens(strContent);
	  ckvpInto->setValue(strConfigDetailName, strContent);
	} break;
		      
	case libconfig::Setting::TypeInt:
	case libconfig::Setting::TypeInt64: {
	  int nContent;
	  sBranch.lookupValue(strConfigDetailName, nContent);
	  ckvpInto->setValue(strConfigDetailName, nContent);
	} break;
		      
	case libconfig::Setting::TypeFloat: {
	  int fContent;
	  sBranch.lookupValue(strConfigDetailName, fContent);
	  ckvpInto->setValue(strConfigDetailName, fContent);
	} break;
		      
	case libconfig::Setting::TypeBoolean: {
	  bool bContent;
	  sBranch.lookupValue(strConfigDetailName, bContent);
	  ckvpInto->setValue(strConfigDetailName, bContent);
	} break;
	  
	case libconfig::Setting::TypeGroup: {
	  CKeyValuePair* ckvpChild = ckvpInto->addChild(strConfigDetailName);
	  Setting& sBranchChild = sBranch[strConfigDetailName];
	  
	  if(this->loadIndividualPluginConfigurationBranch(sBranchChild, ckvpChild, strConfigPath + (strConfigPath == "" ? "" : "/") + strConfigDetailName) == false) {
	    return false;
	  }
	} break;
	}
      }
    }
    
    return true;
  }
  
  bool Beliefstate::spreadEvent(Event evEvent) {
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
  
  void Beliefstate::spreadServiceEvent(ServiceEvent seServiceEvent) {
    if(m_psPlugins->spreadServiceEvent(seServiceEvent) == 0) {
      ConfigSettings cfgSet = configSettings();
      if(cfgSet.bDisplayUnhandledServiceEvents) {
	this->warn("Unhandled service event ('" + seServiceEvent.strServiceName + "') dropped.");
	
	if(seServiceEvent.cdDesignator) {
	  //this->warn("Content was:");
	  //seServiceEvent.cdDesignator->printDesignator();
	}
      }
    }
  }
  
  bool Beliefstate::cycle() {
    bool bContinue = true;
    
    if(m_bRun) {
      Result resCycle = m_psPlugins->cycle();
      
      // Forward all status messages collected from the plugins and
      // the core into the event system
      list<StatusMessage> lstCoreMessages = queuedMessages();
      for(list<StatusMessage>::iterator itSM = lstCoreMessages.begin();
	  itSM != lstCoreMessages.end();
	  itSM++) {
	resCycle.lstStatusMessages.push_back(*itSM);
      }
	
      for(list<StatusMessage>::iterator itSM = resCycle.lstStatusMessages.begin();
	  itSM != resCycle.lstStatusMessages.end();
	  itSM++) {
	Event evEvent = defaultEvent("status-message");
	evEvent.msgStatusMessage = *itSM;
	evEvent.bPreempt = false;
	
	if(this->spreadEvent(evEvent)) {
	  this->setRedirectOutput(true);
	}
      }
      
      for(list<Event>::iterator itEv = m_lstGlobalEvents.begin();
	  itEv != m_lstGlobalEvents.end();
	  itEv++) {
	resCycle.lstEvents.push_back(*itEv);
      }
      m_lstGlobalEvents.clear();
      
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
      list<StatusMessage> lstCoreMessages = queuedMessages();
      for(list<StatusMessage>::iterator itSM = lstCoreMessages.begin();
	  itSM != lstCoreMessages.end();
	  itSM++) {
	Event evMessage = defaultEvent("status-message");
	evMessage.msgStatusMessage = *itSM;
	
	this->handleUnhandledEvent(evMessage);
      }
    }
    
    return bContinue;
  }
  
  void Beliefstate::triggerShutdown() {
    m_bRun = false;
  }
  
  void Beliefstate::triggerTerminalResize() {
    m_mtxTerminalResize.lock();
    m_bTerminalWindowResize = true;
    m_mtxTerminalResize.unlock();
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
  
  string Beliefstate::workspaceDirectory() {
    return m_strWorkspaceDirectory;
  }
  
  string Beliefstate::homeDirectory() {
    string strHome = "";
    
    char* cHome = getenv("HOME");
    if(cHome) {
      strHome = cHome;
    }
    
    return strHome;
  }
  
  string Beliefstate::resolveDirectoryTokens(string strSubject) {
    // First, make list of things to replace
    list< pair<string, bool> > lstTokens;
  
    size_t pos = 0;
    while((pos = strSubject.find("$", pos)) != string::npos) {
      size_t offset = 1;
    
      if(pos < strSubject.size() - 1) {
	if(strSubject.at(pos + 1) != ' ') {
	  string strToken = "";
	  bool bInBrackets = false;
	
	  if(strSubject.at(pos + 1) == '{') {
	    offset++;
	    size_t pos_endbracket = strSubject.find("}", pos + 1);
	  
	    if(pos_endbracket != string::npos) {
	      strToken = strSubject.substr(pos + 2, pos_endbracket - (pos + 2));
	      offset += 1 + pos_endbracket - (pos + 2);
	      bInBrackets = true;
	    } else {
	      // This token is invalid (no ending bracket).
	    }
	  } else {
	    size_t pos_space_or_end = strSubject.find_first_of(" /.~_-\\$", pos + 1);
	    if(pos_space_or_end == string::npos) {
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
	    lstTokens.push_back(make_pair(strToken, bInBrackets));
	  }
	}
      }
    
      pos += offset;
    }
    
    for(list< pair<string, bool> >::iterator itToken = lstTokens.begin();
	  itToken != lstTokens.end();
	  itToken++) {
      pair<string, bool> prToken = *itToken;
      string strToken = prToken.first;
      bool bInBrackets = prToken.second;
      string strToReplace = (bInBrackets ? "${" + strToken + "}" : "$" + strToken);
      string strReplacement = this->findTokenReplacement(strToken);
      
      if(strReplacement == "") {
	this->warn("Failed to resolve directory token '" + strToken + "', asserting value \"\".");
      }
      
      this->replaceStringInPlace(strSubject, strToReplace, strReplacement);
    }
    
    return strSubject;
  }
  
  string Beliefstate::findTokenReplacement(string strToken) {
    string strReplacement = "";
    
    if(strToken == "HOME") {
      strReplacement = this->homeDirectory();
    }
    
    return strReplacement;
  }
  
  bool Beliefstate::handleUnhandledEvent(Event evEvent) {
    if(evEvent.strEventName == "status-message") {
      StatusMessage msgStatus = evEvent.msgStatusMessage;
      this->setRedirectOutput(false);
      
      return true;
    }
    
    return false;
  }
}
