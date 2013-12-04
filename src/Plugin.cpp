#include <Plugin.h>


namespace beliefstate {
  namespace plugins {
    Plugin::Plugin() {
      m_nID = createPluginID();
      m_bRunCycle = true;
    }
    
    Plugin::~Plugin() {
      freePluginID(m_nID);
    }
    
    void Plugin::setPluginID(int nID) {
      m_nID = nID;
    }
    
    int Plugin::pluginID() {
      return m_nID;
    }
    
    Result Plugin::init(int argc, char** argv) {
      // Dummy.
      return defaultResult();
    }
    
    Result Plugin::deinit() {
      // Dummy.
      return defaultResult();
    }
    
    Result Plugin::cycle() {
      // Dummy.
      return defaultResult();
    }
    
    void Plugin::setSubscribedToEvent(string strEventName, bool bSubscribed) {
      m_lstSubscribedEventNames.remove(strEventName);
      
      if(bSubscribed) {
	m_lstSubscribedEventNames.push_back(strEventName);
      }
    }
    
    bool Plugin::subscribedToEvent(string strEventName) {
      for(list<string>::iterator itEI = m_lstSubscribedEventNames.begin();
	  itEI != m_lstSubscribedEventNames.end();
	  itEI++) {
	if(*itEI == strEventName) {
	  return true;
	}
      }
      
      return false;
    }
    
    void Plugin::consumeEvent(Event evEvent) {
      // Dummy.
    }
    
    void Plugin::setOffersService(string strServiceName, bool bOffering) {
      m_lstOfferedServices.remove(strServiceName);
      
      if(bOffering) {
	m_lstOfferedServices.push_back(strServiceName);
      }
    }
    
    bool Plugin::offersService(string strServiceName) {
      for(list<string>::iterator itService = m_lstOfferedServices.begin();
	  itService != m_lstOfferedServices.end();
	  itService++) {
	if(*itService == strServiceName) {
	  return true;
	}
      }
      
      return false;
    }
    
    Event Plugin::consumeServiceEvent(ServiceEvent seServiceEvent) {
      Event evReturn = defaultEvent();
      // Dummy.
      return evReturn;
    }
    
    void Plugin::addDependency(string strPluginName) {
      m_lstDependencies.remove(strPluginName);
      m_lstDependencies.push_back(strPluginName);
    }
    
    bool Plugin::dependsOn(string strPluginName) {
      for(list<string>::iterator itDep = m_lstDependencies.begin();
	  itDep != m_lstDependencies.end();
	  itDep++) {
	if(*itDep == strPluginName) {
	  return true;
	}
      }
      
      return false;
    }
    
    list<string> Plugin::dependencies() {
      return m_lstDependencies;
    }
    
    void Plugin::deployCycleData(Result& resDeployTo) {
      m_mtxEventsStore.lock();
      resDeployTo.lstEvents = m_lstEvents;
      m_lstEvents.clear();
      m_mtxEventsStore.unlock();
      
      m_mtxServiceEventsStore.lock();
      resDeployTo.lstServiceEvents = m_lstServiceEvents;
      m_lstServiceEvents.clear();
      m_mtxServiceEventsStore.unlock();
    }
    
    void Plugin::deployEvent(Event evDeploy, bool bWaitForEvent) {
      evDeploy.nOriginID = this->pluginID();
      
      m_mtxEventsStore.lock();
      m_lstEvents.push_back(evDeploy);
      m_mtxEventsStore.unlock();
      
      if(bWaitForEvent) {
	this->waitForEvent(evDeploy);
      }
    }
    
    void Plugin::deployServiceEvent(ServiceEvent seDeploy) {
      seDeploy.nRequesterID = this->pluginID();
      
      m_mtxServiceEventsStore.lock();
      m_lstServiceEvents.push_back(seDeploy);
      m_mtxServiceEventsStore.unlock();
    }
    
    void Plugin::setPluginName(string strName) {
      m_strName = strName;
    }
    
    string Plugin::pluginName() {
      return m_strName;
    }
    
    string Plugin::pluginIdentifierString(bool bBold) {
      stringstream sts;
      sts << colorSpecifierForID(this->pluginID(), bBold);
      sts << "[" << this->pluginName() << "/";
      sts << this->pluginID();
      sts << "]";
      
      return sts.str();
    }
    
    void Plugin::warn(string strMessage) {
      cerr << this->pluginIdentifierString(true) << " " << strMessage << normalColorSpecifier() << endl;
    }
    
    void Plugin::info(string strMessage) {
      cout << this->pluginIdentifierString(false) << " " << strMessage << normalColorSpecifier() << endl;
    }
    
    int Plugin::getTimeStamp() {
      return std::time(0);
    }
    
    
    int Plugin::openNewRequestID() {
      int nID = 0;
      
      while(this->isRequestIDOpen(nID)) {
	nID++;
      }
      
      m_lstOpenRequestIDs.push_back(nID);
      return nID;
    }
    
    bool Plugin::isRequestIDOpen(int nID) {
      for(list<int>::iterator itID = m_lstOpenRequestIDs.begin();
	  itID != m_lstOpenRequestIDs.end();
	  itID++) {
	if(*itID == nID) {
	  return true;
	}
      }
      
      return false;
    }
    
    void Plugin::closeRequestID(int nID) {
      m_lstOpenRequestIDs.remove(nID);
    }
    
    bool Plugin::isAnyRequestIDOpen() {
      return (m_lstOpenRequestIDs.size() > 0);
    }
    
    void Plugin::setRunning(bool bRunCycle) {
      m_mtxRunCycle.lock();
      m_bRunCycle = bRunCycle;
      m_mtxRunCycle.unlock();
    }
    
    bool Plugin::running() {
      m_mtxRunCycle.lock();
      bool bReturn = m_bRunCycle;
      m_mtxRunCycle.unlock();
      
      return bReturn;
    }
    
    void Plugin::waitForEvent(Event evWait) {
      if(evWait.nOpenRequestID != -1) {
	while(this->isRequestIDOpen(evWait.nOpenRequestID) && this->running()) {
	  // Do nothing and wait.
	}
      }
    }
  }
}
