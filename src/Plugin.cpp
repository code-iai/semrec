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


#include <Plugin.h>


namespace semrec {
  namespace plugins {
    Plugin::Plugin() {
      this->setPluginID(createPluginID());
      m_bRunCycle = true;
      m_bDevelopmentPlugin = false;
      m_strVersion = "";
      
      ConfigSettings cfgsetCurrent = configSettings();
      if(cfgsetCurrent.bOnlyDisplayImportant) {
	this->setOnlyDisplayImportant(true);
      }
    }
    
    Plugin::~Plugin() {
      freePluginID(m_nID);
    }
    
    Designator* Plugin::getIndividualConfig() {
      return getPluginConfig(this->pluginName());
    }
    
    void Plugin::setDevelopmentPlugin(bool bDevelopmentPlugin) {
      m_bDevelopmentPlugin = bDevelopmentPlugin;
    }
    
    bool Plugin::developmentPlugin() {
      return m_bDevelopmentPlugin;
    }
    
    void Plugin::setPluginID(int nID) {
      m_nID = nID;
      
      this->setMessagePrefixLabel(this->pluginName() + "/" + this->str(this->pluginID()));
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
    
    void Plugin::setSubscribedToEvent(std::string strEventName, bool bSubscribed) {
      m_lstSubscribedEventNames.remove(strEventName);
      
      if(bSubscribed) {
	m_lstSubscribedEventNames.push_back(strEventName);
      }
    }
    
    bool Plugin::subscribedToEvent(std::string strEventName) {
      for(std::string strCurrentName : m_lstSubscribedEventNames) {
	if(strCurrentName == strEventName) {
	  return true;
	}
      }
      
      return false;
    }
    
    void Plugin::consumeEvent(Event evEvent) {
      // Dummy.
    }
    
    void Plugin::setOffersService(std::string strServiceName, bool bOffering) {
      m_lstOfferedServices.remove(strServiceName);
      
      if(bOffering) {
	m_lstOfferedServices.push_back(strServiceName);
      }
    }
    
    bool Plugin::offersService(std::string strServiceName) {
      for(std::string strCurrentName : m_lstOfferedServices) {
	if(strCurrentName == strServiceName) {
	  return true;
	}
      }
      
      return false;
    }
    
    Event Plugin::consumeServiceEvent(ServiceEvent seServiceEvent) {
      Event evReturn = defaultEvent();
      
      if(seServiceEvent.siServiceIdentifier == SI_RESPONSE) {
	m_mtxReceivedServiceEventResponses.lock();
	m_lstReceivedServiceEventResponses.push_back(seServiceEvent);
	m_mtxReceivedServiceEventResponses.unlock();
      }
      
      return evReturn;
    }
    
    void Plugin::addDependency(std::string strPluginName) {
      m_lstDependencies.remove(strPluginName);
      m_lstDependencies.push_back(strPluginName);
    }
    
    bool Plugin::dependsOn(std::string strPluginName) {
      for(std::string strDepName : m_lstDependencies) {
	if(strDepName == strPluginName) {
	  return true;
	}
      }
      
      return false;
    }
    
    std::list<std::string> Plugin::dependencies() {
      return m_lstDependencies;
    }
    
    void Plugin::deployCycleData(Result& resDeployTo) {
      m_mtxEventsStore.lock();

      if(m_lstEvents.size() > 0) {
	resDeployTo.lstEvents = m_lstEvents;
	m_lstEvents.clear();
      }
      
      m_mtxEventsStore.unlock();
      
      m_mtxServiceEventsStore.lock();
      
      if(m_lstServiceEvents.size() > 0) {
	resDeployTo.lstServiceEvents = m_lstServiceEvents;
	m_lstServiceEvents.clear();
      }
      
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
    
    ServiceEvent Plugin::deployServiceEvent(ServiceEvent seDeploy, bool bWaitForEvent) {
      seDeploy.nRequesterID = this->pluginID();
      
      if(seDeploy.siServiceIdentifier == SI_REQUEST) {
	seDeploy.nServiceEventID = rand();
      }
      
      m_mtxServiceEventsStore.lock();
      m_lstServiceEvents.push_back(seDeploy);
      m_mtxServiceEventsStore.unlock();
      
      if(bWaitForEvent) {
	return this->waitForEvent(seDeploy);
      } else {
	return seDeploy;
      }
    }
    
    void Plugin::setPluginName(std::string strName) {
      m_strName = strName;
      
      this->setPluginID(m_nID);
    }
    
    std::string Plugin::pluginName() {
      return m_strName;
    }
    
    void Plugin::setPluginVersion(std::string strVersion) {
      m_strVersion = strVersion;
    }
    
    std::string Plugin::pluginVersion() {
      return m_strVersion;
    }
    
    std::string Plugin::pluginIdentifierString(bool bBold) {
      return colorSpecifierForID(this->pluginID(), bBold) +
	"[" + this->pluginName() + "/" + this->str(this->pluginID()) + "]";
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
      for(int nCurrentID : m_lstOpenRequestIDs) {
	if(nCurrentID == nID) {
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
    
    ServiceEvent Plugin::waitForEvent(ServiceEvent seWait) {
      bool bGoon = true;
      ServiceEvent seReturn = defaultServiceEvent(seWait.strServiceName);
      
      while(bGoon && this->running()) {
	m_mtxReceivedServiceEventResponses.lock();
	
	for(std::list<ServiceEvent>::iterator itSE = m_lstReceivedServiceEventResponses.begin();
	    itSE != m_lstReceivedServiceEventResponses.end(); itSE++) {
	  if(((*itSE).nServiceEventID == seWait.nServiceEventID) && (*itSE).siServiceIdentifier == SI_RESPONSE) {
	    bGoon = false;
	    seReturn = *itSE;
	    
	    m_lstReceivedServiceEventResponses.erase(itSE);
	    
	    break;
	  }
	}
	
	m_mtxReceivedServiceEventResponses.unlock();
      }
      
      return seReturn;
    }
    
    void Plugin::success(std::string strMessage, bool bImportant) {
      this->coloredText(strMessage, colorSpecifierForID(this->pluginID()), false, bImportant);
    }
    
    void Plugin::info(std::string strMessage, bool bImportant) {
      this->coloredText(strMessage, colorSpecifierForID(this->pluginID()), false, bImportant);
    }
    
    void Plugin::warn(std::string strMessage, bool bImportant) {
      this->coloredText(strMessage, colorSpecifierForID(this->pluginID()), true, bImportant);
    }
    
    void Plugin::fail(std::string strMessage, bool bImportant) {
      this->coloredText(strMessage, colorSpecifierForID(this->pluginID()), true, bImportant);
    }
  }
}
