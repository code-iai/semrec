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


#ifndef __PLUGIN_H__
#define __PLUGIN_H__


// System
#include <cstdlib>
#include <mutex>
#include <list>
#include <string>
#include <sstream>

// Private
#include <Types.h>
#include <ForwardDeclarations.h>
#include <ArbitraryMappingsHolder.h>


using namespace designator_integration;


namespace semrec {
  namespace plugins {
    class Plugin : public ArbitraryMappingsHolder {
    private:
      std::list<std::string> m_lstDependencies;
      std::string m_strName;
      std::string m_strVersion;
      int m_nID;
      bool m_bRunCycle;
      std::mutex m_mtxRunCycle;
      bool m_bDevelopmentPlugin;
      
    protected:
      std::list<Event> m_lstEvents;
      std::mutex m_mtxEventsStore;
      std::list<std::string> m_lstSubscribedEventNames;
      std::list<ServiceEvent> m_lstServiceEvents;
      std::mutex m_mtxServiceEventsStore;
      std::list<std::string> m_lstOfferedServices;
      std::list<int> m_lstOpenRequestIDs;
      std::list<ServiceEvent> m_lstReceivedServiceEventResponses;
      std::mutex m_mtxReceivedServiceEventResponses;
      
    public:
      Plugin();
      virtual ~Plugin();
      
      void setPluginID(int nID);
      int pluginID();
      
      void setPluginName(std::string strName);
      std::string pluginName();
      
      virtual Result init(int argc, char** argv);
      virtual Result deinit();
      
      virtual Result cycle();
      Result cycleResults();
      
      Designator* getIndividualConfig();
      
      void setDevelopmentPlugin(bool bDevelopmentPlugin);
      bool developmentPlugin();
      void setPluginVersion(std::string strVersion);
      std::string pluginVersion();
      
      void setSubscribedToEvent(std::string strEventName, bool bSubscribed);
      bool subscribedToEvent(std::string strEventName);
      virtual void consumeEvent(Event evEvent);
      
      void setOffersService(std::string strServiceName, bool bOffering);
      bool offersService(std::string strServiceName);
      virtual Event consumeServiceEvent(ServiceEvent seServiceEvent);
      
      void addDependency(std::string strPluginName);
      bool dependsOn(std::string strPluginName);
      std::list<std::string> dependencies();
      
      void deployCycleData(Result& resDeployTo);
      
      void deployEvent(Event evDeploy, bool bWaitForEvent = false);
      ServiceEvent deployServiceEvent(ServiceEvent seDeploy, bool bWaitForEvent = false);
      
      std::string pluginIdentifierString(bool bBold);
      
      int openNewRequestID();
      bool isRequestIDOpen(int nID);
      void closeRequestID(int nID);
      bool isAnyRequestIDOpen();
      
      void setRunning(bool bRunCycle);
      bool running();
      
      void waitForEvent(Event evWait);
      ServiceEvent waitForEvent(ServiceEvent seWait);
      
      void success(std::string strMessage, bool bImportant = false);
      void info(std::string strMessage, bool bImportant = false);
      void warn(std::string strMessage, bool bImportant = false);
      void fail(std::string strMessage, bool bImportant = false);
    };
  }
}


#endif /* __PLUGIN_H__ */
