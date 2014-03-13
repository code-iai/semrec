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
#include <UtilityBase.h>

using namespace std;


namespace beliefstate {
  namespace plugins {
    class Plugin : public UtilityBase {
    private:
      list<string> m_lstDependencies;
      string m_strName;
      string m_strVersion;
      int m_nID;
      bool m_bRunCycle;
      mutex m_mtxRunCycle;
      bool m_bDevelopmentPlugin;
      
    protected:
      list<Event> m_lstEvents;
      mutex m_mtxEventsStore;
      list<string> m_lstSubscribedEventNames;
      list<ServiceEvent> m_lstServiceEvents;
      mutex m_mtxServiceEventsStore;
      list<string> m_lstOfferedServices;
      list<int> m_lstOpenRequestIDs;
      
    public:
      Plugin();
      virtual ~Plugin();
      
      void setPluginID(int nID);
      int pluginID();
      
      void setPluginName(string strName);
      string pluginName();
      
      virtual Result init(int argc, char** argv);
      virtual Result deinit();
      
      virtual Result cycle();
      Result cycleResults();
      
      CDesignator* getIndividualConfig();
      
      void setDevelopmentPlugin(bool bDevelopmentPlugin);
      bool developmentPlugin();
      void setPluginVersion(string strVersion);
      string pluginVersion();
      
      void setSubscribedToEvent(string strEventName, bool bSubscribed);
      bool subscribedToEvent(string strEventName);
      virtual void consumeEvent(Event evEvent);
      
      void setOffersService(string strServiceName, bool bOffering);
      bool offersService(string strServiceName);
      virtual Event consumeServiceEvent(ServiceEvent seServiceEvent);
      
      void addDependency(string strPluginName);
      bool dependsOn(string strPluginName);
      list<string> dependencies();
      
      void deployCycleData(Result& resDeployTo);
      
      void deployEvent(Event evDeploy, bool bWaitForEvent = false);
      void deployServiceEvent(ServiceEvent seDeploy);
      
      string pluginIdentifierString(bool bBold);
      /* void warn(string strMessage); */
      /* void info(string strMessage); */
      void unimplemented(string strMessage);
      
      int getTimeStamp();
      
      int openNewRequestID();
      bool isRequestIDOpen(int nID);
      void closeRequestID(int nID);
      bool isAnyRequestIDOpen();
      
      void setRunning(bool bRunCycle);
      bool running();
      
      void waitForEvent(Event evWait);
    };
  }
}


#endif /* __PLUGIN_H__ */
