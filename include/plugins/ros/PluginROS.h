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


#ifndef __PLUGIN_ROS_H__
#define __PLUGIN_ROS_H__


#define PLUGIN_CLASS PluginROS


// System
#include <cstdlib>
#include <iostream>
#include <algorithm>

// Boost
#include <boost/thread.hpp>

// ROS
#include <ros/ros.h>
#include <ros/rate.h>
#include <rosgraph_msgs/Log.h>
#include <ros/callback_queue.h>

// Designators
#include <designators/Designator.h>
#include <designator_integration_msgs/DesignatorCommunication.h>

// Private
#include <Types.h>
#include <ForwardDeclarations.h>
#include <Plugin.h>


namespace semrec {
  namespace plugins {
    class PLUGIN_CLASS : public Plugin {
    private:
      ros::NodeHandle* m_nhHandle;
      ros::ServiceServer m_srvCallback;
      ros::Publisher m_pubLoggedDesignators;
      ros::Publisher m_pubInteractiveCallback;
      ros::Publisher m_pubStatusMessages;
      bool m_bStartedSpinning;
      ros::AsyncSpinner* m_aspnAsyncSpinner;
      bool m_bRoslogMessages;
      std::mutex m_mtxGlobalInputLock;
      bool m_bFirstContextReceived;
      std::mutex m_mtxSpinWorker;
      bool m_bKeepSpinning;
      boost::thread* m_thrdSpinWorker;
      bool m_bSpinWorkerRunning;
      std::mutex m_mtxSpinWorkerRunning;
      
    public:
      PLUGIN_CLASS();
      ~PLUGIN_CLASS();
      
      virtual Result init(int argc, char** argv);
      virtual Result deinit();
      
      virtual Result cycle();

      void setKeepSpinning(bool bKeepSpinning);
      bool keepSpinning();
      void spinWorker();
      void setSpinWorkerRunning(bool bSpinWorkerRunning);
      bool spinWorkerRunning();
      void shutdownSpinWorker();
      
      bool serviceCallback(designator_integration_msgs::DesignatorCommunication::Request &req, designator_integration_msgs::DesignatorCommunication::Response &res);
      
      virtual void consumeEvent(Event evEvent);
      virtual Event consumeServiceEvent(ServiceEvent seServiceEvent);
      
      std::string getDesignatorTypeString(Designator* desigDesignator);
      
      void waitForAssuranceToken(std::string strToken);
    };
  }
  
  extern "C" plugins::PLUGIN_CLASS* createInstance();
  extern "C" void destroyInstance(plugins::PLUGIN_CLASS* icDestroy);
}


#endif /* __PLUGIN_ROS_H__ */
