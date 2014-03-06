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


#ifndef __INTERACTIVE_OBJECT_H__
#define __INTERACTIVE_OBJECT_H__


// System
#include <iostream>
#include <mutex>

// ROS
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <ros/ros.h>

using namespace std;
using namespace visualization_msgs;
using namespace interactive_markers;


namespace beliefstate {
  typedef struct {
    string strLabel;
    string strIdentifier;
    string strParameter;
    uint8_t unMenuEntryID;
  } InteractiveMenuEntry;
  
  typedef struct {
    string strObject;
    string strCommand;
    string strParameter;
  } InteractiveObjectCallbackResult;
  
  class InteractiveObject {
  private:
    Marker m_mkrMarker;
    MenuHandler m_mhMenu;
    InteractiveMarker m_imMarker;
    InteractiveMarkerServer* m_imsServer;
    InteractiveMarkerControl m_imcControl;
    list<InteractiveMenuEntry> m_lstMenuEntries;
    list<InteractiveObjectCallbackResult> m_lstCallbackResults;
    mutex m_mtxCallbackResults;
    
  public:
    InteractiveObject(string strName);
    ~InteractiveObject();
    
    bool insertIntoServer(InteractiveMarkerServer* imsServer);
    bool removeFromServer();
    void clickCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    
    void setMarker(visualization_msgs::Marker mkrMarker);
    void setSize(float fWidth, float fDepth, float fHeight);
    void setPose(string strFixedFrame, geometry_msgs::Pose posPose);
    void setPose(geometry_msgs::Pose posPose);
    
    string name();
    void addMenuEntry(string strLabel, string strIdentifier, string strParameter = "");
    void removeMenuEntry(string strIdentifier, string strParameter = "");
    void clearMenuEntries();
    
    list<InteractiveObjectCallbackResult> callbackResults();
  };
}


#endif /* __INTERACTIVE_OBJECT_H__ */
