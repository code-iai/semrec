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


#include <plugins/interactive/InteractiveObject.h>


namespace semrec {
  InteractiveObject::InteractiveObject(std::string strName) {
    m_imsServer = NULL;
    
    m_imMarker.header.frame_id = "/map";
    m_imMarker.scale = 1;
    m_imMarker.name = strName;
    
    m_mkrMarker.type = visualization_msgs::Marker::CUBE;
    m_mkrMarker.scale.x = m_imMarker.scale * 0.45;
    m_mkrMarker.scale.y = m_imMarker.scale * 0.45;
    m_mkrMarker.scale.z = m_imMarker.scale * 0.45;
    m_mkrMarker.color.r = 0.5;
    m_mkrMarker.color.g = 0.5;
    m_mkrMarker.color.b = 0.5;
    m_mkrMarker.color.a = 1.0;
  }
  
  InteractiveObject::~InteractiveObject() {
  }
  
  void InteractiveObject::setSize(float fWidth, float fDepth, float fHeight) {
    m_mkrMarker.scale.x = m_imMarker.scale * fWidth;
    m_mkrMarker.scale.y = m_imMarker.scale * fDepth;
    m_mkrMarker.scale.z = m_imMarker.scale * fHeight;
    
    this->insertIntoServer(m_imsServer);
  }
  
  void InteractiveObject::setPose(std::string strFixedFrame, geometry_msgs::Pose posPose) {
    m_imMarker.header.frame_id = strFixedFrame;
    m_imMarker.pose = posPose;
    
    this->insertIntoServer(m_imsServer);
  }
  
  void InteractiveObject::setPose(geometry_msgs::Pose posPose) {
    this->setPose(m_imMarker.header.frame_id, posPose);
  }
  
  void InteractiveObject::setMarker(visualization_msgs::Marker mkrMarker) {
    m_mkrMarker = mkrMarker;
    
    this->insertIntoServer(m_imsServer);
  }
  
  void InteractiveObject::clickCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
    interactive_markers::MenuHandler::EntryHandle entHandle = feedback->menu_entry_id;
    
    bool bFound = false;
    InteractiveMenuEntry imeEntry;
    
    for(InteractiveMenuEntry imeEntry : m_lstMenuEntries) {
      if(imeEntry.unMenuEntryID == entHandle) {
	imeEntry = imeEntry;
	bFound = true;
	
	break;
      }
    }
    
    if(bFound) {
      // Send out the symbolic event containing the callback result.
      InteractiveObjectCallbackResult iocrResult;
      iocrResult.strObject = feedback->marker_name;
      iocrResult.strCommand = imeEntry.strIdentifier;
      iocrResult.strParameter = imeEntry.strParameter;
      
      m_mtxCallbackResults.lock();
      m_lstCallbackResults.push_back(iocrResult);
      m_mtxCallbackResults.unlock();
    }
  }
  
  bool InteractiveObject::insertIntoServer(interactive_markers::InteractiveMarkerServer* imsServer) {
    if(ros::ok()) {
      if(imsServer) {
	m_imcControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
	m_imcControl.always_visible = true;
      
	m_imcControl.markers.clear();
	m_imcControl.markers.push_back(m_mkrMarker);
      
	m_imMarker.controls.clear();
	m_imMarker.controls.push_back(m_imcControl);
      
	imsServer->insert(m_imMarker);
	m_mhMenu.apply(*imsServer, m_imMarker.name);
	imsServer->applyChanges();
      
	m_imsServer = imsServer;
      
	return true;
      }
    }
    
    return false;
  }
  
  bool InteractiveObject::removeFromServer() {
    if(ros::ok()) {
      if(m_imsServer) {
	m_imsServer->erase(m_imMarker.name);
	m_imsServer->applyChanges();
	
	return true;
      }
    }
    
    return false;
  }
  
  std::string InteractiveObject::name() {
    return m_imMarker.name;
  }
  
  void InteractiveObject::addMenuEntry(std::string strLabel, std::string strIdentifier, std::string strParameter) {
    InteractiveMenuEntry imeEntry;
    imeEntry.strLabel = strLabel;
    imeEntry.strIdentifier = strIdentifier;
    imeEntry.strParameter = strParameter;
    
    interactive_markers::MenuHandler::EntryHandle entEntry = m_mhMenu.insert(strLabel, boost::bind(&InteractiveObject::clickCallback, this, _1));
    m_mhMenu.setCheckState(entEntry, interactive_markers::MenuHandler::NO_CHECKBOX);
    
    imeEntry.unMenuEntryID = entEntry;
    
    m_lstMenuEntries.push_back(imeEntry);
    
    if(m_imsServer) {
      this->insertIntoServer(m_imsServer);
    }
  }
  
  void InteractiveObject::clearMenuEntries() {
    interactive_markers::MenuHandler mhMenu;
    
    m_lstMenuEntries.clear();
    m_mhMenu = mhMenu;
    
    if(m_imsServer) {
      this->insertIntoServer(m_imsServer);
    }
  }
  
  void InteractiveObject::removeMenuEntry(std::string strIdentifier, std::string strParameter) {
    interactive_markers::MenuHandler mhMenu;
    
    for(std::list<InteractiveMenuEntry>::iterator itIME = m_lstMenuEntries.begin();
	itIME != m_lstMenuEntries.end();
	itIME++) {
      if((*itIME).strIdentifier == strIdentifier && (*itIME).strParameter == strParameter) {
	m_lstMenuEntries.erase(itIME);
	
	break;
      }
    }
    
    for(InteractiveMenuEntry imeEntry : m_lstMenuEntries) {
      interactive_markers::MenuHandler::EntryHandle entEntry = m_mhMenu.insert(imeEntry.strLabel, boost::bind(&InteractiveObject::clickCallback, this, _1));
      m_mhMenu.setCheckState(entEntry, interactive_markers::MenuHandler::NO_CHECKBOX);
    }
    
    m_mhMenu = mhMenu;
    
    if(m_imsServer) {
      this->insertIntoServer(m_imsServer);
    }
  }
  
  std::list<InteractiveObjectCallbackResult> InteractiveObject::callbackResults() {
    std::list<InteractiveObjectCallbackResult> lstResults;
    
    m_mtxCallbackResults.lock();
    lstResults = m_lstCallbackResults;
    m_lstCallbackResults.clear();
    m_mtxCallbackResults.unlock();
    
    return lstResults;
  }
}
