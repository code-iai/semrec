#include <plugins/interactive/InteractiveObject.h>


namespace beliefstate {
  InteractiveObject::InteractiveObject(string strName) {
    m_imsServer = NULL;
    
    m_imMarker.header.frame_id = "/map";
    m_imMarker.scale = 1;
    m_imMarker.name = strName;
    
    m_mkrMarker.type = Marker::CUBE;
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
  
  void InteractiveObject::setPose(string strFixedFrame, geometry_msgs::Pose posPose) {
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
    MenuHandler::EntryHandle entHandle = feedback->menu_entry_id;
    
    bool bFound = false;
    InteractiveMenuEntry imeEntry;
    for(list<InteractiveMenuEntry>::iterator itIME = m_lstMenuEntries.begin();
	itIME != m_lstMenuEntries.end();
	itIME++) {
      if((*itIME).unMenuEntryID == entHandle) {
	imeEntry = *itIME;
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
  
  bool InteractiveObject::insertIntoServer(InteractiveMarkerServer* imsServer) {
    if(ros::ok()) {
      if(imsServer) {
	m_imcControl.interaction_mode = InteractiveMarkerControl::BUTTON;
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
    if(m_imsServer) {
      m_imsServer->erase(m_imMarker.name);
      m_imsServer->applyChanges();
      
      return true;
    }
    
    return false;
  }
  
  string InteractiveObject::name() {
    return m_imMarker.name;
  }
  
  void InteractiveObject::addMenuEntry(string strLabel, string strIdentifier, string strParameter) {
    InteractiveMenuEntry imeEntry;
    imeEntry.strLabel = strLabel;
    imeEntry.strIdentifier = strIdentifier;
    imeEntry.strParameter = strParameter;
    
    MenuHandler::EntryHandle entEntry = m_mhMenu.insert(strLabel, boost::bind(&InteractiveObject::clickCallback, this, _1));
    m_mhMenu.setCheckState(entEntry, MenuHandler::NO_CHECKBOX);
    
    imeEntry.unMenuEntryID = entEntry;
    
    m_lstMenuEntries.push_back(imeEntry);
    
    if(m_imsServer) {
      this->insertIntoServer(m_imsServer);
    }
  }
  
  void InteractiveObject::clearMenuEntries() {
    MenuHandler mhMenu;
    
    m_lstMenuEntries.clear();
    m_mhMenu = mhMenu;
    
    if(m_imsServer) {
      this->insertIntoServer(m_imsServer);
    }
  }
  
  void InteractiveObject::removeMenuEntry(string strIdentifier, string strParameter) {
    MenuHandler mhMenu;
    
    for(list<InteractiveMenuEntry>::iterator itIME = m_lstMenuEntries.begin();
	itIME != m_lstMenuEntries.end();
	itIME++) {
      if((*itIME).strIdentifier == strIdentifier && (*itIME).strParameter == strParameter) {
	m_lstMenuEntries.erase(itIME);
	break;
      }
    }
    
    for(list<InteractiveMenuEntry>::iterator itIME = m_lstMenuEntries.begin();
	itIME != m_lstMenuEntries.end();
	itIME++) {
      MenuHandler::EntryHandle entEntry = m_mhMenu.insert((*itIME).strLabel, boost::bind(&InteractiveObject::clickCallback, this, _1));
      m_mhMenu.setCheckState(entEntry, MenuHandler::NO_CHECKBOX);
    }
    
    m_mhMenu = mhMenu;
    
    if(m_imsServer) {
      this->insertIntoServer(m_imsServer);
    }
  }
  
  list<InteractiveObjectCallbackResult> InteractiveObject::callbackResults() {
    list<InteractiveObjectCallbackResult> lstResults;
    
    m_mtxCallbackResults.lock();
    lstResults = m_lstCallbackResults;
    m_lstCallbackResults.clear();
    m_mtxCallbackResults.unlock();
    
    return lstResults;
  }
}
