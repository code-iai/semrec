#include <plugins/interactive/InteractiveObject.h>


namespace beliefstate {
  InteractiveObject::InteractiveObject() {
    // NOTE(winkler): This is just a test entry.
    MenuHandler::EntryHandle entEntry = m_mhMenu.insert("Entry", boost::bind(&InteractiveObject::clickCallback, this, _1));
    m_mhMenu.setCheckState(entEntry, MenuHandler::NO_CHECKBOX);
    
    m_imMarker.header.frame_id = "/map";
    m_imMarker.pose.position.z = 0.5;
    m_imMarker.scale = 1;
    m_imMarker.name = "marker_name";
    
    m_mkrMarker.type = Marker::CUBE;
    m_mkrMarker.scale.x = m_imMarker.scale * 0.45;
    m_mkrMarker.scale.y = m_imMarker.scale * 0.45;
    m_mkrMarker.scale.z = m_imMarker.scale * 0.45;
    m_mkrMarker.color.r = 0.5;
    m_mkrMarker.color.g = 0.5;
    m_mkrMarker.color.b = 0.5;
    m_mkrMarker.color.a = 1.0;
    
    m_imcControl.interaction_mode = InteractiveMarkerControl::BUTTON;
    m_imcControl.always_visible = true;
    m_imcControl.markers.push_back(m_mkrMarker);
    m_imMarker.controls.push_back(m_imcControl);
  }
  
  InteractiveObject::~InteractiveObject() {
  }
  
  void InteractiveObject::clickCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
    cout << "Got feedback." << endl;
  }
  
  void InteractiveObject::insertIntoServer(InteractiveMarkerServer* imsServer) {
    imsServer->insert(m_imMarker);
    m_mhMenu.apply(*imsServer, m_imMarker.name);
    imsServer->applyChanges();
  }
}
