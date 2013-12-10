#ifndef __INTERACTIVE_OBJECT_H__
#define __INTERACTIVE_OBJECT_H__


// System
#include <iostream>

// ROS
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

using namespace std;
using namespace visualization_msgs;
using namespace interactive_markers;


namespace beliefstate {
  class InteractiveObject {
  private:
    Marker m_mkrMarker;
    MenuHandler m_mhMenu;
    InteractiveMarker m_imMarker;
    InteractiveMarkerControl m_imcControl;
    
  public:
    InteractiveObject();
    ~InteractiveObject();
    
    void insertIntoServer(InteractiveMarkerServer* imsServer);
    void clickCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  };
}


#endif /* __INTERACTIVE_OBJECT_H__ */
