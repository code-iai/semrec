#ifndef __PLUGIN_IMAGE_CAPTURER_H__
#define __PLUGIN_IMAGE_CAPTURER_H__


// System
#include <cstdlib>
#include <iostream>

// ROS
#include <ros/ros.h>

// Designators
#include <designators/CDesignator.h>
#include <designator_integration_msgs/DesignatorCommunication.h>

// Private
#include <Types.h>
#include <ForwardDeclarations.h>
#include <Plugin.h>
#include <plugins/imagecapturer/CImageCapturer.h>

using namespace std;


namespace beliefstate {
  namespace plugins {
    class PluginImageCapturer : public Plugin {
    private:
      CImageCapturer* m_icapImageCapturer;
      
    public:
      PluginImageCapturer();
      ~PluginImageCapturer();
      
      virtual Result init(int argc, char** argv);
      virtual Result deinit();
      
      virtual Result cycle();
      
      virtual void consumeEvent(Event evEvent);
    };
  }
  
  extern "C" plugins::PluginImageCapturer* createInstance();
  extern "C" void destroyInstance(plugins::PluginImageCapturer* icDestroy);
}


#endif /* __PLUGIN_IMAGE_CAPTURER_H__ */
