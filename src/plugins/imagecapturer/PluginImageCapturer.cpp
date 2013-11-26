#include <plugins/imagecapturer/PluginImageCapturer.h>


namespace beliefstate {
  namespace plugins {
    PluginImageCapturer::PluginImageCapturer() {
    }
    
    PluginImageCapturer::~PluginImageCapturer() {
    }
    
    Result PluginImageCapturer::init(int argc, char** argv) {
      Result resInit = defaultResult();
      
      this->setSubscribedToEvent(EI_ADD_IMAGE_FROM_TOPIC, true);
      
      return resInit;
    }
    
    Result PluginImageCapturer::deinit() {
      return defaultResult();
    }
    
    Result PluginImageCapturer::cycle() {
      Result resCycle = defaultResult();
      
      return resCycle;
    }
    
    void PluginImageCapturer::consumeEvent(Event evEvent) {
      cout << "PluginImageCapturer: Consume event!" << endl;
    }
  }
  
  extern "C" plugins::PluginImageCapturer* createInstance() {
    return new plugins::PluginImageCapturer();
  }
  
  extern "C" void destroyInstance(plugins::PluginImageCapturer* icDestroy) {
    delete icDestroy;
  }
}
