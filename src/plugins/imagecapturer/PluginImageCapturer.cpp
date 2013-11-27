#include <plugins/imagecapturer/PluginImageCapturer.h>


namespace beliefstate {
  namespace plugins {
    PluginImageCapturer::PluginImageCapturer() {
      this->addDependency("ros");
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
      this->deployCycleData(resCycle);
      
      return resCycle;
    }
    
    void PluginImageCapturer::consumeEvent(Event evEvent) {
      cout << "PluginImageCapturer: Consume event!" << endl;
      
      // NOTE(winkler): This is a dummy implementation to see if the
      // event distribution works.
      
      Event evImage = defaultEvent();
      evImage.eiEventIdentifier = EI_ADD_IMAGE_FROM_FILE;
      // TODO(winkler): Add filename here!
      
      m_mtxEventsStore.lock();
      m_lstEvents.push_back(evImage);
      m_mtxEventsStore.unlock();
    }
  }
  
  extern "C" plugins::PluginImageCapturer* createInstance() {
    return new plugins::PluginImageCapturer();
  }
  
  extern "C" void destroyInstance(plugins::PluginImageCapturer* icDestroy) {
    delete icDestroy;
  }
}
