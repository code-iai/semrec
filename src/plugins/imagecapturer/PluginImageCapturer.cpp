#include <plugins/imagecapturer/PluginImageCapturer.h>


namespace beliefstate {
  namespace plugins {
    PluginImageCapturer::PluginImageCapturer() {
      m_icapImageCapturer = NULL;
      this->addDependency("ros");
    }
    
    PluginImageCapturer::~PluginImageCapturer() {
    }
    
    Result PluginImageCapturer::init(int argc, char** argv) {
      Result resInit = defaultResult();
      
      m_icapImageCapturer = new CImageCapturer();
      m_icapImageCapturer->publishImages("/logged_images");
      this->setSubscribedToEvent("add-image-from-topic", true);
      
      return resInit;
    }
    
    Result PluginImageCapturer::deinit() {
      if(m_icapImageCapturer) {
	delete m_icapImageCapturer;
      }
      
      return defaultResult();
    }
    
    Result PluginImageCapturer::cycle() {
      Result resCycle = defaultResult();
      this->deployCycleData(resCycle);
      
      return resCycle;
    }
    
    void PluginImageCapturer::consumeEvent(Event evEvent) {
      if(evEvent.strEventName == "add-image-from-topic") {
	if(evEvent.cdDesignator) {
	  string strTopic = evEvent.cdDesignator->stringValue("origin");
	  string strFilename = evEvent.cdDesignator->stringValue("filename");
	  
	  if(strTopic != "") {
	    if(strFilename == "") {
	      strFilename = strTopic;
	      replace(strFilename.begin(), strFilename.end(), '/', '_');
	      strFilename += ".png";
	    }
	    
	    if(m_icapImageCapturer->captureFromTopic(strTopic, strFilename, "/home/winkler/tempicap/")) {
	      this->info("Wrote image from topic '" + strTopic + "' to file '" + strFilename + "'");
	      
	      Event evImage = defaultEvent("add-image-from-file");
	      evImage.cdDesignator = new CDesignator();
	      evImage.cdDesignator->setType(ACTION);
	      evImage.cdDesignator->setValue("filename", strFilename);
	      evImage.nOpenRequestID = evEvent.nOpenRequestID;
	      evImage.bRequest = false;
	      
	      m_mtxEventsStore.lock();
	      m_lstEvents.push_back(evImage);
	      m_mtxEventsStore.unlock();
	    } else {
	      this->warn("Failed to capture image from topic '" + strTopic + "' and write it to '" + strFilename + "'.");
	      
	      Event evImage = defaultEvent("add-image-from-topic");
	      evImage.nOpenRequestID = evEvent.nOpenRequestID;
	      evImage.bRequest = false;
	      
	      m_mtxEventsStore.lock();
	      m_lstEvents.push_back(evImage);
	      m_mtxEventsStore.unlock();
	    }
	  } else {
	    this->warn("No topic was given when requesting to capture an image from a topic.");
	  }
	}
      }
    }
  }
  
  extern "C" plugins::PluginImageCapturer* createInstance() {
    return new plugins::PluginImageCapturer();
  }
  
  extern "C" void destroyInstance(plugins::PluginImageCapturer* icDestroy) {
    delete icDestroy;
  }
}
