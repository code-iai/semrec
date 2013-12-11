#include <plugins/imagecapturer/PluginImageCapturer.h>


namespace beliefstate {
  namespace plugins {
    PLUGIN_CLASS::PLUGIN_CLASS() {
      m_icapImageCapturer = NULL;
      this->addDependency("ros");
    }
    
    PLUGIN_CLASS::~PLUGIN_CLASS() {
    }
    
    Result PLUGIN_CLASS::init(int argc, char** argv) {
      Result resInit = defaultResult();
      
      m_icapImageCapturer = new CImageCapturer();
      m_icapImageCapturer->publishImages("/logged_images");
      this->setSubscribedToEvent("add-image-from-topic", true);
      
      return resInit;
    }
    
    Result PLUGIN_CLASS::deinit() {
      if(m_icapImageCapturer) {
	delete m_icapImageCapturer;
      }
      
      return defaultResult();
    }
    
    Result PLUGIN_CLASS::cycle() {
      Result resCycle = defaultResult();
      this->deployCycleData(resCycle);
      
      return resCycle;
    }
    
    void PLUGIN_CLASS::consumeEvent(Event evEvent) {
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
	    
	    ConfigSettings cfgsetCurrent = configSettings();
	    
	    if(m_icapImageCapturer->captureFromTopic(strTopic, strFilename, cfgsetCurrent.strExperimentDirectory)) {
	      string strFilepath = cfgsetCurrent.strExperimentDirectory + strFilename;
	      this->info("Wrote image from topic '" + strTopic + "' to file '" + strFilepath + "'");
	      
	      Event evImage = eventInResponseTo(evEvent, "add-image-from-file");
	      evImage.cdDesignator = new CDesignator();
	      evImage.cdDesignator->setType(ACTION);
	      evImage.cdDesignator->setValue("origin", strTopic);
	      // NOTE(winkler): We just use the filename here, not the
	      // global filepath. This is due to the fact that all
	      // images are stored relative to the generated .owl file
	      // (i.e. in the same directory). When moving all files
	      // somewhere else, global paths would make finding of
	      // files very difficult.
	      evImage.cdDesignator->setValue("filename", strFilename);
	      
	      m_mtxEventsStore.lock();
	      m_lstEvents.push_back(evImage);
	      m_mtxEventsStore.unlock();
	    } else {
	      this->warn("Failed to capture image from topic '" + strTopic + "' and write it to '" + cfgsetCurrent.strExperimentDirectory + strFilename + "'.");
	      
	      Event evImage = eventInResponseTo(evEvent);
	      
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
  
  extern "C" plugins::PLUGIN_CLASS* createInstance() {
    return new plugins::PLUGIN_CLASS();
  }
  
  extern "C" void destroyInstance(plugins::PLUGIN_CLASS* icDestroy) {
    delete icDestroy;
  }
}
