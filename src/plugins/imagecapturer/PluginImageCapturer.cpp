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


#include <plugins/imagecapturer/PluginImageCapturer.h>


namespace semrec {
  namespace plugins {
    PLUGIN_CLASS::PLUGIN_CLASS() {
      m_icapImageCapturer = NULL;
      
      this->addDependency("ros");
      this->setPluginVersion("0.2");
    }
    
    PLUGIN_CLASS::~PLUGIN_CLASS() {
    }
    
    Result PLUGIN_CLASS::init(int argc, char** argv) {
      Result resInit = defaultResult();
      
      m_icapImageCapturer = new CImageCapturer();
      
      Designator* cdConfig = this->getIndividualConfig();
      m_icapImageCapturer->setTimeout(cdConfig->floatValue("timeout", 5.0f));
      m_icapImageCapturer->publishImages(cdConfig->stringValue("image-publishing-topic", "/logged_images"));
      
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
	  std::string strTopic = evEvent.cdDesignator->stringValue("origin");
	  std::string strFilename = evEvent.cdDesignator->stringValue("filename");
	  
	  if(strTopic != "") {
	    if(strFilename == "") {
	      strFilename = strTopic;
	      replace(strFilename.begin(), strFilename.end(), '/', '_');
	      strFilename += ".jpg";
	    }
	    
	    ConfigSettings cfgsetCurrent = configSettings();
	    
	    this->info("Waiting for image topic '" + strTopic + "'");
	    if(m_icapImageCapturer->captureFromTopic(strTopic, strFilename, cfgsetCurrent.strExperimentDirectory)) {
	      std::string strFilepath = cfgsetCurrent.strExperimentDirectory + strFilename;
	      this->info("Wrote image from topic '" + strTopic + "' to file '" + strFilepath + "'");
	      
	      Event evImage = eventInResponseTo(evEvent, "add-image-from-file");
	      evImage.cdDesignator = new Designator();
	      evImage.cdDesignator->setType(Designator::DesignatorType::ACTION);
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
	      
	      Event evImage = eventInResponseTo(evEvent, "cancel-open-request");
	      
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
