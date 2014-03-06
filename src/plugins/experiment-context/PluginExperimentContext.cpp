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


#include <plugins/experiment-context/PluginExperimentContext.h>


namespace beliefstate {
  namespace plugins {
    PLUGIN_CLASS::PLUGIN_CLASS() {
      m_expFileExporter = new CExporterFileoutput();
      
      this->setPluginVersion("0.3");
      this->addDependency("ros");
    }
    
    PLUGIN_CLASS::~PLUGIN_CLASS() {
      if(m_expFileExporter) {
	delete m_expFileExporter;
      }
    }
    
    Result PLUGIN_CLASS::init(int argc, char** argv) {
      this->setSubscribedToEvent("set-experiment-meta-data", true);
      this->setSubscribedToEvent("export-planlog", true);
      this->setSubscribedToEvent("experiment-start", true);
      this->setSubscribedToEvent("experiment-shutdown", true);
      
      ros::NodeHandle nh;
      m_pubMetadata = nh.advertise<designator_integration_msgs::Designator>("/logged_metadata", 1);
      
      return defaultResult();
    }
    
    Result PLUGIN_CLASS::deinit() {
      return defaultResult();
    }
    
    Result PLUGIN_CLASS::cycle() {
      Result resCycle = defaultResult();
      this->deployCycleData(resCycle);
      
      return resCycle;
    }
    
    void PLUGIN_CLASS::consumeEvent(Event evEvent) {
      if(evEvent.strEventName == "set-experiment-meta-data") {
	if(evEvent.cdDesignator) {
	  string strField = evEvent.cdDesignator->stringValue("field");
	  string strValue = evEvent.cdDesignator->stringValue("value");
	  
	  if(strField != "") {
	    this->info("Set '" + strField + "' to '" + strValue + "'");
	    m_mapValues[strField] = strValue;
	  }
	}
      } else if(evEvent.strEventName == "export-planlog") {
	string strFormat = evEvent.cdDesignator->stringValue("format");
	transform(strFormat.begin(), strFormat.end(), strFormat.begin(), ::tolower);
	
	if(strFormat == "meta") {
	  CDesignator* cdMeta = new CDesignator();
	  cdMeta->setType(ACTION);
	  
	  this->info("Experiment Context Plugin exporting meta-data");
	  
	  if(m_mapValues.find("time-end") == m_mapValues.end()) {
	    stringstream sts;
	    sts << this->getTimeStamp();
	    
	    m_mapValues["time-end"] = sts.str();
	  }
	  
	  ConfigSettings cfgsetCurrent = configSettings();
	  string strMetaFile = cfgsetCurrent.strExperimentDirectory + "metadata.xml";
	  
	  string strMetaXML = "";
	  strMetaXML += "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n\n";
	  strMetaXML += "<meta-data>\n";
	  for(map<string, string>::iterator itValues = m_mapValues.begin();
	      itValues != m_mapValues.end();
	      itValues++) {
	    pair<string, string> prEntry = *itValues;
	    
	    strMetaXML += "  <" + prEntry.first + ">" + prEntry.second + "</" + prEntry.first + ">\n";
	    cdMeta->setValue(prEntry.first, prEntry.second);
	  }
	  strMetaXML += "</meta-data>\n";
	  
	  this->info("Published metadata");
	  m_pubMetadata.publish(cdMeta->serializeToMessage());
	  delete cdMeta;
	  
	  m_expFileExporter->writeToFile(strMetaXML, strMetaFile);
	  
	  this->info("Successfully exported meta-data to '" + strMetaFile + "'");
	}
      } else if(evEvent.strEventName == "experiment-start") {
	stringstream sts;
	sts << this->getTimeStamp();
	m_mapValues["time-start"] = sts.str();
      } else if(evEvent.strEventName == "experiment-shutdown") {
	stringstream sts;
	sts << this->getTimeStamp();
	m_mapValues["time-end"] = sts.str();
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
