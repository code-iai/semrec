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


#include <plugins/owlexporter/PluginOWLExporter.h>


namespace semrec {
  namespace plugins {
    PLUGIN_CLASS::PLUGIN_CLASS() {
      this->setPluginVersion("0.93");
    }
    
    PLUGIN_CLASS::~PLUGIN_CLASS() {
    }
    
    Result PLUGIN_CLASS::init(int argc, char** argv) {
      Result resInit = defaultResult();
      
      this->setSubscribedToEvent("set-experiment-meta-data", true);
      this->setSubscribedToEvent("export-planlog", true);
      this->setSubscribedToEvent("experiment-start", true);
      this->setSubscribedToEvent("experiment-shutdown", true);
      this->setSubscribedToEvent("register-owl-namespace", true);
      this->setSubscribedToEvent("update-absolute-experiment-start-time", true);
      this->setSubscribedToEvent("update-absolute-experiment-end-time", true);
      this->setSubscribedToEvent("start-new-experiment", true);
      
      return resInit;
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
      if(evEvent.strEventName == "export-planlog") {
	if(evEvent.cdDesignator) {
	  std::string strFormat = evEvent.cdDesignator->stringValue("format");
	  transform(strFormat.begin(), strFormat.end(), strFormat.begin(), ::tolower);
	  
	  if(strFormat == "owl") {
	    if(m_mapMetaData.find("time-end") == m_mapMetaData.end()) {
	      //m_mapMetaData["time-end"] = this->getTimeStampStr();
	    }
	    
	    ServiceEvent seGetPlanTree = defaultServiceEvent("symbolic-plan-tree");
	    seGetPlanTree.cdDesignator = new Designator(evEvent.cdDesignator);
	    this->deployServiceEvent(seGetPlanTree);
	  }
	}
      } else if(evEvent.strEventName == "experiment-start") {
	Event evSendOwlExporterVersion = defaultEvent("set-experiment-meta-data");
	evSendOwlExporterVersion.cdDesignator = new Designator();
	evSendOwlExporterVersion.cdDesignator->setType(Designator::DesignatorType::ACTION);
	
	evSendOwlExporterVersion.cdDesignator->setValue("field", "owl-exporter-version");
	evSendOwlExporterVersion.cdDesignator->setValue("value", this->pluginVersion());
	
	//m_mapRegisteredOWLNamespaces.clear();
	
	this->deployEvent(evSendOwlExporterVersion);
	
	//m_mapMetaData["time-start"] = this->getTimeStampStr();
      } else if(evEvent.strEventName == "experiment-shutdown") {
	//m_mapMetaData["time-end"] = this->getTimeStampStr();
      } else if(evEvent.strEventName == "set-experiment-meta-data") {
	if(evEvent.cdDesignator) {
	  std::string strField = evEvent.cdDesignator->stringValue("field");
	  std::string strValue = evEvent.cdDesignator->stringValue("value");
	  
	  if(strField != "") {
	    this->info("Set meta data field '" + strField + "' to '" + strValue + "'");
	    m_mapMetaData[strField] = strValue;
	  }
	}
      } else if(evEvent.strEventName == "register-owl-namespace") {
	if(evEvent.cdDesignator) {
	  std::string strShortcut = evEvent.cdDesignator->stringValue("shortcut");
	  std::string strIRI = evEvent.cdDesignator->stringValue("iri");
	  
	  if(strIRI != "" && strShortcut != "") {
	    m_mapRegisteredOWLNamespaces[strShortcut] = strIRI;
	    this->info("Registered OWL namespace: '" + strShortcut + "' = '" + strIRI + "'");
	  } else {
	    this->warn("Did not register OWL namespace. Insufficient information: '" + strShortcut + "' = '" + strIRI + "'");
	  }
	}
      } else if(evEvent.strEventName == "update-absolute-experiment-start-time") {
	if(evEvent.lstNodes.size() > 0) {
	  if(m_mapMetaData.find("time-start") == m_mapMetaData.end()) {
	    // First entry
	    m_mapMetaData["time-start"] = evEvent.lstNodes.front()->metaInformation()->stringValue("time-start");
	  } else {
	    // Update if necessary
	    std::string strOld = m_mapMetaData["time-start"];
	    std::string strNew = evEvent.lstNodes.front()->metaInformation()->stringValue("time-start");
	    
	    float fOld, fNew;
	    sscanf(strOld.c_str(), "%f", &fOld);
	    sscanf(strNew.c_str(), "%f", &fNew);
	    
	    if(fNew < fOld) {
	      m_mapMetaData["time-start"] = strNew;
	    }
	  }
	}
      } else if(evEvent.strEventName == "update-absolute-experiment-end-time") {
	if(evEvent.lstNodes.size() > 0) {
	  // Every end time overwrites any already existing value, as
	  // it always happens after.
	  if(m_mapMetaData.find("time-end") == m_mapMetaData.end()) {
	    m_mapMetaData["time-end"] = evEvent.lstNodes.front()->metaInformation()->stringValue("time-end");
	  } else {
	    // Update if necessary
	    std::string strOld = m_mapMetaData["time-end"];
	    std::string strNew = evEvent.lstNodes.front()->metaInformation()->stringValue("time-end");
	    
	    float fOld, fNew;
	    sscanf(strOld.c_str(), "%f", &fOld);
	    sscanf(strNew.c_str(), "%f", &fNew);
	    
	    if(fNew > fOld) {
	      m_mapMetaData["time-end"] = strNew;
	    }
	  }
	}
      } else if(evEvent.strEventName == "start-new-experiment") {
	m_mapMetaData.clear();
      }
    }
    
    Event PLUGIN_CLASS::consumeServiceEvent(ServiceEvent seServiceEvent) {
      Event evResult = defaultEvent();
      
      if(seServiceEvent.siServiceIdentifier == SI_RESPONSE) {
	if(seServiceEvent.strServiceName == "symbolic-plan-tree") {
	  if(seServiceEvent.cdDesignator) {
	    if(seServiceEvent.lstResultEvents.size() > 0) {
	      Event evCar = seServiceEvent.lstResultEvents.front();
	      
	      std::string strFormat = seServiceEvent.cdDesignator->stringValue("format");
	      transform(strFormat.begin(), strFormat.end(), strFormat.begin(), ::tolower);
	      
	      if(strFormat == "owl") {
		this->info("OWLExporter Plugin received plan log data. Exporting symbolic log.");
		
		CExporterOwl* expOwl = new CExporterOwl();
		expOwl->setMetaData(m_mapMetaData);
		
		Designator* cdConfig = this->getIndividualConfig();
		std::string strSemanticsDescriptorFile = cdConfig->stringValue("semantics-descriptor-file");
		
		if(strSemanticsDescriptorFile != "") {
		  this->info("Loading semantics descriptor file '" + strSemanticsDescriptorFile + "'");
		  
		  if(expOwl->loadSemanticsDescriptorFile(strSemanticsDescriptorFile) == false) {
		    this->warn("Failed to load semantics descriptor file '" + strSemanticsDescriptorFile + "'.");
		  }
		} else {
		  this->warn("No semantics descriptor file was specified.");
		}
		
		expOwl->configuration()->setValue(std::string("display-successes"), (int)seServiceEvent.cdDesignator->floatValue("show-successes"));
		expOwl->configuration()->setValue(std::string("display-failures"), (int)seServiceEvent.cdDesignator->floatValue("show-fails"));
		expOwl->configuration()->setValue(std::string("max-detail-level"), (int)seServiceEvent.cdDesignator->floatValue("max-detail-level"));
		
		expOwl->setRootNodes(evCar.lstRootNodes);
		
		bool bFailed = false;
		for(Node* ndNode : evCar.lstNodes) {
		  if(ndNode) {
		    expOwl->addNode(ndNode);
		  } else {
		    this->fail("One of the nodes received in the plan log data contains invalid data. Cancelling export. Try again at will.");
		    bFailed = true;
		    break;
		  }
		}
		
		if(!bFailed) {
		  this->info("Parameterizing exporter");
		  
		  expOwl->setDesignatorIDs(evCar.lstDesignatorIDs);
		  expOwl->setDesignatorEquations(evCar.lstEquations);
		  expOwl->setDesignatorEquationTimes(evCar.lstEquationTimes);
		  
		  ConfigSettings cfgsetCurrent = configSettings();
		  expOwl->setOutputFilename(cfgsetCurrent.strExperimentDirectory + seServiceEvent.cdDesignator->stringValue("filename"));
		  expOwl->setRegisteredOWLNamespaces(m_mapRegisteredOWLNamespaces);
		  
		  this->info("Exporting OWL file to '" + expOwl->outputFilename() + "'", true);
		  
		  if(expOwl->runExporter(NULL)) {
		    this->info("Successfully exported OWL file '" + expOwl->outputFilename() + "'", true);
		  } else {
		    this->warn("Failed to export to OWL file '" + expOwl->outputFilename() + "'", true);
		  }
		} else {
		  this->warn("Failed to export to OWL file '" + expOwl->outputFilename() + "'", true);
		}
		
		delete expOwl;
	      }
	    }
	  }
	}
      }
      
      return evResult;
    }
  }
  
  extern "C" plugins::PLUGIN_CLASS* createInstance() {
    return new plugins::PLUGIN_CLASS();
  }
  
  extern "C" void destroyInstance(plugins::PLUGIN_CLASS* icDestroy) {
    delete icDestroy;
  }
}
