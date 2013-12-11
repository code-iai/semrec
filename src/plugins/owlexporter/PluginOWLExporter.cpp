#include <plugins/owlexporter/PluginOWLExporter.h>


namespace beliefstate {
  namespace plugins {
    PLUGIN_CLASS::PLUGIN_CLASS() {
    }
    
    PLUGIN_CLASS::~PLUGIN_CLASS() {
    }
    
    Result PLUGIN_CLASS::init(int argc, char** argv) {
      Result resInit = defaultResult();
      
      this->setSubscribedToEvent("export-planlog", true);
      
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
      if(evEvent.cdDesignator) {
	string strFormat = evEvent.cdDesignator->stringValue("format");
	transform(strFormat.begin(), strFormat.end(), strFormat.begin(), ::tolower);
	
	if(strFormat == "owl") {
	  ServiceEvent seGetPlanTree = defaultServiceEvent("symbolic-plan-tree");
	  seGetPlanTree.cdDesignator = new CDesignator(evEvent.cdDesignator);
	  this->deployServiceEvent(seGetPlanTree);
	}
      }
    }
    
    Event PLUGIN_CLASS::consumeServiceEvent(ServiceEvent seServiceEvent) {
      Event evResult = defaultEvent();
      
      if(seServiceEvent.siServiceIdentifier == SI_RESPONSE) {
	if(seServiceEvent.strServiceName == "symbolic-plan-tree") {
	  if(seServiceEvent.cdDesignator) {
	    if(seServiceEvent.lstResultEvents.size() > 0) {
	      Event evCar = seServiceEvent.lstResultEvents.front();
	      
	      string strFormat = seServiceEvent.cdDesignator->stringValue("format");
	      transform(strFormat.begin(), strFormat.end(), strFormat.begin(), ::tolower);
	      
	      if(strFormat == "owl") {
		this->info("OWLExporter Plugin received plan log data. Exporting symbolic log.");
		
		CExporterOwl *expOwl = new CExporterOwl();
		expOwl->configuration()->setValue(string("display-successes"), (int)seServiceEvent.cdDesignator->floatValue("show-successes"));
		expOwl->configuration()->setValue(string("display-failures"), (int)seServiceEvent.cdDesignator->floatValue("show-fails"));
		expOwl->configuration()->setValue(string("max-detail-level"), (int)seServiceEvent.cdDesignator->floatValue("max-detail-level"));
		
		for(list<Node*>::iterator itN = evCar.lstNodes.begin();
		    itN != evCar.lstNodes.end();
		    itN++) {
		  Node* ndNode = *itN;
		  expOwl->addNode(ndNode);
		}
		
		expOwl->setDesignatorIDs(evCar.lstDesignatorIDs);
		expOwl->setDesignatorEquations(evCar.lstEquations);
		expOwl->setDesignatorEquationTimes(evCar.lstEquationTimes);
		
		ConfigSettings cfgsetCurrent = configSettings();
		expOwl->setOutputFilename(cfgsetCurrent.strExperimentDirectory + seServiceEvent.cdDesignator->stringValue("filename"));
		
		if(expOwl->runExporter(NULL)) {
		  this->info("Successfully exported OWL file '" + expOwl->outputFilename() + "'");
		} else {
		  this->warn("Failed to export to OWL file '" + expOwl->outputFilename() + "'");
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
