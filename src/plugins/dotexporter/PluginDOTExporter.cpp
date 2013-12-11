#include <plugins/dotexporter/PluginDOTExporter.h>


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
	
	if(strFormat == "dot") {
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
	      
	      if(strFormat == "dot") {
		this->info("DOTExporter Plugin received plan log data. Exporting symbolic log.");
		
		CExporterDot *expDot = new CExporterDot();
		expDot->configuration()->setValue(string("display-successes"), (int)seServiceEvent.cdDesignator->floatValue("show-successes"));
		expDot->configuration()->setValue(string("display-failures"), (int)seServiceEvent.cdDesignator->floatValue("show-fails"));
		expDot->configuration()->setValue(string("max-detail-level"), (int)seServiceEvent.cdDesignator->floatValue("max-detail-level"));
		
		for(list<Node*>::iterator itN = evCar.lstNodes.begin();
		    itN != evCar.lstNodes.end();
		    itN++) {
		  Node* ndNode = *itN;
		  expDot->addNode(ndNode);
		}
		
		expDot->setDesignatorIDs(evCar.lstDesignatorIDs);
		expDot->setDesignatorEquations(evCar.lstEquations);
		expDot->setDesignatorEquationTimes(evCar.lstEquationTimes);
		
		ConfigSettings cfgsetCurrent = configSettings();
		expDot->setOutputFilename(cfgsetCurrent.strExperimentDirectory + seServiceEvent.cdDesignator->stringValue("filename"));
		
		if(expDot->runExporter(NULL)) {
		  this->info("Successfully exported DOT file '" + expDot->outputFilename() + "'");
		} else {
		  this->warn("Failed to export to DOT file '" + expDot->outputFilename() + "'");
		}
		
		delete expDot;
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
