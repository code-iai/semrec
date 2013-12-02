#include <plugins/owlexporter/PluginOWLExporter.h>


namespace beliefstate {
  namespace plugins {
    PluginOWLExporter::PluginOWLExporter() {
    }
    
    PluginOWLExporter::~PluginOWLExporter() {
    }
    
    Result PluginOWLExporter::init(int argc, char** argv) {
      Result resInit = defaultResult();
      
      this->setSubscribedToEvent("export-planlog", true);
      
      return resInit;
    }
    
    Result PluginOWLExporter::deinit() {
      return defaultResult();
    }
    
    Result PluginOWLExporter::cycle() {
      Result resCycle = defaultResult();
      this->deployCycleData(resCycle);
      
      return resCycle;
    }
    
    void PluginOWLExporter::consumeEvent(Event evEvent) {
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
    
    Event PluginOWLExporter::consumeServiceEvent(ServiceEvent seServiceEvent) {
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
		//seServiceEvent.cdDesignator->printDesignator();
		
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
		
		expOwl->setOutputFilename("/home/winkler/test-exp.owl");
		if(expOwl->runExporter(NULL)) {
		  this->info("Successfully exported OWL file '" + expOwl->outputFilename() + "'");
		} else {
		  this->warn("Failed to export to OWL file '" + expOwl->outputFilename() + "'");
		}
	      }
	    }
	  }
	}
      }
      
      return evResult;
    }
  }
  
  extern "C" plugins::PluginOWLExporter* createInstance() {
    return new plugins::PluginOWLExporter();
  }
  
  extern "C" void destroyInstance(plugins::PluginOWLExporter* icDestroy) {
    delete icDestroy;
  }
}
