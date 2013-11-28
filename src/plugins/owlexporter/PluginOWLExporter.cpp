#include <plugins/owlexporter/PluginOWLExporter.h>


namespace beliefstate {
  namespace plugins {
    PluginOWLExporter::PluginOWLExporter() {
    }
    
    PluginOWLExporter::~PluginOWLExporter() {
    }
    
    Result PluginOWLExporter::init(int argc, char** argv) {
      Result resInit = defaultResult();
      
      this->setSubscribedToEvent(EI_EXPORT_PLANLOG, true);
      
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
	    string strFormat = seServiceEvent.cdDesignator->stringValue("format");
	    transform(strFormat.begin(), strFormat.end(), strFormat.begin(), ::tolower);
	    
	    if(strFormat == "owl") {
	      this->info("OWLExporter Plugin received plan log data. Do this:");
	      seServiceEvent.cdDesignator->printDesignator();
	      
	      this->info("We got an answer for the tree: ");
	      Event evCar = seServiceEvent.lstResultEvents.front();
	      
	      for(list<Node*>::iterator itN = evCar.lstNodes.begin();
		  itN != evCar.lstNodes.end();
		  itN++) {
		Node* ndNode = *itN;
		
		cout << ndNode->title() << endl;
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
