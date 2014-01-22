#include <plugins/experiment-context/PluginExperimentContext.h>


namespace beliefstate {
  namespace plugins {
    PLUGIN_CLASS::PLUGIN_CLASS() {
      m_expFileExporter = new CExporterFileoutput();
    }
    
    PLUGIN_CLASS::~PLUGIN_CLASS() {
      if(m_expFileExporter) {
	delete m_expFileExporter;
      }
    }
    
    Result PLUGIN_CLASS::init(int argc, char** argv) {
      this->setSubscribedToEvent("set-experiment-meta-data", true);
      this->setSubscribedToEvent("export-planlog", true);
      
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
	  this->info("Experiment Context Plugin exporting meta-data");
	
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
	  }
	  strMetaXML += "</meta-data>\n";
	
	  m_expFileExporter->writeToFile(strMetaXML, strMetaFile);
	
	  this->info("Successfully exported meta-data to '" + strMetaFile + "'");
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
