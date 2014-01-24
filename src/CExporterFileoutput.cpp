#include <CExporterFileoutput.h>


namespace beliefstate {
  CExporterFileoutput::CExporterFileoutput() {
    this->setOutputFilename("");
  }
  
  CExporterFileoutput::~CExporterFileoutput() {
  }
  
  void CExporterFileoutput::setOutputFilename(string strFilename) {
    this->configuration()->setValue("filename", strFilename);
  }
  
  string CExporterFileoutput::outputFilename() {
    return this->configuration()->stringValue("filename");
  }
  
  bool CExporterFileoutput::writeToFile(string strContent, string strFilename) {
    if(strFilename == "") {
      strFilename = this->configuration()->stringValue("filename");
    }
    
    if(strFilename != "") {
      ofstream fsFile;
      fsFile.open(strFilename.c_str());
      fsFile << strContent;
      fsFile.close();
    
      return true;
    }
  
    return false;
  }
}
