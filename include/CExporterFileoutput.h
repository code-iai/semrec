#ifndef __C_EXPORTER_DOT_H__
#define __C_EXPORTER_DOT_H__


// System
#include <string>

// Private
#include <CExporter.h>

using namespace std;


namespace beliefstate {
  class CExporterFileoutput : public CExporter {
  private:
  public:
    CExporterFileoutput();
    ~CExporterFileoutput();
  
    void setOutputFilename(string strFilename);
    string outputFilename();
  
    bool writeToFile(string strContent, string strFilename = "");
  };
}


#endif /* __C_EXPORTER_DOT_H__ */
