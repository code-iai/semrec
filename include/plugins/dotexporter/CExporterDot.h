#ifndef __C_EXPORTER_FILEOUTPUT_H__
#define __C_EXPORTER_FILEOUTPUT_H__


// System
#include <string>

// Private
#include <CExporterFileoutput.h>

using namespace std;


namespace beliefstate {
  class CExporterDot : public CExporterFileoutput {
  private:
  public:
    CExporterDot();
    ~CExporterDot();
  
    virtual bool runExporter(CKeyValuePair* ckvpConfigurationOverlay);
    string generateDotStringForNodes(list<Node*> lstNodes, string strParentID);
    string generateDotImagesStringForNode(Node *ndImages);
    string generateDotObjectsStringForNode(Node *ndObjects);
    string generateDotStringForDescription(list<CKeyValuePair*> lstDescription);  
    string dotEscapeString(string strValue);
  };
}


#endif /* __C_EXPORTER_FILEOUTPUT_H__ */
