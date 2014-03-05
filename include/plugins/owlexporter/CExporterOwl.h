#ifndef __C_EXPORTER_OWL_H__
#define __C_EXPORTER_OWL_H__


// System
#include <string>
#include <list>

// Private
#include <CExporterFileoutput.h>

using namespace std;


namespace beliefstate {
  class CExporterOwl : public CExporterFileoutput {
  private:
    list< pair<string, string> > m_lstEntities;
    
    void addEntity(string strNickname, string strNamespace);
    
  public:
    CExporterOwl();
    ~CExporterOwl();
    
    list<string> gatherClassesForNodes(list<Node*> lstNodes);
    list<string> gatherTimepointsForNodes(list<Node*> lstNodes);
    
    bool loadSemanticsDescriptorFile(string strFilepath);
    
    void prepareEntities(string strNamespaceID, string strNamespace);
    string generateDocTypeBlock();
    string generateXMLNSBlock(string strNamespace);
    string generateOwlImports(string strNamespace);
    string generatePropertyDefinitions();
    string generateClassDefinitions();
    string generateEventIndividualsForNodes(list<Node*> lstNodes, string strNamespace);
    string generateEventIndividuals(string strNamespace);
    string generateObjectIndividualsForNodes(list<Node*> lstNodes, string strNamespace);
    string generateObjectIndividuals(string strNamespace);
    string generateImageIndividualsForNodes(list<Node*> lstNodes, string strNamespace);
    string generateImageIndividuals(string strNamespace);
    string generateDesignatorIndividuals(string strNamespace);
    string generateFailureIndividualsForNodes(list<Node*> lstNodes, string strNamespace);
    string generateFailureIndividuals(string strNamespace);
    string generateTimepointIndividuals(string strNamespace);
    
    string owlClassForNode(Node *ndNode, bool bClassOnly = false, bool bPrologSyntax = false);
    string owlClassForObject(CKeyValuePair *ckvpObject);  
    virtual string nodeIDPrefix(Node* ndInQuestion, string strProposition);
    
    virtual bool runExporter(CKeyValuePair* ckvpConfigurationOverlay);
    string owlEscapeString(string strValue);
    string generateOwlStringForNodes(list<Node*> lstNodes, string strNamespaceID, string strNamespace);
    
    string failureClassForCondition(string strCondition);
  };
}


#endif /* __C_EXPORTER_OWL_H__ */
