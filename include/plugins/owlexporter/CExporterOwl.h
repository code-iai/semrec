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


#ifndef __C_EXPORTER_OWL_H__
#define __C_EXPORTER_OWL_H__


// System
#include <string>
#include <list>
#include <libconfig.h++>

// Private
#include <CExporterFileoutput.h>

using namespace std;
using namespace libconfig;


namespace beliefstate {
  class CExporterOwl : public CExporterFileoutput {
  private:
    list< pair<string, string> > m_lstEntities;
    list< pair<string, string> > m_lstFailureMapping;
    list<string> m_lstDefinedProperties;
    list< pair<string, string> > m_lstAnnotationPurposeMapping;
    string m_strPropertyNamespace;
    string m_strDefaultAnnotation;
    
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
    string resolveDesignatorAnnotationTagName(string strAnnotation);
  };
}


#endif /* __C_EXPORTER_OWL_H__ */
