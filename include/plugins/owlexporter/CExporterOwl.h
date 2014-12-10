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
#include <algorithm>

// LibConfig
#include <libconfig.h++>

// Private
#include <CExporterFileoutput.h>
#include <plugins/owlexporter/OwlIndividual.h>


namespace semrec {
  class CExporterOwl : public CExporterFileoutput {
  private:
    std::list< std::pair<std::string, std::string> > m_lstEntities;
    std::list< std::pair<std::string, std::string> > m_lstFailureMapping;
    std::string m_strDefaultConditionMapping;
    std::string m_strDefaultDesignatorClass;
    std::list< std::pair<std::string, std::string> > m_lstAnnotationPurposeMapping;
    std::string m_strPropertyNamespace;
    std::string m_strDefaultAnnotation;
    std::map<std::string, std::string> m_mapMetaData;
    std::list<std::string> m_lstAnnotatedParameters;
    std::list<std::string> m_lstExportedObjectIndividuals;
    std::map<std::string, std::string> m_mapRegisteredOWLNamespaces;
    std::map<std::string, KeyValuePair*> m_mapDesignators;
    int m_nThrowAndCatchFailureCounter;
    
    void addEntity(std::string strNickname, std::string strNamespace);
    
  public:
    CExporterOwl();
    ~CExporterOwl();
    
    std::list<std::string> gatherClassesForNodes(std::list<Node*> lstNodes);
    std::list<std::string> gatherTimepointsForNodes(std::list<Node*> lstNodes, std::list<Node*> lstTrace);
    void setMetaData(std::map<std::string, std::string> mapMetaData);
    
    bool loadSemanticsDescriptorFile(std::string strFilepath);
    
    void prepareEntities(std::string strNamespaceID, std::string strNamespace);
    std::string generateDocTypeBlock();
    std::string generateXMLNSBlock(std::string strNamespace);
    std::string generateOwlImports(std::string strNamespace);
    std::string generatePropertyDefinitions();
    std::string generateClassDefinitions();
    std::string generateEventIndividualsForNodes(std::list<Node*> lstNodes, std::string strNamespace);
    std::string generateEventIndividuals(std::string strNamespace);
    std::string generateObjectIndividualsForNodes(std::list<Node*> lstNodes, std::string strNamespace);
    std::string generateObjectIndividuals(std::string strNamespace);
    std::string generateImageIndividualsForNodes(std::list<Node*> lstNodes, std::string strNamespace);
    std::string generateImageIndividuals(std::string strNamespace);
    std::string generateDesignatorIndividuals(std::string strNamespace);
    std::list<std::string> collectAllSuccessorDesignatorIDs(std::string strDesigID);
    std::string generateFailureIndividualsForNodes(std::list<Node*> lstNodes, std::string strNamespace);
    std::string generateFailureIndividuals(std::string strNamespace);
    std::string generateTimepointIndividuals(std::string strNamespace);
    std::string generateMetaDataIndividual(std::string strNamespace);
    std::string generateParameterAnnotationInformation(std::string strNamespace);
    
    std::string owlClassForNode(Node *ndNode, bool bClassOnly = false, bool bPrologSyntax = false);
    std::string owlClassForObject(KeyValuePair *ckvpObject);  
    virtual std::string nodeIDPrefix(Node* ndInQuestion, std::string strProposition);
    
    virtual bool runExporter(KeyValuePair* ckvpConfigurationOverlay);
    std::string owlEscapeString(std::string strValue);
    std::string generateOwlStringForNodes(std::list<Node*> lstNodes, std::string strNamespaceID, std::string strNamespace);
    
    std::string failureClassForCondition(std::string strCondition);
    std::string resolveDesignatorAnnotationTagName(std::string strAnnotation);
    
    void setRegisteredOWLNamespaces(std::map<std::string, std::string> mapRegisteredOWLNamespaces);
  };
}


#endif /* __C_EXPORTER_OWL_H__ */
