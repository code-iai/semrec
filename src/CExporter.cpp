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


#include <CExporter.h>


namespace semrec {
  CExporter::CExporter() {
    m_ckvpConfiguration = new KeyValuePair();
  }

  CExporter::~CExporter() {
    // NOTE(winkler): We used to delete the nodes when the exporter
    // shuts down. As in the new system the nodes are managed by a
    // central `symbolic log' entity, this is not necessary
    // anymore. In fact, its harmful to do (as the symbolic log will
    // try to `free' the same node addresses -- this results in a
    // segfault).

    //this->clearNodes();
    
    if(m_ckvpConfiguration) {
      delete m_ckvpConfiguration;
      m_ckvpConfiguration = NULL;
    }
  }

  void CExporter::addNode(Node* ndAdd) {
    m_lstNodes.push_back(ndAdd);
  }
  
  void CExporter::setRootNodes(std::list<Node*> lstRootNodes) {
    m_lstRootNodes = lstRootNodes;
  }
  
  void CExporter::addRootNode(Node* ndRoot) {
    m_lstRootNodes.push_back(ndRoot);
  }
  
  std::list<Node*> CExporter::rootNodes() {
    return m_lstRootNodes;
  }
  
  std::list<Node*> CExporter::nodes() {
    return m_lstNodes;
  }

  void CExporter::clearNodes() {
    for(Node* ndDelete : m_lstNodes) {
      delete ndDelete;
    }
    
    m_lstNodes.clear();
  }
  
  KeyValuePair* CExporter::configuration() {
    return m_ckvpConfiguration;
  }
  
  std::string CExporter::nodeIDPrefix(Node* ndInQuestion, std::string strProposition) {
    std::string strPrefix = ndInQuestion->metaInformation()->stringValue("class");
    
    if(strPrefix == "") {
      strPrefix = strProposition;
    } else {
      strPrefix += "_";
    }
    
    return strPrefix;
  }
  
  void CExporter::renewUniqueIDsForNode(Node *ndRenew) {
    std::string strNodeIDPrefix = this->nodeIDPrefix(ndRenew, "node_");
    ndRenew->setUniqueID(this->generateUniqueID(strNodeIDPrefix));
    
    std::list<Node*> lstSubnodes = ndRenew->subnodes();
    for(Node* ndNode : lstSubnodes) {
      this->renewUniqueIDsForNode(ndNode);
    }
  }
  
  void CExporter::renewUniqueIDs() {
    for(Node* ndNode : m_lstNodes) {
      this->renewUniqueIDsForNode(ndNode);
    }
  }
  
  bool CExporter::runExporter(KeyValuePair* ckvpConfigurationOverlay) {
    // NOTE(winkler): This is a dummy, superclass exporter. It does not
    // actually export anything. Subclass it to get *actual*
    // functionality.
    
    return true;
  }
  
  std::string CExporter::generateRandomIdentifier(std::string strPrefix, unsigned int unLength) {
    std::stringstream sts;
    sts << strPrefix;
    
    for(unsigned int unI = 0; unI < unLength; unI++) {
      int nRandom;
      
      do {
	nRandom = rand() % 122 + 48;
      } while(nRandom < 48 ||
	      (nRandom > 57 && nRandom < 65) ||
	      (nRandom > 90 && nRandom < 97) ||
	      nRandom > 122);
      
      char cRandom = (char)nRandom;
      sts << cRandom;
    }
    
    return sts.str();
  }
  
  std::string CExporter::generateUniqueID(std::string strPrefix, unsigned int unLength) {
    std::string strID;
    
    do {
      strID = this->generateRandomIdentifier(strPrefix, unLength);
    } while(this->uniqueIDPresent(strID));
  
    return strID;
  }

  bool CExporter::uniqueIDPresent(std::string strUniqueID) {
    for(Node* ndNode : m_lstNodes) {
      if(ndNode->includesUniqueID(strUniqueID)) {
	return true;
      }
    }
    
    return false;
  }

  std::string CExporter::replaceString(std::string strOriginal, std::string strReplaceWhat, std::string strReplaceBy) {
    size_t found;
    
    found = strOriginal.find(strReplaceWhat);
    while(found != std::string::npos) {
      strOriginal.replace(found, strReplaceWhat.length(), strReplaceBy);
      found = strOriginal.find(strReplaceWhat, found + strReplaceBy.length());
    };
    
    return strOriginal;
  }
  
  bool CExporter::nodeHasValidDetailLevel(Node* ndDisplay) {
    int nConfigMaxDetailLevel = this->configuration()->floatValue("max-detail-level");
    int nNodeDetailLevel = ndDisplay->metaInformation()->floatValue("detail-level");
    
    return (nNodeDetailLevel <= nConfigMaxDetailLevel);
  }
  
  bool CExporter::nodeDisplayable(Node* ndDisplay) {
    bool bDisplaySuccesses = (this->configuration()->floatValue("display-successes") == 1);
    bool bDisplayFailures = (this->configuration()->floatValue("display-failures") == 1);
    bool bNodeSuccess = (ndDisplay->metaInformation()->floatValue("success") == 1);
    
    if(this->nodeHasValidDetailLevel(ndDisplay)) {
      if((bNodeSuccess && bDisplaySuccesses) || (!bNodeSuccess && bDisplayFailures)) {
	return true;
      }
    }
    
    return false;
  }

  void CExporter::setDesignatorIDs(std::list< std::pair<std::string, std::string> > lstDesignatorIDs) {
    m_lstDesignatorIDs = lstDesignatorIDs;
  }

  void CExporter::setDesignatorEquations(std::list< std::pair<std::string, std::string> > lstDesignatorEquations) {
    m_lstDesignatorEquations = lstDesignatorEquations;
  }
  
  void CExporter::setDesignatorEquationTimes(std::list< std::pair<std::string, std::string> > lstDesignatorEquationTimes) {
    m_lstDesignatorEquationTimes = lstDesignatorEquationTimes;
  }
  
  std::list<std::string> CExporter::designatorIDs() {
    std::list<std::string> lstResult;
    
    for(std::pair<std::string, std::string> prPair : m_lstDesignatorIDs) {
      lstResult.push_back(prPair.second);
    }
    
    return lstResult;
  }
  
  std::list<std::string> CExporter::parentDesignatorsForID(std::string strID) {
    std::list<std::string> lstResult;
    
    for(std::pair<std::string, std::string> prPair : m_lstDesignatorEquations) {
      if(prPair.second == strID) {
	lstResult.push_back(prPair.first);
      }
    }
    
    return lstResult;
  }
  
  std::list<std::string> CExporter::successorDesignatorsForID(std::string strID) {
    std::list<std::string> lstResult;
    
    for(std::pair<std::string, std::string> prPair : m_lstDesignatorEquations) {
      if(prPair.first == strID) {
	lstResult.push_back(prPair.second);
      }
    }
    
    return lstResult;
  }
  
  std::string CExporter::equationTimeForSuccessorID(std::string strID) {
    std::string strReturn = "";
    
    for(std::pair<std::string, std::string> prPair : m_lstDesignatorEquationTimes) {
      if(prPair.first == strID) {
	strReturn = prPair.second;
	break;
      }
    }
    
    return strReturn;
  }
}
