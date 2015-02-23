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


#ifndef __C_EXPORTER_H__
#define __C_EXPORTER_H__


// System
#include <string>
#include <list>
#include <fstream>

// Other
#include <designators/KeyValuePair.h>

// Private
#include <Node.h>
#include <UtilityBase.h>


namespace semrec {
  class CExporter : public UtilityBase {
  private:
    std::list<Node*> m_lstRootNodes;
    std::list< std::pair<std::string, std::string> > m_lstDesignatorIDs;
    std::list< std::pair<std::string, std::string> > m_lstDesignatorEquations;
  
    KeyValuePair* m_ckvpConfiguration;
  
    void renewUniqueIDsForNode(Node *ndRenew);
  
  protected:
    std::list<Node*> m_lstNodes;
    
    std::list< std::pair<std::string, std::string> > m_lstDesignatorEquationTimes;
  
  public:
    CExporter();
    virtual ~CExporter();
    
    KeyValuePair* configuration();
    
    void addNode(Node* ndAdd);
    std::list<Node*> nodes();
    void setRootNodes(std::list<Node*> lstRootNodes);
    void addRootNode(Node* ndRoot);
    std::list<Node*> rootNodes();
    
    void clearNodes();
    
    virtual bool runExporter(KeyValuePair* ckvpConfigurationOverlay);
    
    std::string generateRandomIdentifier(std::string strPrefix, unsigned int unLength = 16);
    std::string generateUniqueID(std::string strPrefix, unsigned int unLength = 16);
    bool uniqueIDPresent(std::string strUniqueID);
    
    void renewUniqueIDs();
    
    virtual std::string nodeIDPrefix(Node* ndInQuestion, std::string strProposition);
    
    std::string replaceString(std::string strOriginal, std::string strReplaceWhat, std::string strReplaceBy);
    virtual bool nodeHasValidDetailLevel(Node* ndDisplay);
    virtual bool nodeDisplayable(Node* ndDisplay);
    
    void setDesignatorIDs(std::list< std::pair<std::string, std::string> > lstDesignatorIDs);
    void setDesignatorEquations(std::list< std::pair<std::string, std::string> > lstDesignatorEquations);
    void setDesignatorEquationTimes(std::list< std::pair<std::string, std::string> > lstDesignatorEquationTimes);  
    
    std::list<std::string> designatorIDs();
    std::list<std::string> parentDesignatorsForID(std::string strID);
    std::list<std::string> successorDesignatorsForID(std::string strID);
    std::string equationTimeForSuccessorID(std::string strID);
  };
}

#endif /* __C_EXPORTER_H__ */
