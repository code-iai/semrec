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
#include <designators/CKeyValuePair.h>

// Private
#include <Node.h>
#include <UtilityBase.h>

using namespace std;


namespace beliefstate {
  class CExporter : public UtilityBase {
  private:
    list<Node*> m_lstNodes;
    list< pair<string, string> > m_lstDesignatorIDs;
    list< pair<string, string> > m_lstDesignatorEquations;
  
    CKeyValuePair* m_ckvpConfiguration;
  
    void renewUniqueIDsForNode(Node *ndRenew);
  
  protected:
    list< pair<string, string> > m_lstDesignatorEquationTimes;
  
  public:
    CExporter();
    virtual ~CExporter();
  
    CKeyValuePair* configuration();
  
    void addNode(Node *ndAdd);
    list<Node*> nodes();
  
    void clearNodes();
  
    virtual bool runExporter(CKeyValuePair* ckvpConfigurationOverlay);
  
    string generateRandomIdentifier(string strPrefix, unsigned int unLength);
    string generateUniqueID(string strPrefix, unsigned int unLength);
    bool uniqueIDPresent(string strUniqueID);
  
    void renewUniqueIDs();
  
    virtual string nodeIDPrefix(Node* ndInQuestion, string strProposition);
  
    string replaceString(string strOriginal, string strReplaceWhat, string strReplaceBy);
    virtual bool nodeHasValidDetailLevel(Node* ndDisplay);
    virtual bool nodeDisplayable(Node* ndDisplay);
  
    void setDesignatorIDs(list< pair<string, string> > lstDesignatorIDs);
    void setDesignatorEquations(list< pair<string, string> > lstDesignatorEquations);
    void setDesignatorEquationTimes(list< pair<string, string> > lstDesignatorEquationTimes);  
  
    list<string> designatorIDs();
    list<string> parentDesignatorsForID(string strID);
    list<string> successorDesignatorsForID(string strID);
    string equationTimeForSuccessorID(string strID);
  };
}

#endif /* __C_EXPORTER_H__ */
