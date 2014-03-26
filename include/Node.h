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


#ifndef __NODE_H__
#define __NODE_H__


// System
#include <string>

// Other
#include <designators/CKeyValuePair.h>

using namespace std;


namespace beliefstate {
  class Node {
  private:
    string m_strTitle;
    string m_strUniqueID;
    int m_nID;
  
    Node *m_ndParent;
    list<CKeyValuePair*> m_lstDescription;
    list<Node*> m_lstSubnodes;
    CKeyValuePair *m_ckvpMetaInformation;
    
    list< pair<string, Node*> > m_lstCaughtFailures;
    
    void init();
    
    void clearDescription();
    void clearSubnodes();
    
  public:
    Node();
    Node(string strTitle);
    Node(list<CKeyValuePair*> lstDescription);
    ~Node();
    
    void setDescription(list<CKeyValuePair*> lstDescription);
    list<CKeyValuePair*> description();
  
    void setTitle(string strTitle);
    string title();
  
    void setParent(Node* ndParent);
    Node* parent();
    Node* relativeWithID(int nID);
  
    void addSubnode(Node* ndAdd);
    list<Node*> subnodes();
  
    void setUniqueID(string strUniqueID);
    string uniqueID();
  
    void setID(int nID);
    int id();
  
    int highestID();
  
    bool includesUniqueID(string strUniqueID);
  
    CKeyValuePair *metaInformation();
  
    void setPrematurelyEnded(bool bPrematurelyEnded);
    bool prematurelyEnded();
    
    CKeyValuePair* addDescriptionListItem(string strDomain, string strPrefix);
    string addImage(string strOrigin, string strFilename, string strTimestamp);
    string addObject(list<CKeyValuePair*> lstDescription);
    string addFailure(string strCondition, string strTimestamp);
    string catchFailure(pair<string, Node*> prCaughtFailure, string strTimestamp);
    Node* emitterForCaughtFailure(string strFailureID, string strTimestamp, int nOffset = 0);
    bool hasFailures();
    void addDesignator(string strType, list<CKeyValuePair*> lstDescription, string strUniqueID, string strAnnotation = "");
    
    void setSuccess(bool bSuccess);
    bool success();
    
    Node* previousNode();
    
    void ensureProperty(string strKey, string strDefaultValue);
  };
}


#endif /* __NODE_H__ */
