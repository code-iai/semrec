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


namespace beliefstate {
  class Node {
  private:
    std::string m_strTitle;
    std::string m_strUniqueID;
    int m_nID;
  
    Node* m_ndParent;
    std::list<CKeyValuePair*> m_lstDescription;
    std::list<Node*> m_lstSubnodes;
    CKeyValuePair* m_ckvpMetaInformation;
    
    std::list< std::pair<std::string, Node*> > m_lstCaughtFailures;
    
    void init();
    
    void clearDescription();
    void clearSubnodes();
    
  public:
    Node();
    Node(std::string strTitle);
    Node(std::list<CKeyValuePair*> lstDescription);
    ~Node();
    
    void setDescription(std::list<CKeyValuePair*> lstDescription);
    std::list<CKeyValuePair*> description();
    
    void setTitle(std::string strTitle);
    std::string title();
    
    void setParent(Node* ndParent);
    Node* parent();
    Node* relativeWithID(int nID, bool bIgnoreSelf = false);
    
    void addSubnode(Node* ndAdd);
    std::list<Node*> subnodes();
    
    void setUniqueID(std::string strUniqueID);
    std::string uniqueID();
    
    void setID(int nID);
    int id();
    
    int highestID();
    
    bool includesUniqueID(std::string strUniqueID);
    
    CKeyValuePair* metaInformation();
    
    void setPrematurelyEnded(bool bPrematurelyEnded);
    bool prematurelyEnded();
    
    CKeyValuePair* addDescriptionListItem(std::string strDomain, std::string strPrefix);
    std::string addImage(std::string strOrigin, std::string strFilename, std::string strTimestamp);
    std::string addObject(std::list<CKeyValuePair*> lstDescription);
    std::string addFailure(std::string strCondition, std::string strTimestamp);
    std::string catchFailure(std::string strFailureID, Node* ndEmitter, std::string strTimestamp);
    void removeCaughtFailure(std::string strFailureID);
    Node* emitterForCaughtFailure(std::string strFailureID, std::string strEmitterID, std::string strTimestamp);
    bool hasFailures();
    void addDesignator(std::string strType, std::list<CKeyValuePair*> lstDescription, std::string strUniqueID, std::string strAnnotation = "");
    
    void setSuccess(bool bSuccess);
    bool success();
    
    Node* previousNode();
    
    void ensureProperty(std::string strKey, std::string strDefaultValue);
  };
}


#endif /* __NODE_H__ */
