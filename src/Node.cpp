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


#include <Node.h>


namespace semrec {
  Node::Node() {
    this->init();
  }
  
  Node::Node(std::string strTitle) {
    this->init();
    this->setTitle(strTitle);
  }
  
  Node::Node(std::list<KeyValuePair*> lstDescription) {
    this->init();
    this->setPrematurelyEnded(false);
    this->setDescription(lstDescription);
  }
  
  Node::~Node() {
    this->clearSubnodes();
    this->clearDescription();
  }
  
  void Node::init() {
    m_strTitle = "";
    m_ckvpMetaInformation = new KeyValuePair();
    m_ndParent = NULL;
    m_nID = 0;
  }
  
  void Node::setDescription(std::list<KeyValuePair*> lstDescription) {
    for(KeyValuePair* ckvpPair : lstDescription) {
      m_lstDescription.push_back(ckvpPair->copy());
    }
  }

  void Node::clearDescription() {
    for(KeyValuePair* ckvpPair : m_lstDescription) {
      delete ckvpPair;
    }
    
    m_lstDescription.clear();
  }
  
  void Node::clearSubnodes() {
    for(Node* ndCurrent : m_lstSubnodes) {
      delete ndCurrent;
    }
    
    m_lstSubnodes.clear();
  }
  
  std::list<KeyValuePair*> Node::description() {
    return m_lstDescription;
  }
  
  void Node::setTitle(std::string strTitle) {
    m_strTitle = strTitle;
  }
  
  std::string Node::title() {
    return m_strTitle;
  }
  
  void Node::addSubnode(Node* ndAdd) {
    ndAdd->setParent(this);
    m_lstSubnodes.push_back(ndAdd);
  }
  
  std::list<Node*> Node::subnodes() {
    return m_lstSubnodes;
  }
  
  void Node::setUniqueID(std::string strUniqueID) {
    m_strUniqueID = strUniqueID;
  }
  
  std::string Node::uniqueID() {
    return m_strUniqueID;
  }
  
  bool Node::includesUniqueID(std::string strUniqueID) {
    bool bReturnvalue = false;
    
    if(m_strUniqueID == strUniqueID) {
      bReturnvalue = true;
    } else {
      for(Node* ndCurrent : m_lstSubnodes) {
	if(ndCurrent->includesUniqueID(strUniqueID)) {
	  bReturnvalue = true;
	  break;
	}
      }
    }
    
    return bReturnvalue;
  }
  
  KeyValuePair* Node::metaInformation() {
    return m_ckvpMetaInformation;
  }
  
  void Node::setID(int nID) {
    m_nID = nID;
  }
  
  int Node::id() {
    return m_nID;
  }
  
  int Node::highestID() {
    int nHighestID = m_nID;
    
    for(Node* ndCurrent : m_lstSubnodes) {
      nHighestID = std::max(nHighestID, ndCurrent->highestID());
    }
    
    return nHighestID;
  }
  
  void Node::setParent(Node* ndParent) {
    m_ndParent = ndParent;
  }
  
  Node* Node::parent() {
    return m_ndParent;
  }
  
  Node* Node::relativeWithID(int nID, bool bIgnoreSelf) {
    if(this->parent() == NULL) {
      return NULL;
    } else {
      if(not bIgnoreSelf && this->id() == nID) {
	return this;
      } else {
	return this->parent()->relativeWithID(nID);
      }
    }
  }
  
  void Node::setPrematurelyEnded(bool bPrematurelyEnded) {
    this->metaInformation()->setValue(std::string("prematurely-ended"), (bPrematurelyEnded ? 1 : 0));
  }
  
  bool Node::prematurelyEnded() {
    return (this->metaInformation()->floatValue("prematurely-ended") == 0 ? false : true);
  }
  
  KeyValuePair* Node::addDescriptionListItem(std::string strDomain, std::string strPrefix) {
    KeyValuePair* ckvpList = this->metaInformation()->addChild(strDomain);
    
    std::stringstream sts;
    sts << strPrefix << "-";
    sts << ckvpList->children().size();
    
    return ckvpList->addChild(sts.str());
  }
  
  std::string Node::addImage(std::string strOrigin, std::string strFilename, std::string strTimestamp) {
    KeyValuePair* ckvpImage = this->addDescriptionListItem("images", "image");
    
    ckvpImage->setValue(std::string("origin"), strOrigin);
    ckvpImage->setValue(std::string("filename"), strFilename);
    ckvpImage->setValue(std::string("time-capture"), strTimestamp);
    
    return ckvpImage->key();
  }
  
  std::string Node::addObject(std::list<KeyValuePair*> lstDescription) {
    KeyValuePair* ckvpObject = this->addDescriptionListItem("objects", "object");
    
    for(KeyValuePair* ckvpPair : lstDescription) {
      ckvpObject->addChild(ckvpPair->copy());
    }
    
    return ckvpObject->key();
  }
  
  std::string Node::addFailure(std::string strCondition, std::string strTimestamp) {
    KeyValuePair* ckvpFailure = this->addDescriptionListItem("failures", "failure");
    
    ckvpFailure->setValue(std::string("condition"), strCondition);
    ckvpFailure->setValue(std::string("time-fail"), strTimestamp);
    
    return ckvpFailure->key();
  }
  
  std::string Node::catchFailure(std::string strFailureID, Node* ndEmitter, std::string strTimestamp) {
    std::stringstream sts;
    sts << ndEmitter;
    
    KeyValuePair* ckvpFailure = this->addDescriptionListItem("caught_failures", "caught_failure");
    
    ckvpFailure->setValue(std::string("failure-id"), strFailureID);
    ckvpFailure->setValue(std::string("time-catch"), strTimestamp);
    ckvpFailure->setValue(std::string("emitter-id"), sts.str());
    
    m_lstCaughtFailures.push_back(std::make_pair(strFailureID, ndEmitter));
    
    return ckvpFailure->key();
  }
  
  void Node::removeCaughtFailure(std::string strFailureID) {
    for(std::list< std::pair<std::string, Node*> >::iterator itPr = m_lstCaughtFailures.begin();
	itPr != m_lstCaughtFailures.end();
	itPr++) {
      std::pair<std::string, Node*> prCurrent = *itPr;
      
      if(prCurrent.first == strFailureID) {
	m_lstCaughtFailures.erase(itPr);
	break;
      }
    }
    
    KeyValuePair* ckvpCaughtFailures = this->metaInformation()->childForKey("caught_failures");
    if(ckvpCaughtFailures) {
      for(KeyValuePair* ckvpCaughtFailure : ckvpCaughtFailures->children()) {
	std::string strCurrentFailureID = ckvpCaughtFailure->stringValue("failure-id");
	
	if(strCurrentFailureID == strFailureID) {
	  ckvpCaughtFailures->removeChildForKey(ckvpCaughtFailure->key());
	  break;
	}
      }
    }
  }
  
  Node* Node::emitterForCaughtFailure(std::string strFailureID, std::string strEmitterID, std::string strTimestamp) {
    for(std::pair<std::string, Node*> prCurrent : m_lstCaughtFailures) {
      std::stringstream sts;
      sts << prCurrent.second;
      
      if(prCurrent.first == strFailureID && sts.str() == strEmitterID) {
	return prCurrent.second;
      }
    }
    
    return NULL;
  }
  
  bool Node::hasFailures() {
    KeyValuePair* ckvpList = this->metaInformation()->childForKey("failures");
    
    if(ckvpList) {
      return (ckvpList->children().size() > 0);
    }
    
    return false;
  }
  
  void Node::addDesignator(std::string strType, std::list<KeyValuePair*> lstDescription, std::string strUniqueID, std::string strAnnotation) {
    KeyValuePair* ckvpDesignator = this->addDescriptionListItem("designators", "designator");
    
    ckvpDesignator->setValue(std::string("type"), strType);
    ckvpDesignator->setValue(std::string("id"), strUniqueID);
    ckvpDesignator->setValue(std::string("annotation"), strAnnotation);
    
    KeyValuePair* ckvpDescription = ckvpDesignator->addChild("description");
    for(KeyValuePair* ckvpChild : lstDescription) {
      ckvpDescription->addChild(ckvpChild);
    }
  }
  
  void Node::setSuccess(bool bSuccess) {
    this->metaInformation()->setValue(std::string("success"), (bSuccess ? 1 : 0));
  }
  
  void Node::setSuccess(int nSuccess) {
    this->setSuccess(nSuccess == 0 ? false : true);
  }
  
  bool Node::success() {
    return (this->metaInformation()->floatValue(std::string("success")) == 1 ? true : false);
  }
  
  Node* Node::previousNode() {
    Node* ndPrevious = NULL;
    
    if(m_ndParent != NULL) {
      std::list<Node*> lstNodes = m_ndParent->subnodes();
      Node* ndLast = NULL;
      
      for(Node* ndNode : lstNodes) {
	if(ndNode == this) {
	  ndPrevious = ndLast;
	  break;
	} else {
	  ndLast = ndNode;
	}
      }
    }
    
    return ndPrevious;
  }
  
  void Node::ensureProperty(std::string strKey, std::string strDefaultValue) {
    if(this->metaInformation()->childForKey(strKey) == NULL) {
      this->metaInformation()->setValue(strKey, strDefaultValue);
    }
    
    for(Node* ndChild : m_lstSubnodes) {
      ndChild->ensureProperty(strKey, strDefaultValue);
    }
  }
}
