#include <Node.h>


namespace beliefstate {
  Node::Node() {
    this->init();
  }

  Node::Node(string strTitle) {
    this->init();
    this->setTitle(strTitle);
  }

  Node::Node(list<CKeyValuePair*> lstDescription) {
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
    m_ckvpMetaInformation = new CKeyValuePair();
    m_ndParent = NULL;
    m_nID = 0;
  }

  void Node::setDescription(list<CKeyValuePair*> lstDescription) {
    for(list<CKeyValuePair*>::iterator itPair = lstDescription.begin();
	itPair != lstDescription.end();
	itPair++) {
      CKeyValuePair *ckvpPair = *itPair;
    
      m_lstDescription.push_back(ckvpPair->copy());
    }
  }

  void Node::clearDescription() {
    for(list<CKeyValuePair*>::iterator itPair = m_lstDescription.begin();
	itPair != m_lstDescription.end();
	itPair++) {
      CKeyValuePair *ckvpPair = *itPair;
      
      delete ckvpPair;
    }
  
    m_lstDescription.clear();
  }

  void Node::clearSubnodes() {
    for(list<Node*>::iterator itNode = m_lstSubnodes.begin();
	itNode != m_lstSubnodes.end();
	itNode++) {
      Node *ndCurrent = *itNode;
    
      delete ndCurrent;
    }
  
    m_lstSubnodes.clear();
  }

  list<CKeyValuePair*> Node::description() {
    return m_lstDescription;
  }

  void Node::setTitle(string strTitle) {
    m_strTitle = strTitle;
  }

  string Node::title() {
    return m_strTitle;
  }

  void Node::addSubnode(Node* ndAdd) {
    ndAdd->setParent(this);
    m_lstSubnodes.push_back(ndAdd);
  }

  list<Node*> Node::subnodes() {
    return m_lstSubnodes;
  }

  void Node::setUniqueID(string strUniqueID) {
    m_strUniqueID = strUniqueID;
  }

  string Node::uniqueID() {
    return m_strUniqueID;
  }

  bool Node::includesUniqueID(string strUniqueID) {
    bool bReturnvalue = false;
  
    if(m_strUniqueID == strUniqueID) {
      bReturnvalue = true;
    } else {
      for(list<Node*>::iterator itNode = m_lstSubnodes.begin();
	  itNode != m_lstSubnodes.end();
	  itNode++) {
	Node *ndCurrent = *itNode;
      
	if(ndCurrent->includesUniqueID(strUniqueID)) {
	  bReturnvalue = true;
	  break;
	}
      }
    }
  
    return bReturnvalue;
  }

  CKeyValuePair *Node::metaInformation() {
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
  
    for(list<Node*>::iterator itNode = m_lstSubnodes.begin();
	itNode != m_lstSubnodes.end();
	itNode++) {
      Node *ndCurrent = *itNode;
    
      nHighestID = max(nHighestID, ndCurrent->highestID());
    }
  
    return nHighestID;
  }

  void Node::setParent(Node* ndParent) {
    m_ndParent = ndParent;
  }

  Node* Node::parent() {
    return m_ndParent;
  }

  void Node::setPrematurelyEnded(bool bPrematurelyEnded) {
    this->metaInformation()->setValue(string("prematurely-ended"), (bPrematurelyEnded ? 1 : 0));
  }

  bool Node::prematurelyEnded() {
    return (this->metaInformation()->floatValue("prematurely-ended") == 0 ? false : true);
  }

  CKeyValuePair* Node::addDescriptionListItem(string strDomain, string strPrefix) {
    CKeyValuePair* ckvpList = this->metaInformation()->addChild(strDomain);
    
    stringstream sts;
    sts << strPrefix << "-";
    sts << ckvpList->children().size();
  
    return ckvpList->addChild(sts.str());
  }
  
  void Node::addImage(string strOrigin, string strFilename) {
    CKeyValuePair* ckvpImage = this->addDescriptionListItem("images", "image");
    
    ckvpImage->setValue(string("origin"), strOrigin);
    ckvpImage->setValue(string("filename"), strFilename);
  }
  
  void Node::addObject(list<CKeyValuePair*> lstDescription) {
    CKeyValuePair* ckvpObject = this->addDescriptionListItem("objects", "object");
    
    for(list<CKeyValuePair*>::iterator itPair = lstDescription.begin();
	itPair != lstDescription.end();
	itPair++) {
      ckvpObject->addChild((*itPair)->copy());
    }
  }

  void Node::addFailure(string strCondition, string strTimestamp) {
    CKeyValuePair* ckvpFailure = this->addDescriptionListItem("failures", "failure");
    
    ckvpFailure->setValue(string("condition"), strCondition);
    ckvpFailure->setValue(string("time-fail"), strTimestamp);
  }
  
  bool Node::hasFailures() {
    CKeyValuePair* ckvpList = this->metaInformation()->childForKey("failures");
    
    if(ckvpList) {
      return (ckvpList->children().size() > 0);
    }
    
    return false;
  }
  
  void Node::addDesignator(string strType, list<CKeyValuePair*> lstDescription, string strUniqueID, string strAnnotation) {
    CKeyValuePair* ckvpDesignator = this->addDescriptionListItem("designators", "designator");
    
    ckvpDesignator->setValue(string("type"), strType);
    // TODO(winkler): Add description here!
    ckvpDesignator->setValue(string("id"), strUniqueID);
    ckvpDesignator->setValue(string("annotation"), strAnnotation);
  }

  void Node::setSuccess(bool bSuccess) {
    this->metaInformation()->setValue(string("success", (bSuccess ? 1 : 0)));
  }

  bool Node::success() {
    return (this->metaInformation()->floatValue(string("success")) == 1 ? true : false);
  }
  
  Node* Node::previousNode() {
    Node* ndPrevious = NULL;
    
    if(m_ndParent != NULL) {
      list<Node*> lstNodes = m_ndParent->subnodes();
      Node* ndLast = NULL;
      
      for(list<Node*>::iterator itNode = lstNodes.begin();
	  itNode != lstNodes.end();
	  itNode++) {
	Node* ndNode = *itNode;
	
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
}
