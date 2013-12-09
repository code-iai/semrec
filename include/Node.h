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
    void addImage(string strOrigin, string strFilename);
    void addObject(list<CKeyValuePair*> lstDescription);
    void addFailure(string strCondition, string strTimestamp);
    void addDesignator(string strType, list<CKeyValuePair*> lstDescription, string strUniqueID, string strAnnotation = "");
  
    void setSuccess(bool bSuccess);
    bool success();
    
    Node* previousNode();
  };
}


#endif /* __NODE_H__ */
