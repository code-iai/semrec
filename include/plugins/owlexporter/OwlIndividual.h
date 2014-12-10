#ifndef __OWL_INDIVIDUAL_H__
#define __OWL_INDIVIDUAL_H__


// System
#include <string>
#include <list>
#include <algorithm>
#include <iostream>


namespace semrec {
  class OwlIndividual {
  public:
    typedef struct _OwlProperty {
      std::string strTag;
      std::string strDataType;
      std::string strResource;
      std::string strContent;
    } OwlProperty;
    
  private:
    std::list<OwlProperty> m_lstProperties;
    std::string m_strID;
    std::string m_strType;
    
    static std::list<std::string> m_lstIssuedProperties;
    static std::list<std::string> m_lstIssuedTypes;
    
  protected:
  public:
    OwlIndividual();
    ~OwlIndividual();
    
    void setID(std::string strID);
    void setType(std::string strType);
    
    void addDataProperty(std::string strTag, std::string strDataType, std::string strContent);
    void addResourceProperty(std::string strTag, std::string strResource);
    void addContentProperty(std::string strTag, std::string strContent);
    
    void issueProperty(std::string strProperty);
    void issueType(std::string strType);
    
    std::string indent(int nSpaces);
    
    std::string print(int nIndentation = 1, int nIndentationPerLevel = 4);
    
    static void resetIssuedProperties();
    static void resetIssuedTypes();
    static void resetIssuedInformation();
    
    static std::list<std::string> issuedProperties();
    static std::list<std::string> issuedTypes();
  };
}


#endif /* __OWL_INDIVIDUAL_H__ */
