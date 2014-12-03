#ifndef __OWL_INDIVIDUAL_H__
#define __OWL_INDIVIDUAL_H__


// System
#include <string>
#include <list>


namespace beliefstate {
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
    
  protected:
  public:
    OwlIndividual();
    ~OwlIndividual();
    
    void setID(std::string strID);
    void setType(std::string strType);
    
    void addDataProperty(std::string strTag, std::string strDataType, std::string strContent);
    void addResourceProperty(std::string strTag, std::string strResource);
    void addContentProperty(std::string strTag, std::string strContent);
    
    std::string indent(int nSpaces);
    
    std::string print(int nIndentation = 1, int nIndentationPerLevel = 4);
  };
}


#endif /* __OWL_INDIVIDUAL_H__ */
