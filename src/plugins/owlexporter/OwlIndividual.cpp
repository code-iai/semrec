#include <plugins/owlexporter/OwlIndividual.h>


namespace beliefstate {
  OwlIndividual::OwlIndividual() {
    //
  }
  
  OwlIndividual::~OwlIndividual() {
    //
  }

  void OwlIndividual::setID(std::string strID) {
    m_strID = strID;
  }
  
  void OwlIndividual::setType(std::string strType) {
    m_strType = strType;
  }
  
  void OwlIndividual::addDataProperty(std::string strTag, std::string strDataType, std::string strContent) {
    OwlProperty opProperty;
    opProperty.strTag = strTag;
    opProperty.strDataType = strDataType;
    opProperty.strContent = strContent;
    
    m_lstProperties.push_back(opProperty);
  }
  
  void OwlIndividual::addResourceProperty(std::string strTag, std::string strResource) {
    OwlProperty opProperty;
    opProperty.strTag = strTag;
    opProperty.strResource = strResource;
    
    m_lstProperties.push_back(opProperty);
  }
  
  void OwlIndividual::addContentProperty(std::string strTag, std::string strContent) {
    OwlProperty opProperty;
    opProperty.strTag = strTag;
    opProperty.strContent = strContent;
    
    m_lstProperties.push_back(opProperty);
  }
  
  std::string OwlIndividual::indent(int nSpaces) {
    std::string strIndentation = "";
    
    for(int nI = 0; nI < nSpaces; nI++) {
      strIndentation += " ";
    }
    
    return strIndentation;
  }
  
  std::string OwlIndividual::print(int nIndentation, int nIndentationPerLevel) {
    std::string strOwl = "";
    
    if(strOwl != "") {
      strOwl += this->indent(nIndentation * nIndentationPerLevel) + "\n";
    }
    
    strOwl += this->indent(nIndentation * nIndentationPerLevel) +
      "<owl:namedIndividual rdf:about=\"" + m_strID + "\">\n";
    
    strOwl += this->indent((nIndentation + 1) * nIndentationPerLevel) +
      "<rdf:type rdf:resource=\"" + m_strType + "\"/>\n";
    
    for(OwlProperty opProperty : m_lstProperties) {
      strOwl += this->indent((nIndentation + 1) * nIndentationPerLevel);
      strOwl += "<" + opProperty.strTag;
      
      if(opProperty.strDataType != "") {
	strOwl += " rdf:datatype=\"" + opProperty.strDataType + "\">" + opProperty.strContent;
	strOwl += "</" + opProperty.strTag + ">";
      } else if(opProperty.strResource != "") {
	strOwl += " rdf:resource=\"" + opProperty.strResource + "\"/>";
      } else {
	strOwl += ">" + opProperty.strContent;
	strOwl += "</" + opProperty.strTag + ">";
      }
      
      strOwl += "\n";
    }
    
    strOwl += this->indent(nIndentation * nIndentationPerLevel) +
      "</owl:namedIndividual>\n";
    strOwl += this->indent(nIndentation * nIndentationPerLevel) + "\n";
    
    return strOwl;
  }
}
