#include <plugins/owlexporter/OwlIndividual.h>


namespace beliefstate {
  std::list<std::string> OwlIndividual::m_lstIssuedProperties;
  std::list<std::string> OwlIndividual::m_lstIssuedTypes;
  
  
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
    
    this->issueType(strType);
  }
  
  void OwlIndividual::addDataProperty(std::string strTag, std::string strDataType, std::string strContent) {
    OwlProperty opProperty;
    opProperty.strTag = strTag;
    opProperty.strDataType = strDataType;
    opProperty.strContent = strContent;
    
    m_lstProperties.push_back(opProperty);
    
    this->issueProperty(strTag);
  }
  
  void OwlIndividual::addResourceProperty(std::string strTag, std::string strResource) {
    OwlProperty opProperty;
    opProperty.strTag = strTag;
    opProperty.strResource = strResource;
    
    m_lstProperties.push_back(opProperty);
    
    this->issueProperty(strTag);
  }
  
  void OwlIndividual::addContentProperty(std::string strTag, std::string strContent) {
    OwlProperty opProperty;
    opProperty.strTag = strTag;
    opProperty.strContent = strContent;
    
    m_lstProperties.push_back(opProperty);
    
    this->issueProperty(strTag);
  }
  
  void OwlIndividual::issueProperty(std::string strProperty) {
    std::string strPropertyFormatted = strProperty;
    
    size_t posColon = strPropertyFormatted.find(":");
    if(posColon != std::string::npos) {
      // Namespace found, convert it into short form
      strPropertyFormatted = "&" + strPropertyFormatted.substr(0, posColon) + ";" + strPropertyFormatted.substr(posColon + 1);
    }
    
    if(std::find(m_lstIssuedProperties.begin(), m_lstIssuedProperties.end(), strPropertyFormatted) == m_lstIssuedProperties.end()) {
      m_lstIssuedProperties.push_back(strPropertyFormatted);
    }
  }
  
  void OwlIndividual::issueType(std::string strType) {
    if(std::find(m_lstIssuedTypes.begin(), m_lstIssuedTypes.end(), strType) == m_lstIssuedTypes.end()) {
      m_lstIssuedTypes.push_back(strType);
    }
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
  
  void OwlIndividual::resetIssuedProperties() {
    m_lstIssuedProperties.clear();
  }
  
  void OwlIndividual::resetIssuedTypes() {
    m_lstIssuedTypes.clear();
  }
  
  void OwlIndividual::resetIssuedInformation() {
    OwlIndividual::resetIssuedProperties();
    OwlIndividual::resetIssuedTypes();
  }
  
  std::list<std::string> OwlIndividual::issuedProperties() {
    return m_lstIssuedProperties;
  }
  
  std::list<std::string> OwlIndividual::issuedTypes() {
    return m_lstIssuedTypes;
  }
}
