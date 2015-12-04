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


#include <semrec/plugins/owlexporter/OwlIndividual.h>


namespace semrec {
  std::list<std::string> OwlIndividual::m_lstIssuedProperties;
  std::list<std::string> OwlIndividual::m_lstIssuedTypes;
  std::map<std::string, std::string> OwlIndividual::s_mapStaticResources;
  std::map<std::string, std::string> OwlIndividual::s_mapStaticProperties;
  
  
  OwlIndividual::OwlIndividual() {
    //
  }
  
  OwlIndividual::~OwlIndividual() {
    //
  }
  
  void OwlIndividual::addStaticResource(std::string strKey, std::string strResource) {
    s_mapStaticResources[strKey] = strResource;
  }
  
  void OwlIndividual::addStaticProperty(std::string strKey, std::string strProperty) {
    s_mapStaticProperties[strKey] = strProperty;
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
      "<owl:NamedIndividual rdf:about=\"" + m_strID + "\">\n";
    
    if(m_strType != "") {
      strOwl += this->indent((nIndentation + 1) * nIndentationPerLevel) +
	"<rdf:type rdf:resource=\"" + m_strType + "\"/>\n";
    }
    
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
    
    // Add static resources
    for(std::pair<std::string, std::string> prResource : s_mapStaticResources) {
      strOwl += this->indent((nIndentation + 1) * nIndentationPerLevel);
      strOwl += "<knowrob:" + prResource.first + " rdf:resource=\"" + prResource.second + "\"/>";
    }
    
    // Add static properties
    for(std::pair<std::string, std::string> prProperty : s_mapStaticProperties) {
      strOwl += this->indent((nIndentation + 1) * nIndentationPerLevel);
      strOwl += "<knowrob:" + prProperty.first + " rdf:datatype=\"&xsd;string\">" + prProperty.second + "</knowrob:" + prProperty.first + ">\n";
    }
    
    strOwl += this->indent(nIndentation * nIndentationPerLevel) +
      "</owl:NamedIndividual>\n";
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
