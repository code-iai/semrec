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


#include <plugins/owlexporter/CExporterOwl.h>


namespace beliefstate {
  CExporterOwl::CExporterOwl() {
    m_strPropertyNamespace = "";
    m_strDefaultAnnotation = "";
    
    this->setMessagePrefixLabel("owl-exporter-aux");
  }
  
  CExporterOwl::~CExporterOwl() {
  }
  
  void CExporterOwl::setMetaData(std::map<std::string, std::string> mapMetaData) {
    m_mapMetaData = mapMetaData;
  }
  
  bool CExporterOwl::loadSemanticsDescriptorFile(std::string strFilepath) {
    if(this->fileExists(strFilepath)) {
      libconfig::Config cfgConfig;
      
      try {
	cfgConfig.readFile(strFilepath.c_str());
	
	if(cfgConfig.exists("condition-mappings")) {
	  libconfig::Setting &sConditionMappings = cfgConfig.lookup("condition-mappings");
	  
	  if(sConditionMappings.exists("mappings")) {
	    libconfig::Setting &sMappings = sConditionMappings["mappings"];
	    
	    for(int nI = 0; nI < sMappings.getLength(); nI++) {
	      std::string strTo;
	      
	      if(sMappings[nI].lookupValue("to", strTo)) {
		if(sMappings[nI].exists("from")) {
		  libconfig::Setting &sFrom = sMappings[nI]["from"];
		  
		  for(int nJ = 0; nJ < sFrom.getLength(); nJ++) {
		    std::string strFrom = sFrom[nJ];
		    
		    m_lstFailureMapping.push_back(make_pair(strFrom, strTo));
		  }
		}
	      } else {
		this->warn("Condition mapping without 'to' field. Ignoring.");
	      }
	    }
	  }
	}
	
	m_lstDefinedProperties.clear();
	m_lstAnnotationPurposeMapping.clear();
	
	if(cfgConfig.exists("structure")) {
	  libconfig::Setting &sStructure = cfgConfig.lookup("structure");
	  
	  m_strPropertyNamespace = "";
	  if(sStructure.exists("property-namespace")) {
	    sStructure.lookupValue("property-namespace", m_strPropertyNamespace);
	  }
	  
	  if(m_strPropertyNamespace == "") {
	    this->warn("You didn't specify the 'structure/property-namespace' parameter on the semantics descriptor file. Your OWL classes will have no namespace prepended. Is this intended?");
	  }
	  
	  if(sStructure.exists("defined-properties")) {
	    libconfig::Setting &sDefinedProperties = sStructure["defined-properties"];
	    
	    for(int nI = 0; nI < sDefinedProperties.getLength(); nI++) {
	      std::string strProperty = sDefinedProperties[nI];
	      m_lstDefinedProperties.push_back(m_strPropertyNamespace + strProperty);
	    }
	  }
	  
	  m_strDefaultAnnotation = "";
	  if(sStructure.exists("default-annotation-purpose")) {
	    sStructure.lookupValue("default-annotation-purpose", m_strDefaultAnnotation);
	  }
	  
	  if(m_strDefaultAnnotation == "") {
	    this->warn("You didn't specify the 'structure/default-annotation-purpose' parameter on the semantics descriptor file. Your designator attachments without a defined annotation will be empty and produce a faulty OWL file. Is this intended?");
	  }
	  
	  if(sStructure.exists("annotation-purposes")) {
	    libconfig::Setting &sPurposes = sStructure["annotation-purposes"];
	    
	    for(int nI = 0; nI < sPurposes.getLength(); nI++) {
	      libconfig::Setting &sPurpose = sPurposes[nI];
	      
	      std::string strFrom;
	      std::string strTo;
	      
	      sPurpose.lookupValue("from", strFrom);
	      sPurpose.lookupValue("to", strTo);
	      
	      if(strFrom != "" && strTo != "") {
		m_lstAnnotationPurposeMapping.push_back(make_pair(strFrom, strTo));
	      } else {
		this->warn("Invalid annotation purpose mapping: '" + strFrom + "' -> '" + strTo + "'. Discarding.");
	      }
	    }
	  }
	}
	
	return true;
      } catch(libconfig::ParseException e) {
        std::stringstream sts;
        sts << e.getLine();
	
        this->fail("Error while parsing semantics descriptor file '" + strFilepath + "': " + e.getError() + ", on line " + sts.str());
      } catch(...) {
	this->fail("Undefined error while parsing semantics descriptor file '" + strFilepath + "'");
      }
    } else {
      this->fail("Semantics descriptor file not found: '" + strFilepath + "'.");
    }
    
    return false;
  }
  
  void CExporterOwl::prepareEntities(std::string strNamespaceID, std::string strNamespace) {
    m_lstEntities.clear();
  
    this->addEntity("owl", "http://www.w3.org/2002/07/owl#");
    this->addEntity("xsd", "http://www.w3.org/2001/XMLSchema#");
    this->addEntity("knowrob", "http://knowrob.org/kb/knowrob.owl#");
    this->addEntity("rdfs", "http://www.w3.org/2000/01/rdf-schema#");
    this->addEntity("rdf", "http://www.w3.org/1999/02/22-rdf-syntax-ns#");
    this->addEntity(strNamespaceID, strNamespace + "#");
  }

  void CExporterOwl::addEntity(std::string strNickname, std::string strNamespace) {
    m_lstEntities.push_back(make_pair(strNickname, strNamespace));
  }

  std::string CExporterOwl::generateDocTypeBlock() {
    std::string strDot = "<!DOCTYPE rdf:RDF [\n";
    
    for(std::pair<std::string, std::string> prEntity : m_lstEntities) {
      strDot += "    <!ENTITY " + prEntity.first + " \"" + prEntity.second + "\" >\n";
    }
  
    strDot += "]>\n\n";
    
    return strDot;
  }

  std::string CExporterOwl::generateXMLNSBlock(std::string strNamespace) {
    std::string strDot = "<rdf:RDF xmlns=\"" + strNamespace + "#\"\n";
    strDot += "     xml:base=\"" + strNamespace + "\"\n";
    
    for(std::list< std::pair<std::string, std::string> >::iterator itPair = m_lstEntities.begin();
	itPair != m_lstEntities.end();
	itPair++) {
      std::pair<std::string, std::string> prEntity = *itPair;
      
      if(itPair != m_lstEntities.begin()) {
	strDot += "\n";
      }

      strDot += "     xmlns:" + prEntity.first + "=\"" + prEntity.second + "\"";
    }

    strDot += ">\n\n";
    return strDot;
  }
  
  std::string CExporterOwl::generateOwlImports(std::string strNamespace) {
    std::string strDot = "";
    std::string strImportNamespace = "package://knowrob_common/owl/knowrob.owl";
    
    strDot += "    <owl:Ontology rdf:about=\"" + strNamespace + "\">\n";
    strDot += "        <owl:imports rdf:resource=\"" + strImportNamespace + "\"/>\n";
    strDot += "    </owl:Ontology>\n\n";
    
    return strDot;
  }

  std::string CExporterOwl::generatePropertyDefinitions() {
    std::string strDot = "    <!-- Property Definitions -->\n\n";
    
    for(std::string strProperty : m_lstDefinedProperties) {
      strDot += "    <owl:ObjectProperty rdf:about=\"" + strProperty + "\"/>\n\n";
    }
    
    return strDot;
  }
  
  std::list<std::string> CExporterOwl::gatherClassesForNodes(std::list<Node*> lstNodes) {
    std::list<std::string> lstClasses;
    
    for(Node* ndCurrent : lstNodes) {
      if(ndCurrent) {
	std::list<std::string> lstClassesSubnodes = this->gatherClassesForNodes(ndCurrent->subnodes());
	
	if(ndCurrent->metaInformation()->stringValue("class") != "") {
	  lstClassesSubnodes.push_back(ndCurrent->metaInformation()->stringValue("classnamespace") + ndCurrent->metaInformation()->stringValue("class"));
	} else {
	  lstClassesSubnodes.push_back(this->owlClassForNode(ndCurrent));
	}
	
	for(std::string strClassSubnode : lstClassesSubnodes) {
	  bool bExists = false;
	  
	  for(std::string strClassNode : lstClasses) {
	    if(strClassSubnode == strClassNode) {
	      bExists = true;
	      break;
	    }
	  }
	  
	  if(!bExists) {
	    lstClasses.push_back(strClassSubnode);
	  }
	}
      } else {
	this->fail("Class for invalid node requested!");
      }
    }
    
    return lstClasses;
  }
  
  std::list<std::string> CExporterOwl::gatherTimepointsForNodes(std::list<Node*> lstNodes) {
    std::list<std::string> lstTimepoints;
    
    for(Node* ndCurrent : lstNodes) {
      if(ndCurrent) {
	// Gather node timepoints
	std::list<std::string> lstTimepointsSubnodes = this->gatherTimepointsForNodes(ndCurrent->subnodes());
	lstTimepointsSubnodes.push_back(ndCurrent->metaInformation()->stringValue("time-start"));
	lstTimepointsSubnodes.push_back(ndCurrent->metaInformation()->stringValue("time-end"));
	
	// Gather failure timepoints
	CKeyValuePair *ckvpFailures = ndCurrent->metaInformation()->childForKey("failures");
	
	if(ckvpFailures) {
	  std::list<CKeyValuePair*> lstFailures = ckvpFailures->children();
	  
	  unsigned int unIndex = 0;
	  for(CKeyValuePair* ckvpFailure : lstFailures) {
	    lstTimepointsSubnodes.push_back(ckvpFailure->stringValue("time-fail"));
	  }
	}
	
	// Gather image timepoints
	CKeyValuePair *ckvpImages = ndCurrent->metaInformation()->childForKey("images");
	
	if(ckvpImages) {
	  std::list<CKeyValuePair*> lstImages = ckvpImages->children();
	  
	  unsigned int unIndex = 0;
	  for(CKeyValuePair* ckvpImage : lstImages) {
	    lstTimepointsSubnodes.push_back(ckvpImage->stringValue("time-capture"));
	  }
	}
	
	// Gather designator equation timepoints
	for(std::pair<std::string, std::string> prPair : m_lstDesignatorEquationTimes) {
	  lstTimepointsSubnodes.push_back(prPair.second);
	}
	
	// Unify all timepoints
	for(std::string strTimepointSubnode : lstTimepointsSubnodes) {
	  bool bExists = false;
	  
	  for(std::string strTimepointNode : lstTimepoints) {
	    if(strTimepointSubnode == strTimepointNode) {
	      bExists = true;
	      break;
	    }
	  }
	  
	  if(!bExists) {
	    lstTimepoints.push_back(strTimepointSubnode);
	  }
	}
      } else {
	this->fail("Timepoints for invalid node requested!");
      }
    }
    
    return lstTimepoints;
  }
  
  std::string CExporterOwl::generateClassDefinitions() {
    std::string strDot = "    <!-- Class Definitions -->\n\n";
    
    std::list<std::string> lstClasses = this->gatherClassesForNodes(this->nodes());
    lstClasses.push_back("&knowrob;TimePoint");
    
    for(std::string strClass : lstClasses) {
      strDot += "    <owl:Class rdf:about=\"" + strClass + "\"/>\n\n";
    }
    
    return strDot;
  }
  
  std::string CExporterOwl::nodeIDPrefix(Node* ndInQuestion, std::string strProposition) {
    std::string strPrefix = CExporter::nodeIDPrefix(ndInQuestion, strProposition);
    std::string strOwlClass = this->owlClassForNode(ndInQuestion, true);
    
    if(strPrefix == "") {
      strPrefix = strOwlClass + "_";
    } else if(strPrefix == strProposition) {
      if(strOwlClass != "") {
	strPrefix = strOwlClass + "_";
      }
    }
    
    return strPrefix;
  }
  
  std::string CExporterOwl::generateEventIndividualsForNodes(std::list<Node*> lstNodes, std::string strNamespace) {
    std::string strDot = "";
    
    Node* ndLastDisplayed = NULL;
    for(std::list<Node*>::iterator itNode = lstNodes.begin();
	itNode != lstNodes.end();
	itNode++) {
      Node* ndCurrent = *itNode;
      
      if(ndCurrent) {
	if(this->nodeDisplayable(ndCurrent)) {
	  std::string strOwlClass;
	  
	  if(ndCurrent->metaInformation()->stringValue("class") != "") {
	    strOwlClass = ndCurrent->metaInformation()->stringValue("classnamespace") + ndCurrent->metaInformation()->stringValue("class");
	  } else {
	    strOwlClass = this->owlClassForNode(ndCurrent);
	  }
	  
	  // NOTE: The position of this line causes that newer nodes
	  // are always further up in the log. If this is not
	  // intended, move it below the generation code for the
	  // current node.
	  strDot += this->generateEventIndividualsForNodes(ndCurrent->subnodes(), strNamespace);
	  
	  // NOTE: Here, the generation code the the current node
	  // begins.
	  strDot += "    <owl:namedIndividual rdf:about=\"&" + strNamespace + ";" + ndCurrent->uniqueID() + "\">\n";
	  strDot += "        <rdf:type rdf:resource=\"" + strOwlClass + "\"/>\n";
	  strDot += "        <knowrob:taskContext rdf:datatype=\"&xsd;string\">" + ndCurrent->title() + "</knowrob:taskContext>\n";
	  strDot += "        <knowrob:taskSuccess rdf:datatype=\"&xsd;boolean\">" + (ndCurrent->success() ? string("true") : string("false")) + "</knowrob:taskSuccess>\n";
	  strDot += "        <knowrob:startTime rdf:resource=\"&" + strNamespace + ";timepoint_" + ndCurrent->metaInformation()->stringValue("time-start") + "\"/>\n";
	  strDot += "        <knowrob:endTime rdf:resource=\"&" + strNamespace + ";timepoint_" + ndCurrent->metaInformation()->stringValue("time-end") + "\"/>\n";
	  
	  if(ndCurrent->title() == "GOAL-ACHIEVE") {
	    std::list<CKeyValuePair*> lstDescription = ndCurrent->description();
	    std::string strPattern = "";
	    
	    for(CKeyValuePair* ckvpNow : lstDescription) {
	      if(ckvpNow->key() == "PATTERN") {
		strPattern = ckvpNow->stringValue();
		break;
	      }
	    }
	    
	    if(strPattern != "") {
	      strDot += "        <knowrob:goalContext rdf:datatype=\"&xsd;string\">" + strPattern + "</knowrob:goalContext>\n";
	    }
	  }
	  
	  std::list<Node*> lstSubnodes = ndCurrent->subnodes();
	  for(Node* ndSubnode : lstSubnodes) {
	    if(this->nodeDisplayable(ndSubnode)) {
	      strDot += "        <knowrob:subAction rdf:resource=\"&" + strNamespace + ";" + ndSubnode->uniqueID() + "\"/>\n";
	    }
	  }
	  
	  if(ndLastDisplayed) {
	    strDot += "        <knowrob:previousAction rdf:resource=\"&" + strNamespace + ";" + ndLastDisplayed->uniqueID() + "\"/>\n";
	  }
	  
	  std::list<Node*>::iterator itPostEvent = itNode;
	  itPostEvent++;
	  while(itPostEvent != lstNodes.end()) {
	    if(this->nodeDisplayable(*itPostEvent)) {
	      strDot += "        <knowrob:nextAction rdf:resource=\"&" + strNamespace + ";" + (*itPostEvent)->uniqueID() + "\"/>\n";
	      break;
	    }
	    
	    itPostEvent++;
	  }
	  
	  // Object references here.
	  CKeyValuePair* ckvpObjects = ndCurrent->metaInformation()->childForKey("objects");
	  
	  if(ckvpObjects) {
	    std::list<CKeyValuePair*> lstObjects = ckvpObjects->children();
	    
	    unsigned int unIndex = 0;
	    for(CKeyValuePair* ckvpObject : lstObjects) {
	      std::string strDefClass = ckvpObject->stringValue("_class");
	      std::string strDefClassNamespace = ckvpObject->stringValue("_classnamespace");
	      std::string strDefProperty = ckvpObject->stringValue("_property");
	      
	      if(strDefClass == "") {
		strDefClass = "object";
	      }
	      
	      if(strDefProperty == "") {
		if(strOwlClass == "&knowrob;VisualPerception") {
		  strDefProperty = "knowrob:detectedObject";
		} else {
		  strDefProperty = "knowrob:objectActedOn";
		}
	      }
	      
	      if(strDefClassNamespace == "") {
		strDefClassNamespace = "&" + strNamespace + ";";
	      }
	      
	      std::string strObjectID = strDefClass + "_" + ckvpObject->stringValue("__id");
	      strDot += "        <" + strDefProperty + " rdf:resource=\"" + strDefClassNamespace + strObjectID +"\"/>\n";
	    }
	  }
	  
	  // Image references here.
	  CKeyValuePair *ckvpImages = ndCurrent->metaInformation()->childForKey("images");
	  
	  if(ckvpImages) {
	    std::list<CKeyValuePair*> lstImages = ckvpImages->children();
	    
	    unsigned int unIndex = 0;
	    for(CKeyValuePair* ckvpImage : lstImages) {
	      std::stringstream sts;
	      sts << ndCurrent->uniqueID() << "_image_" << unIndex;
	      
	      strDot += "        <knowrob:capturedImage rdf:resource=\"&" + strNamespace + ";" + sts.str() +"\"/>\n";
	    }
	  }
	  
	  // Failure references here.
	  CKeyValuePair *ckvpFailures = ndCurrent->metaInformation()->childForKey("failures");
	  
	  if(ckvpFailures) {
	    std::list<CKeyValuePair*> lstFailures = ckvpFailures->children();
	    
	    unsigned int unIndex = 0;
	    for(CKeyValuePair* ckvpFailure : lstFailures) {
	      std::stringstream sts;
	      sts << ndCurrent->uniqueID() << "_failure_" << unIndex;
	      strDot += "        <knowrob:eventFailure rdf:resource=\"&" + strNamespace + ";" + sts.str() + "\"/>\n";
	      m_nThrowAndCatchFailureCounter++;
	    }
	  }
	  
	  // Caught failure here.
	  CKeyValuePair *ckvpCaughtFailures = ndCurrent->metaInformation()->childForKey("caught_failures");
	  
	  if(ckvpCaughtFailures) {
	    std::list<CKeyValuePair*> lstCaughtFailures = ckvpCaughtFailures->children();
	    
	    unsigned int unIndex = 0;
	    for(CKeyValuePair* ckvpCaughtFailure : lstCaughtFailures) {
	      Node* ndFailureEmitter = ndCurrent->emitterForCaughtFailure(ckvpCaughtFailure->stringValue("failure-id"), ckvpCaughtFailure->stringValue("emitter-id"), ckvpCaughtFailure->stringValue("time-catch"));
	      
	      if(ndFailureEmitter) {
		std::string strCaughtFailure = ndFailureEmitter->uniqueID() + "_" + ckvpCaughtFailure->stringValue("failure-id");
		m_nThrowAndCatchFailureCounter--;
		strDot += "        <knowrob:caughtFailure rdf:resource=\"&" + strNamespace + ";" + strCaughtFailure + "\"/>\n";
	      } else {
		this->warn("No emitter for failure '" + ckvpCaughtFailure->stringValue("failure-id") + "'.");
	      }
	    }
	  }
	  
	  // Designator references here.
	  CKeyValuePair *ckvpDesignators = ndCurrent->metaInformation()->childForKey("designators");
	  
	  if(ckvpDesignators) {
	    std::list<CKeyValuePair*> lstDesignators = ckvpDesignators->children();
	    
	    unsigned int unIndex = 0;
	    for(CKeyValuePair* ckvpDesignator : lstDesignators) {
	      std::string strAnnotation = ckvpDesignator->stringValue("annotation");
	      std::string strDesigID = ckvpDesignator->stringValue("id");
	      
	      if(strAnnotation == "parameter-annotation") { // Special treatment for parameter annotations
		CKeyValuePair* ckvpChildren = ckvpDesignator->childForKey("description");
		
		if(ckvpChildren) {
		  for(CKeyValuePair* ckvpChild : ckvpChildren->children()) {
		    std::string strKey = ckvpChild->key();
		    std::stringstream sts;
		    bool bSupportedType = true;
		    
		    switch(ckvpChild->type()) {
		    case FLOAT:
		      sts << ckvpChild->floatValue();
		      break;
		      
		    case STRING:
		      sts << ckvpChild->stringValue();
		      break;
		      
		    default:
		      this->warn("Unsupported parameter annotation type for key '" + strKey + "'.");
		      bSupportedType = false;
		      break;
		    }
		    
		    if(bSupportedType) {
		      if(find(m_lstAnnotatedParameters.begin(), m_lstAnnotatedParameters.end(), strKey) == m_lstAnnotatedParameters.end()) {
			m_lstAnnotatedParameters.push_back(strKey);
		      }
		      
		      strDot += "        <knowrob:" + strKey + ">" + sts.str() + "</knowrob:" + strKey + ">\n";
		      strDot += "        <knowrob:annotatedParameterType rdf:datatype=\"&xsd;string\">" + strKey + "</knowrob:annotatedParameterType>\n";
		    }
		  }
		}
	      }
	      
	      std::string strDesigPurpose = this->resolveDesignatorAnnotationTagName(strAnnotation);
	      strDot += "        <knowrob:" + strDesigPurpose + " rdf:resource=\"&" + strNamespace + ";" + strDesigID + "\"/>\n";
	    }
	  }
	  
	  strDot += "    </owl:namedIndividual>\n\n";
	  ndLastDisplayed = ndCurrent;
	}
      } else {
	this->fail("Generation of event individual for node with invalid content requested!");
      }
    }
    
    return strDot;
  }
  
  std::string CExporterOwl::resolveDesignatorAnnotationTagName(std::string strAnnotation) {
    std::string strDesigPurpose = "";
    
    for(std::pair<std::string, std::string> prMapping : m_lstAnnotationPurposeMapping) {
      if(prMapping.first == strAnnotation) {
	strDesigPurpose = prMapping.second;
	break;
      }
    }
    
    if(strDesigPurpose == "") {
      strDesigPurpose = m_strDefaultAnnotation;
    }
    
    return strDesigPurpose;
  }
  
  std::string CExporterOwl::generateEventIndividuals(std::string strNamespace) {
    std::string strDot = "    <!-- Event Individuals -->\n\n";
    strDot += this->generateEventIndividualsForNodes(this->nodes(), strNamespace);
    
    return strDot;
  }
  
  std::string CExporterOwl::owlClassForObject(CKeyValuePair *ckvpObject) {
    return "&knowrob;HumanScaleObject";
  }
  
  std::string CExporterOwl::failureClassForCondition(std::string strCondition) {
    std::string strFailureClass = "CRAMFailure";
    
    for(std::pair<std::string, std::string> prCurrent : m_lstFailureMapping) {
      if(strCondition == prCurrent.first) {
	strFailureClass = prCurrent.second;
	break;
      }
    }
    
    return strFailureClass;
  }
  
  std::string CExporterOwl::generateFailureIndividualsForNodes(std::list<Node*> lstNodes, std::string strNamespace) {
    std::string strDot = "";
    
    for(Node* ndCurrent : lstNodes) {
      if(ndCurrent) {
	CKeyValuePair *ckvpFailures = ndCurrent->metaInformation()->childForKey("failures");
	
	if(ckvpFailures) {
	  std::list<CKeyValuePair*> lstFailures = ckvpFailures->children();
	  
	  unsigned int unIndex = 0;
	  for(CKeyValuePair* ckvpFailure : lstFailures) {
	    std::stringstream sts;
	    sts << ndCurrent->uniqueID() << "_failure_" << unIndex;
	    
	    std::string strCondition = ckvpFailure->stringValue("condition");
	    std::string strTimestamp = ckvpFailure->stringValue("time-fail");
	    
	    std::string strFailureClass = this->failureClassForCondition(strCondition);
	    
	    strDot += "    <owl:namedIndividual rdf:about=\"&" + strNamespace + ";" + sts.str() + "\">\n";
	    strDot += "        <rdf:type rdf:resource=\"&knowrob;" + strFailureClass + "\"/>\n";
	    strDot += "        <rdfs:label rdf:datatype=\"&xsd;string\">" + this->owlEscapeString(strCondition) + "</rdfs:label>\n";
	    strDot += "        <knowrob:startTime rdf:resource=\"&" + strNamespace + ";timepoint_" + strTimestamp + "\"/>\n";
	    strDot += "    </owl:namedIndividual>\n\n";
	  }
	}
	
	strDot += this->generateFailureIndividualsForNodes(ndCurrent->subnodes(), strNamespace);
      } else {
	this->fail("Failure node with invalid content!");
      }
    }
    
    return strDot;
  }
  
  std::string CExporterOwl::generateObjectIndividualsForNodes(std::list<Node*> lstNodes, std::string strNamespace) {
    std::string strDot = "";
    
    for(Node* ndCurrent : lstNodes) {
      if(ndCurrent) {
	CKeyValuePair* ckvpObjects = ndCurrent->metaInformation()->childForKey("objects");
	
	if(ckvpObjects) {
	  std::list<CKeyValuePair*> lstObjects = ckvpObjects->children();
	  
	  unsigned int unIndex = 0;
	  for(CKeyValuePair* ckvpObject : lstObjects) {
	    std::string strDesignatorID = ckvpObject->stringValue("__id");
	    
	    std::string strDefClass = ckvpObject->stringValue("_class");
	    std::string strDefClassNamespace = ckvpObject->stringValue("_classnamespace");
	    
	    std::string strOwlClass = strDefClass;
	    if(strOwlClass == "") {
	      strOwlClass = this->owlClassForObject(ckvpObject);
	    } else {
	      strOwlClass = strDefClassNamespace + strDefClass;
	    }
	    
	    if(strDefClass == "") {
	      strDefClass = "object";
	    }
	    
	    if(strDefClassNamespace == "") {
	      strDefClassNamespace = "&" + strNamespace + ";";
	    }
	    
	    std::string strObjectID = strDefClass + "_" + ckvpObject->stringValue("__id");
	    
	    if(find(m_lstExportedObjectIndividuals.begin(), m_lstExportedObjectIndividuals.end(), strObjectID) == m_lstExportedObjectIndividuals.end()) {
	      strDot += "    <owl:namedIndividual rdf:about=\"" + strDefClassNamespace + strObjectID + "\">\n";
	      strDot += "        <knowrob:designator rdf:resource=\"&" + strNamespace + ";" + strDesignatorID + "\"/>\n";
	      strDot += "        <rdf:type rdf:resource=\"" + strOwlClass + "\"/>\n";
	      strDot += "    </owl:namedIndividual>\n\n";
	      
	      m_lstExportedObjectIndividuals.push_back(strObjectID);
	    }
	  }
	}
	
	strDot += this->generateObjectIndividualsForNodes(ndCurrent->subnodes(), strNamespace);
      } else {
	this->fail("Generation of object individual for node with invalid content requested!");
      }
    }
    
    return strDot;
  }
  
  std::string CExporterOwl::generateObjectIndividuals(std::string strNamespace) {
    std::string strDot = "    <!-- Object Individuals -->\n\n";
    strDot += this->generateObjectIndividualsForNodes(this->nodes(), strNamespace);
    
    return strDot;
  }
  
  std::string CExporterOwl::generateImageIndividualsForNodes(std::list<Node*> lstNodes, std::string strNamespace) {
    std::string strDot = "";
    
    for(Node* ndCurrent : lstNodes) {
      if(ndCurrent) {
	CKeyValuePair* ckvpImages = ndCurrent->metaInformation()->childForKey("images");
	
	if(ckvpImages) {
	  std::list<CKeyValuePair*> lstImages = ckvpImages->children();
	  
	  unsigned int unIndex = 0;
	  for(CKeyValuePair* ckvpImage : lstImages) {
	    std::stringstream sts;
	    sts << ndCurrent->uniqueID() << "_image_" << unIndex;
	    
	    std::string strOwlClass = "&knowrob;CameraImage";
	    std::string strFilename = ckvpImage->stringValue("filename");
	    std::string strTopic = ckvpImage->stringValue("origin");
	    std::string strCaptureTime = ckvpImage->stringValue("time-capture");
	    
	    strDot += "    <owl:namedIndividual rdf:about=\"&" + strNamespace + ";" + sts.str() + "\">\n";
	    strDot += "        <knowrob:linkToImageFile rdf:datatype=\"&xsd;string\">" + strFilename + "</knowrob:linkToImageFile>\n";
	    strDot += "        <knowrob:rosTopic rdf:datatype=\"&xsd;string\">" + strTopic + "</knowrob:rosTopic>\n";
	    strDot += "        <knowrob:captureTime rdf:resource=\"&" + strNamespace + ";timepoint_" + strCaptureTime + "\"/>\n";
	    strDot += "        <rdf:type rdf:resource=\"" + strOwlClass + "\"/>\n";
	    strDot += "    </owl:namedIndividual>\n\n";
	  }
	}
	
	strDot += this->generateImageIndividualsForNodes(ndCurrent->subnodes(), strNamespace);
      } else {
	this->fail("Image node with invalid content!");
      }
    }
    
    return strDot;
  }
  
  std::string CExporterOwl::generateImageIndividuals(std::string strNamespace) {
    std::string strDot = "    <!-- Image Individuals -->\n\n";
    strDot += this->generateImageIndividualsForNodes(this->nodes(), strNamespace);
    
    return strDot;
  }
  
  std::string CExporterOwl::generateDesignatorIndividuals(std::string strNamespace) {
    std::string strDot = "    <!-- Designator Individuals -->\n\n";
    
    std::list<std::string> lstDesigIDs = this->designatorIDs();
    
    for(std::string strID : lstDesigIDs) {
      strDot += "    <owl:namedIndividual rdf:about=\"&" + strNamespace + ";" + strID + "\">\n";
      strDot += "        <rdf:type rdf:resource=\"&knowrob;CRAMDesignator\"/>\n";
      
      std::list<std::string> lstSuccessorIDs = this->successorDesignatorsForID(strID);
      for(std::string strID2 : lstSuccessorIDs) {
	strDot += "        <knowrob:successorDesignator rdf:resource=\"&" + strNamespace + ";" + strID2 + "\"/>\n";
      }
      
      std::string strEquationTime = this->equationTimeForSuccessorID(strID);
      if(strEquationTime != "") {
	strDot += "        <knowrob:equationTime rdf:resource=\"&" + strNamespace + ";timepoint_" + strEquationTime + "\"/>\n";
      }
      
      strDot += "    </owl:namedIndividual>\n\n";
    }
    
    return strDot;
  }
  
  std::string CExporterOwl::generateFailureIndividuals(std::string strNamespace) {
    std::string strDot = "    <!-- Failure Individuals -->\n\n";
    strDot += this->generateFailureIndividualsForNodes(this->nodes(), strNamespace);
    
    return strDot;
  }
  
  std::string CExporterOwl::generateTimepointIndividuals(std::string strNamespace) {
    std::string strDot = "    <!-- Timepoint Individuals -->\n\n";
    
    std::list<std::string> lstTimepoints = this->gatherTimepointsForNodes(this->nodes());
    for(std::string strTimepoint : lstTimepoints) {
      strDot += "    <owl:namedIndividual rdf:about=\"&" + strNamespace + ";timepoint_" + strTimepoint + "\">\n";
      strDot += "        <rdf:type rdf:resource=\"&knowrob;TimePoint\"/>\n";
      strDot += "    </owl:namedIndividual>\n\n";
    }
    
    return strDot;
  }
  
  std::string CExporterOwl::generateMetaDataIndividual(std::string strNamespace) {
    std::string strDot = "    <!-- Meta Data Individual -->\n\n";
    std::string strUniqueName = this->generateUniqueID("ExperimentMetaData_", 8);
    
    strDot += "    <owl:namedIndividual rdf:about=\"&" + strNamespace + ";" + strUniqueName + "\">\n";
    strDot += "        <rdf:type rdf:resource=\"&knowrob;ExperimentMetaData\"/>\n";
    
    std::list<Node*> lstRootNodes = this->rootNodes();
    for(Node* ndRoot : lstRootNodes) {
      strDot += "        <knowrob:subAction rdf:resource=\"&" + strNamespace + ";" + ndRoot->uniqueID() + "\"/>\n";
    }
    
    for(std::pair<std::string, std::string> prEntry : m_mapMetaData) {
      std::string strCamelCaseKey = prEntry.first;
      int nCharCount = prEntry.first.length();
      
      for(int nI = 0; nI < nCharCount; nI++) {
	if(strCamelCaseKey[nI] == '-') {
	  std::string strTemp = strCamelCaseKey.substr(nI + 1, 1);
	  transform(strTemp.begin(), strTemp.end(), strTemp.begin(), ::toupper);
	  strCamelCaseKey.erase(nI, 2);
	  strCamelCaseKey.insert(nI, strTemp);
	  nCharCount--;
	}
      }
      
      strDot += "        <knowrob:" + strCamelCaseKey + " rdf:datatype=\"&xsd;string\">" + prEntry.second + "</knowrob:" + strCamelCaseKey + ">\n";
    }
    strDot += "    </owl:namedIndividual>\n\n";
    
    return strDot;
  }
  
  std::string CExporterOwl::generateParameterAnnotationInformation(std::string strNamespace) {
    std::string strDot = "    <!-- Parameter Annotation Information Individual -->\n\n";
    std::string strUniqueName = this->generateUniqueID("AnnotationInformation_", 8);
    
    strDot += "    <owl:namedIndividual rdf:about=\"&" + strNamespace + ";" + strUniqueName + "\">\n";
    strDot += "        <rdf:type rdf:resource=\"&knowrob;AnnotationInformation\"/>\n";
    
    for(std::string strParameterAnnotation : m_lstAnnotatedParameters) {
      strDot += "        <knowrob:annotatedParameterType rdf:datatype=\"&xsd;string\">" + strParameterAnnotation + "</knowrob:annotatedParameterType>\n";
    }
    
    strDot += "    </owl:namedIndividual>\n\n";
    
    return strDot;
  }
  
  std::string CExporterOwl::owlClassForNode(Node *ndNode, bool bClassOnly, bool bPrologSyntax) {
    std::string strName = "";
    
    if(ndNode) {
      strName = ndNode->title();
    }
    
    std::string strPlainPrefix = "knowrob";
    std::string strPrefix = (bPrologSyntax ? strPlainPrefix + ":" : "&" + strPlainPrefix + ";");
    std::string strClass = "CRAMAction";
    
    if(strName == "WITH-DESIGNATORS") {
      // Is this right? Or is there a more fitting type for that?
      strClass = "WithDesignators";
    } else if(strName == "TAG") {
      strClass = "Tag";
    } else if(strName.substr(0, 5) == "GOAL-") {
      // This is a goal definition.
      std::string strGoal = strName.substr(5);
      
      // Missing yet:
      /*
	PREVENT
	MAINTAIN
	INFORM (speech act, add information to belief state from outside)
      */
      
      if(strGoal == "PERCEIVE-OBJECT") {
	strClass = "CRAMPerceive";
      } else if(strGoal == "ACHIEVE") {
	strClass = "CRAMAchieve";
      } else if(strGoal == "PERFORM") { // Should go into another structure (?)
	strClass = "CRAMPerform";
      } else if(strGoal == "MONITOR-ACTION") {
	strClass = "CRAMMonitor";
      } else if(strGoal == "PERFORM-ON-PROCESS-MODULE") {
	strClass = "PerformOnProcessModule";
      } else {
	strClass = "DeclarativeGoal";
      }
    } else if(strName.substr(0, 8) == "RESOLVE-") {
      // This is a designator resolution.
      std::string strDesigType = strName.substr(8);
      
      if(strDesigType == "LOCATION-DESIGNATOR") {
	strClass = "ResolveLocationDesignator";
      } else if(strDesigType == "ACTION-DESIGNATOR") {
	strClass = "ResolveActionDesignator";
      }
    } else if(strName.substr(0, 21) == "REPLACEABLE-FUNCTION-") {
      // This is an internal function name
      std::string strFunction = strName.substr(21);
      
      if(strFunction == "NAVIGATE") {
	strClass = "BaseMovement"; // NOTE(winkler): was 'Navigate'
      }
    } else if(strName.substr(0, 8) == "PERFORM-") {
      // This is the performance of probably a designator
      std::string strPerformer = strName.substr(8);
      
      if(strPerformer == "ACTION-DESIGNATOR") {
	CKeyValuePair* ckvpDescription = NULL;
	std::list<CKeyValuePair*> lstDesc = ndNode->description();
	
	for(CKeyValuePair* ckvpCurrent : lstDesc) {
	  if(ckvpCurrent->key() == "DESCRIPTION") {
	    ckvpDescription = ckvpCurrent;
	    break;
	  }
	}
	
	bool bSpecializedDesignator = true;
	if(ckvpDescription) {
	  std::string strTo = ckvpDescription->stringValue("TO");
	  std::string strType = ckvpDescription->stringValue("TYPE");
	  
	  if(strTo == "GRASP") {
	    // Specializer: Grasping.
	    strClass = "PickingUpAnObject";
	  } else if(strTo == "LIFT") {
	    // Specializer: Lifting.
	    strClass = "LiftingAnObject";
	  } else if(strTo == "CARRY") {
	    // Specializer: Carrying.
	    strClass = "CarryingAnObject";
	  } else if(strTo == "PERCEIVE") {
	    // Specializer: Perceiving.
	    strClass = "PerceivingObjects";
	  } else if(strTo == "PUT-DOWN") {
	    // Specializer: Putting down.
	    strClass = "PuttingDownAnObject";
	  } else if(strTo == "PARK") {
	    // Specializer: Putting down.
	    strClass = "ParkingArms";
	  } else if(strType == "NAVIGATION") {
	    // Specializer: Navigating.
	    strClass = "Navigate";
	  } else {
	    // Fallback.
	    bSpecializedDesignator = false;
	  }
	} else {
	  // Fallback.
	  bSpecializedDesignator = false;
	}
	
	if(bSpecializedDesignator == false) {
	  // Default class if no specializer could be found.
	  strClass = "PerformActionDesignator";
	}
      }
    } else if(strName == "UIMA-PERCEIVE") {
      strClass = "UIMAPerception"; // NOTE(winkler): was 'VisualPerception'
    } else if(strName == "FIND-OBJECTS") {
      strClass = "FindingObjects";
    } else if(strName == "OBJECT-IDENTITY-RESOLUTION") {
      strClass = "ObjectIdentityResolution";
    } else if(strName == "BELIEF-STATE-UPDATE") {
      strClass = "BeliefStateUpdate";
    } else if(strName == "MOTION-PLANNING") {
      strClass = "MotionPlanning";
    } else if(strName == "MOTION-EXECUTION") {
      strClass = "MotionExecution";
    } else if(strName == "AT-LOCATION") {
      strClass = "AtLocation";
    } else if(strName == "VOLUNTARY-BODY-MOVEMENT-ARMS") {
      strClass = "ArmMovement";
    } else if(strName == "VOLUNTARY-BODY-MOVEMENT-HEAD") {
      strClass = "HeadMovement";
    } else if(strName == "WITH-FAILURE-HANDLING") {
      strClass = "WithFailureHandling";
    } else if(strName == "WITH-POLICY") {
      strClass = "WithPolicy";
    }
    
    return (bClassOnly ? "" : strPrefix) + (bPrologSyntax ? "'" + strClass + "'" : strClass);
  }
  
  bool CExporterOwl::runExporter(CKeyValuePair* ckvpConfigurationOverlay) {
    m_lstAnnotatedParameters.clear();
    m_lstExportedObjectIndividuals.clear();
    
    m_nThrowAndCatchFailureCounter = 0;
    
    this->info("Renewing unique IDs");
    this->renewUniqueIDs();
    
    this->info("Generating XML");
    if(this->outputFilename() != "") {
      std::string strOwl = "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n\n";
      // NOTE(winkler): This used to be `random'. Changed this due to
      // non-necessity of such a long namespace.
      // this->generateRandomIdentifier("namespace_", 8);
      std::string strNamespaceID = "log";
      std::string strNamespace = "http://ias.cs.tum.edu/kb/cram_log.owl";// + strNamespaceID;
      
      // Prepare content
      this->info(" - Preparing content");
      this->prepareEntities(strNamespaceID, strNamespace);
      
      // Generate source
      this->info(" - Generating source");
      strOwl += this->generateOwlStringForNodes(this->nodes(), strNamespaceID, strNamespace);
      
      // Write the .owl file
      this->info(" - Writing file");
      return this->writeToFile(strOwl);
    } else {
      this->fail("No output filename was given. Cancelling.");
    }
    
    return false;
  }
  
  std::string CExporterOwl::owlEscapeString(std::string strValue) {
    return strValue;
  }
  
  std::string CExporterOwl::generateOwlStringForNodes(std::list<Node*> lstNodes, std::string strNamespaceID, std::string strNamespace) {
    std::string strOwl = "";
    
    // Assemble OWL source
    this->info(" - Block: DocType");
    strOwl += this->generateDocTypeBlock();
    this->info(" - Block: XMLNS");
    strOwl += this->generateXMLNSBlock(strNamespace);
    this->info(" - Block: Imports");
    strOwl += this->generateOwlImports(strNamespace);
    this->info(" - Block: PropDefs");
    strOwl += this->generatePropertyDefinitions();
    this->info(" - Block: ClassDefs");
    strOwl += this->generateClassDefinitions();
    this->info(" - Block: EvtIndivs");
    strOwl += this->generateEventIndividuals(strNamespaceID);
    this->info(" - Block: ObjIndivs");
    strOwl += this->generateObjectIndividuals(strNamespaceID);
    this->info(" - Block: ImgIndivs");
    strOwl += this->generateImageIndividuals(strNamespaceID);
    this->info(" - Block: DesigIndivs");
    strOwl += this->generateDesignatorIndividuals(strNamespaceID);
    this->info(" - Block: FailIndivs");
    strOwl += this->generateFailureIndividuals(strNamespaceID);
    this->info(" - Block: TPIndivs");
    strOwl += this->generateTimepointIndividuals(strNamespaceID);
    this->info(" - Meta Data");
    strOwl += this->generateMetaDataIndividual(strNamespaceID);
    this->info(" - Parameter Annotations");
    strOwl += this->generateParameterAnnotationInformation(strNamespaceID);
    strOwl += "</rdf:RDF>\n";
    
    if(m_nThrowAndCatchFailureCounter != 0) {
      this->warn("Throw/Catch failure counter is != 0: " + this->str(m_nThrowAndCatchFailureCounter));
    }
    
    return strOwl;
  }
}
