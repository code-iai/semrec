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
  
  bool CExporterOwl::loadSemanticsDescriptorFile(string strFilepath) {
    if(this->fileExists(strFilepath)) {
      Config cfgConfig;
      
      try {
	cfgConfig.readFile(strFilepath.c_str());
	
	if(cfgConfig.exists("condition-mappings")) {
	  Setting &sConditionMappings = cfgConfig.lookup("condition-mappings");
	  
	  if(sConditionMappings.exists("mappings")) {
	    Setting &sMappings = sConditionMappings["mappings"];
	    
	    for(int nI = 0; nI < sMappings.getLength(); nI++) {
	      string strTo;
	      
	      if(sMappings[nI].lookupValue("to", strTo)) {
		if(sMappings[nI].exists("from")) {
		  Setting &sFrom = sMappings[nI]["from"];
		  
		  for(int nJ = 0; nJ < sFrom.getLength(); nJ++) {
		    string strFrom = sFrom[nJ];
		    
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
	  Setting &sStructure = cfgConfig.lookup("structure");
	  
	  m_strPropertyNamespace = "";
	  if(sStructure.exists("property-namespace")) {
	    sStructure.lookupValue("property-namespace", m_strPropertyNamespace);
	  }
	  
	  if(m_strPropertyNamespace == "") {
	    this->warn("You didn't specify the 'structure/property-namespace' parameter on the semantics descriptor file. Your OWL classes will have no namespace prepended. Is this intended?");
	  }
	  
	  if(sStructure.exists("defined-properties")) {
	    Setting &sDefinedProperties = sStructure["defined-properties"];
	    
	    for(int nI = 0; nI < sDefinedProperties.getLength(); nI++) {
	      string strProperty = sDefinedProperties[nI];
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
	    Setting &sPurposes = sStructure["annotation-purposes"];
	    
	    for(int nI = 0; nI < sPurposes.getLength(); nI++) {
	      Setting &sPurpose = sPurposes[nI];
	      
	      string strFrom;
	      string strTo;
	      
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
      } catch(ParseException e) {
        stringstream sts;
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
  
  void CExporterOwl::prepareEntities(string strNamespaceID, string strNamespace) {
    m_lstEntities.clear();
  
    this->addEntity("owl", "http://www.w3.org/2002/07/owl#");
    this->addEntity("xsd", "http://www.w3.org/2001/XMLSchema#");
    this->addEntity("knowrob", "http://ias.cs.tum.edu/kb/knowrob.owl#");
    this->addEntity("rdfs", "http://www.w3.org/2000/01/rdf-schema#");
    this->addEntity("rdf", "http://www.w3.org/1999/02/22-rdf-syntax-ns#");
    this->addEntity(strNamespaceID, strNamespace + "#");
  }

  void CExporterOwl::addEntity(string strNickname, string strNamespace) {
    m_lstEntities.push_back(make_pair(strNickname, strNamespace));
  }

  string CExporterOwl::generateDocTypeBlock() {
    string strDot = "<!DOCTYPE rdf:RDF [\n";
  
    for(list< pair<string, string> >::iterator itPair = m_lstEntities.begin();
	itPair != m_lstEntities.end();
	itPair++) {
      pair<string, string> prEntity = *itPair;
    
      strDot += "    <!ENTITY " + prEntity.first + " \"" + prEntity.second + "\" >\n";
    }
  
    strDot += "]>\n\n";
    return strDot;
  }

  string CExporterOwl::generateXMLNSBlock(string strNamespace) {
    string strDot = "<rdf:RDF xmlns=\"" + strNamespace + "#\"\n";
    strDot += "     xml:base=\"" + strNamespace + "\"\n";
  
    for(list< pair<string, string> >::iterator itPair = m_lstEntities.begin();
	itPair != m_lstEntities.end();
	itPair++) {
      pair<string, string> prEntity = *itPair;
    
      if(itPair != m_lstEntities.begin()) {
	strDot += "\n";
      }

      strDot += "     xmlns:" + prEntity.first + "=\"" + prEntity.second + "\"";
    }

    strDot += ">\n\n";
    return strDot;
  }

  string CExporterOwl::generateOwlImports(string strNamespace) {
    string strDot = "";
    
    string strImportNamespace = "knowrob.owl"; //"http://ias.cs.tum.edu/kb/knowrob.owl";
    
    strDot += "    <owl:Ontology rdf:about=\"" + strNamespace + "\">\n";
    strDot += "        <owl:imports rdf:resource=\"" + strImportNamespace + "\"/>\n";
    strDot += "    </owl:Ontology>\n\n";
    
    return strDot;
  }

  string CExporterOwl::generatePropertyDefinitions() {
    string strDot = "    <!-- Property Definitions -->\n\n";
    
    for(list<string>::iterator itProperty = m_lstDefinedProperties.begin();
	itProperty != m_lstDefinedProperties.end();
	itProperty++) {
      strDot += "    <owl:ObjectProperty rdf:about=\"" + *itProperty + "\"/>\n\n";
    }
  
    return strDot;
  }

  list<string> CExporterOwl::gatherClassesForNodes(list<Node*> lstNodes) {
    list<string> lstClasses;
  
    for(list<Node*>::iterator itNode = lstNodes.begin();
	itNode != lstNodes.end();
	itNode++) {
      Node *ndCurrent = *itNode;
    
      list<string> lstClassesSubnodes = this->gatherClassesForNodes(ndCurrent->subnodes());
      lstClassesSubnodes.push_back(this->owlClassForNode(ndCurrent));
    
      for(list<string>::iterator itClassSubnode = lstClassesSubnodes.begin();
	  itClassSubnode != lstClassesSubnodes.end();
	  itClassSubnode++) {
	bool bExists = false;
      
	for(list<string>::iterator itClassNode = lstClasses.begin();
	    itClassNode != lstClasses.end();
	    itClassNode++) {
	  if(*itClassSubnode == *itClassNode) {
	    bExists = true;
	    break;
	  }
	}
      
	if(!bExists) {
	  lstClasses.push_back(*itClassSubnode);
	}
      }
    }
  
    return lstClasses;
  }

  list<string> CExporterOwl::gatherTimepointsForNodes(list<Node*> lstNodes) {
    list<string> lstTimepoints;
    
    for(list<Node*>::iterator itNode = lstNodes.begin();
	itNode != lstNodes.end();
	itNode++) {
      Node *ndCurrent = *itNode;
      
      // Gather node timepoints
      list<string> lstTimepointsSubnodes = this->gatherTimepointsForNodes(ndCurrent->subnodes());
      lstTimepointsSubnodes.push_back(ndCurrent->metaInformation()->stringValue("time-start"));
      lstTimepointsSubnodes.push_back(ndCurrent->metaInformation()->stringValue("time-end"));
      
      // Gather failure timepoints
      CKeyValuePair *ckvpFailures = ndCurrent->metaInformation()->childForKey("failures");
      
      if(ckvpFailures) {
	list<CKeyValuePair*> lstFailures = ckvpFailures->children();
	
	unsigned int unIndex = 0;
	for(list<CKeyValuePair*>::iterator itFailure = lstFailures.begin();
	    itFailure != lstFailures.end();
	    itFailure++, unIndex++) {
	  CKeyValuePair *ckvpFailure = *itFailure;
	  lstTimepointsSubnodes.push_back(ckvpFailure->stringValue("time-fail"));
	}
      }
      
      // Gather image timepoints
      CKeyValuePair *ckvpImages = ndCurrent->metaInformation()->childForKey("images");
      
      if(ckvpImages) {
	list<CKeyValuePair*> lstImages = ckvpImages->children();
	
	unsigned int unIndex = 0;
	for(list<CKeyValuePair*>::iterator itImage = lstImages.begin();
	    itImage != lstImages.end();
	    itImage++, unIndex++) {
	  CKeyValuePair *ckvpImage = *itImage;
	  lstTimepointsSubnodes.push_back(ckvpImage->stringValue("time-capture"));
	}
      }
      
      // Gather designator equation timepoints
      for(list< pair<string, string> >::iterator itPair = m_lstDesignatorEquationTimes.begin();
	  itPair != m_lstDesignatorEquationTimes.end();
	  itPair++) {
	lstTimepointsSubnodes.push_back((*itPair).second);
      }
    
      // Unify all timepoints
      for(list<string>::iterator itTimepointSubnode = lstTimepointsSubnodes.begin();
	  itTimepointSubnode != lstTimepointsSubnodes.end();
	  itTimepointSubnode++) {
	bool bExists = false;
      
	for(list<string>::iterator itTimepointNode = lstTimepoints.begin();
	    itTimepointNode != lstTimepoints.end();
	    itTimepointNode++) {
	  if(*itTimepointSubnode == *itTimepointNode) {
	    bExists = true;
	    break;
	  }
	}
      
	if(!bExists) {
	  lstTimepoints.push_back(*itTimepointSubnode);
	}
      }
    }
  
    return lstTimepoints;
  }

  string CExporterOwl::generateClassDefinitions() {
    string strDot = "    <!-- Class Definitions -->\n\n";
  
    list<string> lstClasses = this->gatherClassesForNodes(this->nodes());
    lstClasses.push_back("&knowrob;TimePoint");
  
    for(list<string>::iterator itClass = lstClasses.begin();
	itClass != lstClasses.end();
	itClass++) {
      strDot += "    <owl:Class rdf:about=\"" + *itClass + "\"/>\n\n";
    }
  
    return strDot;
  }

  string CExporterOwl::nodeIDPrefix(Node* ndInQuestion, string strProposition) {
    string strPrefix = CExporter::nodeIDPrefix(ndInQuestion, strProposition);
    strPrefix = this->owlClassForNode(ndInQuestion, true) + "_";
  
    return strPrefix;
  }

  string CExporterOwl::generateEventIndividualsForNodes(list<Node*> lstNodes, string strNamespace) {
    string strDot = "";
    
    Node *ndLastDisplayed = NULL;
    for(list<Node*>::iterator itNode = lstNodes.begin();
	itNode != lstNodes.end();
	itNode++) {
      Node *ndCurrent = *itNode;
      
      if(this->nodeDisplayable(ndCurrent)) {
	string strOwlClass = this->owlClassForNode(ndCurrent);
	strDot += this->generateEventIndividualsForNodes(ndCurrent->subnodes(), strNamespace);
	
	strDot += "    <owl:namedIndividual rdf:about=\"&" + strNamespace + ";" + ndCurrent->uniqueID() + "\">\n";
	strDot += "        <rdf:type rdf:resource=\"" + strOwlClass + "\"/>\n";
	strDot += "        <knowrob:taskContext rdf:datatype=\"&xsd;string\">" + ndCurrent->title() + "</knowrob:taskContext>\n";
	strDot += "        <knowrob:startTime rdf:resource=\"&" + strNamespace + ";timepoint_" + ndCurrent->metaInformation()->stringValue("time-start") + "\"/>\n";
	strDot += "        <knowrob:endTime rdf:resource=\"&" + strNamespace + ";timepoint_" + ndCurrent->metaInformation()->stringValue("time-end") + "\"/>\n";
	
	if(ndCurrent->title() == "GOAL-ACHIEVE") {
	  list<CKeyValuePair*> lstDescription = ndCurrent->description();
	  string strPattern = "";
	  
	  for(list<CKeyValuePair*>::iterator itDesc = lstDescription.begin();
	      itDesc != lstDescription.end();
	      itDesc++) {
	    CKeyValuePair *prNow = *itDesc;
	  
	    if(prNow->key() == "PATTERN") {
	      strPattern = prNow->stringValue();
	      break;
	    }
	  }
	
	  if(strPattern != "") {
	    strDot += "        <knowrob:goalContext rdf:datatype=\"&xsd;string\">" + strPattern + "</knowrob:goalContext>\n";
	  }
	}
	
	list<Node*> lstSubnodes = ndCurrent->subnodes();
	for(list<Node*>::iterator itSubnode = lstSubnodes.begin();
	    itSubnode != lstSubnodes.end();
	    itSubnode++) {
	  Node *ndSubnode = *itSubnode;
	  
	  if(this->nodeDisplayable(ndSubnode)) {
	    strDot += "        <knowrob:subAction rdf:resource=\"&" + strNamespace + ";" + ndSubnode->uniqueID() + "\"/>\n";
	  }
	}
	
	if(ndLastDisplayed) {
	  strDot += "        <knowrob:previousAction rdf:resource=\"&" + strNamespace + ";" + ndLastDisplayed->uniqueID() + "\"/>\n";
	}
	
	list<Node*>::iterator itPostEvent = itNode;
	itPostEvent++;
	while(itPostEvent != lstNodes.end()) {
	  if(this->nodeDisplayable(*itPostEvent)) {
	    strDot += "        <knowrob:nextAction rdf:resource=\"&" + strNamespace + ";" + (*itPostEvent)->uniqueID() + "\"/>\n";
	    break;
	  }
	
	  itPostEvent++;
	}
	
	// Object references here.
	CKeyValuePair *ckvpObjects = ndCurrent->metaInformation()->childForKey("objects");
	
	if(ckvpObjects) {
	  list<CKeyValuePair*> lstObjects = ckvpObjects->children();
    
	  unsigned int unIndex = 0;
	  for(list<CKeyValuePair*>::iterator itObject = lstObjects.begin();
	      itObject != lstObjects.end();
	      itObject++, unIndex++) {
	    CKeyValuePair *ckvpObject = *itObject;
      
	    stringstream sts;
	    sts << ndCurrent->uniqueID() << "_object_" << unIndex;
	  
	    if(strOwlClass == "&knowrob;VisualPerception") {
	      strDot += "        <knowrob:detectedObject rdf:resource=\"&" + strNamespace + ";" + sts.str() +"\"/>\n";
	    } else {
	      strDot += "        <knowrob:objectActedOn rdf:resource=\"&" + strNamespace + ";" + sts.str() +"\"/>\n";
	    }
	  }
	}
	
	// Image references here.
	CKeyValuePair *ckvpImages = ndCurrent->metaInformation()->childForKey("images");
	
	if(ckvpImages) {
	  list<CKeyValuePair*> lstImages = ckvpImages->children();
	  
	  unsigned int unIndex = 0;
	  for(list<CKeyValuePair*>::iterator itImage = lstImages.begin();
	      itImage != lstImages.end();
	      itImage++, unIndex++) {
	    CKeyValuePair *ckvpImage = *itImage;
	    
	    stringstream sts;
	    sts << ndCurrent->uniqueID() << "_image_" << unIndex;
	    
	    strDot += "        <knowrob:capturedImage rdf:resource=\"&" + strNamespace + ";" + sts.str() +"\"/>\n";
	  }
	}
	
	// Failure references here.
	CKeyValuePair *ckvpFailures = ndCurrent->metaInformation()->childForKey("failures");
      
	if(ckvpFailures) {
	  list<CKeyValuePair*> lstFailures = ckvpFailures->children();
	
	  unsigned int unIndex = 0;
	  for(list<CKeyValuePair*>::iterator itFailure = lstFailures.begin();
	      itFailure != lstFailures.end();
	      itFailure++, unIndex++) {
	    CKeyValuePair *ckvpFailure = *itFailure;
	  
	    stringstream sts;
	    sts << ndCurrent->uniqueID() << "_failure_" << unIndex;
	    strDot += "        <knowrob:eventFailure rdf:resource=\"&" + strNamespace + ";" + sts.str() + "\"/>\n";
	  }
	}
	
	// Caught failure here.
	CKeyValuePair *ckvpCaughtFailures = ndCurrent->metaInformation()->childForKey("caught_failures");
	
	if(ckvpCaughtFailures) {
	  list<CKeyValuePair*> lstCaughtFailures = ckvpCaughtFailures->children();
	  
	  unsigned int unIndex = 0;
	  for(list<CKeyValuePair*>::iterator itCaughtFailure = lstCaughtFailures.begin();
	      itCaughtFailure != lstCaughtFailures.end();
	      itCaughtFailure++, unIndex++) {
	    CKeyValuePair *ckvpCaughtFailure = *itCaughtFailure;
	    
	    Node* ndFailureEmitter = ndCurrent->emitterForCaughtFailure(ckvpCaughtFailure->stringValue("failure-id"), ckvpCaughtFailure->stringValue("time-catch"), unIndex);
	    string strCaughtFailure = ndFailureEmitter->uniqueID() + "_" + ckvpCaughtFailure->stringValue("failure-id");
	    
	    strDot += "        <knowrob:caughtFailure rdf:resource=\"&" + strNamespace + ";" + strCaughtFailure + "\"/>\n";
	  }
	}
	
	// Designator references here.
	CKeyValuePair *ckvpDesignators = ndCurrent->metaInformation()->childForKey("designators");
	
	if(ckvpDesignators) {
	  list<CKeyValuePair*> lstDesignators = ckvpDesignators->children();
	
	  unsigned int unIndex = 0;
	  for(list<CKeyValuePair*>::iterator itDesignator = lstDesignators.begin();
	      itDesignator != lstDesignators.end();
	      itDesignator++, unIndex++) {
	    CKeyValuePair *ckvpDesignator = *itDesignator;
	    
	    string strAnnotation = ckvpDesignator->stringValue("annotation");
	    string strDesigID = ckvpDesignator->stringValue("id");
	    
	    string strDesigPurpose = this->resolveDesignatorAnnotationTagName(strAnnotation);
	    
	    strDot += "        <knowrob:" + strDesigPurpose + " rdf:resource=\"&" + strNamespace + ";" + strDesigID + "\"/>\n";
	  }
	}
	
	strDot += "    </owl:namedIndividual>\n\n";
	ndLastDisplayed = ndCurrent;
      }
    }
    
    return strDot;
  }
  
  string CExporterOwl::resolveDesignatorAnnotationTagName(string strAnnotation) {
    string strDesigPurpose = "";
    
    for(list< pair<string, string> >::iterator itP = m_lstAnnotationPurposeMapping.begin();
	itP != m_lstAnnotationPurposeMapping.end();
	itP++) {
      pair<string, string> prMapping = *itP;
      
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
  
  string CExporterOwl::generateEventIndividuals(string strNamespace) {
    string strDot = "    <!-- Event Individuals -->\n\n";
    strDot += this->generateEventIndividualsForNodes(this->nodes(), strNamespace);
  
    return strDot;
  }

  string CExporterOwl::owlClassForObject(CKeyValuePair *ckvpObject) {
    return "&knowrob;HumanScaleObject";
  }
  
  string CExporterOwl::failureClassForCondition(string strCondition) {
    string strFailureClass = "CRAMFailure";
    
    for(list< pair<string, string> >::iterator itPair = m_lstFailureMapping.begin();
	itPair != m_lstFailureMapping.end();
	itPair++) {
      pair<string, string> prCurrent = *itPair;
      
      if(strCondition == prCurrent.first) {
	strFailureClass = prCurrent.second;
	break;
      }
    }
    
    return strFailureClass;
  }
  
  string CExporterOwl::generateFailureIndividualsForNodes(list<Node*> lstNodes, string strNamespace) {
    string strDot = "";
    
    for(list<Node*>::iterator itNode = lstNodes.begin();
	itNode != lstNodes.end();
	itNode++) {
      Node *ndCurrent = *itNode;
      
      CKeyValuePair *ckvpFailures = ndCurrent->metaInformation()->childForKey("failures");
      
      if(ckvpFailures) {
	list<CKeyValuePair*> lstFailures = ckvpFailures->children();
      
	unsigned int unIndex = 0;
	for(list<CKeyValuePair*>::iterator itFailure = lstFailures.begin();
	    itFailure != lstFailures.end();
	    itFailure++, unIndex++) {
	  CKeyValuePair *ckvpFailure = *itFailure;
	
	  stringstream sts;
	  sts << ndCurrent->uniqueID() << "_failure_" << unIndex;
	
	  string strCondition = ckvpFailure->stringValue("condition");
	  string strTimestamp = ckvpFailure->stringValue("time-fail");
	  
	  string strFailureClass = this->failureClassForCondition(strCondition);
	  
	  strDot += "    <owl:namedIndividual rdf:about=\"&" + strNamespace + ";" + sts.str() + "\">\n";
	  strDot += "        <rdf:type rdf:resource=\"&knowrob;" + strFailureClass + "\"/>\n";
	  strDot += "        <rdfs:label rdf:datatype=\"&xsd;string\">" + this->owlEscapeString(strCondition) + "</rdfs:label>\n";
	  strDot += "        <knowrob:startTime rdf:resource=\"&" + strNamespace + ";timepoint_" + strTimestamp + "\"/>\n";
	  strDot += "    </owl:namedIndividual>\n\n";
	}
      }
    
      strDot += this->generateFailureIndividualsForNodes(ndCurrent->subnodes(), strNamespace);
    }
  
    return strDot;
  }

  string CExporterOwl::generateObjectIndividualsForNodes(list<Node*> lstNodes, string strNamespace) {
    string strDot = "";
  
    for(list<Node*>::iterator itNode = lstNodes.begin();
	itNode != lstNodes.end();
	itNode++) {
      Node *ndCurrent = *itNode;
      CKeyValuePair *ckvpObjects = ndCurrent->metaInformation()->childForKey("objects");
    
      if(ckvpObjects) {
	list<CKeyValuePair*> lstObjects = ckvpObjects->children();
      
	unsigned int unIndex = 0;
	for(list<CKeyValuePair*>::iterator itObject = lstObjects.begin();
	    itObject != lstObjects.end();
	    itObject++, unIndex++) {
	  CKeyValuePair *ckvpObject = *itObject;
	
	  stringstream sts;
	  sts << ndCurrent->uniqueID() << "_object_" << unIndex;
	
	  string strDesignatorID = ckvpObject->stringValue("__id");
	  string strOwlClass = this->owlClassForObject(ckvpObject);
	  strDot += "    <owl:namedIndividual rdf:about=\"&" + strNamespace + ";" + sts.str() + "\">\n";
	  strDot += "        <knowrob:designator rdf:resource=\"&" + strNamespace + ";" + strDesignatorID + "\"/>\n";
	  strDot += "        <rdf:type rdf:resource=\"" + strOwlClass + "\"/>\n";
	  strDot += "    </owl:namedIndividual>\n\n";
	}
      }
    
      strDot += this->generateObjectIndividualsForNodes(ndCurrent->subnodes(), strNamespace);
    }
  
    return strDot;
  }

  string CExporterOwl::generateObjectIndividuals(string strNamespace) {
    string strDot = "    <!-- Object Individuals -->\n\n";
    strDot += this->generateObjectIndividualsForNodes(this->nodes(), strNamespace);
  
    return strDot;
  }

  string CExporterOwl::generateImageIndividualsForNodes(list<Node*> lstNodes, string strNamespace) {
    string strDot = "";
    
    for(list<Node*>::iterator itNode = lstNodes.begin();
	itNode != lstNodes.end();
	itNode++) {
      Node *ndCurrent = *itNode;
      CKeyValuePair *ckvpImages = ndCurrent->metaInformation()->childForKey("images");
      
      if(ckvpImages) {
	list<CKeyValuePair*> lstImages = ckvpImages->children();
	
	unsigned int unIndex = 0;
	for(list<CKeyValuePair*>::iterator itImage = lstImages.begin();
	    itImage != lstImages.end();
	    itImage++, unIndex++) {
	  CKeyValuePair *ckvpImage = *itImage;
	  
	  stringstream sts;
	  sts << ndCurrent->uniqueID() << "_image_" << unIndex;
	  
	  string strOwlClass = "&knowrob;CameraImage";
	  string strFilename = ckvpImage->stringValue("filename");
	  string strTopic = ckvpImage->stringValue("origin");
	  string strCaptureTime = ckvpImage->stringValue("time-capture");
	  
	  strDot += "    <owl:namedIndividual rdf:about=\"&" + strNamespace + ";" + sts.str() + "\">\n";
	  strDot += "        <knowrob:linkToImageFile rdf:datatype=\"&xsd;string\">" + strFilename + "</knowrob:linkToImageFile>\n";
	  strDot += "        <knowrob:rosTopic rdf:datatype=\"&xsd;string\">" + strTopic + "</knowrob:rosTopic>\n";
	  strDot += "        <knowrob:captureTime rdf:resource=\"&" + strNamespace + ";timepoint_" + strCaptureTime + "\"/>\n";
	  strDot += "        <rdf:type rdf:resource=\"" + strOwlClass + "\"/>\n";
	  strDot += "    </owl:namedIndividual>\n\n";
	}
      }
      
      strDot += this->generateImageIndividualsForNodes(ndCurrent->subnodes(), strNamespace);
    }
  
    return strDot;
  }

  string CExporterOwl::generateImageIndividuals(string strNamespace) {
    string strDot = "    <!-- Image Individuals -->\n\n";
    strDot += this->generateImageIndividualsForNodes(this->nodes(), strNamespace);
  
    return strDot;
  }

  string CExporterOwl::generateDesignatorIndividuals(string strNamespace) {
    string strDot = "    <!-- Designator Individuals -->\n\n";
    
    list<string> lstDesigIDs = this->designatorIDs();
    
    for(list<string>::iterator itID = lstDesigIDs.begin();
	itID != lstDesigIDs.end();
	itID++) {
      string strID = *itID;
      
      strDot += "    <owl:namedIndividual rdf:about=\"&" + strNamespace + ";" + strID + "\">\n";
      strDot += "        <rdf:type rdf:resource=\"&knowrob;CRAMDesignator\"/>\n";
      
      list<string> lstSuccessorIDs = this->successorDesignatorsForID(strID);
      for(list<string>::iterator itID2 = lstSuccessorIDs.begin();
	  itID2 != lstSuccessorIDs.end();
	  itID2++) {
	string strID2 = *itID2;
	
	strDot += "        <knowrob:successorDesignator rdf:resource=\"&" + strNamespace + ";" + strID2 + "\"/>\n";
      }
      
      string strEquationTime = this->equationTimeForSuccessorID(strID);
      if(strEquationTime != "") {
	strDot += "        <knowrob:equationTime rdf:resource=\"&" + strNamespace + ";timepoint_" + strEquationTime + "\"/>\n";
      }
      
      strDot += "    </owl:namedIndividual>\n\n";
    }
    
    return strDot;
  }

  string CExporterOwl::generateFailureIndividuals(string strNamespace) {
    string strDot = "    <!-- Failure Individuals -->\n\n";
    strDot += this->generateFailureIndividualsForNodes(this->nodes(), strNamespace);
    
    return strDot;
  }

  string CExporterOwl::generateTimepointIndividuals(string strNamespace) {
    string strDot = "    <!-- Timepoint Individuals -->\n\n";
  
    list<string> lstTimepoints = this->gatherTimepointsForNodes(this->nodes());
    for(list<string>::iterator itTimepoint = lstTimepoints.begin();
	itTimepoint != lstTimepoints.end();
	itTimepoint++) {
      strDot += "    <owl:NamedIndividual rdf:about=\"&" + strNamespace + ";timepoint_" + *itTimepoint + "\">\n";
      strDot += "        <rdf:type rdf:resource=\"&knowrob;TimePoint\"/>\n";
      strDot += "    </owl:NamedIndividual>\n\n";
    }
  
    return strDot;
  }

  string CExporterOwl::owlClassForNode(Node *ndNode, bool bClassOnly, bool bPrologSyntax) {
    string strName = ndNode->title();
    
    string strPlainPrefix = "knowrob";
    string strPrefix = (bPrologSyntax ? strPlainPrefix + ":" : "&" + strPlainPrefix + ";");
    string strClass = "CRAMAction";
    
    if(strName == "WITH-DESIGNATORS") {
      // Is this right? Or is there a more fitting type for that?
      strClass = "WithDesignators";
    } else if(strName.substr(0, 5) == "GOAL-") {
      // This is a goal definition.
      string strGoal = strName.substr(5);
    
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
      string strDesigType = strName.substr(8);
    
      if(strDesigType == "LOCATION-DESIGNATOR") {
	strClass = "ResolveLocationDesignator";
      } else if(strDesigType == "ACTION-DESIGNATOR") {
	strClass = "ResolveActionDesignator";
      }
    } else if(strName.substr(0, 21) == "REPLACEABLE-FUNCTION-") {
      // This is an internal function name
      string strFunction = strName.substr(21);
      
      if(strFunction == "NAVIGATE") {
	strClass = "BaseMovement"; // NOTE(winkler): was 'Navigate'
      }
    } else if(strName.substr(0, 8) == "PERFORM-") {
      // This is the performance of probably a designator
      string strPerformer = strName.substr(8);
    
      if(strPerformer == "ACTION-DESIGNATOR") {
	strClass = "PerformActionDesignator"; // Added
      }
    } else if(strName == "UIMA-PERCEIVE") {
      strClass = "UIMAPerception"; // NOTE(winkler): was 'VisualPerception'
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
    this->info("Renewing unique IDs");
    this->renewUniqueIDs();
    
    this->info("Generating XML");
    if(this->outputFilename() != "") {
      string strOwl = "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n\n";
      // NOTE(winkler): This used to be `random'. Changed this due to
      // non-necessity of such a long namespace.
      // this->generateRandomIdentifier("namespace_", 8);
      string strNamespaceID = "log";
      string strNamespace = "http://ias.cs.tum.edu/kb/cram_log.owl";// + strNamespaceID;
      
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
  
  string CExporterOwl::owlEscapeString(string strValue) {
    return strValue;
  }
  
  string CExporterOwl::generateOwlStringForNodes(list<Node*> lstNodes, string strNamespaceID, string strNamespace) {
    string strOwl = "";
    
    // Assemble OWL source
    strOwl += this->generateDocTypeBlock();
    strOwl += this->generateXMLNSBlock(strNamespace);
    strOwl += this->generateOwlImports(strNamespace);
    strOwl += this->generatePropertyDefinitions();
    strOwl += this->generateClassDefinitions();
    strOwl += this->generateEventIndividuals(strNamespaceID);
    strOwl += this->generateObjectIndividuals(strNamespaceID);
    strOwl += this->generateImageIndividuals(strNamespaceID);
    strOwl += this->generateDesignatorIndividuals(strNamespaceID);
    strOwl += this->generateFailureIndividuals(strNamespaceID);
    strOwl += this->generateTimepointIndividuals(strNamespaceID);
    strOwl += "</rdf:RDF>\n";
  
    return strOwl;
  }
}
