#include <plugins/owlexporter/CExporterOwl.h>



namespace beliefstate {
  CExporterOwl::CExporterOwl() {
  }

  CExporterOwl::~CExporterOwl() {
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
  
    strDot += "    <owl:Ontology rdf:about=\"" + strNamespace + "\">\n";
    strDot += "        <owl:imports rdf:resource=\"http://ias.cs.tum.edu/kb/knowrob.owl\"/>\n";
    strDot += "    </owl:Ontology>\n\n";
  
    return strDot;
  }

  string CExporterOwl::generatePropertyDefinitions() {
    string strDot = "    <!-- Property Definitions -->\n\n";
  
    list<string> lstProperties;
    lstProperties.push_back("&knowrob;startTime");
    lstProperties.push_back("&knowrob;endTime");
    lstProperties.push_back("&knowrob;previousEvent");
    lstProperties.push_back("&knowrob;nextEvent");
    lstProperties.push_back("&knowrob;subAction");
    lstProperties.push_back("&knowrob;detectedObject");
    lstProperties.push_back("&knowrob;objectActedOn");
    lstProperties.push_back("&knowrob;eventFailure");
    lstProperties.push_back("&knowrob;designator");
    lstProperties.push_back("&knowrob;equationTime");
    lstProperties.push_back("&knowrob;successorDesignator");
    lstProperties.push_back("&knowrob;taskContext");
    lstProperties.push_back("&knowrob;goalContext");
  
    for(list<string>::iterator itProperty = lstProperties.begin();
	itProperty != lstProperties.end();
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
	  strDot += "        <knowrob:previousEvent rdf:resource=\"&" + strNamespace + ";" + ndLastDisplayed->uniqueID() + "\"/>\n";
	}
      
	list<Node*>::iterator itPostEvent = itNode;
	itPostEvent++;
	while(itPostEvent != lstNodes.end()) {
	  if(this->nodeDisplayable(*itPostEvent)) {
	    strDot += "        <knowrob:nextEvent rdf:resource=\"&" + strNamespace + ";" + (*itPostEvent)->uniqueID() + "\"/>\n";
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
	
	// Designator references here.
	CKeyValuePair *ckvpDesignators = ndCurrent->metaInformation()->childForKey("designators");
	
	if(ckvpDesignators) {
	  list<CKeyValuePair*> lstDesignators = ckvpDesignators->children();
	
	  unsigned int unIndex = 0;
	  for(list<CKeyValuePair*>::iterator itDesignator = lstDesignators.begin();
	      itDesignator != lstDesignators.end();
	      itDesignator++, unIndex++) {
	    CKeyValuePair *ckvpDesignator = *itDesignator;
	    
	    string strDesigPurpose;
	    string strAnnotation = ckvpDesignator->stringValue("annotation");
	    string strDesigID = ckvpDesignator->stringValue("id");
	    
	    if(strAnnotation == "perception-request") {
	      strDesigPurpose = "perceptionRequest";
	    } else if(strAnnotation == "perception-result") {
	      strDesigPurpose = "perceptionResult";
	    } else if(strAnnotation == "object-acted-on") {
	      strDesigPurpose = "objectActedOn";
	    } else if(strAnnotation == "putdown-location") {
	      strDesigPurpose = "putdownLocation";
	    } else if(strAnnotation == "voluntary-movement-details") {
	      strDesigPurpose = "voluntaryMovementDetails";
	    } else {
	      strDesigPurpose = "designator";
	    }
	  
	    strDot += "        <knowrob:" + strDesigPurpose + " rdf:resource=\"&" + strNamespace + ";" + strDesigID + "\"/>\n";
	  }
	}
	
	strDot += "    </owl:namedIndividual>\n\n";
	ndLastDisplayed = ndCurrent;
      }
    }
    
    return strDot;
  }

  string CExporterOwl::generateEventIndividuals(string strNamespace) {
    string strDot = "    <!-- Event Individuals -->\n\n";
    strDot += this->generateEventIndividualsForNodes(this->nodes(), strNamespace);
  
    return strDot;
  }

  string CExporterOwl::owlClassForObject(CKeyValuePair *ckvpObject) {
    return "&knowrob;HumanScaleObject";
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
	
	  string strFailureClass = "genericPlanFailure";
	  list< pair<string, string> > lstFailureMapping;
	
	  // NOTE(winkler): Define all known failure class mappings here.
	  // CRAM-PLAN-FAILURES
	  lstFailureMapping.push_back(make_pair("CRAM-PLAN-FAILURES:OBJECT-NOT-FOUND", "ObjectNotFound"));
	  lstFailureMapping.push_back(make_pair("CRAM-PLAN-FAILURES:OBJECT-NOT-FOUND-DESIG", "ObjectNotFound"));
	  lstFailureMapping.push_back(make_pair("CRAM-PLAN-FAILURES:OBJECT-LOST", "ObjectLost"));
	  lstFailureMapping.push_back(make_pair("CRAM-PLAN-FAILURES:MANIPULATION-FAILURE", "ManipulationFailed"));
	  lstFailureMapping.push_back(make_pair("CRAM-PLAN-FAILURES:MANIPULATION-FAILED", "ManipulationFailed"));
	  lstFailureMapping.push_back(make_pair("CRAM-PLAN-FAILURES:MANIPULATION-PICKUP-FAILED", "ManipulationFailed"));
	  lstFailureMapping.push_back(make_pair("CRAM-PLAN-FAILURES:MANIPULATION-POSE-OCCUPIED", "ManipulationPoseOccupied"));
	  lstFailureMapping.push_back(make_pair("CRAM-PLAN-FAILURES:MANIPULATION-POSE-UNREACHABLE", "ManipulationPoseUnreachable"));
	  lstFailureMapping.push_back(make_pair("CRAM-PLAN-FAILURES:LOCATION-NOT-REACHED-FAILURE", "LocationNotReached"));
	  lstFailureMapping.push_back(make_pair("CRAM-PLAN-FAILURES:NAVIGATION-FAILURE", "LocationNotReached"));
	  lstFailureMapping.push_back(make_pair("CRAM-PLAN-FAILURES:NAVIGATION-FAILURE-LOCATION", "LocationNotReached"));
	  lstFailureMapping.push_back(make_pair("CRAM-PLAN-FAILURES:LOCATION-REACHED-BUT-NOT-TERMINATED", "LocationNotReached"));
	  // CRAM-MOVEIT
	  lstFailureMapping.push_back(make_pair("CRAM-MOVEIT:MOVEIT-FAILURE", "ManipulationFailed"));
	  lstFailureMapping.push_back(make_pair("CRAM-MOVEIT:PLANNING-FAILED", "ManipulationFailed"));
	  lstFailureMapping.push_back(make_pair("CRAM-MOVEIT:INVALID-MOTION-PLAN", "ManipulationFailed"));
	  lstFailureMapping.push_back(make_pair("CRAM-MOVEIT:MOTION-PLAN-INVALIDATED-BY-ENVIRONMENT-CHANGE", "ManipulationFailed"));
	  lstFailureMapping.push_back(make_pair("CRAM-MOVEIT:CONTROL-FAILED", "ManipulationFailed"));
	  lstFailureMapping.push_back(make_pair("CRAM-MOVEIT:UNABLE-TO-ACQUIRE-SENSOR-DATA", "ManipulationFailed"));
	  lstFailureMapping.push_back(make_pair("CRAM-MOVEIT:TIMED-OUT", "ManipulationFailed"));
	  lstFailureMapping.push_back(make_pair("CRAM-MOVEIT:PREEMPTED", "ManipulationFailed"));
	  lstFailureMapping.push_back(make_pair("CRAM-MOVEIT:START-STATE-IN-COLLISION", "ManipulationFailed"));
	  lstFailureMapping.push_back(make_pair("CRAM-MOVEIT:START-STATE-VIOLATES-PATH-CONSTRAINTS", "ManipulationFailed"));
	  lstFailureMapping.push_back(make_pair("CRAM-MOVEIT:GOAL-IN-COLLISION", "ManipulationFailed"));
	  lstFailureMapping.push_back(make_pair("CRAM-MOVEIT:GOAL-VIOLATES-PATH-CONSTRAINTS", "ManipulationFailed"));
	  lstFailureMapping.push_back(make_pair("CRAM-MOVEIT:GOAL-CONSTRAINTS-VIOLATED", "ManipulationFailed"));
	  lstFailureMapping.push_back(make_pair("CRAM-MOVEIT:INVALID-GROUP-NAME", "ManipulationFailed"));
	  lstFailureMapping.push_back(make_pair("CRAM-MOVEIT:INVALID-GOAL-CONSTRAINT", "ManipulationFailed"));
	  lstFailureMapping.push_back(make_pair("CRAM-MOVEIT:INVALID-ROBOT-STATE", "ManipulationFailed"));
	  lstFailureMapping.push_back(make_pair("CRAM-MOVEIT:INVALID-LINK-NAME", "ManipulationFailed"));
	  lstFailureMapping.push_back(make_pair("CRAM-MOVEIT:INVALID-OBJECT-NAME", "ManipulationFailed"));
	  lstFailureMapping.push_back(make_pair("CRAM-MOVEIT:FRAME-TRANSFORM-FAILURE", "ManipulationFailed"));
	  lstFailureMapping.push_back(make_pair("CRAM-MOVEIT:COLLISION-CHECKING-UNAVAILABLE", "ManipulationFailed"));
	  lstFailureMapping.push_back(make_pair("CRAM-MOVEIT:ROBOT-STATE-STALE", "ManipulationFailed"));
	  lstFailureMapping.push_back(make_pair("CRAM-MOVEIT:SENSOR-INFO-STALE", "ManipulationFailed"));
	  lstFailureMapping.push_back(make_pair("CRAM-MOVEIT:NO-IK-SOLUTION", "ManipulationFailed"));
	  lstFailureMapping.push_back(make_pair("CRAM-MOVEIT:NO-COLLISION-SHAPES-DEFINED", "ManipulationFailed"));
	  lstFailureMapping.push_back(make_pair("CRAM-MOVEIT:POSE-NOT-TRANSFORMABLE-INTO-LINK", "ManipulationFailed"));
	
	  for(list< pair<string, string> >::iterator itPair = lstFailureMapping.begin();
	      itPair != lstFailureMapping.end();
	      itPair++) {
	    pair<string, string> prCurrent = *itPair;
	  
	    if(strCondition == prCurrent.first) {
	      strFailureClass = prCurrent.second;
	      break;
	    }
	  }
	
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
	
	  // 	string strDesignatorID = ckvpObject->stringValue("__id");
	  string strOwlClass = "&knowrob;CameraImage";
	  strDot += "    <owl:namedIndividual rdf:about=\"&" + strNamespace + ";" + sts.str() + "\">\n";
	  //strDot += "        <knowrob:designator rdf:resource=\"&" + strNamespace + ";" + strDesignatorID + "\"/>\n";
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
    
      // NOTE(winkler): Don't generate properties for parent designators
      // (single lists are enough here)

      // list<string> lstParentIDs = this->parentDesignatorsForID(strID);
      // for(list<string>::iterator itID2 = lstParentIDs.begin();
      // 	itID2 != lstParentIDs.end();
      // 	itID2++) {
      //   string strID2 = *itID2;
      
      //   strDot += "        <knowrob:parentDesignator rdf:resource=\"&" + strNamespace + ";" + strID2 + "\"/>\n";
      // }
    
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

  string CExporterOwl::owlClassForNode(Node *ndNode, bool bClassOnly) {
    string strName = ndNode->title();
    string strPrefix = "&knowrob;";
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
	strClass = "Navigate";
      }
    } else if(strName.substr(0, 8) == "PERFORM-") {
      // This is the performance of probably a designator
      string strPerformer = strName.substr(8);
    
      if(strPerformer == "ACTION-DESIGNATOR") {
	strClass = "PerformActionDesignator"; // Added
      }
    } else if(strName == "UIMA-PERCEIVE") {
      strClass = "VisualPerception";
    } else if(strName == "AT-LOCATION") {
      strClass = "AtLocation";
    } else if(strName == "VOLUNTARY-BODY-MOVEMENT") {
      strClass = "VoluntaryBodyMovement";
    }
  
    return (bClassOnly ? "" : strPrefix) + strClass;
  }

  bool CExporterOwl::runExporter(CKeyValuePair* ckvpConfigurationOverlay) {
    this->renewUniqueIDs();
    
    if(this->outputFilename() != "") {
      string strOwl = "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n\n";
      // NOTE(winkler): This used to be `random'. Changed this due to
      // non-necessity of such a long namespace.
      // this->generateRandomIdentifier("namespace_", 8);
      string strNamespaceID = "log";
      string strNamespace = "http://ias.cs.tum.edu/kb/cram_log.owl";// + strNamespaceID;
      
      // Prepare content
      this->prepareEntities(strNamespaceID, strNamespace);
      
      // Generate source
      strOwl += this->generateOwlStringForNodes(this->nodes(), strNamespaceID, strNamespace);
      
      // Write the .owl file
      return this->writeToFile(strOwl);
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
