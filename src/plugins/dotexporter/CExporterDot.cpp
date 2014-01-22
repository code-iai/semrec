#include <plugins/dotexporter/CExporterDot.h>


namespace beliefstate {
  CExporterDot::CExporterDot() {
  }

  CExporterDot::~CExporterDot() {
  }

  bool CExporterDot::runExporter(CKeyValuePair* ckvpConfigurationOverlay) {
    this->renewUniqueIDs();
    
    if(this->outputFilename() != "") {
      string strGraphID = this->generateRandomIdentifier("plangraph_", 8);
      string strToplevelID = this->generateUniqueID("node_", 8);
      
      string strDot = "digraph " + strGraphID + " {\n";
      
      strDot += "  " + strToplevelID + " [shape=doublecircle, style=bold, label=\"top-level\"];\n";
      
      strDot += this->generateDotStringForNodes(this->nodes(), strToplevelID);
      
      strDot += "}\n";
      
      return this->writeToFile(strDot);
    }
    
    return false;
  }
  
  string CExporterDot::generateDotStringForDescription(list<CKeyValuePair*> lstDescription) {
    string strDot = "";
  
    for(list<CKeyValuePair*>::iterator itPair = lstDescription.begin();
	itPair != lstDescription.end();
	itPair++) {
      CKeyValuePair *ckvpCurrent = *itPair;
    
      if(ckvpCurrent->key().at(0) != '_') {
	string strValue = "?";
	bool bEscape = true;
      
	if(ckvpCurrent->type() == STRING) {
	  strValue = ckvpCurrent->stringValue();
	} else if(ckvpCurrent->type() == FLOAT) {
	  stringstream sts;
	  sts << ckvpCurrent->floatValue();
	  strValue = sts.str();
	} else if(ckvpCurrent->type() == POSE) {
	  bEscape = false;
	  geometry_msgs::Pose psPose = ckvpCurrent->poseValue();
	  stringstream stsPS;
	
	  stsPS << "|{position |{{x | " << psPose.position.x << "} "
		<< "|{y | " << psPose.position.y << "} "
		<< "|{z | " << psPose.position.z << "}}} "
		<< "|{orientation |{{x | " << psPose.orientation.x << "} "
		<< "|{y | " <<psPose.orientation.y << "} "
		<< "|{z | " << psPose.orientation.z << "} "
		<< "|{w | " << psPose.orientation.w << "}}}}";
	
	  strValue = stsPS.str();
	} else if(ckvpCurrent->type() == POSESTAMPED) {
	  bEscape = false;
	  geometry_msgs::PoseStamped psPoseStamped = ckvpCurrent->poseStampedValue();
	  stringstream stsPS;
	
	  stsPS << "{{frame-id | \\\"" << psPoseStamped.header.frame_id << "\\\"} "
		<< "|{position |{{x | " << psPoseStamped.pose.position.x << "} "
		<< "|{y | " << psPoseStamped.pose.position.y << "} "
		<< "|{z | " << psPoseStamped.pose.position.z << "}}} "
		<< "|{orientation |{{x | " << psPoseStamped.pose.orientation.x << "} "
		<< "|{y | " <<psPoseStamped.pose.orientation.y << "} "
		<< "|{z | " << psPoseStamped.pose.orientation.z << "} "
		<< "|{w | " << psPoseStamped.pose.orientation.w << "}}}}";
	
	  strValue = stsPS.str();
	} else {
	  stringstream sts;
	  sts << (int)ckvpCurrent->type();
	  this->warn("Careful: unknown type code for field (" + sts.str() + ")");
	}
	
	if(bEscape) {
	  strValue = this->dotEscapeString(strValue);
	}
	
	strDot += "|{" + this->dotEscapeString(ckvpCurrent->key()) + " | " + strValue + "}";
      }
    }
  
    return strDot;
  }

  string CExporterDot::generateDotStringForNodes(list<Node*> lstNodes, string strParentID) {
    string strDot = "";
  
    for(list<Node*>::iterator itNode = lstNodes.begin();
	itNode != lstNodes.end();
	itNode++) {
      Node *ndCurrent = *itNode;
    
      if(this->nodeDisplayable(ndCurrent)) {
	string strNodeID = ndCurrent->uniqueID();
    
	string strFillColor;
	string strEdgeColor;
	if(ndCurrent->metaInformation()->floatValue("success") == 1) {
	  strFillColor = "#ddffdd";
	  strEdgeColor = "green";
	} else {
	  strFillColor = "#ffdddd";
	  strEdgeColor = "red";
	}
    
	string strParameters = this->generateDotStringForDescription(ndCurrent->description());
	string strLabel = "{" + this->dotEscapeString(ndCurrent->title()) + strParameters + "}";
    
	strDot += "\n  " + strNodeID + " [shape=Mrecord, style=filled, fillcolor=\"" + strFillColor + "\", label=\"" + strLabel + "\"];\n";
	strDot += "  edge [color=\"" + strEdgeColor + "\", label=\"\"];\n";
	strDot += "  " + strParentID + " -> " + strNodeID + ";\n";
      
	// Images
	strDot += this->generateDotImagesStringForNode(ndCurrent);
      
	// Objects
	strDot += this->generateDotObjectsStringForNode(ndCurrent);
      
	// Subnodes
	strDot += this->generateDotStringForNodes(ndCurrent->subnodes(), strNodeID);
      }
    }
  
    return strDot;
  }

  string CExporterDot::generateDotImagesStringForNode(Node *ndImages) {
    string strDot = "";
  
    CKeyValuePair *ckvpImages = ndImages->metaInformation()->childForKey("images");
  
    if(ckvpImages) {
      list<CKeyValuePair*> lstChildren = ckvpImages->children();
  
      unsigned int unIndex = 0;
      for(list<CKeyValuePair*>::iterator itChild = lstChildren.begin();
	  itChild != lstChildren.end();
	  itChild++, unIndex++) {
	CKeyValuePair *ckvpChild = *itChild;
    
	string strOrigin = ckvpChild->stringValue("origin");
	string strFilename = ckvpChild->stringValue("filename");
    
	stringstream sts;
	sts << ndImages->uniqueID() << "_image_" << unIndex;
    
	strDot += "  " + sts.str() + " [shape=box, label=\"" + strOrigin + "\", width=\"6cm\", height=\"6cm\", fixedsize=true, imagescale=true, image=\"" + strFilename + "\"];\n";
	strDot += "  edge [color=\"black\", label=\"camera image\"];\n";
	strDot += "  " + sts.str() + " -> " + ndImages->uniqueID() + ";\n";
      }
    }
  
    return strDot;
  }

  string CExporterDot::generateDotObjectsStringForNode(Node *ndObjects) {
    string strDot = "";
  
    CKeyValuePair *ckvpObjects = ndObjects->metaInformation()->childForKey("objects");
  
    if(ckvpObjects) {
      list<CKeyValuePair*> lstChildren = ckvpObjects->children();
  
      unsigned int unIndex = 0;
      for(list<CKeyValuePair*>::iterator itChild = lstChildren.begin();
	  itChild != lstChildren.end();
	  itChild++, unIndex++) {
	CKeyValuePair *ckvpChild = *itChild;
    
	stringstream sts;
	sts << ndObjects->uniqueID() << "_object_" << unIndex;
    
	string strParameters = this->generateDotStringForDescription(ckvpChild->children());
	string strTitle = "Some Object";
	string strLabel = "{" + this->dotEscapeString(strTitle) + strParameters + "}";
    
	strDot += "  " + sts.str() + " [shape=Mrecord, label=\"" + strLabel + "\"];\n";
	strDot += "  edge [color=\"black\", label=\"associated object\"];\n";
	strDot += "  " + sts.str() + " -> " + ndObjects->uniqueID() + ";\n";
      }
    }
  
    return strDot;
  }

  string CExporterDot::dotEscapeString(string strValue) {
    strValue = this->replaceString(strValue, "\n", "\\n");
    strValue = this->replaceString(strValue, "{", "\\{");
    strValue = this->replaceString(strValue, "}", "\\}");
    strValue = this->replaceString(strValue, "<", "\\<");
    strValue = this->replaceString(strValue, ">", "\\>");
    strValue = this->replaceString(strValue, "\"", "\\\"");
    strValue = this->replaceString(strValue, "|", "\\|");
  
    return strValue;
  }
}
