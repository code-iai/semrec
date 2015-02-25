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


#include <plugins/dotexporter/CExporterDot.h>


namespace semrec {
  CExporterDot::CExporterDot() {
  }
  
  CExporterDot::~CExporterDot() {
  }
  
  bool CExporterDot::runExporter(KeyValuePair* ckvpConfigurationOverlay) {
    if(this->outputFilename() != "") {
      this->renewUniqueIDs();
      int nMaxDetailLevel = this->configuration()->floatValue("max-detail-level");
      
      std::string strGraphID = this->generateRandomIdentifier("plangraph_");
      std::string strToplevelID = this->generateUniqueID("node_");
      
      std::string strDot = "digraph " + strGraphID + " {\n";
      
      strDot += "  " + strToplevelID + " [shape=doublecircle, style=bold, label=\"top-level\"];\n";
      
      int nMinusOne = -1;
      strDot += this->generateDotStringForNodes(this->nodes(), strToplevelID, nMinusOne);
      
      strDot += "}\n";
      
      return this->writeToFile(strDot);
    }
    
    return false;
  }
  
  int CExporterDot::countNodes(std::list<Node*> lstNodes) {
    int nCount = 0;
    
    for(Node* ndNode : lstNodes) {
      nCount++;
      
      nCount += this->countNodes(ndNode->subnodes());
    }
    
    return nCount;
  }
  
  bool CExporterDot::runSequentialExporter() {
    bool bReturnvalue = false;
    
    if(this->outputFilename() != "") {
      this->renewUniqueIDs();
      int nMaxDetailLevel = this->configuration()->floatValue("max-detail-level");
      
      std::string strGraphID = this->generateRandomIdentifier("plangraph_");
      std::string strToplevelID = this->generateUniqueID("node_");
      bReturnvalue = true;
      
      int nNodeCount = this->countNodes(m_lstNodes);
      
      for(int nI = 1; nI < nNodeCount + 1; nI++) {
	std::string strDot = "digraph " + strGraphID + " {\n";
	strDot += "  " + strToplevelID + " [shape=doublecircle, style=bold, label=\"top-level\"];\n";
	
	int nUseIndex = nI;
	strDot += this->generateDotStringForNodes(this->nodes(), strToplevelID, nUseIndex);
	
	strDot += "}\n";
	
	char acPaddedIndex[80];
	sprintf(acPaddedIndex, "%08d", nI - 1);
	
	if(!this->writeToFile(strDot, this->outputFilename() + "." + std::string(acPaddedIndex))) {
	  bReturnvalue = false;
	  
	  break;
	}
      }
      
      std::string strTimeTable = "";
      int nIndex = 0;
      
      for(std::string strTime : m_lstTimeTableTimePoints) {
	std::string strTimePoint = this->str(nIndex) + ", " + strTime;
	
	strTimeTable += strTimePoint + "\n";
	
	nIndex++;
      }
      
      this->writeToFile(strTimeTable, this->outputFilename() + ".timetable.csv");
    }
    
    return bReturnvalue;
  }
  
  std::string CExporterDot::generateDotStringForDescription(std::list<KeyValuePair*> lstDescription, int nTimeStart, int nTimeEnd) {
    std::string strDot = "";
    
    if(nTimeStart > -1) {
      strDot += "|{time-start | " + this->str(nTimeStart) + "}";
    }
    
    if(nTimeEnd > -1) {
      strDot += "|{time-end | " + this->str(nTimeEnd) + "}";
    }
    
    for(KeyValuePair* ckvpCurrent : lstDescription) {
      if(ckvpCurrent->key().at(0) != '_') {
	std::string strValue = "?";
	bool bEscape = true;
	
	if(ckvpCurrent->type() == KeyValuePair::ValueType::STRING) {
	  strValue = ckvpCurrent->stringValue();
	} else if(ckvpCurrent->type() == KeyValuePair::ValueType::FLOAT) {
	  strValue = this->str(ckvpCurrent->floatValue());
	} else if(ckvpCurrent->type() == KeyValuePair::ValueType::POSE) {
	  bEscape = false;
	  geometry_msgs::Pose psPose = ckvpCurrent->poseValue();
	  std::stringstream stsPS;
	  
	  stsPS << "|{position |{{x | " << psPose.position.x << "} "
		<< "|{y | " << psPose.position.y << "} "
		<< "|{z | " << psPose.position.z << "}}} "
		<< "|{orientation |{{x | " << psPose.orientation.x << "} "
		<< "|{y | " <<psPose.orientation.y << "} "
		<< "|{z | " << psPose.orientation.z << "} "
		<< "|{w | " << psPose.orientation.w << "}}}}";
	
	  strValue = stsPS.str();
	} else if(ckvpCurrent->type() == KeyValuePair::ValueType::POSESTAMPED) {
	  bEscape = false;
	  geometry_msgs::PoseStamped psPoseStamped = ckvpCurrent->poseStampedValue();
	  std::stringstream stsPS;
	  
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
	  // NOTE(winkler): This actually IS a valid warning. The only
	  // issue is that we don't have a proper failure handling
	  // mechanism here, yet. Also, it doesn't hurt the current
	  // logs.
	  
	  /*stringstream sts;
	  sts << (int)ckvpCurrent->type();
	  this->warn("Careful: unknown type code for field (" + sts.str() + ")");*/
	}
	
	if(bEscape) {
	  strValue = this->dotEscapeString(strValue);
	}
	
	strDot += "|{" + this->dotEscapeString(ckvpCurrent->key()) + " | " + strValue + "}";
      }
    }
  
    return strDot;
  }

  std::string CExporterDot::generateDotStringForNodes(std::list<Node*> lstNodes, std::string strParentID, int& nIndex) {
    std::string strDot = "";
    
    for(Node* ndCurrent : lstNodes) {
      if(this->nodeDisplayable(ndCurrent)) {
	std::string strNodeID = ndCurrent->uniqueID();
	
	std::string strFillColor;
	std::string strEdgeColor;
	
	if(ndCurrent->metaInformation()->floatValue("success") == 1) {
	  strFillColor = "#ddffdd";
	  strEdgeColor = "green";
	} else {
	  strFillColor = "#ffdddd";
	  strEdgeColor = "red";
	}
	
	std::string strParameters = this->generateDotStringForDescription(ndCurrent->description(),
									  ndCurrent->metaInformation()->floatValue("time-start"),
									  ndCurrent->metaInformation()->floatValue("time-end"));
	std::string strLabel = "{" + this->dotEscapeString(ndCurrent->title()) + strParameters + "}";
	bool bVisible = nIndex > 0 || nIndex == -1;
	
	if(nIndex > 0) {
	  nIndex--;
	  
	  if(nIndex == 0) {
	    // This stretch just ended
	    m_lstTimeTableTimePoints.push_back(ndCurrent->metaInformation()->stringValue("time-start"));
	  }
	}
	
	strDot += "\n  " + strNodeID + " [shape=Mrecord, style=" + (bVisible ? "filled" : "invis") + ", fillcolor=\"" + strFillColor + "\", label=\"" + strLabel + "\"];\n";
	strDot += "  edge [color=\"" + strEdgeColor + "\", label=\"\"" + (bVisible ? "" : ", style=invis") + "];\n";
	strDot += "  " + strParentID + " -> " + strNodeID + ";\n";
	
	// Images
	strDot += this->generateDotImagesStringForNode(ndCurrent, bVisible);
	
	// Objects
	strDot += this->generateDotObjectsStringForNode(ndCurrent, bVisible);
	
	// Subnodes
	strDot += this->generateDotStringForNodes(ndCurrent->subnodes(), strNodeID, nIndex);
      } else if(this->nodeHasValidDetailLevel(ndCurrent)) {
	// Node has valid detail level. So the failed displayability
	// was due to this and not due to a failed success/failure
	// check. Display its children and use this node's parent id
	// for their parent id.
	
	// Subnodes
	strDot += this->generateDotStringForNodes(ndCurrent->subnodes(), strParentID, nIndex);
      }
    }
    
    return strDot;
  }
  
  std::string CExporterDot::generateDotImagesStringForNode(Node *ndImages, bool bVisible) {
    std::string strDot = "";
    
    KeyValuePair *ckvpImages = ndImages->metaInformation()->childForKey("images");
    
    if(ckvpImages) {
      std::list<KeyValuePair*> lstChildren = ckvpImages->children();
      
      unsigned int unIndex = 0;
      for(KeyValuePair* ckvpChild : lstChildren) {
	std::string strOrigin = ckvpChild->stringValue("origin");
	std::string strFilename = ckvpChild->stringValue("filename");
	
	std::stringstream sts;
	sts << ndImages->uniqueID() << "_image_" << unIndex;
	
	strDot += "  " + sts.str() + " [" + (bVisible ? "" : "style=invis, ") + "shape=box, label=\"" + strOrigin + "\", width=\"6cm\", height=\"6cm\", fixedsize=true, imagescale=true, image=\"" + strFilename + "\"];\n";
	strDot += "  edge [" + (bVisible ? "" : std::string("style=invis, ")) + "color=\"black\", label=\"camera image\"];\n";
	strDot += "  " + sts.str() + " -> " + ndImages->uniqueID() + ";\n";
      }
    }
    
    return strDot;
  }
  
  std::string CExporterDot::generateDotObjectsStringForNode(Node *ndObjects, bool bVisible) {
    std::string strDot = "";
    
    KeyValuePair *ckvpObjects = ndObjects->metaInformation()->childForKey("objects");
    
    if(ckvpObjects) {
      std::list<KeyValuePair*> lstChildren = ckvpObjects->children();
      
      unsigned int unIndex = 0;
      for(KeyValuePair* ckvpChild : lstChildren) {
	std::string strDefClass = ckvpChild->stringValue("_class");
	std::string strDefProperty = ckvpChild->stringValue("_property");
	
	if(strDefClass == "") {
	  strDefClass = "object";
	}
	
	if(strDefProperty == "") {
	  strDefProperty = "knowrob:objectActedOn";
	}
	
	std::string strObjectID = strDefClass + "_" + ckvpChild->stringValue("__id");
	
	// std::stringstream sts;
	// sts << ndObjects->uniqueID() << "_object_" << unIndex;
	
	std::string strParameters = this->generateDotStringForDescription(ckvpChild->children());
	std::string strTitle = strObjectID;
	std::string strLabel = "{" + this->dotEscapeString(strTitle) + strParameters + "}";
	
	strDot += "  " + strObjectID + " [" + (bVisible ? "" : "style=invis, ") + "shape=Mrecord, label=\"" + strLabel + "\"];\n";
	strDot += "  edge [" + (bVisible ? "" : std::string("style=invis, ")) + "color=\"black\", label=\"" + strDefProperty + "\"];\n";
	strDot += "  " + strObjectID + " -> " + ndObjects->uniqueID() + ";\n";
      }
    }
    
    return strDot;
  }

  std::string CExporterDot::dotEscapeString(std::string strValue) {
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
