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
	  strValue = this->str(ckvpCurrent->floatValue());
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
