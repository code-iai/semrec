/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Institute for Artificial Intelligence,
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


#include <plugins/prediction/DecisionTree.h>


namespace semrec {
  DecisionTree::DecisionTree() {
    this->init();
  }
  
  DecisionTree::DecisionTree(std::string strFile) {
    this->init("", strFile);
  }
  
  DecisionTree::~DecisionTree() {
    if(m_jsnDecisionTree) {
      delete m_jsnDecisionTree;
    }
  }
  
  void DecisionTree::init(std::string strMessagePrefix, std::string strFile) {
    if(strMessagePrefix == "") {
      strMessagePrefix = "dtree-aux";
    }
    
    this->setMessagePrefixLabel(strMessagePrefix);
    
    m_jsnDecisionTree = new JSON();
  }
  
  bool DecisionTree::load(std::string strFile) {
    bool bReturn = false;
    
    if(strFile != "") {
      std::ifstream ifFile(strFile.c_str());
      
      if(ifFile.good()) {
	std::string strJSON((std::istreambuf_iterator<char>(ifFile)),
			    (std::istreambuf_iterator<char>()));
	
	m_jsnDecisionTree->parse(strJSON);
	
	if(m_jsnDecisionTree->rootProperty()) {
	  bReturn = true;
	}
      }
    }
    
    return bReturn;
  }
  
  Property* DecisionTree::evaluate(KeyValuePair* ckvpFeatures) {
    Property* prReturn = NULL;
    
    if(m_jsnDecisionTree) {
      if(m_jsnDecisionTree->rootProperty()) {
	prReturn = this->evaluate(m_jsnDecisionTree->rootProperty(), ckvpFeatures);
      }
    }
    
    return prReturn;
  }
  
  bool DecisionTree::relationSatisfied(Property* prRelation, KeyValuePair* ckvpFeatures) {
    // This is a relation, evaluate it
    bool bResult = false;
    Property* prOperator = NULL;
    Property* prVariable = NULL;
    Property* prValue = NULL;
    
    std::string strOperator = prRelation->key();
    prVariable = prRelation->namedSubProperty("variable");
    prValue = prRelation->namedSubProperty("value");

    if(strOperator != "" && prVariable && prValue) {
      if(ckvpFeatures->childForKey(prVariable->getString())) {
	if(strOperator == "=") { // Operator: "="
	  switch(prValue->type()) {
	  case Property::String: {
	    std::string strFeature = ckvpFeatures->stringValue(prVariable->getString());
	    bResult = (strFeature == prValue->getString());
	  } break;
	    
	  case Property::Integer: {
	    int nFeature = (int)ckvpFeatures->floatValue(prVariable->getString());
	    bResult = (nFeature == prValue->getInteger());
	  } break;
	    
	  case Property::Double: {
	    float fFeature = ckvpFeatures->floatValue(prVariable->getString());
	    bResult = (fFeature == (float)prValue->getDouble());
	  } break;
	    
	  case Property::Boolean: {
	    bool bFeature = (ckvpFeatures->floatValue(prVariable->getString()) != 0.0);
	    bResult = (bFeature == prValue->getBoolean());
	  } break;
	    
	  default: {
	    this->warn("Unknown property value type, not evaluating.");
	  } break;
	  }
	} else if(strOperator == "<") { // Operator: "<"
	  switch(prValue->type()) {
	  case Property::Integer: {
	    int nFeature = (int)ckvpFeatures->floatValue(prVariable->getString());
	    bResult = (nFeature < prValue->getInteger());
	  } break;
	    
	  case Property::Double: {
	    float fFeature = ckvpFeatures->floatValue(prVariable->getString());
	    bResult = (fFeature < (float)prValue->getDouble());
	  } break;
	    
	  default: {
	    this->warn("Unknown property value type, not evaluating.");
	  } break;
	  }
	} else if(strOperator == "<=") { // Operator: "<="
	  switch(prValue->type()) {
	  case Property::Integer: {
	    int nFeature = (int)ckvpFeatures->floatValue(prVariable->getString());
	    bResult = (nFeature <= prValue->getInteger());
	  } break;
	  
	  case Property::Double: {
	    float fFeature = ckvpFeatures->floatValue(prVariable->getString());
	    bResult = (fFeature <= (float)prValue->getDouble());
	  } break;
	  
	  default: {
	    this->warn("Unknown property value type, not evaluating.");
	  } break;
	  }
	} else if(strOperator == ">") { // Operator: ">"
	  switch(prValue->type()) {
	  case Property::Integer: {
	    int nFeature = (int)ckvpFeatures->floatValue(prVariable->getString());
	    bResult = (nFeature > prValue->getInteger());
	  } break;
	  
	  case Property::Double: {
	    float fFeature = ckvpFeatures->floatValue(prVariable->getString());
	    bResult = (fFeature > (float)prValue->getDouble());
	  } break;
	  
	  default: {
	    this->warn("Unknown property value type, not evaluating.");
	  } break;
	  }
	} else if(strOperator == ">=") { // Operator: ">="
	  switch(prValue->type()) {
	  case Property::Integer: {
	    int nFeature = (int)ckvpFeatures->floatValue(prVariable->getString());
	    bResult = (nFeature >= prValue->getInteger());
	  } break;
	  
	  case Property::Double: {
	    float fFeature = ckvpFeatures->floatValue(prVariable->getString());
	    bResult = (fFeature >= (float)prValue->getDouble());
	  } break;
	  
	  default: {
	    this->warn("Unknown property value type, not evaluating.");
	  } break;
	  }
	} else if(strOperator == "!=") { // Operator: "!="
	  switch(prValue->type()) {
	  case Property::String: {
	    std::string strFeature = ckvpFeatures->stringValue(prVariable->getString());
	    bResult = (strFeature != prValue->getString());
	  } break;
	  
	  case Property::Integer: {
	    int nFeature = (int)ckvpFeatures->floatValue(prVariable->getString());
	    bResult = (nFeature != prValue->getInteger());
	  } break;
	  
	  case Property::Double: {
	    float fFeature = ckvpFeatures->floatValue(prVariable->getString());
	    bResult = (fFeature != (float)prValue->getDouble());
	  } break;
	  
	  case Property::Boolean: {
	    bool bFeature = (ckvpFeatures->floatValue(prVariable->getString()) != 0.0);
	    bResult = (bFeature != prValue->getBoolean());
	  } break;
	  
	  default: {
	    this->warn("Unknown property value type, not evaluating.");
	  } break;
	}
	} else {
	  this->warn("Unknown operator: '" + strOperator + "'");
	}
      } else {
	this->missingFeature(strOperator, prVariable->getString());
      }
    } else {
      this->warn("Missing operator, variable, or value.");
    }
    
    return bResult;
  }
  
  Property* DecisionTree::evaluate(Property* prTree, KeyValuePair* ckvpFeatures) {
    Property* prResult = NULL;
    
    if(prTree && ckvpFeatures) {
      Property* prAction = prTree->namedSubProperty("relation");
      
      if(prAction) {
	bool bResult = this->relationSatisfied(prAction->subProperties().front(), ckvpFeatures);
	
	prResult = prTree->namedSubProperty((bResult ? "true" : "false"));
	if(prResult) {
	  std::list<Property*> lstSubActions = prResult->subProperties();
	    
	  for(Property* prSubAction : lstSubActions) {
	    prResult = this->evaluate(prSubAction, ckvpFeatures);
	      
	    if(prResult) {
	      break;
	    }
	  }
	}
      } else if((prAction = prTree->namedSubProperty("result")) != NULL) {
	// This is a leaf (i.e. result), return it
	prResult = prAction;
      } else if(prTree->type() == Property::Array) {
	for(Property* prSubAction : prTree->subProperties()) {
	  prResult = this->evaluate(prSubAction, ckvpFeatures);
	    
	  if(prResult) {
	    break;
	  }
	}
      } else {
	// This is something unknown, report it
	this->warn("Cannot evaluate decision tree branch:");
	prTree->print();
      }
    }
      
    return prResult;
  }
  
  std::vector<KeyValuePair*> DecisionTree::invert(Property* prTargetResult, KeyValuePair* ckvpFeatures) {
    std::vector<KeyValuePair*> vecSolutions;
    
    // First, find all branches with the target result.
    std::vector<Property*> vecSolutionsTemp = this->findBranchesWithResult(prTargetResult);
    std::vector< std::vector<Property*> > vecSolutionsStraightened = this->straightenResultBranches(vecSolutionsTemp);
    for(Property* prSolution : vecSolutionsTemp) {
      delete prSolution;
    }
    
    // Second, filter out any relations that are already satisfied via
    // ckvpFeatures.
    std::vector< std::vector<Property*> > vecSols;
    
    for(std::vector<Property*> vecSolution : vecSolutionsStraightened) {
      std::vector<Property*> vecSolutionNew;
      
      for(std::vector<Property*>::iterator itSolutionStep = vecSolution.begin();
	  itSolutionStep != vecSolution.end();
	  ++itSolutionStep) {
	if(!this->relationSatisfied(*itSolutionStep, ckvpFeatures)) {
	  vecSolutionNew.push_back(*itSolutionStep);
	} else {
	  delete *itSolutionStep;
	}
      }
      
      vecSols.push_back(vecSolutionNew);
    }
    
    for(std::vector<Property*> vecSolution : vecSols) {
      if(vecSolution.size() > 0) {
	KeyValuePair* ckvpSolution = new KeyValuePair("solution", KeyValuePair::ValueType::LIST);
      
	for(Property* prSolutionStep : vecSolution) {
	  KeyValuePair* ckvpStep = new KeyValuePair("step", KeyValuePair::ValueType::LIST);
	  ckvpSolution->addChild(ckvpStep);
	  
	  ckvpStep->setValue("relation", prSolutionStep->key());
	  
	  std::list<std::string> lstCopyFields;
	  lstCopyFields.push_back("variable");
	  lstCopyFields.push_back("value");
	
	  for(std::string strFieldName : lstCopyFields) {
	    Property* prField = prSolutionStep->namedSubProperty(strFieldName);
	  
	    switch(prField->type()) {
	    case Property::String:
	      ckvpStep->setValue(strFieldName, prField->getString());
	      break;

	    case Property::Integer:
	      ckvpStep->setValue(strFieldName, prField->getInteger());
	      break;

	    case Property::Double:
	      ckvpStep->setValue(strFieldName, prField->getDouble());
	      break;

	    case Property::Boolean:
	      ckvpStep->setValue(strFieldName, prField->getBoolean());
	      break;
	    
	    default:
	      this->warn("Unknown property type.");
	      break;
	    }
	  }
	
	  delete prSolutionStep;
	}
      
	vecSolutions.push_back(ckvpSolution);
      }
    }
    
    return vecSolutions;
  }
  
  std::vector< std::vector<Property*> > DecisionTree::straightenResultBranches(std::vector<Property*> vecSolutionBranches) {
    std::vector< std::vector<Property*> > vecSolutionsStraightened;
    
    for(Property* prSolutionBranch : vecSolutionBranches) {
      vecSolutionsStraightened.push_back(this->straightenResultBranch(prSolutionBranch));
    }
    
    return vecSolutionsStraightened;
  }
  
  std::vector<Property*> DecisionTree::straightenResultBranch(Property* prSolutionBranch) {
    std::vector<Property*> vecStraightened;
    
    // We're only interested in relations, as we already know the
    // target result.
    if(prSolutionBranch->key() == "relation") {
      if(prSolutionBranch->subProperties().size() == 1) {
	vecStraightened.push_back(new Property(prSolutionBranch->subProperties().front())); // Copy
      }
    }
    
    for(Property* prSub : prSolutionBranch->subProperties()) {
      std::vector<Property*> vecSubStraightened = this->straightenResultBranch(prSub);
      
      for(Property* prSubStraightened : vecSubStraightened) {
	vecStraightened.push_back(new Property(prSubStraightened));
	delete prSubStraightened; // Got copied
      }
    }
    
    return vecStraightened;
  }
  
  std::vector<Property*> DecisionTree::findBranchesWithResult(Property* prTargetResult, Property* prStart) {
    std::vector<Property*> vecSolutions;
    
    if(!prStart) {
      prStart = m_jsnDecisionTree->rootProperty();
    }
    
    Property* prResult = prStart->namedSubProperty("result");
    if(prResult) {
      // This is the result leaf, we're at our destination. Check its
      // value vs. prTargetResult.
      
      // TODO(winkler): Extend so that not only string results can be
      // used (i.e. check for the type of both `Property's and compare
      // them accordingly.
      if(prResult->getString() == prTargetResult->getString()) {
	// Yes, this is indeed the result we're looking for. Add a
	// copy to the solutions and terminate.
	vecSolutions.push_back(new Property(prStart));
      } else {
	// No, this was not the result we were looking for. Silently
	// terminate and return nothing.
      }
    } else {
      // This is not a leaf node. We need to recurse from here, and
      // interprete the returned results appropriately.
      
      // Preserve for later.
      Property* prRelation = prStart->namedSubProperty("relation");
      
      // Try all branches except `relation', as that is (similar to
      // `result') reserved.
      for(Property* prBranch : prStart->subProperties()) {
	if(prBranch->key() != "relation") {
	  // Yes, this is a trieable branch.
	  std::vector<Property*> vecSubSolutions = this->findBranchesWithResult(prTargetResult, prBranch);
	  
	  // Process them here.
	  for(Property* prSubSolution : vecSubSolutions) {
 	    Property* prSolutionBranch = new Property("", Property::Object);
	    
	    if(prRelation) {
	      prSolutionBranch->addSubProperty(new Property(prRelation));
	    }
	    
	    prSolutionBranch->addSubProperty(new Property(prSubSolution));
	    delete prSubSolution; // Not used anymore, got deep-copied
	    
	    vecSolutions.push_back(prSolutionBranch);
	  }
	}
      }
    }
    
    return vecSolutions;
  }
  
  void DecisionTree::missingFeature(std::string strOperator, std::string strFeatureName) {
    this->warn("Decision Tree: '" + strFeatureName + "' missing from feature space while evaluating operator '" + strOperator + "'.");
  }
  
  void DecisionTree::missingOperand(std::string strOperator) {
    this->warn("Decision Tree: Operand missing while evaluating operator '" + strOperator + "'.");
  }
}
