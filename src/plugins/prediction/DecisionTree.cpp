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


namespace beliefstate {
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
	std::string strJSON((istreambuf_iterator<char>(ifFile)),
			    (istreambuf_iterator<char>()));
	
	m_jsnDecisionTree->parse(strJSON);
	
	if(m_jsnDecisionTree->rootProperty()) {
	  bReturn = true;
	}
      }
    }
    
    return bReturn;
  }
  
  Property* DecisionTree::evaluate(CKeyValuePair* ckvpFeatures) {
    Property* prReturn = NULL;
    
    if(m_jsnDecisionTree) {
      if(m_jsnDecisionTree->rootProperty()) {
	prReturn = this->evaluate(m_jsnDecisionTree->rootProperty(), ckvpFeatures);
      }
    }
    
    return prReturn;
  }
  
  Property* DecisionTree::evaluate(Property* prTree, CKeyValuePair* ckvpFeatures) {
    Property* prResult = NULL;
    
    if(prTree && ckvpFeatures) {
      Property* prAction = prTree->namedSubProperty("relation");
	
      if(prAction) {
	// This is a relation, evaluate it
	bool bResult = false;
	Property* prOperator = NULL;
	Property* prVariable = NULL;
	Property* prValue = NULL;
	  
	// Operator: "="
	if((prOperator = prAction->namedSubProperty("=")) != NULL) {
	  prVariable = prOperator->namedSubProperty("variable");
	  prValue = prOperator->namedSubProperty("value");
	    
	  if(prVariable && prValue) {
	    // Check for equality: "="
	    if(ckvpFeatures->childForKey(prVariable->getString())) {
	      switch(prValue->type()) {
	      case Property::String: {
		std::string strFeature = ckvpFeatures->stringValue(prVariable->getString());
		bResult = (strFeature == prValue->getString());
		std::cout << "Checking " << prVariable->getString() << " (= " << strFeature << ") == " << prValue->getString() << " : "
			  << (bResult ? "True" : "False") << std::endl;
	      } break;
		  
	      case Property::Integer: {
		int nFeature = (int)ckvpFeatures->floatValue(prVariable->getString());
		bResult = (nFeature == prValue->getInteger());
		std::cout << "Checking " << prVariable->getString() << " (= " << nFeature << ") == " << prValue->getInteger() << " : "
			  << (bResult ? "True" : "False") << std::endl;
	      } break;
		  
	      case Property::Double: {
		float fFeature = ckvpFeatures->floatValue(prVariable->getString());
		bResult = (fFeature == (float)prValue->getDouble());
		  
		std::cout << "Checking " << prVariable->getString() << " (= " << fFeature << ") == " << (float)prValue->getDouble() << " : "
			  << (bResult ? "True" : "False") << std::endl;
	      } break;
		  
	      case Property::Boolean: {
		bool bFeature = (ckvpFeatures->floatValue(prVariable->getString()) != 0.0);
		bResult = (bFeature == prValue->getBoolean());
		std::cout << "Checking " << prVariable->getString() << " (= " << bFeature << ") == " << prValue->getBoolean() << " : "
			  << (bResult ? "True" : "False") << std::endl;
	      } break;
		  
	      default: {
		this->warn("Unknown property value type, not evaluating.");
	      } break;
	      }
	    } else {
	      this->missingFeature("=", prVariable->getString());
	    }
	  } else {
	    this->missingOperand("=");
	  }
	} else if((prOperator = prAction->namedSubProperty("<")) != NULL) {
	  // Operator: "<"
	  prVariable = prOperator->namedSubProperty("variable");
	  prValue = prOperator->namedSubProperty("value");
	    
	  if(prOperator) {
	    if(prVariable && prValue) {
	      if(ckvpFeatures->childForKey(prVariable->getString())) {
		// Check for less than: "<"
		switch(prValue->type()) {
		case Property::Integer: {
		  int nFeature = (int)ckvpFeatures->floatValue(prVariable->getString());
		  bResult = (nFeature < prValue->getInteger());
		  std::cout << "Checking " << prVariable->getString() << " (= " << nFeature << ") < " << prValue->getInteger() << " : "
			    << (bResult ? "True" : "False") << std::endl;
		} break;
		    
		case Property::Double: {
		  float fFeature = ckvpFeatures->floatValue(prVariable->getString());
		  bResult = (fFeature < (float)prValue->getDouble());
		  std::cout << "Checking " << prVariable->getString() << " (= " << fFeature << ") < " << (float)prValue->getDouble() << " : "
			    << (bResult ? "True" : "False") << std::endl;
		} break;
		    
		default: {
		  this->warn("Unknown property value type, not evaluating.");
		} break;
		}
	      } else {
		this->missingFeature("<", prVariable->getString());
	      }
	    } else {
	      this->missingOperand("<");
	    }
	  }
	} else if((prOperator = prAction->namedSubProperty("<=")) != NULL) {
	  prVariable = prOperator->namedSubProperty("variable");
	  prValue = prOperator->namedSubProperty("value");
	    
	  if(prVariable && prValue) {
	    if(ckvpFeatures->childForKey(prVariable->getString())) {
	      // Check for equality: "<="
	      switch(prValue->type()) {
	      case Property::Integer: {
		int nFeature = (int)ckvpFeatures->floatValue(prVariable->getString());
		bResult = (nFeature <= prValue->getInteger());
		std::cout << "Checking " << prVariable->getString() << " (= " << nFeature << ") <= " << prValue->getInteger() << " : "
			  << (bResult ? "True" : "False") << std::endl;
	      } break;
		  
	      case Property::Double: {
		float fFeature = ckvpFeatures->floatValue(prVariable->getString());
		bResult = (fFeature <= (float)prValue->getDouble());
		std::cout << "Checking " << prVariable->getString() << " (= " << fFeature << ") <= " << (float)prValue->getDouble() << " : "
			  << (bResult ? "True" : "False") << std::endl;
	      } break;
		  
	      default: {
		this->warn("Unknown property value type, not evaluating.");
	      } break;
	      }
	    } else {
	      this->missingFeature("<=", prVariable->getString());
	    }
	  } else {
	    this->missingOperand("<=");
	  }
	} else if((prOperator = prAction->namedSubProperty(">")) != NULL) {
	  prVariable = prOperator->namedSubProperty("variable");
	  prValue = prOperator->namedSubProperty("value");
	    
	  if(prVariable && prValue) {
	    if(ckvpFeatures->childForKey(prVariable->getString())) {
	      // Check for equality: ">"
	      switch(prValue->type()) {
	      case Property::Integer: {
		int nFeature = (int)ckvpFeatures->floatValue(prVariable->getString());
		bResult = (nFeature > prValue->getInteger());
		std::cout << "Checking " << prVariable->getString() << " (= " << nFeature << ") > " << prValue->getInteger() << " : "
			  << (bResult ? "True" : "False") << std::endl;
	      } break;
		  
	      case Property::Double: {
		float fFeature = ckvpFeatures->floatValue(prVariable->getString());
		bResult = (fFeature > (float)prValue->getDouble());
		std::cout << "Checking " << prVariable->getString() << " (= " << fFeature << ") > " << (float)prValue->getDouble() << " : "
			  << (bResult ? "True" : "False") << std::endl;
	      } break;
		  
	      default: {
		this->warn("Unknown property value type, not evaluating.");
	      } break;
	      }
	    } else {
	      this->missingFeature(">", prVariable->getString());
	    }
	  } else {
	    this->missingOperand(">");
	  }
	} else if((prOperator = prAction->namedSubProperty(">=")) != NULL) {
	  prVariable = prOperator->namedSubProperty("variable");
	  prValue = prOperator->namedSubProperty("value");
	    
	  if(prVariable && prValue) {
	    if(ckvpFeatures->childForKey(prVariable->getString())) {
	      // Check for equality: ">="
	      switch(prValue->type()) {
	      case Property::Integer: {
		int nFeature = (int)ckvpFeatures->floatValue(prVariable->getString());
		bResult = (nFeature >= prValue->getInteger());
		std::cout << "Checking " << prVariable->getString() << " (= " << nFeature << ") >= " << prValue->getInteger() << " : "
			  << (bResult ? "True" : "False") << std::endl;
	      } break;
		  
	      case Property::Double: {
		float fFeature = ckvpFeatures->floatValue(prVariable->getString());
		bResult = (fFeature >= (float)prValue->getDouble());
		std::cout << "Checking " << prVariable->getString() << " (= " << fFeature << ") >= " << (float)prValue->getDouble() << " : "
			  << (bResult ? "True" : "False") << std::endl;
	      } break;
		  
	      default: {
		this->warn("Unknown property value type, not evaluating.");
	      } break;
	      }
	    } else {
	      this->missingFeature(">=", prVariable->getString());
	    }
	  } else {
	    this->missingOperand(">=");
	  }
	} else if((prOperator = prAction->namedSubProperty("!=")) != NULL) {
	  prVariable = prOperator->namedSubProperty("variable");
	  prValue = prOperator->namedSubProperty("value");
	    
	  if(prVariable && prValue) {
	    if(ckvpFeatures->childForKey(prVariable->getString())) {
	      // Check for inequality: "!="
	      switch(prValue->type()) {
	      case Property::String: {
		std::string strFeature = ckvpFeatures->stringValue(prVariable->getString());
		bResult = (strFeature != prValue->getString());
		std::cout << "Checking " << prVariable->getString() << " (= " << strFeature << ") != " << prValue->getString() << " : "
			  << (bResult ? "True" : "False") << std::endl;
	      } break;
		  
	      case Property::Integer: {
		int nFeature = (int)ckvpFeatures->floatValue(prVariable->getString());
		bResult = (nFeature != prValue->getInteger());
		std::cout << "Checking " << prVariable->getString() << " (= " << nFeature << ") != " << prValue->getInteger() << " : "
			  << (bResult ? "True" : "False") << std::endl;
	      } break;
		  
	      case Property::Double: {
		float fFeature = ckvpFeatures->floatValue(prVariable->getString());
		bResult = (fFeature != (float)prValue->getDouble());
		std::cout << "Checking " << prVariable->getString() << " (= " << fFeature << ") != " << (float)prValue->getDouble() << " : "
			  << (bResult ? "True" : "False") << std::endl;
	      } break;
		  
	      case Property::Boolean: {
		bool bFeature = (ckvpFeatures->floatValue(prVariable->getString()) != 0.0);
		bResult = (bFeature != prValue->getBoolean());
		std::cout << "Checking " << prVariable->getString() << " (= " << bFeature << ") != " << prValue->getBoolean() << " : "
			  << (bResult ? "True" : "False") << std::endl;
	      } break;
		  
	      default: {
		this->warn("Unknown property value type, not evaluating.");
	      } break;
	      }
	    } else {
	      this->missingFeature("!=", prVariable->getString());
	    }
	  } else {
	    this->missingOperand("!=");
	  }
	}
	  
	prResult = prTree->namedSubProperty((bResult ? "true" : "false"));
	if(prResult) {
	  list<Property*> lstSubActions = prResult->subProperties();
	    
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
	std::cout << "Returning result:" << std::endl;
	prResult->print();
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
  
  std::vector<CKeyValuePair*> DecisionTree::invert(Property* prTargetResult, CKeyValuePair* ckvpFeatures) {
    std::vector<CKeyValuePair*> vecSolutions;
    
    // TODO(winkler): Implement the actual solver here. The result
    // needs to be a list of variable sets that consist of features
    // that need to be changed in order to get to the target result
    // property. Prior features are given in ckvpFeatures, which act
    // as a base for solving. Solutions need to be sorted by the
    // length of their solution path/amount of variables to change, in
    // an ascending manner.
    
    this->fail("Inverting decision trees is not implemented yet. You supplied the following information:");
    
    if(prTargetResult) {
      this->fail("Target Result:");
      prTargetResult->print();
    }
    
    if(ckvpFeatures) {
      std::string strFeatures = "";
      
      for(CKeyValuePair* ckvpFeature : ckvpFeatures->children()) {
	if(strFeatures != "") {
	  strFeatures += ", ";
	}
	
	strFeatures += ckvpFeature->key();
      }
      
      this->fail("Features: " + strFeatures);
    }
    
    //m_jsnDecisionTree->rootProperty()->print();
    
    // First, find all branches with the targetted result
    std::vector<Property*> vecSolutionsTemp = this->findBranchesWithResult(prTargetResult);
    for(Property* prSolution : vecSolutionsTemp) {
      std::cout << "Solution:" << std::endl;
      prSolution->print();
    }
    
    return vecSolutions;
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
	this->info("Found a valid solution.");
	vecSolutions.push_back(new Property(prStart));
      } else {
	// No, this was not the result we were looking for. Silently
	// terminate and return nothing.
      }
    } else {
      // This is not a leaf node. We need to recurse from here, and
      // interprete the returned results appropriately.
      
      // Try all branches except `relation', as that is (similar to
      // `result') reserved.
      for(Property* prBranch : prStart->subProperties()) {
	if(prBranch->key() != "relation") {
	  // Yes, this is a trieable branch.
	  std::vector<Property*> vecSubSolutions = this->findBranchesWithResult(prTargetResult, prBranch);
	  
	  // Process them here.
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
