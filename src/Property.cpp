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


#include <Property.h>


namespace semrec {
  Property::Property(std::string strKey, PropertyType ptType) {
    m_strValue = "";
    m_dValue = 0.0f;
    
    this->set(ptType);
    this->setKey(strKey);
  }
  
  Property::Property(Property* prTemplate, bool bDeepCopy) {
    // Copy constructor
    this->set(prTemplate->type());
    this->setKey(prTemplate->key());
    
    switch(prTemplate->type()) {
    case String: {
      this->set(prTemplate->getString());
    } break;
      
    case Integer: {
      this->set(prTemplate->getInteger());
    } break;
      
    case Double: {
      this->set(prTemplate->getDouble());
    } break;
      
    case Boolean: {
      this->set(prTemplate->getBoolean());
    } break;
      
    case Object:
    case Array: {
      if(bDeepCopy) {
	for(Property* prSubProperty : prTemplate->subProperties()) {
	  this->addSubProperty(new Property(prSubProperty));
	}
      }
    } break;
    }
  }
  
  Property::~Property() {
    for(Property* prDelete : m_lstSubProperties) {
      delete prDelete;
    }
    
    m_lstSubProperties.clear();
  }
  
  void Property::setKey(std::string strKey) {
    m_strKey = strKey;
  }
  
  std::string Property::key() {
    return m_strKey;
  }
  
  void Property::set(Property::PropertyType ptType) {
    m_ptType = ptType;
  }
  
  Property::PropertyType Property::type() {
    return m_ptType;
  }
  
  void Property::set(std::string strValue) {
    m_strValue = strValue;
  }
  
  std::string Property::getString() {
    return m_strValue;
  }
  
  void Property::set(int nValue) {
    m_dValue = nValue;
  }
  
  int Property::getInteger() {
    return (int)m_dValue;
  }
  
  void Property::set(double dValue) {
    m_dValue = dValue;
  }
  
  double Property::getDouble() {
    return m_dValue;
  }
  
  void Property::set(bool bValue) {
    m_dValue = (bValue ? 1 : 0);
  }
  
  bool Property::getBoolean() {
    return (m_dValue != 0);
  }
  
  void Property::addSubProperty(Property* prSubProperty) {
    m_lstSubProperties.push_back(prSubProperty);
  }
  
  std::list<Property*> Property::subProperties() {
    return m_lstSubProperties;
  }
  
  void Property::print(int nIndentationLevel, bool bPrintKey) {
    std::string strIndent = "";
    
    for(int nI = 0; nI < nIndentationLevel; nI++) {
      strIndent += " ";
    }
    
    std::cout << strIndent;
    
    if(bPrintKey) {
      std::cout << this->key() << ": ";
    }
    
    switch(this->type()) {
    case String:
      std::cout << "\"" << this->getString() << "\"" << std::endl;
      break;
      
    case Integer:
      std::cout << this->getInteger() << std::endl;
      break;
      
    case Boolean:
      std::cout << this->getBoolean() << std::endl;
      break;
      
    case Double:
      std::cout << this->getDouble() << std::endl;
      break;
      
    case Array:
      std::cout << std::endl;
      
      for(Property* prCurrent : m_lstSubProperties) {
        prCurrent->print(nIndentationLevel + 1);
      }
      break;
      
    case Object:
      std::cout << std::endl;
      
      for(Property* prCurrent : m_lstSubProperties) {
        prCurrent->print(nIndentationLevel + 1);
      }
      break;
    }
  }
  
  Property* Property::namedSubProperty(std::string strKey) {
    for(Property* prCurrent : m_lstSubProperties) {
      if(prCurrent->key() == strKey) {
        return prCurrent;
      }
    }
    
    return NULL;
  }
}
