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


namespace beliefstate {
  Property::Property(string strKey, PropertyType ptType) {
    m_strValue = "";
    m_dValue = 0.0f;

    this->set(ptType);
    this->setKey(strKey);
  }

  Property::~Property() {
    for(list<Property*>::iterator itP = m_lstSubProperties.begin();
        itP != m_lstSubProperties.end();
        itP++) {
      delete *itP;
    }

    m_lstSubProperties.clear();
  }

  void Property::setKey(string strKey) {
    m_strKey = strKey;
  }

  string Property::key() {
    return m_strKey;
  }

  void Property::set(Property::PropertyType ptType) {
    m_ptType = ptType;
  }

  Property::PropertyType Property::type() {
    return m_ptType;
  }

  void Property::set(string strValue) {
    m_strValue = strValue;
  }

  string Property::getString() {
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

  list<Property*> Property::subProperties() {
    return m_lstSubProperties;
  }

  void Property::print(int nIndentationLevel, bool bPrintKey) {
    string strIndent = "";
    for(int nI = 0; nI < nIndentationLevel; nI++) {
      strIndent += " ";
    }
    
    cout << strIndent;
    if(bPrintKey) {
      cout << this->key() << ": ";
    }
    
    switch(this->type()) {
    case String:
      cout << this->getString() << endl;
      break;

    case Integer:
      cout << this->getInteger() << endl;
      break;

    case Boolean:
      cout << this->getBoolean() << endl;
      break;

    case Double:
      cout << this->getDouble() << endl;
      break;

    case Array:
      cout << endl;
      
      for(Property* prCurrent : m_lstSubProperties) {
        prCurrent->print(nIndentationLevel + 1);//, false);
      }
      break;

    case Object:
      cout << endl;

      for(Property* prCurrent : m_lstSubProperties) {
        prCurrent->print(nIndentationLevel + 1);
      }
      break;
    }
  }

  Property* Property::namedSubProperty(string strKey) {
    for(list<Property*>::iterator itP = m_lstSubProperties.begin();
        itP != m_lstSubProperties.end();
        itP++) {
      Property* prCurrent = *itP;

      if(prCurrent->key() == strKey) {
        return prCurrent;
      }
    }

    return NULL;
  }
}
