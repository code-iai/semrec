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


#include <JSON.h>


namespace semrec {
  JSON::JSON() {
    m_prRootProperty = NULL;
  }
  
  JSON::~JSON() {
    if(m_prRootProperty) {
      delete m_prRootProperty;
      m_prRootProperty = NULL;
    }
  }
  
  void JSON::parse(std::string strJSON, std::string strMimeType) {
    if(strMimeType == "") {
      strMimeType = "application/json";
    }
    
    if(!m_prRootProperty) {
      delete m_prRootProperty;
    }
    
    m_prRootProperty = new Property("root", Property::Object);
    
    if(strMimeType == "application/json") {
      struct json_tokener* tok;
      struct json_object* jobj;
      enum json_tokener_error jteError;
      
      tok = json_tokener_new_ex(1000);
      
      if(!tok) {
	std::cerr << "Couldn't initialize json_tokener." << std::endl;
      } else {
	jobj = json_tokener_parse_ex(tok, strJSON.c_str(), strJSON.length());
	jteError = tok->err;
	
	if(jteError == json_tokener_success) {
	  if(jobj != NULL) {
	    this->parse(jobj, m_prRootProperty);
	  } else {
	    std::cerr << "Failed to parse JSON: " << json_tokener_error_desc(jteError) << std::endl;
	  }
	  
	  jobj = NULL;
	} else {
	  std::cerr << "Failed to parse JSON: " << json_tokener_error_desc(jteError) << std::endl;
	}
	
	json_tokener_free(tok);
      }
    } else {
      this->parseXML(strJSON);
    }
  }
  
  Property* JSON::parseXML(std::string strXML) {
    int nOffset = 0;
    
    return this->parseXML(strXML, nOffset);
  }
  
  Property* JSON::parseXML(std::string strXML, int& nOffset) {
    // TODO(winkler): Implement this.
  }
  
  void JSON::parse(json_object* jobj, Property* prParent) {
    switch(json_object_get_type(jobj)) {
    case json_type_boolean:
      prParent->set(Property::Boolean);
      prParent->set(json_object_get_boolean(jobj));
      break;
      
    case json_type_double:
      prParent->set(Property::Double);
      prParent->set(json_object_get_double(jobj));
      break;
      
    case json_type_int:
      prParent->set(Property::Integer);
      prParent->set(json_object_get_int(jobj));
      break;
      
    case json_type_string:
      prParent->set(Property::String);
      prParent->set(std::string(json_object_get_string(jobj)));
      break;
      
    case json_type_array:
      prParent->set(Property::Array);
      this->parseArray(jobj, NULL, prParent);
      break;
      
    case json_type_object: {
      json_object_object_foreach(jobj, key, val) {
        Property* prChild = new Property();
        prParent->addSubProperty(prChild);
        prChild->setKey(key);
        prChild->set(Property::Object);
	
        this->parse(val, prChild);
      }
    } break;
      
    case json_type_null: {
      std::cout << "Null?" << std::endl;
    } break;
      
    default:
      break;
    }
  }
  
  void JSON::parseArray(json_object* jobj, char* key, Property* prParent) {
    json_object* jarray = jobj, *jvalue;
    
    if(key) {
      jarray = json_object_object_get(jobj, key);
    }
    
    for(int nI = 0; nI < json_object_array_length(jarray); nI++) {
      Property* prChild = new Property();
      prChild->set(Property::Array);
      prParent->addSubProperty(prChild);
      
      if(key) {
        prChild->setKey(std::string(key));
      }
      
      jvalue = json_object_array_get_idx(jarray, nI);
      
      if(json_object_get_type(jvalue) == json_type_array) {
        this->parseArray(jvalue, NULL, prChild);
      } else {
        this->parse(jvalue, prChild);
      }
    }
  }
  
  Property* JSON::rootProperty() {
    return m_prRootProperty;
  }
  
  std::string JSON::encode(Property* prEncode) {
    std::string strEncoded = "";
    
    if(prEncode == NULL) {
      prEncode = m_prRootProperty;
    }
    
    if(prEncode->key() != "" && prEncode != m_prRootProperty) {
      strEncoded += "\"" + prEncode->key() + "\" : ";
    }
    
    switch(prEncode->type()) {
    case Property::String:
      strEncoded += "\"" + prEncode->getString() + "\"";
      break;
      
    case Property::Integer: {
      std::stringstream sts;
      sts << prEncode->getInteger();
      strEncoded += "\"" + sts.str() + "\"";
    } break;
      
    case Property::Double: {
      std::stringstream sts;
      sts << prEncode->getDouble();
      strEncoded += "\"" + sts.str() + "\"";
    } break;
      
    case Property::Boolean:
      strEncoded += std::string("\"") + (prEncode->getBoolean() ? "true" : "false") + "\"";
      break;
      
    case Property::Object: {
      strEncoded += "{";
      std::list<Property*> lstSubProperties = prEncode->subProperties();
      
      for(std::list<Property*>::iterator itP = lstSubProperties.begin();
          itP != lstSubProperties.end();
          itP++) {
        Property* prCurrent = *itP;
	
        if(itP != lstSubProperties.begin()) {
          strEncoded += ", ";
        }
	
        strEncoded += this->encode(prCurrent);
      }
      
      strEncoded += "}";
    } break;
      
    case Property::Array: {
      strEncoded += "[";
      std::list<Property*> lstSubProperties = prEncode->subProperties();
      
      for(std::list<Property*>::iterator itP = lstSubProperties.begin();
          itP != lstSubProperties.end();
          itP++) {
        Property* prCurrent = *itP;
	
        if(itP != lstSubProperties.begin()) {
          strEncoded += ", ";
        }
	
        strEncoded += this->encode(prCurrent);
      }
      
      strEncoded += "]";
    } break;

    default:
      break;
    }

    return strEncoded;
  }
}
