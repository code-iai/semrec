#include <JSON.h>


namespace beliefstate {
  JSON::JSON() {
    m_prRootProperty = NULL;
  }

  JSON::~JSON() {
    if(m_prRootProperty) {
      delete m_prRootProperty;
      m_prRootProperty = NULL;
    }
  }

  void JSON::parse(string strJSON, string strMimeType) {
    if(strMimeType == "") {
      strMimeType = "application/json";
    }
    
    if(!m_prRootProperty) {
      delete m_prRootProperty;
    }

    m_prRootProperty = new Property("root", Property::Object);
    
    if(strMimeType == "application/json") {
      json_object* jobj = json_tokener_parse(strJSON.c_str());
      this->parse(jobj, m_prRootProperty);
    } else {
      this->parseXML(strJSON);
    }
  }
  
  Property* JSON::parseXML(string strXML) {
    int nOffset = 0;
    
    return this->parseXML(strXML, nOffset);
  }
  
  Property* JSON::parseXML(string strXML, int& nOffset) {
    
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
      prParent->set(string(json_object_get_string(jobj)));
      break;

    case json_type_array:
      prParent->set(Property::Array);
      this->parseArray(jobj, NULL, prParent);
      break;

    case json_type_object:
      json_object_object_foreach(jobj, key, val) {
        Property* prChild = new Property();
        prParent->addSubProperty(prChild);
        prChild->setKey(key);
        prChild->set(Property::Object);

        this->parse(val, prChild);
      }
      break;

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
        prChild->setKey(string(key));
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

  string JSON::encode(Property* prEncode) {
    string strEncoded = "";

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
      stringstream sts;
      sts << prEncode->getInteger();
      strEncoded += "\"" + sts.str() + "\"";
    } break;

    case Property::Double: {
      stringstream sts;
      sts << prEncode->getDouble();
      strEncoded += "\"" + sts.str() + "\"";
    } break;

    case Property::Boolean:
      strEncoded += string("\"") + (prEncode->getBoolean() ? "true" : "false") + "\"";
      break;

    case Property::Object: {
      strEncoded += "{";
      list<Property*> lstSubProperties = prEncode->subProperties();

      for(list<Property*>::iterator itP = lstSubProperties.begin();
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
      list<Property*> lstSubProperties = prEncode->subProperties();

      for(list<Property*>::iterator itP = lstSubProperties.begin();
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
