#ifndef __JSON_H__
#define __JSON_H__


// System
#include <string>
#include <iostream>
#include <sstream>

using namespace std;


// LibJSON
#include <json/json.h>


// Private
#include <Property.h>


namespace beliefstate {
  class JSON {
  private:
    Property* m_prRootProperty;

  protected:
  public:
    JSON();
    ~JSON();

    void parse(string strJSON, string strMimeType = "");
    void parse(json_object* jobj, Property* prParent);
    void parseArray(json_object* jobj, char* key, Property* prParent);

    Property* parseXML(string strXML);
    Property* parseXML(string strXML, int& nOffset);
    
    Property* rootProperty();

    string encode(Property* prEncode = NULL);
  };
}


#endif /* __JSON_H__ */
