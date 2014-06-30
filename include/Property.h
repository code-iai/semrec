#ifndef __PROPERTY_H__
#define __PROPERTY_H__


// System
#include <iostream>
#include <list>
#include <string>

using namespace std;


namespace beliefstate {
  class Property {
  public:
    typedef enum {
      String,
      Integer,
      Double,
      Boolean,
      Array,
      Object
    } PropertyType;

  private:
    list<Property*> m_lstSubProperties;
    string m_strKey;
    string m_strValue;
    double m_dValue;
    PropertyType m_ptType;

  protected:
  public:
    Property(string strKey = "", PropertyType ptType = String);
    ~Property();

    void setKey(string strKey);
    string key();

    void set(PropertyType ptType);
    PropertyType type();

    void set(string strValue);
    string getString();

    void set(int nValue);
    int getInteger();

    void set(double dValue);
    double getDouble();

    void set(bool bValue);
    bool getBoolean();

    void addSubProperty(Property* prSubProperty);
    list<Property*> subProperties();

    void print(int nIndentationLevel = 0, bool bPrintKey = true);

    Property* namedSubProperty(string strKey);
  };
}


#endif /* __PROPERTY_H__ */
