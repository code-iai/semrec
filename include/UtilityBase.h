#ifndef __UTILITY_BASE_H__
#define __UTILITY_BASE_H__


// System
#include <iostream>
#include <string>
#include <fstream>

using namespace std;


namespace beliefstate {
  class UtilityBase {
  private:
    string m_strMessagePrefixLabel;
    
  public:
    UtilityBase();
    ~UtilityBase();
    
    void setMessagePrefixLabel(string strMessagePrefixLabel);
    string messagePrefixLabel();
    
    void coloredText(string strText, string strColorValue, bool bBold = false);
    void success(string strMessage);
    void info(string strMessage);
    void warn(string strMessage);
    void fail(string strMessage);
    
    bool fileExists(string strFileName);
    
    string stripPostfix(string strString, string strPostfix);
  };
}


#endif /* __UTILITY_BASE_H__ */
