#include <UtilityBase.h>


namespace beliefstate {
  UtilityBase::UtilityBase() {
    m_strMessagePrefixLabel = "";
  }
  
  UtilityBase::~UtilityBase() {
  }
  
  void UtilityBase::setMessagePrefixLabel(string strMessagePrefixLabel) {
    m_strMessagePrefixLabel = strMessagePrefixLabel;
  }
  
  string UtilityBase::messagePrefixLabel() {
    return m_strMessagePrefixLabel;
  }
  
  void UtilityBase::info(string strMessage) {
    cout << "[ " << m_strMessagePrefixLabel << " ] " << strMessage << endl;
  }
  
  void UtilityBase::coloredText(string strText, string strColorValue, bool bBold) {
    cout << "\033[" << (bBold ? "1" : "0") << ";" << strColorValue << "m" << strText << "\033[0m" << endl;
  }
  
  void UtilityBase::success(string strMessage) {
    this->coloredText("[ " + m_strMessagePrefixLabel + " ] " + strMessage, "32");
  }
  
  void UtilityBase::warn(string strMessage) {
    this->coloredText("[ " + m_strMessagePrefixLabel + " ] " + strMessage, "33", true);
  }
  
  void UtilityBase::fail(string strMessage) {
    this->coloredText("[ " + m_strMessagePrefixLabel + " ] " + strMessage, "31", true);
  }
  
  bool UtilityBase::fileExists(string strFileName) {
    ifstream ifile(strFileName.c_str());
    
    if(ifile) {
      ifile.close();
      
      return true;
    }
    
    return false;
  }
  
  string UtilityBase::stripPostfix(string strString, string strPostfix) {
    if(strString.length() >= strPostfix.length()) {
      if(strString.substr(strString.length() - strPostfix.length()) == strPostfix) {
	return strString.substr(0, strString.length() - strPostfix.length());
      }
    }
    
    return strString;
  }
}
