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
    // TODO(winkler): Mark this as a warning
    this->coloredText("[ " + m_strMessagePrefixLabel + " ] " + strMessage, "33", true);
  }
  
  void UtilityBase::fail(string strMessage) {
    this->coloredText("[ " + m_strMessagePrefixLabel + " ] " + strMessage, "31", true);
  }
}
