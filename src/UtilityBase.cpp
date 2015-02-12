/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Institute for Artificial Intelligence,
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


#include <UtilityBase.h>


namespace semrec {
  bool UtilityBase::m_bRedirectOutput = false;
  bool UtilityBase::m_bQuiet = false;
  
  
  UtilityBase::UtilityBase() {
    m_strMessagePrefixLabel = "";
    m_bOnlyDisplayImportant = false;
    m_nTimeFloatingPointPrecision = 0;
  }
  
  UtilityBase::~UtilityBase() {
  }
  
  void UtilityBase::setQuiet(bool bQuiet) {
    m_bQuiet = bQuiet;
  }
  
  bool UtilityBase::quiet() {
    return m_bQuiet;
  }
  
  void UtilityBase::setMessagePrefixLabel(std::string strMessagePrefixLabel) {
    m_strMessagePrefixLabel = strMessagePrefixLabel;
  }
  
  std::string UtilityBase::messagePrefixLabel() {
    return m_strMessagePrefixLabel;
  }
  
  void UtilityBase::coloredText(std::string strText, std::string strColorValue, bool bBold, bool bImportant) {
    if((!m_bOnlyDisplayImportant || (m_bOnlyDisplayImportant && bImportant)) && !m_bQuiet) {
      StatusMessage msgStatus = queueMessage(strColorValue, bBold, this->messagePrefixLabel(), strText);
      
      if(!m_bRedirectOutput) {
	this->outputColoredStatusMessage(msgStatus);
      }
    }
  }
  
  void UtilityBase::info(std::string strMessage, bool bImportant) {
    this->coloredText(strMessage, "37", false, bImportant);
  }
  
  void UtilityBase::success(std::string strMessage, bool bImportant) {
    this->coloredText(strMessage, "32", false, bImportant);
  }
  
  void UtilityBase::warn(std::string strMessage, bool bImportant) {
    this->coloredText(strMessage, "33", true, bImportant);
  }
  
  void UtilityBase::fail(std::string strMessage, bool bImportant) {
    this->coloredText(strMessage, "31", true, bImportant);
  }
  
  bool UtilityBase::fileExists(std::string strFileName) {
    bool bGood = false;
    std::ifstream ifile(strFileName.c_str(), std::ifstream::in);
    
    if(ifile) {
      bGood = ifile.good();
      
      ifile.close();
    }
    
    return bGood;
  }
  
  std::string UtilityBase::stripPostfix(std::string strString, std::string strPostfix) {
    if(strString.length() >= strPostfix.length()) {
      if(strString.substr(strString.length() - strPostfix.length()) == strPostfix) {
	return strString.substr(0, strString.length() - strPostfix.length());
      }
    }
    
    return strString;
  }
  
  void UtilityBase::outputColoredStatusMessage(StatusMessage msgStatus) {
    std::cout << "\033[" << (msgStatus.bBold ? "1" : "0") << ";" << msgStatus.strColorCode << "m"
	      << "[ " << msgStatus.strPrefix << " ] " << msgStatus.strMessage << "\033[0m" << std::endl;
  }
  
  void UtilityBase::setRedirectOutput(bool bRedirect) {
    m_bRedirectOutput = bRedirect;
  }
  
  void UtilityBase::replaceStringInPlace(std::string& subject, const std::string& search, const std::string& replace) {
    size_t pos = 0;
    
    while((pos = subject.find(search, pos)) != std::string::npos) {
      subject.replace(pos, search.length(), replace);
      pos += replace.length();
    }
  }
  
  double UtilityBase::getTimeStampPrecise() {
    ros::Time timeNow = ros::Time::now();
    
    return timeNow.toSec();
  }

  std::string UtilityBase::getTimeStampStr(double dTime) {
    char cTimeBuffer[256];
    std::string strFormat = "%30." + this->str(m_nTimeFloatingPointPrecision) + "f";
    
    sprintf(cTimeBuffer, strFormat.c_str(), dTime);
    
    std::string strTime = cTimeBuffer;
    std::stringstream stsTrimmer;
    stsTrimmer << strTime;
    strTime.clear();
    stsTrimmer >> strTime;
      
    return strTime;
  }
  
  std::string UtilityBase::getTimeStampStr() {
    return this->getTimeStampStr(this->getTimeStampPrecise());
  }
  
  std::string UtilityBase::str(float fValue) {
    std::stringstream sts;
    sts.imbue(std::locale(sts.getloc(), new std::numpunct<char>()));
    
    sts << fValue;
    
    return sts.str();
  }
  
  std::string UtilityBase::str(double dValue) {
    std::stringstream sts;
    sts.imbue(std::locale(sts.getloc(), new std::numpunct<char>()));
    
    sts << dValue;
    
    return sts.str();
  }
  
  std::string UtilityBase::str(int nValue) {
    std::stringstream sts;
    sts.imbue(std::locale(sts.getloc(), new std::numpunct<char>()));
    
    sts << nValue;
    
    return sts.str();
  }
  
  void UtilityBase::setOnlyDisplayImportant(bool bOnly) {
    m_bOnlyDisplayImportant = bOnly;
  }
  
  bool UtilityBase::onlyDisplayImportant() {
    return m_bOnlyDisplayImportant;
  }
  
  void UtilityBase::setTimeFloatingPointPrecision(int nPrecision) {
    m_nTimeFloatingPointPrecision = nPrecision;
  }
}
