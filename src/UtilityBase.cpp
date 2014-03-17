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


namespace beliefstate {
  bool UtilityBase::m_bRedirectOutput;
  
  
  UtilityBase::UtilityBase() {
    m_strMessagePrefixLabel = "";
    m_bRedirectOutput = false;
  }
  
  UtilityBase::~UtilityBase() {
  }
  
  void UtilityBase::setMessagePrefixLabel(string strMessagePrefixLabel) {
    m_strMessagePrefixLabel = strMessagePrefixLabel;
  }
  
  string UtilityBase::messagePrefixLabel() {
    return m_strMessagePrefixLabel;
  }
  
  void UtilityBase::coloredText(string strText, string strColorValue, bool bBold) {
    StatusMessage msgStatus = queueMessage(strColorValue, bBold, this->messagePrefixLabel(), strText);
    
    if(!m_bRedirectOutput) {
      this->outputColoredStatusMessage(msgStatus);
    }
  }
  
  void UtilityBase::info(string strMessage) {
    this->coloredText(strMessage, "37");
  }
  
  void UtilityBase::success(string strMessage) {
    this->coloredText(strMessage, "32");
  }
  
  void UtilityBase::warn(string strMessage) {
    this->coloredText(strMessage, "33", true);
  }
  
  void UtilityBase::fail(string strMessage) {
    this->coloredText(strMessage, "31", true);
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
  
  void UtilityBase::outputColoredStatusMessage(StatusMessage msgStatus) {
    cout << "\033[" << (msgStatus.bBold ? "1" : "0") << ";" << msgStatus.strColorCode << "m"
	 << "[ " << msgStatus.strPrefix << " ] " << msgStatus.strMessage << "\033[0m" << endl;
  }
  
  void UtilityBase::setRedirectOutput(bool bRedirect) {
    m_bRedirectOutput = bRedirect;
  }
}
