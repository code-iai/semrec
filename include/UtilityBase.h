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


#ifndef __UTILITY_BASE_H__
#define __UTILITY_BASE_H__


// System
#include <iostream>
#include <string>
#include <fstream>
#include <chrono>

// Private
#include <Types.h>
#include <ForwardDeclarations.h>


namespace semrec {
  class UtilityBase {
  private:
    static bool m_bRedirectOutput;
    static bool m_bQuiet;
    std::string m_strMessagePrefixLabel;
    bool m_bOnlyDisplayImportant;
    int m_nTimeFloatingPointPrecision;
    
  public:
    UtilityBase();
    ~UtilityBase();
    
    void setQuiet(bool bQuiet);
    bool quiet();
    
    void setMessagePrefixLabel(std::string strMessagePrefixLabel);
    std::string messagePrefixLabel();
    
    void coloredText(std::string strText, std::string strColorValue, bool bBold = false, bool bImportant = false);
    virtual void success(std::string strMessage, bool bImportant = false);
    virtual void info(std::string strMessage, bool bImportant = false);
    virtual void warn(std::string strMessage, bool bImportant = false);
    virtual void fail(std::string strMessage, bool bImportant = false);
    
    void setOnlyDisplayImportant(bool bOnly);
    bool onlyDisplayImportant();
    
    bool fileExists(std::string strFileName);
    
    std::string stripPostfix(std::string strString, std::string strPostfix);
    
    void outputColoredStatusMessage(StatusMessage msgStatus);
    void setRedirectOutput(bool bRedirect);
    
    /*! \brief Partial string replace function
      
      Replaces 'search' by 'replace' in string 'subject'.
      
      \param subject The string to search in
      \param search The string to search for
      \param replace The string to use as a replacement */
    void replaceStringInPlace(std::string& subject, const std::string& search, const std::string& replace);
    
    double getTimeStampPrecise();
    std::string getTimeStampStr();
    std::string getTimeStampStr(double dTime);
    
    std::string str(float fValue);
    std::string str(double dValue);
    std::string str(int nValue);
    
    void setTimeFloatingPointPrecision(int nPrecision);
  };
}


#endif /* __UTILITY_BASE_H__ */
