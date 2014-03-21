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

// Private
#include <Types.h>
#include <ForwardDeclarations.h>

using namespace std;


namespace beliefstate {
  class UtilityBase {
  private:
    static bool m_bRedirectOutput;
    string m_strMessagePrefixLabel;
    
  public:
    UtilityBase();
    ~UtilityBase();
    
    void setMessagePrefixLabel(string strMessagePrefixLabel);
    string messagePrefixLabel();
    
    void coloredText(string strText, string strColorValue, bool bBold = false);
    virtual void success(string strMessage);
    virtual void info(string strMessage);
    virtual void warn(string strMessage);
    virtual void fail(string strMessage);
    
    bool fileExists(string strFileName);
    
    string stripPostfix(string strString, string strPostfix);
    
    void outputColoredStatusMessage(StatusMessage msgStatus);
    void setRedirectOutput(bool bRedirect);
    
    /*! \brief Partial string replace function
      
      Replaces 'search' by 'replace' in string 'subject'.
      
      \param subject The string to search in
      
      \param search The string to search for
      
      \param replace The string to use as a replacement */
    void replaceStringInPlace(string& subject, const string& search, const string& replace);
  };
}


#endif /* __UTILITY_BASE_H__ */
