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


#ifndef __OWL_INDIVIDUAL_H__
#define __OWL_INDIVIDUAL_H__


// System
#include <string>
#include <list>
#include <algorithm>
#include <iostream>
#include <map>


namespace semrec {
  class OwlIndividual {
  public:
    typedef struct _OwlProperty {
      std::string strTag;
      std::string strDataType;
      std::string strResource;
      std::string strContent;
    } OwlProperty;
    
  private:
    static std::map<std::string, std::string> s_mapStaticResources;
    static std::map<std::string, std::string> s_mapStaticProperties;
    
    std::list<OwlProperty> m_lstProperties;
    std::string m_strID;
    std::string m_strType;
    
    static std::list<std::string> m_lstIssuedProperties;
    static std::list<std::string> m_lstIssuedTypes;
    
  protected:
  public:
    OwlIndividual();
    ~OwlIndividual();
    
    static void addStaticResource(std::string strKey, std::string strResource);
    static void addStaticProperty(std::string strKey, std::string strProperty);
    
    void setID(std::string strID);
    void setType(std::string strType);
    
    void addDataProperty(std::string strTag, std::string strDataType, std::string strContent);
    void addResourceProperty(std::string strTag, std::string strResource);
    void addContentProperty(std::string strTag, std::string strContent);
    
    void issueProperty(std::string strProperty);
    void issueType(std::string strType);
    
    std::string indent(int nSpaces);
    
    std::string print(int nIndentation = 1, int nIndentationPerLevel = 4);
    
    static void resetIssuedProperties();
    static void resetIssuedTypes();
    static void resetIssuedInformation();
    
    static std::list<std::string> issuedProperties();
    static std::list<std::string> issuedTypes();
  };
}


#endif /* __OWL_INDIVIDUAL_H__ */
