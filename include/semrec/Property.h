/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Institute for Artificial Intelligence,
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


#ifndef __PROPERTY_H__
#define __PROPERTY_H__


// System
#include <iostream>
#include <list>
#include <string>


namespace semrec {
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
    std::list<Property*> m_lstSubProperties;
    std::string m_strKey;
    std::string m_strValue;
    double m_dValue;
    PropertyType m_ptType;
    
  protected:
  public:
    Property(std::string strKey = "", PropertyType ptType = String);
    Property(Property* prTemplate, bool bDeepCopy = true);
    ~Property();
    
    void setKey(std::string strKey);
    std::string key();
    
    void set(PropertyType ptType);
    PropertyType type();
    
    void set(std::string strValue);
    std::string getString();
    
    void set(int nValue);
    int getInteger();
    
    void set(double dValue);
    double getDouble();
    
    void set(bool bValue);
    bool getBoolean();
    
    void addSubProperty(Property* prSubProperty);
    std::list<Property*> subProperties();
    
    void print(int nIndentationLevel = 0, bool bPrintKey = true);
    
    Property* namedSubProperty(std::string strKey);
  };
}


#endif /* __PROPERTY_H__ */
