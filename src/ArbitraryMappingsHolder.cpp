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


#include <ArbitraryMappingsHolder.h>


namespace semrec {
  ArbitraryMappingsHolder::ArbitraryMappingsHolder() {
  }
  
  ArbitraryMappingsHolder::~ArbitraryMappingsHolder() {
    for(KeyValuePair* ckvpMapping : m_lstArbitraryMappings) {
      delete ckvpMapping;
    }
    
    m_lstArbitraryMappings.clear();
  }
  
  void ArbitraryMappingsHolder::parseBranch(libconfig::Setting& sBranch) {
    // TODO(winkler): Actually parse the mapping branch here and
    // assert it into a new part of the designator mapping storage.
  }
  
  bool ArbitraryMappingsHolder::loadArbitraryMappingsFile(std::string strFilepath) {
    if(this->fileExists(strFilepath)) {
      libconfig::Config cfgConfig;
      
      try {
	cfgConfig.readFile(strFilepath.c_str());
	
	if(cfgConfig.exists("arbitrary-mappings")) {
	  libconfig::Setting &sArbitraryMappings = cfgConfig.lookup("arbitrary-mappings");
	  
	  if(sArbitraryMappings.exists("mappings")) {
	    libconfig::Setting &sMappings = sArbitraryMappings["mappings"];
	    
	    for(int nI = 0; nI < sMappings.getLength(); nI++) {
	      this->parseBranch(sMappings[nI]);
	    }
	  }
	}
	
	return true;
      } catch(libconfig::ParseException e) {
	std::stringstream sts;
        sts << e.getLine();
	
        this->fail("Error while parsing arbitrary mappings file '" + strFilepath + "': " + e.getError() + ", on line " + sts.str());
      } catch(...) {
	this->fail("Undefined error while parsing arbitrary mappings file '" + strFilepath + "'");
      }
    } else {
      this->fail("Arbitrary mappings file not found: '" + strFilepath + "'.");
    }
    
    return false;
  }
  
  std::list<KeyValuePair*> ArbitraryMappingsHolder::arbitraryMappings() {
    return m_lstArbitraryMappings;
  }
}
