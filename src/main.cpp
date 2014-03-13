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


// System
#include <iostream>
#include <cstdlib>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <getopt.h>

// Private
#include <Beliefstate.h>

using namespace std;
using namespace beliefstate;


// Global variable for shutdown triggering
Beliefstate* bsBeliefstate;


void printHelp(string strExecutableName) {
  cout << "Usage: " << strExecutableName << " [options]" << endl << endl;
  
  cout << "Available options are:" << endl;
  cout << "  -h, --help\t\tPrint this help" << endl;
  cout << "  -c, --config <file>\tLoad config file <file> instead of the default one" << endl;
  cout << endl;
  
  cout << "Should any questions arise, feel free to send an email to winkler@cs.uni-bremen.de" << endl;
}

void catchSIGTERMandSIGINT(int nSignum) {
  bsBeliefstate->triggerShutdown();
}

int main(int argc, char** argv) {
  // Read command line parameters
  int nC, option_index = 0;
  static struct option long_options[] = {{"config",           required_argument, 0, 'c'},
					 {"help",             no_argument,       0, 'h'},
					 {0,                  0,                 0, 0}};
  
  string strConfigFile = "";
  bool bQuit = false;
  
  while((nC = getopt_long(argc, argv, "c:h", long_options, &option_index)) != -1) {
    switch(nC) {
    case 'c': {
      strConfigFile = string(optarg);
    } break;
      
    case 'h': {
      printHelp(string(argv[0]));
      bQuit = true;
    } break;
      
    default: {
    } break;
    }
  }
  
  if(bQuit == false) {
    bsBeliefstate = new Beliefstate(argc, argv);
    bsBeliefstate->info("Starting beliefstate system.");
    
    Result resInit = bsBeliefstate->init(strConfigFile);
  
    if(resInit.bSuccess) {
      // Catch SIGTERM and SIGINT and bind them to the callback function
      // catchSIGTERMandSIGINT. This will trigger the shutdown mechanism
      // in the Beliefstate instance.
      struct sigaction action;
      memset(&action, 0, sizeof(struct sigaction));
      action.sa_handler = catchSIGTERMandSIGINT;
      sigaction(SIGTERM, &action, NULL);
      sigaction(SIGINT, &action, NULL);
    
      while(bsBeliefstate->cycle()) {
	// Idle here at will.
      }
    } else {
      bsBeliefstate->fail("Initialization of the beliefstate system failed. Being a quitter.");
    }
    
    cout << "\r";
    bsBeliefstate->info("Exiting gracefully.");
    bsBeliefstate->cycle();
    
    bsBeliefstate->deinit();
    bsBeliefstate->cycle();
    
    delete bsBeliefstate;
  }
  
  return EXIT_SUCCESS;
}
