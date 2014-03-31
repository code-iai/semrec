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
#include <BeliefstateROS.h>

using namespace std;
using namespace beliefstate;


// Storage of former signal handlers
typedef void (*Handler)(int signum);
Handler hdlrOldSIGWINCH = SIG_IGN;


// Global variable for shutdown triggering
BeliefstateROS* g_bsBeliefstate;


void printHelp(string strExecutableName) {
  cout << "Beliefstate System (version " + g_bsBeliefstate->version() + ") by Jan Winkler <winkler@cs.uni-bremen.de>" << endl;
  cout << "Licensed under BSD. https://www.github.com/fairlight1337/beliefstate" << endl << endl;
  cout << "Usage: " << strExecutableName << " [options]" << endl << endl;
  
  cout << "Available options are:" << endl;
  cout << "  -h, --help\t\tPrint this help" << endl;
  cout << "  -c, --config <file>\tLoad config file <file> instead of the default one" << endl;
  cout << endl;
  
  cout << "Should any questions arise, feel free to send an email to winkler@cs.uni-bremen.de" << endl;
}

void catchHandler(int nSignum) {
  switch(nSignum) {
  case SIGTERM:
  case SIGINT: {
    g_bsBeliefstate->triggerShutdown();
  }
    
  case SIGWINCH: {
    if(hdlrOldSIGWINCH != SIG_IGN && hdlrOldSIGWINCH != SIG_DFL) {
      (*hdlrOldSIGWINCH)(SIGWINCH);
    }
    
    g_bsBeliefstate->triggerTerminalResize();
  }
  }
}

int main(int argc, char** argv) {
  g_bsBeliefstate = new BeliefstateROS(argc, argv);
  
  // Read command line parameters
  int nC, option_index = 0;
  static struct option long_options[] = {{"config", required_argument, 0, 'c'},
					 {"help",   no_argument,       0, 'h'},
					 {0,        0,                 0, 0}};
  
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
    g_bsBeliefstate->info("Starting beliefstate system (version " + g_bsBeliefstate->version() + ".");
    
    Result resInit = g_bsBeliefstate->init(strConfigFile);
  
    if(resInit.bSuccess) {
      // Catch SIGTERM and SIGINT and bind them to the callback function
      // catchSIGTERMandSIGINT. This will trigger the shutdown mechanism
      // in the Beliefstate instance.
      struct sigaction action;
      memset(&action, 0, sizeof(struct sigaction));
      action.sa_handler = catchHandler;
      sigaction(SIGTERM, &action, NULL);
      sigaction(SIGINT, &action, NULL);
      
      hdlrOldSIGWINCH = signal(SIGWINCH, SIG_IGN);
      sigaction(SIGWINCH, &action, NULL);
      
      while(g_bsBeliefstate->cycle()) {
	// Idle here at will.
      }
    } else {
      g_bsBeliefstate->fail("Initialization of the beliefstate system failed. Being a quitter.");
    }
    
    cout << "\r";
    g_bsBeliefstate->info("Exiting gracefully.");
    g_bsBeliefstate->cycle();
    
    g_bsBeliefstate->deinit();
    g_bsBeliefstate->cycle();
    
    delete g_bsBeliefstate;
  }
  
  return EXIT_SUCCESS;
}
