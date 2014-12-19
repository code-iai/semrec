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
#include <string>

// Private
#include <SemanticHierarchyRecorderROS.h>


// Storage of former signal handlers
typedef void (*Handler)(int signum);
Handler hdlrOldSIGWINCH = SIG_IGN;


// Global variable for shutdown triggering
semrec::SemanticHierarchyRecorderROS* g_srRecorder;


void printHelp(std::string strExecutableName) {
  std::cout << "Semantic Hierarchy Recorder System (version \033[1;37m" + g_srRecorder->version() + "\033[0;37m) by Jan Winkler <winkler@cs.uni-bremen.de>" << std::endl;
  std::cout << "Licensed under BSD. https://www.github.com/code-iai/ros-semrec" << std::endl << std::endl;
  std::cout << "Usage: " << strExecutableName << " [options]" << std::endl << std::endl;
  
  std::cout << "Available options are:" << std::endl;
  std::cout << "  -h, --help\t\tPrint this help" << std::endl;
  std::cout << "  -c, --config <file>\tLoad config file <file> instead of the default one" << std::endl;
  std::cout << "  -q, --quiet\t\tStart in quiet mode (command line output is suppressed)" << std::endl;
  std::cout << std::endl;
  
  std::cout << "Should any questions arise, feel free to send an email to winkler@cs.uni-bremen.de" << std::endl;
}

void catchHandler(int nSignum) {
  switch(nSignum) {
  case SIGTERM:
  case SIGINT: {
    g_srRecorder->triggerShutdown();
  } break;
    
  case SIGWINCH: {
    if(hdlrOldSIGWINCH != SIG_IGN && hdlrOldSIGWINCH != SIG_DFL) {
      (*hdlrOldSIGWINCH)(SIGWINCH);
    }
    
    g_srRecorder->triggerTerminalResize();
  } break;
    
  default:
    break;
  }
}

int main(int argc, char** argv) {
  g_srRecorder = new semrec::SemanticHierarchyRecorderROS(argc, argv);
  
  // Read command line parameters
  int nC, option_index = 0;
  static struct option long_options[] = {{"config", required_argument, 0, 'c'},
				         {"quiet",  no_argument,       0, 'q'},
				         {"help",   no_argument,       0, 'h'},
				         {0,        0,                 0, 0}};
  
  std::string strConfigFile = "";
  bool bQuit = false;
  bool bQuiet = false;
  
  while((nC = getopt_long(argc, argv, "c:qh", long_options, &option_index)) != -1) {
    switch(nC) {
    case 'c': {
      strConfigFile = std::string(optarg);
    } break;
      
    case 'q': {
      bQuiet = true;
    } break;
      
    case 'h': {
      printHelp(std::string(argv[0]));
      bQuit = true;
    } break;
      
    default: {
    } break;
    }
  }
  
  if(bQuit == false) {
    g_srRecorder->setQuiet(bQuiet);
    
    g_srRecorder->info("Starting semantic hierarchy recorder system (version \033[1;37m" + g_srRecorder->version() + "\033[0;37m).");
    
    semrec::Result resInit = g_srRecorder->init(strConfigFile);
    
    if(resInit.bSuccess) {
      // Catch SIGTERM and SIGINT and bind them to the callback function
      // catchSIGTERMandSIGINT. This will trigger the shutdown mechanism
      // in the semrec instance.
      struct sigaction action;
      memset(&action, 0, sizeof(struct sigaction));
      action.sa_handler = catchHandler;
      sigaction(SIGTERM, &action, NULL);
      sigaction(SIGINT, &action, NULL);
      
      hdlrOldSIGWINCH = signal(SIGWINCH, SIG_IGN);
      sigaction(SIGWINCH, &action, NULL);
      
      g_srRecorder->info("Initialization complete, ready for action.", true);
      while(g_srRecorder->cycle()) {
	// Idle here at will.
	usleep(10);
      }
    } else {
      g_srRecorder->fail("Initialization of the recorder system failed. Being a quitter.");
    }
    
    std::cout << "\r";
    g_srRecorder->info("Exiting gracefully.");
    g_srRecorder->cycle();
    
    g_srRecorder->deinit();
    g_srRecorder->cycle();
    
    delete g_srRecorder;
  }
  
  return EXIT_SUCCESS;
}
