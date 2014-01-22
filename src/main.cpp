// System
#include <iostream>
#include <cstdlib>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

// Private
#include <Beliefstate.h>
//#include <ForwardDeclarations.h>

using namespace std;
using namespace beliefstate;


Beliefstate* bsBeliefstate;


void catchSIGTERMandSIGINT(int nSignum) {
  bsBeliefstate->triggerShutdown();
}

int main(int argc, char** argv) {
  cout << "[ core ] Starting beliefstate system." << endl;
  
  bsBeliefstate = new Beliefstate(argc, argv);
  Result resInit = bsBeliefstate->init();
  
  if(resInit.bSuccess) {
    // Catch SIGTERM and SIGINT and bind them to the callback function
    // catchSIGTERMandSIGINT.
    struct sigaction action;
    memset(&action, 0, sizeof(struct sigaction));
    action.sa_handler = catchSIGTERMandSIGINT;
    sigaction(SIGTERM, &action, NULL);
    sigaction(SIGINT, &action, NULL);
    
    while(bsBeliefstate->cycle()) {
    }
  } else {
    cerr << "\033[1;31m[ core ]Initialization of the beliefstate system failed. Being a quitter." << "\033[0m" << endl;
  }
  
  cout << "[ core ] Exiting gracefully." << endl;
  
  bsBeliefstate->deinit();
  delete bsBeliefstate;
  
  return EXIT_SUCCESS;
}
