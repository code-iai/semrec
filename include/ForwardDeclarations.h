#ifndef __FORWARD_DECLARATIONS_H__
#define __FORWARD_DECLARATIONS_H__


// Private
#include <Types.h>


namespace beliefstate {
  // Context specific functions
  int createContextID();
  bool contextIDTaken(int nID);
  void freeContextID();
  
  // Result container specific functions
  Result defaultResult();
  
  // Event container specific functions
  Event defaultEvent();
}


#endif /* __FORWARD_DECLARATIONS_H__ */
