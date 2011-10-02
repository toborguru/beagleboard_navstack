// ITwistEndpoint.hpp

#ifndef GUARD_ITwistEndpoint
#define GUARD_ITwistEndpoint

#include "ITwistListener.hpp"

namespace diff_drive_core
{
class ITwistEndpoint
{
public:
  // Virtual destructor to pass pointer ownership without exposing base class [Meyers, 2005, Item 7]
  virtual ~ITwistEndpoint() {}
  virtual void Subscribe() = 0;
  virtual void Unsubscribe() = 0;
  virtual void Attach( ITwistListener& twist_listener ) = 0; 
};
}

#endif /* GUARD_ITwistEndpoint */
