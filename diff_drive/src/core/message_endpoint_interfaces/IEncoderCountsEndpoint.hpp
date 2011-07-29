// IEncoderCountEndpoint.hpp

#ifndef GUARD_IEncoderCountsEndpoint
#define GUARD_IEncoderCountsEndpoint

#include "IEncoderCountsListener.hpp"

namespace diff_drive_core
{
class IEncoderCountsEndpoint
{
public:
  // Virtual destructor to pass pointer ownership without exposing base class [Meyers, 2005, Item 7]
  virtual ~IEncoderCountsEndpoint() {}
  virtual void Subscribe();
  virtual void Unsubscribe();
  virtual void Attach( IEncoderCountsListener& encoderCountsListener ); 
};
}

#endif /* GUARD_IEncoderCountEndpoint */
