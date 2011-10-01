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
  virtual void Subscribe() = 0;
  virtual void Unsubscribe() = 0;
  virtual void Attach( IEncoderCountsListener& encoderCountsListener ) = 0; 
};
}

#endif /* GUARD_IEncoderCountEndpoint */
