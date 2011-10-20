// EncoderCountsI2CEndpoint.hpp
 
#ifndef GUARD_EncoderCountsEndpoint
#define GUARD_EncoderCountsEndpoint
 
#include <ros/ros.h>
#include "IEncoderCountsEndpoint.hpp"
 
namespace diff_drive_message_endpoints
{
  class EncoderCountsEndpoint : public diff_drive_core::IEncoderCountsEndpoint
  { 
    public:
      EncoderCountsEndpoint();
      ~EncoderCountsEndpoint();
 
    private:
  };
}
 
#endif /* GUARD_EncoderCountsEndpoint */
