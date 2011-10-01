// EncoderCountI2CEndpoint.hpp
 
#ifndef GUARD_EncoderCountI2CEndpoint
#define GUARD_EncoderCountI2CEndpoint
 
#include <ros/ros.h>
#include "IEncoderCountEndpoint.hpp"
 
namespace diff_drive_message_endpoints
{
  class EncoderCountEndpoint : public diff_drive_core::IEncoderCountEndpoint
  { 
    public:
      EncoderCountEndpoint();
      ~EncoderCountEndpoint();
 
    private:
  };
}
 
#endif /* GUARD_EncoderCountI2CEndpoint */
