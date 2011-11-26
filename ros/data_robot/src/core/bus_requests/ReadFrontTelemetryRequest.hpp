// ReadFrontTelemetryRequest.hpp

#include "BusAddresses.hpp"
#include "BusRequest.hpp"
#include "Telemetry.hpp"

#ifndef GUARD_ReadFrontTelemetryRequest
#define GUARD_ReadFrontTelemetryRequest

namespace data_robot_core
{
/** This class is a BusRequest that is used to read the encoder counts from 
 *  the robot.
 *
 */
class ReadFrontTelemetryRequest : public BusRequest
{
public:
  ReadFrontTelemetryRequest( bool is_blockable = true, bool is_lockable = true );

  // Virtual destructor to pass pointer ownership without exposing base class [Meyers, 2005, Item 7]
  virtual ~ReadFrontTelemetryRequest() {};

  FrontShellTelemetry_T GetTelemetry();

private:

};
}
 
#endif /* GUARD_ReadFrontTelemetryRequest */
