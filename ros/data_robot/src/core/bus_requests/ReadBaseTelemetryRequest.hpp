// ReadBaseTelemetryRequest.hpp

#include "BusRequest.hpp"
#include "Telemetry.hpp"

#ifndef GUARD_ReadBaseTelemetryRequest
#define GUARD_ReadBaseTelemetryRequest

namespace data_robot_core
{
/** This class is a BusRequest that is used to read the encoder counts from 
 *  the robot.
 *
 */
class ReadBaseTelemetryRequest : public BusRequest
{
public:
  ReadBaseTelemetryRequest( bool is_blockable = true, bool is_lockable = true );

  // Virtual destructor to pass pointer ownership without exposing base class [Meyers, 2005, Item 7]
  virtual ~ReadBaseTelemetryRequest() {};

  BaseTelemetry_T GetTelemetry();

private:
};
}
 
#endif /* GUARD_ReadBaseTelemetryRequest */
