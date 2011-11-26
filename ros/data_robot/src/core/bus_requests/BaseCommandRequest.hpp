// BaseCommandRequest.hpp

#include "BusRequest.hpp"

#ifndef GUARD_BaseCommandRequest
#define GUARD_BaseCommandRequest

namespace data_robot_core
{
/** This class is a BusRequest that is used to send arbitrary commands
 *  the robot.
 *
 */
class BaseCommandRequest : public BusRequest
{
public:
  BaseCommandRequest( bool is_blockable = true, bool is_lockable = true );

  // Virtual destructor to pass pointer ownership without exposing base class [Meyers, 2005, Item 7]
  virtual ~BaseCommandRequest() {};

  void  SetCommand( uint8_t command, uint8_t parameter );

  void  SetCommand( uint16_t command );

private:
};
}
 
#endif /* GUARD_BaseCommandRequest */
