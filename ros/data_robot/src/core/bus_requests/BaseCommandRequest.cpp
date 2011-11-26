// BaseCommandRequest.cpp

#include "BusAddresses.hpp"
#include "BaseCommandRequest.hpp"

namespace data_robot_core
{
BaseCommandRequest::BaseCommandRequest( bool is_blockable, bool is_lockable )
                    : BusRequest( is_blockable, is_lockable )
{
  uint16_t address;

  address = BB_TRAINER + ( COMMAND_OFFSET << 8 );

  SetAddress( (uint8_t*)&address, ADDRESS_SIZE );
  SetRequestType( REQUEST_WRITE );
}

void BaseCommandRequest::SetCommand( uint8_t command, uint8_t parameter )
{
  uint8_t command_tuple[2];

  command_tuple[0] = command;
  command_tuple[1] = parameter;

  SetDataBufferSize( sizeof(command_tuple) );
  SetData( command_tuple, sizeof(command_tuple) );
}

void BaseCommandRequest::SetCommand( uint16_t command )
{
  SetDataBufferSize( sizeof(command) );
  SetData( (uint8_t*)&command, sizeof(command) );
}
}
