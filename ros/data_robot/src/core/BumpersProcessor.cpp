#include "BumpersProcessor.hpp"

namespace data_robot_core
{
/** Default constructor.
 *  @param  front_bumper_index  Bit position of of the front bumper.
 *  @param  front_left_bumper_index  Bit position of of the front left bumper.
 *  @param  front_right_bumper_index  Bit position of of the front right bumper.
 *  @param  rear_bumper_index  Bit position of of the rear bumper.
 *  @param  rear_left_bumper_index  Bit position of of the rear left bumper.
 *  @param  rear_right_bumper_index  Bit position of of the rear right bumper.
 */
BumpersProcessor::BumpersProcessor( int8_t front_bumper_index,
                                    int8_t front_left_bumper_index,
                                    int8_t front_right_bumper_index,
                                    int8_t rear_bumper_index,
                                    int8_t rear_left_bumper_index,
                                    int8_t rear_right_bumper_index)
                : _front_index( front_bumper_index ), 
                  _front_left_index( front_left_bumper_index ), 
                  _front_right_index( front_right_bumper_index ), 
                  _rear_index( rear_bumper_index ), 
                  _rear_left_index( rear_left_bumper_index ), 
                  _rear_right_index( rear_right_bumper_index ),
                  _current_bumps( 0 )
{
}

/** Function to report new bumper status to this processor.
 *
 *  A mask is used so that bumper data can be reported from multiple sources. All
 *  active bumpers will be kept until masked with a 0 state.
 */
void    BumpersProcessor::AddNewData( uint8_t bumpers_mask,  uint8_t bumpers_state )
{
  _current_bumps = CalculateBumps( _current_bumps, bumpers_mask, bumpers_state );
}

/** Access Function. 
 *  @returns Bit-field of the bumpers currently pressed.
 */
uint8_t BumpersProcessor::GetCurrentBumps() const
{
  return _current_bumps;
}

/** Access Function. 
 *  @returns All current bumps combined into a directional enumeration.
 */
data_robot::Bumpers BumpersProcessor::GetBumpDirection() const
{
  return CalculateBumpDirection( _current_bumps );
}

/** Access Function. 
 *  @returns The index in the bumper bit-field that represents the front center bumper.
 */
int8_t  BumpersProcessor::GetFrontBumperIndex() const
{
  return _front_index;
}

/** Access Function. 
 *  @param index The index in the bumper bit-field that represents the front center bumper.
 */
void    BumpersProcessor::SetFrontBumperIndex(  int8_t index )
{
  _front_index = index;
}

/** Access Function. 
 *  @returns The index in the bumper bit-field that represents the front left bumper.
 */
int8_t  BumpersProcessor::GetFrontLeftBumperIndex() const
{
  return _front_left_index;
}

/** Access Function. 
 *  @param index The index in the bumper bit-field that represents the front left bumper.
 */
void    BumpersProcessor::SetFrontLeftBumperIndex(  int8_t index )
{
  _front_left_index = index;
}

/** Access Function. 
 *  @returns The index in the bumper bit-field that represents the front right bumper.
 */
int8_t  BumpersProcessor::GetFrontRightBumperIndex() const
{
  return _front_right_index;
}

/** Access Function. 
 *  @param index The index in the bumper bit-field that represents the front right bumper.
 */
void    BumpersProcessor::SetFrontRightBumperIndex( int8_t index )
{
  _front_right_index = index;
}

/** Access Function. 
 *  @returns The index in the bumper bit-field that represents the rear center bumper.
 */
int8_t  BumpersProcessor::GetRearBumperIndex() const
{
  return _rear_index;
}

/** Access Function. 
 *  @param index The index in the bumper bit-field that represents the rear center bumper.
 */
void    BumpersProcessor::SetRearBumperIndex( int8_t index )
{
  _rear_index = index;
}

/** Access Function. 
 *  @returns The index in the bumper bit-field that represents the rear left bumper.
 */
int8_t  BumpersProcessor::GetRearLeftBumperIndex() const
{
  return _rear_left_index;
}

/** Access Function. 
 *  @param index The index in the bumper bit-field that represents the rear left bumper.
 */
void    BumpersProcessor::SetRearLeftBumperIndex( int8_t index )
{
  _rear_left_index = index;
}

/** Access Function. 
 *  @returns The index in the bumper bit-field that represents the rear right bumper.
 */
int8_t  BumpersProcessor::GetRearRightBumperIndex() const
{
  return _rear_right_index;
}

/** Access Function. 
 *  @param index The index in the bumper bit-field that represents the rear right bumper.
 */
void    BumpersProcessor::SetRearRightBumperIndex(  int8_t index )
{
  _rear_right_index = index;
}

/** Calculate the current bumpers bit-field given old and new data, with a mask 
 *  to determine which new bits to process.
 */
uint8_t BumpersProcessor::CalculateBumps( uint8_t old_bumps, uint8_t new_bump_mask, uint8_t new_bumps )
{
  uint8_t bumpers_state;
  uint8_t preserved_bumps;

  // Preserve data
  preserved_bumps = old_bumps & ~new_bump_mask;

  // Find new bumps
  bumpers_state = new_bumps & new_bump_mask;

  // Combine them
  return bumpers_state | preserved_bumps;
}

data_robot::Bumpers BumpersProcessor::CalculateBumpDirection( uint8_t bumpers_state ) const
{
  bool front =        false;
  bool front_left =   false;
  bool front_right =  false;
  bool rear =         false;
  bool rear_left =    false;
  bool rear_right =   false;

  data_robot::Bumpers combined_bumpers;
  data_robot::Bumpers front_bumpers;
  data_robot::Bumpers rear_bumpers;

  if ( _front_index > 0 )
  {
    if ( bumpers_state & (1 << _front_index) )
    {
      front = true;
    }
  }

  if ( _front_left_index > 0 )
  {
    if ( bumpers_state & (1 << _front_left_index) )
    {
      front_left = true;
    }
  }

  if ( _front_right_index > 0 )
  {
    if ( bumpers_state & (1 << _front_right_index) )
    {
      front_right = true;
    }
  }

  if ( _rear_index > 0 )
  {
    if ( bumpers_state & (1 << _rear_index) )
    {
      rear = true;
    }
  }

  if ( _rear_left_index > 0 )
  {
    if ( bumpers_state & (1 << _rear_left_index) )
    {
      rear_left = true;
    }
  }

  if ( _rear_right_index > 0 )
  {
    if ( bumpers_state & (1 << _rear_right_index) )
    {
      rear_right = true;
    }
  }

  front_bumpers.bump_direction = data_robot::Bumpers::NONE;
  rear_bumpers.bump_direction = data_robot::Bumpers::NONE;

  if ( front | (front_left && front_right) )
  {
    front_bumpers.bump_direction = data_robot::Bumpers::FRONT;
  }
  else if ( front_left )
  {
    front_bumpers.bump_direction = data_robot::Bumpers::FRONT_LEFT;
  }
  else if ( front_right )
  {
    front_bumpers.bump_direction = data_robot::Bumpers::FRONT_RIGHT;
  }

  if ( rear | (rear_left && rear_right) )
  {
    rear_bumpers.bump_direction = data_robot::Bumpers::REAR;
  }
  else if ( rear_left )
  {
    rear_bumpers.bump_direction = data_robot::Bumpers::REAR_LEFT;
  }
  else if ( rear_right )
  {
    rear_bumpers.bump_direction = data_robot::Bumpers::REAR_RIGHT;
  }

  if ( front_bumpers.bump_direction != data_robot::Bumpers::NONE )
  {
    if ( rear_bumpers.bump_direction != data_robot::Bumpers::NONE )
    {
      combined_bumpers.bump_direction = data_robot::Bumpers::WEDGED; 
    }
    else
    {
      combined_bumpers = front_bumpers;
    }
  }
  else
  {
    combined_bumpers = rear_bumpers;
  }

  return combined_bumpers;
}
}
