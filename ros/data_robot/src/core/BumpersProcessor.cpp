#include "BumpersProcessor.hpp"

namespace data_robot_core
{
/** Default constructor.
 *  @param  front_bumper_mask  Bit position of of the front bumper.
 *  @param  front_left_bumper_mask  Bit position of of the front left bumper.
 *  @param  front_right_bumper_mask  Bit position of of the front right bumper.
 *  @param  rear_bumper_mask  Bit position of of the rear bumper.
 *  @param  rear_left_bumper_mask  Bit position of of the rear left bumper.
 *  @param  rear_right_bumper_mask  Bit position of of the rear right bumper.
 */
BumpersProcessor::BumpersProcessor( uint32_t  front_bumper_mask,
                                    uint32_t  front_left_bumper_mask,
                                    uint32_t  front_right_bumper_mask,
                                    uint32_t  rear_bumper_mask,
                                    uint32_t  rear_left_bumper_mask,
                                    uint32_t  rear_right_bumper_mask )
                : _front_mask( front_bumper_mask ), 
                  _front_left_mask( front_left_bumper_mask ), 
                  _front_right_mask( front_right_bumper_mask ), 
                  _rear_mask( rear_bumper_mask ), 
                  _rear_left_mask( rear_left_bumper_mask ), 
                  _rear_right_mask( rear_right_bumper_mask ),
                  _current_bumps( 0 )
{
}

/** Function to report new bumper status to this processor.
 *
 *  A mask is used so that bumper data can be reported from multiple sources. All
 *  active bumpers will be kept until masked with a 0 state.
 */
void      BumpersProcessor::AddNewData( uint32_t bumpers_mask,  uint32_t bumpers_state )
{
  _current_bumps = CalculateBumps( _current_bumps, bumpers_mask, bumpers_state );
}

/** Access Function. 
 *  @returns Bit-field of the bumpers currently pressed.
 */
uint32_t  BumpersProcessor::GetCurrentBumps() const
{
  return _current_bumps;
}

/** Access Function. 
 *  @returns All current bumps combined into a directional enumeration.
 */
uint8_t   BumpersProcessor::GetBumpDirection() const
{
  return CalculateBumpDirection( _current_bumps );
}

/** Performs a read-modify-write on the bumper mask to add a bit at the given index.
 *  @param index Bit index to add to the bumper mask. If negative nothing is changed.
 */
void      BumpersProcessor::AddFrontBumperIndex( int8_t index )
{
  if ( (index >= 0) && (index < 32) )
  {
    _front_mask |= 1<<index;
  }
}

/** Access Function. 
 *  @returns The mask in the bumper bit-field that represents the front center bumper.
 */
uint32_t  BumpersProcessor::GetFrontBumperMask() const
{
  return _front_mask;
}

/** Access Function. 
 *  @param mask The mask in the bumper bit-field that represents the front center bumper.
 */
void      BumpersProcessor::SetFrontBumperMask(  uint32_t mask )
{
  _front_mask = mask;
}

/** Performs a read-modify-write on the bumper mask to add a bit at the given index.
 *  @param index Bit index to add to the bumper mask. If negative nothing is changed.
 */
void      BumpersProcessor::AddFrontLeftBumperIndex( int8_t index )
{
  if ( (index >= 0) && (index < 32) )
  {
    _front_left_mask |= 1<<index;
  }
}

/** Access Function. 
 *  @returns The mask in the bumper bit-field that represents the front left bumper.
 */
uint32_t  BumpersProcessor::GetFrontLeftBumperMask() const
{
  return _front_left_mask;
}

/** Access Function. 
 *  @param mask The mask in the bumper bit-field that represents the front left bumper.
 */
void      BumpersProcessor::SetFrontLeftBumperMask(  uint32_t mask )
{
  _front_left_mask = mask;
}

/** Performs a read-modify-write on the bumper mask to add a bit at the given index.
 *  @param index Bit index to add to the bumper mask. If negative nothing is changed.
 */
void      BumpersProcessor::AddFrontRightBumperIndex( int8_t index )
{
  if ( (index >= 0) && (index < 32) )
  {
    _front_right_mask |= 1<<index;
  }
}

/** Access Function. 
 *  @returns The mask in the bumper bit-field that represents the front right bumper.
 */
uint32_t  BumpersProcessor::GetFrontRightBumperMask() const
{
  return _front_right_mask;
}

/** Access Function. 
 *  @param mask The mask in the bumper bit-field that represents the front right bumper.
 */
void      BumpersProcessor::SetFrontRightBumperMask( uint32_t mask )
{
  _front_right_mask = mask;
}

/** Performs a read-modify-write on the bumper mask to add a bit at the given index.
 *  @param index Bit index to add to the bumper mask. If negative nothing is changed.
 */
void      BumpersProcessor::AddRearBumperIndex( int8_t index )
{
  if ( (index >= 0) && (index < 32) )
  {
    _rear_mask |= 1<<index;
  }
}

/** Access Function. 
 *  @returns The mask in the bumper bit-field that represents the rear center bumper.
 */
uint32_t  BumpersProcessor::GetRearBumperMask() const
{
  return _rear_mask;
}

/** Access Function. 
 *  @param mask The mask in the bumper bit-field that represents the rear center bumper.
 */
void      BumpersProcessor::SetRearBumperMask( uint32_t mask )
{
  _rear_mask = mask;
}

/** Performs a read-modify-write on the bumper mask to add a bit at the given index.
 *  @param index Bit index to add to the bumper mask. If negative nothing is changed.
 */
void      BumpersProcessor::AddRearLeftBumperIndex( int8_t index )
{
  if ( (index >= 0) && (index < 32) )
  {
    _rear_left_mask |= 1<<index;
  }
}

/** Access Function. 
 *  @returns The mask in the bumper bit-field that represents the rear left bumper.
 */
uint32_t  BumpersProcessor::GetRearLeftBumperMask() const
{
  return _rear_left_mask;
}

/** Access Function. 
 *  @param mask The mask in the bumper bit-field that represents the rear left bumper.
 */
void      BumpersProcessor::SetRearLeftBumperMask( uint32_t mask )
{
  _rear_left_mask = mask;
}

/** Performs a read-modify-write on the bumper mask to add a bit at the given index.
 *  @param index Bit index to add to the bumper mask. If negative nothing is changed.
 */
void      BumpersProcessor::AddRearRightBumperIndex( int8_t index )
{
  if ( (index >= 0) && (index < 32) )
  {
    _rear_right_mask |= 1<<index;
  }
}

/** Access Function. 
 *  @returns The mask in the bumper bit-field that represents the rear right bumper.
 */
uint32_t  BumpersProcessor::GetRearRightBumperMask() const
{
  return _rear_right_mask;
}

/** Access Function. 
 *  @param mask The mask in the bumper bit-field that represents the rear right bumper.
 */
void      BumpersProcessor::SetRearRightBumperMask(  uint32_t mask )
{
  _rear_right_mask = mask;
}

/** Calculate the current bumpers bit-field given old and new data, with a mask 
 *  to determine which new bits to process.
 */
uint32_t  BumpersProcessor::CalculateBumps( uint32_t old_bumps, uint32_t new_bump_mask, uint32_t new_bumps )
{
  uint32_t bumpers_state;
  uint32_t preserved_bumps;

  // Preserve data
  preserved_bumps = old_bumps & ~new_bump_mask;

  // Find new bumps
  bumpers_state = new_bumps & new_bump_mask;

  // Combine them
  return bumpers_state | preserved_bumps;
}

uint8_t   BumpersProcessor::CalculateBumpDirection( uint32_t bumpers_state ) const
{
  bool front =        false;
  bool front_left =   false;
  bool front_right =  false;
  bool rear =         false;
  bool rear_left =    false;
  bool rear_right =   false;
 
  uint8_t combined_direction =  0;
  uint8_t front_direction =     0;
  uint8_t rear_direction =      0;

  // Process raw bumpers
  if ( bumpers_state & _front_mask )
  {
    front = true;
  }

  if ( bumpers_state & _front_left_mask )
  {
    front_left = true;
  }

  if ( bumpers_state & _front_right_mask )
  {
    front_right = true;
  }
  
  if ( bumpers_state & _rear_mask )
  {
    rear = true;
  }

  if ( bumpers_state & _rear_left_mask )
  {
    rear_left = true;
  }

  if ( bumpers_state & _rear_right_mask )
  {
    rear_right = true;
  }

  // Process combined bumps
  if ( front | (front_left && front_right) )
  {
    front_direction = data_robot::Bumpers::FRONT;
  }
  else if ( front_left )
  {
    front_direction = data_robot::Bumpers::FRONT_LEFT;
  }
  else if ( front_right )
  {
    front_direction = data_robot::Bumpers::FRONT_RIGHT;
  }

  if ( rear | (rear_left && rear_right) )
  {
    rear_direction = data_robot::Bumpers::REAR;
  }
  else if ( rear_left )
  {
    rear_direction = data_robot::Bumpers::REAR_LEFT;
  }
  else if ( rear_right )
  {
    rear_direction = data_robot::Bumpers::REAR_RIGHT;
  }

  if ( front_direction != data_robot::Bumpers::NONE )
  {
    if ( rear_direction != data_robot::Bumpers::NONE )
    {
      combined_direction = data_robot::Bumpers::WEDGED; 
    }
    else
    {
      combined_direction = front_direction;
    }
  }
  else
  {
    combined_direction = rear_direction;
  }

  return combined_direction;
}
}
