// BumpersProcessor.hpp
 
#ifndef GUARD_BumpersProcessor
#define GUARD_BumpersProcessor

#include "data_robot/Bumpers.h"

namespace data_robot_core
{ 
class BumpersProcessor
{
public:

  BumpersProcessor( uint32_t front_bumper_mask =        0,
                    uint32_t front_left_bumper_mask =   0,
                    uint32_t front_right_bumper_mask =  0,
                    uint32_t rear_bumper_mask =         0,
                    uint32_t rear_left_bumper_mask =    0,
                    uint32_t rear_right_bumper_mask =   0 );

  void      AddNewData( uint32_t bumpers_mask,  uint32_t bumpers_state );

  uint32_t  GetCurrentBumps() const;

  uint8_t   GetBumpDirection() const;

  // Bumper Geometry Functions
  void      AddFrontBumperIndex(  int8_t    index );
  uint32_t  GetFrontBumperMask()  const;
  void      SetFrontBumperMask(   uint32_t  mask );

  void      AddFrontLeftBumperIndex(  int8_t   index );
  uint32_t  GetFrontLeftBumperMask()  const;
  void      SetFrontLeftBumperMask(   uint32_t mask );

  void      AddFrontRightBumperIndex( int8_t   index );
  uint32_t  GetFrontRightBumperMask() const;
  void      SetFrontRightBumperMask(  uint32_t mask );

  void      AddRearBumperIndex( int8_t   index );
  uint32_t  GetRearBumperMask() const;
  void      SetRearBumperMask(  uint32_t mask );

  void      AddRearLeftBumperIndex( int8_t   index );
  uint32_t  GetRearLeftBumperMask() const;
  void      SetRearLeftBumperMask(  uint32_t mask );

  void      AddRearRightBumperIndex(  int8_t   index );
  uint32_t  GetRearRightBumperMask()  const;
  void      SetRearRightBumperMask(   uint32_t mask );

private:
  uint32_t  CalculateBumps( uint32_t old_bumps, uint32_t new_bump_mask, uint32_t new_bumps );
  uint8_t   CalculateBumpDirection( uint32_t bumpers_state ) const;

  uint32_t  _front_mask;
  uint32_t  _front_left_mask;
  uint32_t  _front_right_mask;
  uint32_t  _rear_mask;
  uint32_t  _rear_left_mask;
  uint32_t  _rear_right_mask;

  uint32_t _current_bumps;
};
}
 
#endif /* GUARD_BumpersProcessor */
