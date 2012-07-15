// BumpersProcessor.hpp
 
#ifndef GUARD_BumpersProcessor
#define GUARD_BumpersProcessor

#include "data_robot/Bumpers.h"

namespace data_robot_core
{ 
  class BumpersProcessor
  {
    public:
  
      BumpersProcessor( int8_t front_bumper_index =       -1,
                        int8_t front_left_bumper_index =  -1,
                        int8_t front_right_bumper_index = -1,
                        int8_t rear_bumper_index =        -1,
                        int8_t rear_left_bumper_index =   -1,
                        int8_t rear_right_bumper_index =  -1 );

      void    AddNewData( uint8_t bumpers_mask,  uint8_t bumpers_state );

      uint8_t GetCurrentBumps() const;

      data_robot::Bumpers GetBumpDirection() const;

      // Bumper Geometry Functions
      int8_t  GetFrontBumperIndex() const;
      void    SetFrontBumperIndex(  int8_t index );

      int8_t  GetFrontLeftBumperIndex() const;
      void    SetFrontLeftBumperIndex(  int8_t index );

      int8_t  GetFrontRightBumperIndex()  const;
      void    SetFrontRightBumperIndex(   int8_t index );

      int8_t  GetRearBumperIndex()  const;
      void    SetRearBumperIndex(   int8_t index );

      int8_t  GetRearLeftBumperIndex()  const;
      void    SetRearLeftBumperIndex(   int8_t index );

      int8_t  GetRearRightBumperIndex() const;
      void    SetRearRightBumperIndex(  int8_t index );

    private:
      uint8_t CalculateBumps( uint8_t old_bumps, uint8_t new_bump_mask, uint8_t new_bumps );
      data_robot::Bumpers CalculateBumpDirection( uint8_t bumpers_state ) const;
      
      int8_t  _front_index;
      int8_t  _front_left_index;
      int8_t  _front_right_index;
 
      int8_t  _rear_index;
      int8_t  _rear_left_index;
      int8_t  _rear_right_index;

      uint8_t _current_bumps;
  };
}
 
#endif /* GUARD_BumpersProcessor */
