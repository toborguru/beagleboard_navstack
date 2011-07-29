// BaseModel.hpp
 
#ifndef GUARD_BaseModel
#define GUARD_BaseModel

#include "IEncoderCountsEndpoint.hpp"

#include "diff_drive/EncoderCounts.h"
#include "diff_drive/TickVelocity.h"

namespace diff_drive_core
{ 
  class BaseModel
  {
    public:
  
      BaseModel(  double wheel_radius = 0.5, 
                  double wheel_base = 1.0,
                  double wheel_ratio = 1.0,
                  int16_t wheel_ticks = 100 );
 
      void    AddEncoderCounts(diff_drive::EncoderCounts new_counts); 
      diff_drive::TickVelocity VelocityToTicks(double linear_vel, double angular_vel);
 
      double  GetX();
      void    SetX(double x);

      double  GetY();
      void    SetY(double y);

      double  GetTheta();
      void    SetTheta(double theta);

      double  GetWheelRadius();
      void    SetWheelRadius(double wheel_radius);

      double  GetWheelBase();
      void    SetWheelBase(double wheel_base);

      double  GetWheelRatio();
      void    SetWheelRatio(double wheel_ratio);

      double  GetWheelTicks();
      void    SetWheelTicks(uint16_t wheel_ticks);
 
    private:
      void  CalculateProperties();
      double  _x;             // meters
      double  _y;             // meters 
      double  _theta;         // radians
      double  _wheel_radius;  // meters
      double  _wheel_base;    // wheel separation - meters
      double  _wheel_ratio;   // Ratio of wheel diameter differences
      uint16_t  _wheel_ticks; // Number of ticks in one complete wheel rotation

      uint16_t  _ticks_per_meter; 
      uint16_t  _ticks_per_radian; 
  };
}
 
#endif /* GUARD_BaseModel */
