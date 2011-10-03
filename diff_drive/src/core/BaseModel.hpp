// BaseModel.hpp
 
#ifndef GUARD_BaseModel
#define GUARD_BaseModel

#include "diff_drive/EncoderCounts.h"
#include "diff_drive/TickVelocity.h"

namespace diff_drive_core
{ 
  class BaseModel
  {
    public:
  
      BaseModel(  const double    wheel_radius  = 0.5, 
                  const uint16_t  wheel_ticks   = 100,
                  const double    wheel_base    = 1.0,
                  const double    wheel_ratio   = 1.0,
                  const double    stasis_radius = 0.0,
                  const int16_t   stasis_ticks  = -1 );
 
      void      ConvertCounts( const diff_drive::EncoderCounts new_counts ); 
      diff_drive::TickVelocity VelocityToTicks( const double linear_vel, const double angular_vel ) const;

      // Current State
      double    GetDeltaX() const;
      double    GetDeltaY() const;
      double    GetDeltaTheta() const;
      double    GetDeltaStasis() const;

      double    GetLinearVelocity() const;
      double    GetAngularVelocity() const;
      double    GetStasisVelocity() const;

      // Base Geometry Properties
      double    GetWheelRadius() const;
      void      SetWheelRadius(const double wheel_radius);

      double    GetWheelBase() const;
      void      SetWheelBase(const double wheel_base);

      double    GetWheelRatio() const;
      void      SetWheelRatio(const double wheel_ratio);

      uint16_t  GetWheelTicks() const;
      void      SetWheelTicks(const uint16_t wheel_ticks);

      double    GetStasisRadius() const;
      void      SetStasisRadius(const double stasis_radius);
 
      int16_t   GetStasisTicks() const;
      void      SetStasisTicks(const int16_t stasis_ticks);

      double    GetTicksPerMeter() const;
      double    GetTicksPerRadian() const;
      double    GetStasisTicksPerMeter() const;

    private:
      // Base Geometry Property  Functions
      void      CalculateProperties();

      double    CalculateTicksPerMeter( const double wheel_radius, 
                                        const uint16_t wheel_ticks ) const;
      
      double    CalculateTicksPerRadian(  const double wheel_base, 
                                          const double ticks_per_meter ) const;

      // State Functions 
      double    CalculateDeltaTheta(  const double left_distance, 
                                      const double right_distance, 
                                      const double wheel_base ) const;
      
      double    CalculateDeltaX(  const double average_distance, 
                                  const double delta_theta ) const;
      
      double    CalculateDeltaY(  const double average_distance, 
                                  const double delta_theta ) const;
  
      double    CalculateDistance(  const int16_t ticks,
                                    const double ticks_per_meter,
                                    const double correction_factor = 1.0 ) const;

      double    CalculateVelocity(  const double distance,
                                    const double seconds ) const;

      // Base Geometry Property Variables
      double    _wheel_radius;    // meters
      double    _wheel_base;      // wheel separation - meters
      double    _wheel_ratio;     // Ratio of wheel diameter differences
      uint16_t  _wheel_ticks;     // Number of ticks in one complete wheel rotation
      double    _stasis_radius;   // Radius of the stasis wheel, if enabled.
      int16_t   _stasis_ticks;    // Number of ticks in one complete wheel rotation, 
                                  // negative numbers disables stasis wheel support.

      double _ticks_per_meter; 
      double _ticks_per_radian; 
      double _stasis_ticks_per_meter;

      // Current State Variables
      double    _delta_x;           // meters
      double    _delta_y;           // meters 
      double    _delta_theta;       // radians
      double    _delta_stasis;      // meters
     
      double    _linear_velocity;   // meters/sec
      double    _angular_velocity;  // radians/sec
      double    _stasis_velocity;   // meters/sec
  };
}
 
#endif /* GUARD_BaseModel */
