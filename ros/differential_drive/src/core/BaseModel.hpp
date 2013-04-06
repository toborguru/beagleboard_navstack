// BaseModel.hpp
 
#ifndef GUARD_BaseModel
#define GUARD_BaseModel

#include <pthread.h>

#include "differential_drive/EncoderCounts.h"
#include "differential_drive/TickVelocity.h"

namespace differential_drive_core
{ 
typedef struct
{
  double theta;   // radians
  double linear;  // meters
  double stasis;  // meters
} BaseDistance_T;

typedef struct
{
  double linear;  // meters/sec
  double angular; // radians/sec
  double stasis;  // meters/sec
} BaseVelocities_T;

typedef struct
{
  double left_in_right_out;  // multiplier ratio
  double right_in_left_out;  // multiplier ratio
} BaseCorrections_T;

typedef struct
{
  double    wheel_radius;   // meters                                           
  uint32_t  wheel_ticks;    // wheel separation - meters                        
  double    wheel_base;     // Ratio of wheel diameter differences              
  double    wheel_ratio;    // Number of ticks in one complete wheel rotation   
  double    stasis_radius;  // Radius of the stasis wheel, if enabled.          
  int32_t   stasis_ticks;   // Number of ticks in one complete wheel rotation,  
} BaseGeometry_T;         // negative numbers disables stasis wheel support.

typedef struct
{
  double ticks_per_meter; 
  double meters_per_tick;
  double ticks_per_radian;
  double radians_per_tick;
  double stasis_ticks_per_meter;
  double meters_per_stasis_tick;
} TickRates_T;

class BaseModel
{
public:

  BaseModel(  double    wheel_radius  = 0.0, 
              uint32_t  wheel_ticks   = 0,
              double    wheel_base    = 0.0,
              double    wheel_ratio   = 1.0,
              double    stasis_radius = 0.0,
              int32_t   stasis_ticks  = -1 );

  BaseModel(  BaseGeometry_T  base_geometry );

  void      ConvertCounts(  BaseDistance_T* p_delta_position, 
                            BaseVelocities_T* p_velocity,
                            differential_drive::EncoderCounts const & new_counts ) const; 

  differential_drive::TickVelocity ConvertVelocity( double linear_vel, double angular_vel ) const;

  // Base Geometry Functions
  BaseGeometry_T  GetBaseGeometry() const;
  bool            SetBaseGeometry( BaseGeometry_T const & geometry );

  bool      CheckGeometryValid( BaseGeometry_T const & geometry ) const;
  bool      CheckGeometryStasisValid( BaseGeometry_T const & geometry ) const;

  bool      GetSetupValid() const;
  bool      GetStasisValid() const;

  double    GetWheelRadius() const;
  bool      SetWheelRadius(double wheel_radius);

  double    GetWheelBase() const;
  bool      SetWheelBase(double wheel_base);

  double    GetWheelRatio() const;
  bool      SetWheelRatio(double wheel_ratio);

  uint32_t  GetWheelTicks() const;
  bool      SetWheelTicks(uint32_t wheel_ticks);

  double    GetStasisRadius() const;
  bool      SetStasisRadius(double stasis_radius);

  int32_t   GetStasisTicks() const;
  bool      SetStasisTicks(int32_t stasis_ticks);

  double    GetTicksPerMeter() const;
  double    GetMetersPerTick() const;

  double    GetTicksPerRadian() const;
  double    GetRadiansPerTick() const;
  
  double    GetStasisTicksPerMeter() const;
  double    GetMetersPerStasisTick() const;

private:
  differential_drive::TickVelocity VelocityToTicks( double linear_vel, 
                                                    double angular_vel,
                                                    BaseGeometry_T const &  base_geometry,
                                                    TickRates_T const & tick_rates,
                                                    BaseCorrections_T const & corrections ) const;

  BaseDistance_T  CountsToDistance( differential_drive::EncoderCounts const & counts, 
                                    BaseGeometry_T const & geometry, 
                                    TickRates_T const & rates,
                                    BaseCorrections_T const & corrections ) const;

  BaseVelocities_T  DistanceToVelocity( BaseDistance_T const & distance, double seconds ) const;

  // Base Geometry Property  Functions
  TickRates_T CalculateTickRates( BaseGeometry_T const & geometry ) const;

  double      CalculateTicksPerMeter( double wheel_radius, 
                                      uint32_t wheel_ticks ) const;
  
  double      CalculateMetersPerTick( double wheel_radius, 
                                      uint32_t wheel_ticks ) const;
  
  double      CalculateTicksPerRadian(  double wheel_base, 
                                        double ticks_per_meter ) const;

  double      CalculateRadiansPerTick(  double wheel_base, 
                                        double ticks_per_meter ) const;

  BaseCorrections_T CalculateCorrections( double wheel_ratio ) const; 

  // State Functions 
  double      CalculateDeltaTheta(  double left_distance, 
                                    double right_distance, 
                                    double wheel_base ) const;
  
  double      CalculateDeltaX(  double average_distance, 
                                double delta_theta ) const;
  
  double      CalculateDeltaY(  double average_distance, 
                                double delta_theta ) const;

  double      CalculateDistance(  int32_t ticks,
                                  double meters_per_tick,
                                  double correction_factor = 1.0 ) const;

  double      CalculateVelocity(  double distance,
                                  double seconds ) const;

  // Base Geometry Property Variables
  BaseGeometry_T    _base_geometry;                                  

  TickRates_T       _tick_rates; 

  BaseCorrections_T _corrections;

  pthread_mutex_t*  _p_lock_mutex;
};
}

#endif /* GUARD_BaseModel */
