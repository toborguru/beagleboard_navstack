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
  double    wheel_radius;   // meters                                           
  uint16_t  wheel_ticks;    // wheel separation - meters                        
  double    wheel_base;     // Ratio of wheel diameter differences              
  double    wheel_ratio;    // Number of ticks in one complete wheel rotation   
  double    stasis_radius;  // Radius of the stasis wheel, if enabled.          
  int16_t   stasis_ticks;   // Number of ticks in one complete wheel rotation,  
} BaseGeometry_T;         // negative numbers disables stasis wheel support.

typedef struct
{
  double ticks_per_meter; 
  double ticks_per_radian; 
  double stasis_ticks_per_meter;
} TickRates_T;

class BaseModel
{
public:

  BaseModel(  double    wheel_radius  = 0.0, 
              uint16_t  wheel_ticks   = 0,
              double    wheel_base    = 0,
              double    wheel_ratio   = 1.0,
              double    stasis_radius = 0.0,
              int16_t   stasis_ticks  = -1 );

  BaseModel(  BaseGeometry_T  base_geometry );

  void      ConvertCounts(  BaseDistance_T* p_delta_position, 
                            BaseVelocities_T* p_velocity,
                            differential_drive::EncoderCounts new_counts ) const; 

  differential_drive::TickVelocity VelocityToTicks( double linear_vel, double angular_vel ) const;

  // Base Geometry Functions
  BaseGeometry_T  GetBaseGeometry() const;
  void            SetBaseGeometry( BaseGeometry_T geometry );

  bool      GetSetupValid() const;
  bool      GetStasisValid() const;

  double    GetWheelRadius() const;
  void      SetWheelRadius(double wheel_radius);

  double    GetWheelBase() const;
  void      SetWheelBase(double wheel_base);

  double    GetWheelRatio() const;
  void      SetWheelRatio(double wheel_ratio);

  uint16_t  GetWheelTicks() const;
  void      SetWheelTicks(uint16_t wheel_ticks);

  double    GetStasisRadius() const;
  void      SetStasisRadius(double stasis_radius);

  int16_t   GetStasisTicks() const;
  void      SetStasisTicks(int16_t stasis_ticks);

  double    GetTicksPerMeter() const;
  double    GetTicksPerRadian() const;
  double    GetStasisTicksPerMeter() const;

private:
  BaseDistance_T  CountsToDistance( differential_drive::EncoderCounts counts, 
                                    BaseGeometry_T geometry, 
                                    const TickRates_T* p_rates = NULL ) const;

  BaseVelocities_T  DistanceToVelocity( BaseDistance_T distance, double seconds ) const;

  // Base Geometry Property  Functions
  TickRates_T CalculateTickRates( BaseGeometry_T geometry ) const;

  double      CalculateTicksPerMeter( double wheel_radius, 
                                      uint16_t wheel_ticks ) const;
  
  double      CalculateTicksPerRadian(  double wheel_base, 
                                        double ticks_per_meter ) const;

  // State Functions 
  double      CalculateDeltaTheta(  double left_distance, 
                                    double right_distance, 
                                    double wheel_base ) const;
  
  double      CalculateDeltaX(  double average_distance, 
                                double delta_theta ) const;
  
  double      CalculateDeltaY(  double average_distance, 
                                double delta_theta ) const;

  double      CalculateDistance(  int16_t ticks,
                                  double ticks_per_meter,
                                  double correction_factor = 1.0 ) const;

  double      CalculateVelocity(  double distance,
                                  double seconds ) const;

  double      LeftInRightOutCorrection( double wheel_ratio ) const; 
  double      RightInLeftOutCorrection( double wheel_ratio ) const; 

  // Base Geometry Property Variables
  BaseGeometry_T  _base_geometry;                                  

  TickRates_T     _tick_rates; 

  pthread_mutex_t  *_p_lock_mutex;
};
}

#endif /* GUARD_BaseModel */
