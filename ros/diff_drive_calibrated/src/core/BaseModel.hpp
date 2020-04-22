// BaseModel.hpp
 
#ifndef GUARD_BaseModel
#define GUARD_BaseModel

#include <pthread.h>

#include "diff_drive_calibrated/EncoderCounts.h"
#include "diff_drive_calibrated/TickVelocity.h"

namespace diff_drive_core
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
  double left_in_right_out;     // multiplier ratio
  double right_in_left_out;     // multiplier ratio
  double angle_pos_in_neg_out;  // multiplier ratio
  double angle_neg_in_pos_out;  // multiplier ratio
} BaseCorrections_T;

typedef struct
{
  double    wheel_radius;   // meters                                           
  uint32_t  wheel_ticks;    // wheel separation - meters                        
  double    wheel_base;     // Ratio of wheel diameter differences              
  double    wheel_ratio;    // Number of ticks in one complete wheel rotation   
  double    stasis_radius;  // Radius of the stasis wheel, if enabled.          
  int32_t   stasis_ticks;   // Number of ticks in one complete wheel rotation,  
} BaseGeometry_T;           // negative numbers disables stasis wheel support.

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

  void      convertCounts(  BaseDistance_T* p_delta_position, 
                            BaseVelocities_T* p_velocity,
                            diff_drive_calibrated::EncoderCounts const & new_counts ) const; 

  diff_drive_calibrated::TickVelocity convertVelocity( double linear_vel, double angular_vel ) const;

  // Base Geometry Functions
  BaseGeometry_T  getBaseGeometry() const;
  bool            setBaseGeometry( BaseGeometry_T const & geometry );

  bool      checkGeometryValid( BaseGeometry_T const & geometry ) const;
  bool      checkGeometryStasisValid( BaseGeometry_T const & geometry ) const;

  bool      getSetupValid() const;
  bool      getStasisValid() const;

  double    getWheelRadius() const;
  bool      setWheelRadius(double wheel_radius);

  double    getWheelBase() const;
  bool      setWheelBase(double wheel_base);

  double    getWheelRatio() const;
  bool      setWheelRatio(double wheel_ratio);

  uint32_t  getWheelTicks() const;
  bool      setWheelTicks(uint32_t wheel_ticks);

  double    getStasisRadius() const;
  bool      setStasisRadius(double stasis_radius);

  int32_t   getStasisTicks() const;
  bool      setStasisTicks(int32_t stasis_ticks);

  double    getTicksPerMeter() const;
  double    getMetersPerTick() const;

  double    getTicksPerRadian() const;
  double    getRadiansPerTick() const;
  
  double    getStasisTicksPerMeter() const;
  double    getMetersPerStasisTick() const;

  double    getLeftInRightOutCorrection() const;
  double    getRightInLeftOutCorrection() const;

  double    getAnglePosInNegOutCorrection() const;
  double    getAngleNegInPosOutCorrection() const;

private:
  diff_drive_calibrated::TickVelocity velocityToTicks( double linear_vel, 
                                                    double angular_vel,
                                                    BaseGeometry_T const &  base_geometry,
                                                    TickRates_T const & tick_rates,
                                                    BaseCorrections_T const & corrections ) const;

  BaseDistance_T  countsToDistance( diff_drive_calibrated::EncoderCounts const & counts, 
                                    BaseGeometry_T const & geometry, 
                                    TickRates_T const & rates,
                                    BaseCorrections_T const & corrections ) const;

  BaseVelocities_T  distanceToVelocity( BaseDistance_T const & distance, double milli_seconds ) const;

  // Base Geometry Property  Functions
  TickRates_T calculateTickRates( BaseGeometry_T const & geometry ) const;

  double      calculateTicksPerMeter( double wheel_radius, 
                                      uint32_t wheel_ticks ) const;
  
  double      calculateMetersPerTick( double wheel_radius, 
                                      uint32_t wheel_ticks ) const;
  
  double      calculateTicksPerRadian(  double wheel_base, 
                                        double ticks_per_meter ) const;

  double      calculateRadiansPerTick(  double wheel_base, 
                                        double ticks_per_meter ) const;

  BaseCorrections_T calculateCorrections( double wheel_ratio ) const; 

  // State Functions 
  double      calculateDeltaTheta(  double left_distance, 
                                    double right_distance, 
                                    double wheel_base,
                                    double correction_factor = 1.0,
                                    double inv_correction_factor = 1.0 ) const;
  
  double      calculateDeltaX(  double average_distance, 
                                double delta_theta ) const;
  
  double      calculateDeltaY(  double average_distance, 
                                double delta_theta ) const;

  double      calculateDistance(  int32_t ticks,
                                  double meters_per_tick,
                                  double correction_factor = 1.0 ) const;

  double      calculateVelocity(  double distance,
                                  double milli_seconds ) const;

  // Base Geometry Property Variables
  BaseGeometry_T    _base_geometry;                                  

  TickRates_T       _tick_rates; 

  BaseCorrections_T _corrections;

  pthread_mutex_t*  _p_lock_mutex;
};
}

#endif /* GUARD_BaseModel */
