#include <cmath>

#include "BaseModel.hpp"

namespace differential_drive_core
{
static int RoundToInt( double r ) 
{
  return (r > 0.0) ? (r + 0.5) : (r - 0.5); 
}


/** Default constructor.
 *  @param  wheel_radius  The radius of the differential drive wheels in meters.
 *  @param  wheel_base    The separation  of the drive wheels in meters. This 
 *                        value can be corrected using UMBMark calibration 
 *                        techniques.
 *  @param  wheel_ratio   The left wheel radius/ right wheel radius. Ideally this
 *                        should be 1.0, it rarely is. This value can be found
 *                        using UMBMark calibration techniques.
 *  @param  wheel_ticks   The number of encoder counts in a full wheel revolution.
 *  @param  stasis_radius The radius of the stasis wheel if one is in use.
 *  @param  stasis_ticks  The number of encoder counts in a full stasis wheel 
 *                        revolution. If this number is negative the stasis wheel
 *                        calculations are disabled.
 */
BaseModel::BaseModel( double    wheel_radius,
                      uint32_t  wheel_ticks,
                      double    wheel_base,
                      double    wheel_ratio,
                      double    stasis_radius,
                      int32_t   stasis_ticks)
{
  _base_geometry.wheel_radius = wheel_radius;
  _base_geometry.wheel_ticks = wheel_ticks;
  _base_geometry.wheel_base = wheel_base;
  _base_geometry.wheel_ratio = wheel_ratio;
  _base_geometry.stasis_radius = stasis_radius;
  _base_geometry.stasis_ticks = stasis_ticks;

  _p_lock_mutex = new pthread_mutex_t;
  pthread_mutex_init( _p_lock_mutex, NULL );

  pthread_mutex_lock( _p_lock_mutex );

  _tick_rates = CalculateTickRates( _base_geometry );
  _corrections = CalculateCorrections( _base_geometry.wheel_ratio );

  pthread_mutex_unlock( _p_lock_mutex );
}

BaseModel::BaseModel( BaseGeometry_T new_geometry )
{
  _base_geometry = new_geometry;

  _p_lock_mutex = new pthread_mutex_t;
  pthread_mutex_init( _p_lock_mutex, NULL );

  pthread_mutex_lock( _p_lock_mutex );

  _tick_rates = CalculateTickRates( _base_geometry );
  _corrections = CalculateCorrections( _base_geometry.wheel_ratio );

  pthread_mutex_unlock( _p_lock_mutex );
}

/** Calculates the distance and velocities associated with @a new_counts.
 *
 *  @param  new_counts  Contains the newly received encoder counts.
 */
void BaseModel::ConvertCounts(  BaseDistance_T* p_delta_position,
                                BaseVelocities_T* p_velocity,
                                differential_drive::EncoderCounts const & new_counts ) const
{
  double seconds;

  pthread_mutex_lock( _p_lock_mutex );

  // convert milliseconds to seconds
  seconds = new_counts.dt_ms / 1000.0;

  *p_delta_position = CountsToDistance( new_counts, _base_geometry, _tick_rates, _corrections );

  *p_velocity = DistanceToVelocity( *p_delta_position, seconds );

  pthread_mutex_unlock( _p_lock_mutex );
}

/** Accepts an desired linear and angular velocity and returns the velocities
 *  in ticks/sec. This function does not alter the current model state.
 */
differential_drive::TickVelocity BaseModel::ConvertVelocity(  double linear_vel, 
                                                              double angular_vel ) const
{
  differential_drive::TickVelocity new_velocity;

  pthread_mutex_lock( _p_lock_mutex );

  new_velocity = VelocityToTicks( linear_vel, angular_vel, _base_geometry, _tick_rates, _corrections );

  pthread_mutex_unlock( _p_lock_mutex );

  return new_velocity;
}

BaseGeometry_T BaseModel::GetBaseGeometry() const
{
  return _base_geometry;
}

/** @returns true if the geometry is valid and the internal data member has been 
 *  updated.
 */
bool BaseModel::SetBaseGeometry( BaseGeometry_T const & geometry )
{
  if ( CheckGeometryValid(geometry) )
  {
    pthread_mutex_lock( _p_lock_mutex );

    _base_geometry = geometry;
    _tick_rates = CalculateTickRates( _base_geometry );
    _corrections = CalculateCorrections( _base_geometry.wheel_ratio );

    pthread_mutex_unlock( _p_lock_mutex );

    return true;
  }

  return false;
}

bool BaseModel::CheckGeometryValid( BaseGeometry_T const & geometry ) const
{
  bool valid = true;

  if ( geometry.wheel_radius <= 0.0 )
  {
    valid = false;
  }
  else if ( geometry.wheel_ticks <= 0 )
  {
    valid = false;
  }
  else if ( geometry.wheel_base <= 0.0 )
  {
    valid = false;
  }
  else if ( geometry.wheel_ratio <= 0.0 )
  {
    valid = false;
  }

  return valid;
}

bool BaseModel::CheckGeometryStasisValid( BaseGeometry_T const & geometry ) const 
{
  bool valid = true;

  if ( geometry.stasis_radius <= 0.0 )
  {
    valid = false;
  }
  else if ( geometry.stasis_ticks <= 0 )
  {
    valid = false;
  }

  return valid;
}

/** Returns true if the internal geometry defines a valid configuration.
 */
bool BaseModel::GetSetupValid() const
{
  bool valid;

  pthread_mutex_lock( _p_lock_mutex );

  valid = CheckGeometryValid( _base_geometry );

  pthread_mutex_unlock( _p_lock_mutex );

  return valid;
}

/** Returns true if the stasis wheel parameters define a valid configuration.
 */
bool BaseModel::GetStasisValid() const
{
  bool valid;

  pthread_mutex_lock( _p_lock_mutex );

  valid = CheckGeometryStasisValid( _base_geometry );

  pthread_mutex_unlock( _p_lock_mutex );

  return valid;
}

/** Returns the drive wheel radius in meters.
 */
double BaseModel::GetWheelRadius() const
{
  return _base_geometry.wheel_radius;
}

/** Sets the drive wheel radius in meters.
 *  @returns true if @wheel_radius is positive and the internal data member was updated.
 */
bool BaseModel::SetWheelRadius( double wheel_radius)
{
  if ( wheel_radius > 0.0 )
  {
    pthread_mutex_lock( _p_lock_mutex );

    _base_geometry.wheel_radius = wheel_radius;

    _tick_rates = CalculateTickRates( _base_geometry );

    pthread_mutex_unlock( _p_lock_mutex );

    return true;
  }

  return false;
}

/** Returns the drive wheel separation in meters.
 */
double BaseModel::GetWheelBase() const
{
  return _base_geometry.wheel_base;
}

/** Sets the drive wheel separation in meters.
 *  @returns true if @wheel_base is positive and the internal data member was updated.
 */
bool BaseModel::SetWheelBase( double wheel_base)
{
  if ( wheel_base > 0.0 )
  {
    pthread_mutex_lock( _p_lock_mutex );

    _base_geometry.wheel_base = wheel_base;

    _tick_rates = CalculateTickRates( _base_geometry );

    pthread_mutex_unlock( _p_lock_mutex );

    return true;
  }

  return false;
}

/** Returns the drive wheel radius ratio (ie. left radius/ right radius).
 *  
 *  Ideally this value should be 1.0 but it rarely is.
 */
double BaseModel::GetWheelRatio() const
{
  return _base_geometry.wheel_ratio;
}

/** Sets the drive wheel radius ratio (ie. left radius/ right radius).
 *  
 *  Ideally this value should be 1.0 but it rarely is.
 *
 *  @returns true if @wheel_ratio is positive and the internal data member was updated.
 */
bool BaseModel::SetWheelRatio( double wheel_ratio)
{
  if ( wheel_ratio > 0.0 )
  {
    pthread_mutex_lock( _p_lock_mutex );

    _base_geometry.wheel_ratio = wheel_ratio;

    _tick_rates = CalculateTickRates( _base_geometry );
    _corrections = CalculateCorrections( _base_geometry.wheel_ratio );

    pthread_mutex_unlock( _p_lock_mutex );

    return true;
  }

  return false;
}

/** Return the number of encoder ticks in one full rotation of the drive wheels.
 */
uint32_t BaseModel::GetWheelTicks() const
{
  return _base_geometry.wheel_ticks;
}

/** Sets the number of encoder ticks in one full rotation of the drive wheels.
 *
 *  @returns true if @wheel_ticks is positive and the internal data member was updated.
 */
bool BaseModel::SetWheelTicks( uint32_t wheel_ticks)
{
  if ( wheel_ticks > 0 )
  {
    pthread_mutex_lock( _p_lock_mutex );

    _base_geometry.wheel_ticks = wheel_ticks;

    _tick_rates = CalculateTickRates( _base_geometry );

    pthread_mutex_unlock( _p_lock_mutex );

    return true;
  }

  return false;
}

/** Returns the radius of the stasis wheel if one is used in meters.
 */
double BaseModel::GetStasisRadius() const
{
  return _base_geometry.stasis_radius;
}

/** Sets the radius of the stasis wheel if one is used in meters.
 *
 *  @returns  true if @stasis_radius is positive and the internal data member was updated.
 *            Otherwise a value of 0.0 is stored and the stasis wheel is "disabled".
 */
bool BaseModel::SetStasisRadius( double stasis_radius)
{
  if ( stasis_radius > 0.0 )
  {
    pthread_mutex_lock( _p_lock_mutex );

    _base_geometry.stasis_radius = stasis_radius;

    _tick_rates = CalculateTickRates( _base_geometry );

    pthread_mutex_unlock( _p_lock_mutex );

    return true;
  }
  else
  {
    pthread_mutex_lock( _p_lock_mutex );

    _base_geometry.stasis_radius = 0.0;

    _tick_rates = CalculateTickRates( _base_geometry );

    pthread_mutex_unlock( _p_lock_mutex );
  }

  return false;
}

/** Returns the number of encoder ticks in one full revolution of the stasis
 *  wheel. 
 *
 *  If negative stasis wheel calculations are disabled.
 */
int32_t BaseModel::GetStasisTicks() const
{
  return _base_geometry.stasis_ticks;
}

/** Sets the number of encoder ticks in one full revolution of the stasis
 *  wheel. 
 *
 *  If negative stasis wheel calculations are disabled.
 *
 *  @returns  true if @stasis_radius is positive and the internal data member was updated.
 *            Otherwise a value of -1 is stored and the stasis wheel is "disabled".
 */
bool BaseModel::SetStasisTicks( int32_t stasis_ticks)
{
  if ( stasis_ticks > 0 )
  {
    pthread_mutex_lock( _p_lock_mutex );

    _base_geometry.stasis_ticks = stasis_ticks;

    _tick_rates = CalculateTickRates( _base_geometry );

    pthread_mutex_unlock( _p_lock_mutex );

    return true;
  }
  else
  {
    pthread_mutex_lock( _p_lock_mutex );

    _base_geometry.stasis_ticks = -1;

    _tick_rates = CalculateTickRates( _base_geometry );

    pthread_mutex_unlock( _p_lock_mutex );
  }

  return false;
}

/** Returns the calculated encoder ticks per meter for the drive wheels.
 */
double BaseModel::GetTicksPerMeter() const
{
  return _tick_rates.ticks_per_meter;
}

/** Returns the calculated encoder ticks per meter for the drive wheels.
 */
double BaseModel::GetMetersPerTick() const
{
  return _tick_rates.meters_per_tick;
}

/** Returns the calculated difference in encoder ticks needed to turn 
 *  one radian.
 */
double BaseModel::GetTicksPerRadian() const
{
  return _tick_rates.ticks_per_radian;
}

/** Returns the calculated difference in encoder ticks needed to turn 
 *  one radian.
 */
double BaseModel::GetRadiansPerTick() const
{
  return _tick_rates.radians_per_tick;
}

/** Returns the calculated encoder ticks per meter for the stasis wheel.
 */
double BaseModel::GetStasisTicksPerMeter() const
{
  return _tick_rates.stasis_ticks_per_meter;
}

/** Returns the calculated encoder ticks per meter for the stasis wheel.
 */
double BaseModel::GetMetersPerStasisTick() const
{
  return _tick_rates.meters_per_stasis_tick;
}

/** Accepts an desired linear and angular velocity and returns the velocities
 *  in ticks/sec. 
 */
differential_drive::TickVelocity BaseModel::VelocityToTicks(  const double linear_vel, 
                                                              const double angular_vel,
                                                              BaseGeometry_T const & base_geometry,
                                                              TickRates_T const & tick_rates,
                                                              BaseCorrections_T const & corrections ) const
{
  differential_drive::TickVelocity new_velocity;
  double linear_ticks;
  double angular_ticks;
  double left_corrected;
  double right_corrected;

  linear_ticks = linear_vel * tick_rates.ticks_per_meter;
  angular_ticks = angular_vel * tick_rates.ticks_per_radian;
                                            
  left_corrected = ( linear_ticks - (angular_ticks * 0.5) )
                    * corrections.right_in_left_out;

  right_corrected = ( linear_ticks + ( angular_ticks * 0.5) )
                    * corrections.left_in_right_out;

  new_velocity.linear_ticks_sec = RoundToInt( (right_corrected + left_corrected) * 0.5 );
  new_velocity.angular_ticks_sec = RoundToInt( right_corrected - left_corrected );

#if 0
  std::cout << "Left Cor: " << left_corrected 
            << " Right Cor: " << right_corrected 
            << " Lin Ticks: " << new_velocity.linear_ticks_sec
            << " Ang Ticks: " << new_velocity.angular_ticks_sec
            << std::endl;
#endif

  return new_velocity;
}

/** Converts encoder counts to the changes in X, Y and theta.
 */
BaseDistance_T  BaseModel::CountsToDistance(  differential_drive::EncoderCounts const & counts, 
                                              BaseGeometry_T const & geometry, 
                                              TickRates_T const & rates,
                                              BaseCorrections_T const & corrections ) const
{
  BaseDistance_T  delta_position;

  double left_distance;
  double right_distance;
  double stasis_distance;
  double average_distance;

  left_distance = CalculateDistance(  counts.left_count, rates.meters_per_tick, 
                                      corrections.left_in_right_out );

  right_distance = CalculateDistance( counts.right_count, rates.meters_per_tick, 
                                      corrections.right_in_left_out );

  if ( geometry.stasis_ticks > 0 )
  {
    stasis_distance = CalculateDistance( counts.stasis_count, rates.meters_per_stasis_tick );
  }
  else
  {
    stasis_distance = 0.0;
  }

  average_distance = ( left_distance + right_distance ) * 0.5;

  delta_position.theta = CalculateDeltaTheta( left_distance, right_distance, geometry.wheel_base );
  delta_position.linear = average_distance;
  delta_position.stasis = stasis_distance;

  return delta_position;
}

/** Converts base distances into velocities given the elapsed time.
 */
BaseVelocities_T  BaseModel::DistanceToVelocity( BaseDistance_T const & distance, double seconds ) const
{
  BaseVelocities_T velocities;

  velocities.linear = CalculateVelocity( distance.linear, seconds );
  velocities.angular = CalculateVelocity( distance.theta, seconds );
  velocities.stasis = CalculateVelocity( distance.stasis, seconds );

  return velocities;
}

/** Calculates drive wheel ticks/meter, ticks/radian, and stasis wheel 
 *  ticks/meter.
 */
TickRates_T BaseModel::CalculateTickRates( BaseGeometry_T const & geometry ) const
{
  TickRates_T rates;

  rates.ticks_per_meter = CalculateTicksPerMeter( geometry.wheel_radius, 
                                                  geometry.wheel_ticks );

  rates.meters_per_tick = CalculateMetersPerTick( geometry.wheel_radius, 
                                                  geometry.wheel_ticks );

  rates.ticks_per_radian = CalculateTicksPerRadian( geometry.wheel_base, 
                                                    rates.ticks_per_meter );

  rates.radians_per_tick = CalculateRadiansPerTick( geometry.wheel_base, 
                                                    rates.meters_per_tick );

  if ( (geometry.stasis_ticks > 0) && (geometry.stasis_radius > 0.0) )
  {
    rates.stasis_ticks_per_meter = CalculateTicksPerMeter( geometry.stasis_radius, 
                                                           geometry.stasis_ticks );

    rates.meters_per_stasis_tick = CalculateMetersPerTick( geometry.stasis_radius, 
                                                           geometry.stasis_ticks );
  }
  else
  {
    rates.stasis_ticks_per_meter = 0.0;
    rates.meters_per_stasis_tick = 0.0;
  }

  return rates;
}

/** Calculates the ticks required to travel 1 meter.
 *  @param wheel_radius The wheel radius in meters.
 *  @param wheel_ticks  The number of encoder ticks in one full revolution.
 *  @returns The numbers of encoder ticks required to travel 1 meter.
 */
double BaseModel::CalculateTicksPerMeter( double wheel_radius, uint32_t wheel_ticks) const
{
  double wheel_circumference;

  wheel_circumference = wheel_radius * 2.0 * M_PI;

  return (double)wheel_ticks / wheel_circumference;
}

/** Calculates the distance traveled in 1 tick.
 *  @param wheel_radius The wheel radius in meters.
 *  @param wheel_ticks  The number of encoder ticks in one full revolution.
 *  @returns The distance traveled in 1 tick. 
 */
double BaseModel::CalculateMetersPerTick( double wheel_radius, uint32_t wheel_ticks) const
{
  double wheel_circumference;

  wheel_circumference = wheel_radius * 2.0 * M_PI;

  return wheel_circumference / (double) wheel_ticks;
}

/** Calculates the difference in ticks between the drive wheels to turn one radian.
 *  @param wheel_base       The separation between the drive wheels in meters.
 *  @param ticks_per_meter  The number of encoder ticks for the drive wheels to 
 *                          travel 1 meter.
 *  @returns The difference of encoder ticks needed to turn one radian.
 */
double BaseModel::CalculateTicksPerRadian(  double wheel_base, double ticks_per_meter ) const
{
  return ticks_per_meter * wheel_base;
}

/** Calculates the amount turned for a difference of 1 tick between the two wheels.
 *  @param wheel_base       The separation between the drive wheels in meters.
 *  @param ticks_per_meter  The number of encoder ticks for the drive wheels to 
 *                          travel 1 meter.
 *  @returns The amount turned for a difference of 1 tick between the two wheels.
 */
double BaseModel::CalculateRadiansPerTick(  double wheel_base, double meters_per_tick ) const
{
  return meters_per_tick / wheel_base;
}

/** Returns the change in angle based on difference in distance traveled by each wheel.
 */
double BaseModel::CalculateDeltaTheta(  double left_distance,
                                        double right_distance,
                                        double wheel_base ) const
{
  return ( right_distance - left_distance ) / wheel_base;
}

/** Returns the linear distance in meters for a given number of encoder ticks.
 */
double BaseModel::CalculateDistance(  int32_t ticks, 
                                      double meters_per_tick,
                                      double correction_factor ) const
{
  return (double)ticks * meters_per_tick * correction_factor;
}

/** Distance correction factor for the given wheel ratio.
 *
 *  Calibrated dead reckoning, as per "A Quick & Easy Guide to Calibrating Your 
 *  Robot Using UMBMark" at: http://www.dprg.org/articles/2009-02a/
 *  where wheel_ratio = Ed
 */
BaseCorrections_T BaseModel::CalculateCorrections( double wheel_ratio ) const
{
  BaseCorrections_T corrections;

  corrections.left_in_right_out = 2.0 / (( 1.0 / wheel_ratio ) + 1.0);
  corrections.right_in_left_out = 2.0 / (1.0 + wheel_ratio);

  return corrections;
}

/** Returns the linear velocity in meters/sec for a given distance and time.
 */
double BaseModel::CalculateVelocity(  double distance, double seconds ) const
{
  return distance / seconds;
}
}
