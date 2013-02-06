#include <cmath>

#include "BaseModel.hpp"

namespace differential_drive_core
{
static int RoundInt( double r ) 
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
                      uint16_t  wheel_ticks,
                      double    wheel_base,
                      double    wheel_ratio,
                      double    stasis_radius,
                      int16_t   stasis_ticks)
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

  pthread_mutex_unlock( _p_lock_mutex );
}

BaseModel::BaseModel( BaseGeometry_T new_geometry )
{
  _base_geometry = new_geometry;

  _p_lock_mutex = new pthread_mutex_t;
  pthread_mutex_init( _p_lock_mutex, NULL );

  pthread_mutex_lock( _p_lock_mutex );

  _tick_rates = CalculateTickRates( _base_geometry );

  pthread_mutex_unlock( _p_lock_mutex );
}

/** Calculates the distance and velocities associated with @a new_counts.
 *
 *  @param  new_counts  Contains the newly received encoder counts.
 */
void BaseModel::ConvertCounts(  BaseDistance_T* p_delta_position,
                                BaseVelocities_T* p_velocity,
                                differential_drive::EncoderCounts new_counts ) const
{
  double seconds;

  // convert milliseconds to seconds
  seconds = new_counts.dt_ms / 1000.0;

  pthread_mutex_lock( _p_lock_mutex );

  *p_delta_position = CountsToDistance( new_counts, _base_geometry, &_tick_rates );

  pthread_mutex_unlock( _p_lock_mutex );

  *p_velocity = DistanceToVelocity( *p_delta_position, seconds );
}

/** Accepts an desired linear and angular velocity and returns the velocities
 *  in ticks/sec. This function does not alter the current model state.
 */
differential_drive::TickVelocity BaseModel::VelocityToTicks( const double linear_vel, const double angular_vel ) const
{
  differential_drive::TickVelocity new_velocity;
  double linear_ticks;
  double angular_ticks;
  double left_corrected;
  double right_corrected;

  pthread_mutex_lock( _p_lock_mutex );

  linear_ticks = linear_vel * _tick_rates.ticks_per_meter;
  angular_ticks = angular_vel * _tick_rates.ticks_per_radian;
                                            

  left_corrected = ( linear_ticks - (angular_ticks / 2.0) )
                    * RightInLeftOutCorrection( _base_geometry.wheel_ratio );

  right_corrected = ( linear_ticks + ( angular_ticks / 2.0) )
                    * LeftInRightOutCorrection( _base_geometry.wheel_ratio );

  pthread_mutex_unlock( _p_lock_mutex );
                     
  new_velocity.linear_ticks_sec = RoundInt( (right_corrected + left_corrected) / 2.0 );
  new_velocity.angular_ticks_sec = RoundInt( right_corrected - left_corrected );

#if 0
  std::cout << "Left Cor: " << left_corrected 
            << " Right Cor: " << right_corrected 
            << " Lin Ticks: " << new_velocity.linear_ticks_sec
            << " Ang Ticks: " << new_velocity.angular_ticks_sec
            << std::endl;
#endif

  return new_velocity;
}

BaseGeometry_T BaseModel::GetBaseGeometry() const
{
  return _base_geometry;
}

void BaseModel::SetBaseGeometry( BaseGeometry_T geometry )
{
  pthread_mutex_lock( _p_lock_mutex );

  _base_geometry = geometry;

  pthread_mutex_unlock( _p_lock_mutex );
}

bool BaseModel::GetSetupValid() const
{
  bool valid = true;

  pthread_mutex_lock( _p_lock_mutex );

  if ( GetWheelRadius() <= 0.0 )
  {
    valid = false;
  }
  else if ( GetWheelTicks() <= 0 )
  {
    valid = false;
  }
  else if ( GetWheelBase() <= 0.0 )
  {
    valid = false;
  }
  else if ( GetWheelRatio() <= 0.0 )
  {
    valid = false;
  }

  pthread_mutex_unlock( _p_lock_mutex );

  return valid;
}

bool BaseModel::GetStasisValid() const
{
  bool valid = true;

  pthread_mutex_lock( _p_lock_mutex );

  if ( GetStasisRadius() <= 0.0 )
  {
    valid = false;
  }
  else if ( GetStasisTicks() <= 0 )
  {
    valid = false;
  }

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
 */
void BaseModel::SetWheelRadius(const double wheel_radius)
{
  pthread_mutex_lock( _p_lock_mutex );

  _base_geometry.wheel_radius = wheel_radius;

  _tick_rates = CalculateTickRates( _base_geometry );

  pthread_mutex_unlock( _p_lock_mutex );
}

/** Returns the drive wheel separation in meters.
 */
double BaseModel::GetWheelBase() const
{
  return _base_geometry.wheel_base;
}

/** Sets the drive wheel separation in meters.
 */
void BaseModel::SetWheelBase(const double wheel_base)
{
  pthread_mutex_lock( _p_lock_mutex );

  _base_geometry.wheel_base = wheel_base;

  _tick_rates = CalculateTickRates( _base_geometry );

  pthread_mutex_unlock( _p_lock_mutex );
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
 */
void BaseModel::SetWheelRatio(const double wheel_ratio)
{
  pthread_mutex_lock( _p_lock_mutex );

  _base_geometry.wheel_ratio = wheel_ratio;

  _tick_rates = CalculateTickRates( _base_geometry );

  pthread_mutex_unlock( _p_lock_mutex );
}

/** Return the number of encoder ticks in one full rotation of the drive wheels.
 */
uint16_t BaseModel::GetWheelTicks() const
{
  return _base_geometry.wheel_ticks;
}

/** Sets the number of encoder ticks in one full rotation of the drive wheels.
 */
void BaseModel::SetWheelTicks(const uint16_t wheel_ticks)
{
  pthread_mutex_lock( _p_lock_mutex );

  _base_geometry.wheel_ticks = wheel_ticks;

  _tick_rates = CalculateTickRates( _base_geometry );

  pthread_mutex_unlock( _p_lock_mutex );
}

/** Returns the radius of the stasis wheel if one is used in meters.
 */
double BaseModel::GetStasisRadius() const
{
  return _base_geometry.stasis_radius;
}

/** Sets the radius of the stasis wheel if one is used in meters.
 */
void BaseModel::SetStasisRadius(const double stasis_radius)
{
  pthread_mutex_lock( _p_lock_mutex );

  _base_geometry.stasis_radius = stasis_radius;

  _tick_rates = CalculateTickRates( _base_geometry );

  pthread_mutex_unlock( _p_lock_mutex );
}

/** Returns the number of encoder ticks in one full revolution of the stasis
 *  wheel. 
 *
 *  If negative stasis wheel calculations are disabled.
 */
int16_t BaseModel::GetStasisTicks() const
{
  return _base_geometry.stasis_ticks;
}

/** Sets the number of encoder ticks in one full revolution of the stasis
 *  wheel. 
 *
 *  If negative stasis wheel calculations are disabled.
 */
void BaseModel::SetStasisTicks(const int16_t stasis_ticks)
{
  pthread_mutex_lock( _p_lock_mutex );

  _base_geometry.stasis_ticks = stasis_ticks;

  _tick_rates = CalculateTickRates( _base_geometry );

  pthread_mutex_unlock( _p_lock_mutex );
}

/** Returns the calculated encoder ticks per meter for the drive wheels.
 */
double BaseModel::GetTicksPerMeter() const
{
  return _tick_rates.ticks_per_meter;
}

/** Returns the calculated difference in encoder ticks needed to turn 
 *  one radian.
 */
double BaseModel::GetTicksPerRadian() const
{
  return _tick_rates.ticks_per_radian;
}

/** Returns the calculated encoder ticks per meter for the stasis wheel.
 */
double BaseModel::GetStasisTicksPerMeter() const
{
  return _tick_rates.stasis_ticks_per_meter;
}

/** Converts encoder counts to the changes in X, Y and theta.
 */
BaseDistance_T  BaseModel::CountsToDistance(  differential_drive::EncoderCounts counts, 
                                              BaseGeometry_T geometry, 
                                              const TickRates_T* p_rates ) const
{
  BaseDistance_T  delta_position;
  TickRates_T rates;

  double left_distance;
  double right_distance;
  double stasis_distance;
  double average_distance;

  if ( p_rates == NULL )
  {
    rates = CalculateTickRates( geometry );
    p_rates = &rates;
  }

  left_distance = CalculateDistance(  counts.left_count, p_rates->ticks_per_meter, 
                                      LeftInRightOutCorrection( geometry.wheel_ratio ) );

  right_distance = CalculateDistance( counts.right_count, p_rates->ticks_per_meter, 
                                      RightInLeftOutCorrection( geometry.wheel_ratio ) );

  if ( geometry.stasis_ticks > 0 )
  {
    stasis_distance = CalculateDistance( counts.stasis_count, p_rates->stasis_ticks_per_meter );
  }
  else
  {
    stasis_distance = 0.0;
  }

  average_distance = ( left_distance + right_distance ) / 2.0;

  delta_position.theta = CalculateDeltaTheta( left_distance, right_distance, geometry.wheel_base );
  delta_position.linear = average_distance;
  delta_position.stasis = stasis_distance;

  return delta_position;
}

/** Converts base distances into velocities given the elapsed time.
 */
BaseVelocities_T  BaseModel::DistanceToVelocity( BaseDistance_T distance, double seconds ) const
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
TickRates_T BaseModel::CalculateTickRates( BaseGeometry_T geometry ) const
{
  TickRates_T rates;

  rates.ticks_per_meter = CalculateTicksPerMeter( geometry.wheel_radius, 
                                                  geometry.wheel_ticks );
  rates.ticks_per_radian = CalculateTicksPerRadian( geometry.wheel_base, 
                                                    rates.ticks_per_meter );

  if ( geometry.stasis_ticks > 0)
  {
    rates.stasis_ticks_per_meter = CalculateTicksPerMeter( geometry.stasis_radius, 
                                                              geometry.stasis_ticks );
  }
  else
  {
    rates.stasis_ticks_per_meter = 0.0;
  }

  return rates;
}

/** Calculates the ticks required to travel 1 meter.
 *  @param wheel_radius The wheel radius in meters.
 *  @param wheel_ticks  The number of encoder ticks in one full revolution.
 *  @returns The numbers of encoder ticks required to travel 1 meter.
 */
double BaseModel::CalculateTicksPerMeter(const double wheel_radius, const uint16_t wheel_ticks) const
{
  double wheel_circumference;

  wheel_circumference = wheel_radius * 2.0 * M_PI;

  return (double)wheel_ticks / wheel_circumference;
}

/** Calculates the difference in ticks between the drive wheels to turn one radian.
 *  @param wheel_base       The separation between the drive wheels in meters.
 *  @param ticks_per_meter  The number of encoder ticks for the drive wheels to 
 *                          travel 1 meter.
 *  @returns The difference of encoder ticks needed to turn one radian.
 */
double BaseModel::CalculateTicksPerRadian(const double wheel_base, const double ticks_per_meter) const
{
  return wheel_base * ticks_per_meter;
}

/** Returns the change in angle based on difference in distance traveled by each wheel.
 */
double BaseModel::CalculateDeltaTheta(  const double left_distance,
                                        const double right_distance,
                                        const double wheel_base ) const
{
  return ( right_distance - left_distance ) / wheel_base;
}

/** Returns the linear distance in meters for a given number of encoder ticks.
 */
double BaseModel::CalculateDistance(  const int16_t ticks, 
                                      const double ticks_per_meter,
                                      const double correction_factor ) const
{
  return ((double)ticks / ticks_per_meter) * correction_factor;
}

/** Distance correction factor for the given wheel ratio.
 *
 *  Calibrated dead reckoning, as per "A Quick & Easy Guide to Calibrating Your 
 *  Robot Using UMBMark" at: http://www.dprg.org/articles/2009-02a/
 *  where wheel_ratio = Ed
 */
double BaseModel::LeftInRightOutCorrection( const double wheel_ratio ) const
{
  return 2.0 / (( 1.0 / wheel_ratio ) + 1.0);
}

/** Distance correction factor for the given wheel ratio.
 *  
 *  Calibrated dead reckoning, as per "A Quick & Easy Guide to Calibrating Your 
 *  Robot Using UMBMark" at: http://www.dprg.org/articles/2009-02a/
 *  where wheel_ratio = Ed
 */
double BaseModel::RightInLeftOutCorrection( double wheel_ratio ) const
{
  return 2.0 / (1.0 + wheel_ratio);
}

/** Returns the linear velocity in meters/sec for a given distance and time.
 */
double BaseModel::CalculateVelocity(  const double distance,
                                      const double seconds ) const
{
  return distance / seconds;
}
}
