#include <cmath>

#include "BaseModel.hpp"

namespace diff_drive_core
{
static int RoundToInt( double r ) 
{
  return (r > 0.0) ? (r + 0.5) : (r - 0.5); 
}

/** @class BaseModel BaseModel.hpp
 *  This class holds the parameters and caclulations to convert between SI units 
 *  and encoder ticks, for a differentially driven robot with 2 drive wheel and 
 *  an optional stasis wheel.
 *
 *  This class is estimating changes in position and angle based on research from 
 *  Tom Brown of the University of Illinois at Urbana-Champaign as detailed here:
 *  @ref //http://rossum.sourceforge.net/papers/DiffSteer/#d7
 *
 *  Positive numbers for drive wheel radius, wheel base, wheel ratio, and wheel 
 *  encoder ticks indicate a valid geometry. Stasis wheel parameters can be 0 or
 *  negative but this will disable stasis wheel processing.
 *  
 *  This class converts encoder readings based on the current readings only and 
 *  does not track global coordinates or integrate readings.
 *
 */

/** Default constructor.
 *
 *  All arguments have defaults, most of them default to an invalid geometry.
 *
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

  _tick_rates = calculateTickRates( _base_geometry );
  _corrections = calculateCorrections( _base_geometry.wheel_ratio );

  pthread_mutex_unlock( _p_lock_mutex );
}

/** Constructor taking a BaseGeometry_T for initial object configuration.
 *
 *  @param  new_geometry data structure to populate internal object parameters.
 */
BaseModel::BaseModel( BaseGeometry_T new_geometry )
{
  _base_geometry = new_geometry;

  _p_lock_mutex = new pthread_mutex_t;
  pthread_mutex_init( _p_lock_mutex, NULL );

  pthread_mutex_lock( _p_lock_mutex );

  _tick_rates = calculateTickRates( _base_geometry );
  _corrections = calculateCorrections( _base_geometry.wheel_ratio );

  pthread_mutex_unlock( _p_lock_mutex );
}

/** Calculates the distance and velocities associated with @a new_counts.
 *  
 *  @param[out] p_delta_position  The change in position calculated from @p new_counts will be stored here.
 *  @param[out] p_velocity        The current velocity calculated from @p new_counts will be stored here.
 *  @param[in]  new_counts        Contains the newly received encoder counts.
 */
void BaseModel::convertCounts(  BaseDistance_T* p_delta_position,
                                BaseVelocities_T* p_velocity,
                                diff_drive_calibrated::EncoderCounts const & new_counts ) const
{
  pthread_mutex_lock( _p_lock_mutex );

  *p_delta_position = countsToDistance( new_counts, _base_geometry, _tick_rates, _corrections );

  *p_velocity = distanceToVelocity( *p_delta_position, new_counts.dt_ms );

  pthread_mutex_unlock( _p_lock_mutex );
}

/** Accepts linear and angular velocity n SI units and returns the velocities
 *  in ticks/sec. This function does not alter the current model state.
 *
 *  @returns populated TickVelocity data structure.
 */
diff_drive_calibrated::TickVelocity BaseModel::convertVelocity(  double linear_vel, 
                                                              double angular_vel ) const
{
  diff_drive_calibrated::TickVelocity new_velocity;

  pthread_mutex_lock( _p_lock_mutex );

  new_velocity = velocityToTicks( linear_vel, angular_vel, _base_geometry, _tick_rates, _corrections );

  pthread_mutex_unlock( _p_lock_mutex );

  return new_velocity;
}

/** Access function for the internal base geometry.
 *
 *  @returns the current internal base geometry parameters.
 */
BaseGeometry_T BaseModel::getBaseGeometry() const
{
  return _base_geometry;
}

/** Updates the internal base geometry parameters if all parameters are valid. 
 *
 *  @returns true if the geometry is valid and the internal data members have been 
 *  updated.
 */
bool BaseModel::setBaseGeometry( BaseGeometry_T const & geometry )
{
  if ( checkGeometryValid(geometry) )
  {
    pthread_mutex_lock( _p_lock_mutex );

    _base_geometry = geometry;
    _tick_rates = calculateTickRates( _base_geometry );
    _corrections = calculateCorrections( _base_geometry.wheel_ratio );

    pthread_mutex_unlock( _p_lock_mutex );

    return true;
  }

  return false;
}

/** Checks @p geometry to see if all data members describe a valid base geometry.
 *  
 *  Positive numbers for drive wheel radius, wheel base, wheel ratio, and wheel 
 *  encoder ticks indicate a valid geometry. Stasis wheel parameters can be 0 or
 *  negative but this will disable stasis wheel processing.
 *  
 *  @param geometry Data structure to check for validity.
 *
 *  @returns true if a valid geometry is described.
 */
bool BaseModel::checkGeometryValid( BaseGeometry_T const & geometry ) const
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

/** Checks @p geometry to see if the stasis wheel parameters describe a valid 
 *  stasis wheel geometry.
 *
 *  Postive numbers for stasis wheel radius, and shaft encoder ticks indicate 
 *  a valid stasis wheel geometry.
 *
 *  @returns true if a valid stasis wheel geometry is described.
 */
bool BaseModel::checkGeometryStasisValid( BaseGeometry_T const & geometry ) const 
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

/** Checks the internal geometry parameters.
 *
 *  @returns true if the internal geometry defines a valid configuration.
 */
bool BaseModel::getSetupValid() const
{
  bool valid;

  pthread_mutex_lock( _p_lock_mutex );

  valid = checkGeometryValid( _base_geometry );

  pthread_mutex_unlock( _p_lock_mutex );

  return valid;
}

/** Checks the internal configuration of the stasis wheel. 
 *
 *  @returns true if the stasis wheel parameters define a valid configuration.
 */
bool BaseModel::getStasisValid() const
{
  bool valid;

  pthread_mutex_lock( _p_lock_mutex );

  valid = checkGeometryStasisValid( _base_geometry );

  pthread_mutex_unlock( _p_lock_mutex );

  return valid;
}

/** Access function for the internal drive wheel radius.
 *
 *  @returns the drive wheel radius in meters.
 */
double BaseModel::getWheelRadius() const
{
  return _base_geometry.wheel_radius;
}

/** Sets the drive wheel radius in meters.
 *
 *  @returns true if @p wheel_radius is positive and the internal data member was updated.
 */
bool BaseModel::setWheelRadius( double wheel_radius)
{
  if ( wheel_radius > 0.0 )
  {
    pthread_mutex_lock( _p_lock_mutex );

    _base_geometry.wheel_radius = wheel_radius;

    _tick_rates = calculateTickRates( _base_geometry );

    pthread_mutex_unlock( _p_lock_mutex );

    return true;
  }

  return false;
}

/** Access function for the internal drive wheel base.
 *
 *  @returns the drive wheel separation in meters.
 */
double BaseModel::getWheelBase() const
{
  return _base_geometry.wheel_base;
}

/** Sets the drive wheel separation in meters.
 *
 *  @returns true if @p wheel_base is positive and the internal data member was updated.
 */
bool BaseModel::setWheelBase( double wheel_base)
{
  if ( wheel_base > 0.0 )
  {
    pthread_mutex_lock( _p_lock_mutex );

    _base_geometry.wheel_base = wheel_base;

    _tick_rates = calculateTickRates( _base_geometry );

    pthread_mutex_unlock( _p_lock_mutex );

    return true;
  }

  return false;
}

/** Access function for the internal drive wheel ratio. 
 *
 *  The drive wheel ratio describes the difference in wheel radiuses 
 *  (left radius/ right radius).
 *  Ideally this value should be 1.0 but it rarely is.
 *
 *  @returns the drive wheel radius ratio.
 *  
 */
double BaseModel::getWheelRatio() const
{
  return _base_geometry.wheel_ratio;
}

/** Sets the drive wheel radius ratio.
 *  
 *  The drive wheel ratio describes the difference in wheel radiuses 
 *  (left radius/ right radius).
 *  Ideally this value should be 1.0 but it rarely is.
 *
 *  @returns true if @p wheel_ratio is positive and the internal data member was updated.
 */
bool BaseModel::setWheelRatio( double wheel_ratio)
{
  if ( wheel_ratio > 0.0 )
  {
    pthread_mutex_lock( _p_lock_mutex );

    _base_geometry.wheel_ratio = wheel_ratio;

    _tick_rates = calculateTickRates( _base_geometry );
    _corrections = calculateCorrections( _base_geometry.wheel_ratio );

    pthread_mutex_unlock( _p_lock_mutex );

    return true;
  }

  return false;
}

/** Access function for the internal drive wheel encoder count.
 *
 *  @returns the number of encoder ticks in one full rotation of the drive wheels.
 */
uint32_t BaseModel::getWheelTicks() const
{
  return _base_geometry.wheel_ticks;
}

/** Sets the number of encoder ticks in one full rotation of the drive wheels.
 *
 *  @returns true if @p wheel_ticks is positive and the internal data member was updated.
 */
bool BaseModel::setWheelTicks( uint32_t wheel_ticks)
{
  if ( wheel_ticks > 0 )
  {
    pthread_mutex_lock( _p_lock_mutex );

    _base_geometry.wheel_ticks = wheel_ticks;

    _tick_rates = calculateTickRates( _base_geometry );

    pthread_mutex_unlock( _p_lock_mutex );

    return true;
  }

  return false;
}

/** Access function for the internal stasis wheel radius.
 *  
 *  If the value is <= 0 the stasis wheel is disabled.
 *
 *  Returns the radius of the stasis wheel in meters.
 */
double BaseModel::getStasisRadius() const
{
  return _base_geometry.stasis_radius;
}

/** Sets the radius of the stasis wheel in meters.
 *
 *  If @p stasis_radius is <= 0 stasis wheel calculations are disabled.
 *
 *  @returns  true if @p stasis_radius is positive and the internal data member was updated.
 *            Otherwise a value of 0.0 is stored and the stasis wheel is disabled.
 */
bool BaseModel::setStasisRadius( double stasis_radius)
{
  if ( stasis_radius > 0.0 )
  {
    pthread_mutex_lock( _p_lock_mutex );

    _base_geometry.stasis_radius = stasis_radius;

    _tick_rates = calculateTickRates( _base_geometry );

    pthread_mutex_unlock( _p_lock_mutex );

    return true;
  }
  else
  {
    pthread_mutex_lock( _p_lock_mutex );

    _base_geometry.stasis_radius = 0.0;

    _tick_rates = calculateTickRates( _base_geometry );

    pthread_mutex_unlock( _p_lock_mutex );
  }

  return false;
}

/** Access function for the internal stasis wheel encoder counts.
 *
 *  If is value <= 0 stasis wheel calculations are disabled.
 *
 *  @returns the number of encoder ticks in one full revolution of the stasis
 *  wheel. 
 */
int32_t BaseModel::getStasisTicks() const
{
  return _base_geometry.stasis_ticks;
}

/** Sets the number of encoder ticks in one full revolution of the stasis
 *  wheel. 
 *
 *  If @p stasis_ticks is <= 0 stasis wheel calculations are disabled.
 *
 *  @returns  true if @p stasis_radius is positive and the internal data member was updated.
 *            Otherwise a value of -1 is stored and the stasis wheel is disabled.
 */
bool BaseModel::setStasisTicks( int32_t stasis_ticks)
{
  if ( stasis_ticks > 0 )
  {
    pthread_mutex_lock( _p_lock_mutex );

    _base_geometry.stasis_ticks = stasis_ticks;

    _tick_rates = calculateTickRates( _base_geometry );

    pthread_mutex_unlock( _p_lock_mutex );

    return true;
  }
  else
  {
    pthread_mutex_lock( _p_lock_mutex );

    _base_geometry.stasis_ticks = -1;

    _tick_rates = calculateTickRates( _base_geometry );

    pthread_mutex_unlock( _p_lock_mutex );
  }

  return false;
}

/** @returns the calculated encoder ticks per meter for the drive wheels.
 */
double BaseModel::getTicksPerMeter() const
{
  return _tick_rates.ticks_per_meter;
}

/** @returns the calculated meters per encoder tick for the drive wheels.
 */
double BaseModel::getMetersPerTick() const
{
  return _tick_rates.meters_per_tick;
}

/** @returns the calculated difference in encoder ticks needed to turn 
 *  one radian.
 */
double BaseModel::getTicksPerRadian() const
{
  return _tick_rates.ticks_per_radian;
}

/** @returns the calculated radians per difference in encoder ticks.
 */
double BaseModel::getRadiansPerTick() const
{
  return _tick_rates.radians_per_tick;
}

/** @returns the calculated encoder ticks per meter for the stasis wheel.
 */
double BaseModel::getStasisTicksPerMeter() const
{
  return _tick_rates.stasis_ticks_per_meter;
}

/** @returns the calculated meters per encoder tick for the stasis wheel.
 */
double BaseModel::getMetersPerStasisTick() const
{
  return _tick_rates.meters_per_stasis_tick;
}

/** @returns the calculated calculated correction factor for the distance
 *  traveled by the left wheel and the distance required by the right wheel
 *  based on the wheel ratio.
 */
double BaseModel::getLeftInRightOutCorrection() const
{
  return _corrections.left_in_right_out;
}

/** @returns the calculated calculated correction factor for the distance
 *  traveled by the right wheel and the distance required by the left wheel
 *  based on the wheel ratio.
 */
double BaseModel::getRightInLeftOutCorrection() const
{
  return _corrections.right_in_left_out;
}

/** @returns the calculated calculated correction factor for the angle
 *  traversed based on direction and wheel ratio.
 */
double BaseModel::getAnglePosInNegOutCorrection() const
{
  return _corrections.angle_pos_in_neg_out;
}

/** @returns the calculated calculated correction factor for the angle
 *  traversed based on direction and wheel ratio.
 */
double BaseModel::getAngleNegInPosOutCorrection() const
{
  return _corrections.angle_neg_in_pos_out;
}

/** Accepts linear and angular velocity n SI units and returns the velocities
 *  in ticks/sec. This function does not alter the current model state.
 *
 *  @p base_geometry and @p tick_rates are pre-calculated for speed and should 
 *  be calculated from the current values in @p base_geometry if the results 
 *  are to be meaningful.
 *
 *  @param linear_vel Linear velocity in meters/second.
 *  @param angular_vel Angular velocity in radians/second.
 *  @param[in] base_geometry Base geometry used for the calculations.
 *  @param[in] tick_rates Pre-calculated tick rates.
 *  @param[in] corrections Pre-calculated tick corrections.
 *
 *  @returns populated TickVelocity data structure.
 */
diff_drive_calibrated::TickVelocity BaseModel::velocityToTicks(  const double linear_vel, 
                                                              const double angular_vel,
                                                              BaseGeometry_T const & base_geometry,
                                                              TickRates_T const & tick_rates,
                                                              BaseCorrections_T const & corrections ) const
{
  diff_drive_calibrated::TickVelocity new_velocity;
  double linear_ticks;
  double angular_ticks;
  double angle_corrected;
  double left_corrected;
  double right_corrected;

  linear_ticks = linear_vel * tick_rates.ticks_per_meter;
  angular_ticks = angular_vel * tick_rates.ticks_per_radian;

  if ( (corrections.angle_neg_in_pos_out == 1.0) || (angular_ticks == 0) )
  {
    angle_corrected = angular_ticks;
  }
  else if ( angular_ticks > 0 )
  {
    angle_corrected = angular_ticks * corrections.angle_neg_in_pos_out;
  }
  else
  {
    angle_corrected = angular_ticks * corrections.angle_pos_in_neg_out;
  }

  left_corrected = ( linear_ticks - (angle_corrected * 0.5) )
                    * corrections.right_in_left_out;

  right_corrected = ( linear_ticks + (angle_corrected * 0.5) )
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
 *
 *  
 */
BaseDistance_T  BaseModel::countsToDistance(  diff_drive_calibrated::EncoderCounts const & counts, 
                                              BaseGeometry_T const & geometry, 
                                              TickRates_T const & rates,
                                              BaseCorrections_T const & corrections ) const
{
  BaseDistance_T  delta_position;

  double left_distance;
  double right_distance;
  double stasis_distance;
  double average_distance;

  left_distance = calculateDistance(  counts.left_count, rates.meters_per_tick, 
                                      corrections.left_in_right_out );

  right_distance = calculateDistance( counts.right_count, rates.meters_per_tick, 
                                      corrections.right_in_left_out );

  if ( geometry.stasis_ticks > 0 )
  {
    stasis_distance = calculateDistance( counts.stasis_count, rates.meters_per_stasis_tick );
  }
  else
  {
    stasis_distance = 0.0;
  }

  average_distance = ( left_distance + right_distance ) * 0.5;

  delta_position.theta = calculateDeltaTheta( left_distance, right_distance, geometry.wheel_base,
                                              corrections.angle_pos_in_neg_out,
                                              corrections.angle_neg_in_pos_out );
  delta_position.linear = average_distance;
  delta_position.stasis = stasis_distance;

  return delta_position;
}

/** Converts base distances into velocities given the elapsed time.
 */
BaseVelocities_T  BaseModel::distanceToVelocity( BaseDistance_T const & distance, double milli_seconds ) const
{
  BaseVelocities_T velocities;

  if ( milli_seconds != 0.0 )
  {
  velocities.linear = calculateVelocity( distance.linear, milli_seconds );
  velocities.angular = calculateVelocity( distance.theta, milli_seconds );
  velocities.stasis = calculateVelocity( distance.stasis, milli_seconds );
  }
  else
  {
  velocities.linear = 0.0;
  velocities.angular = 0.0;
  velocities.stasis = 0.0;
  }

  return velocities;
}

/** Calculates drive wheel ticks/meter, ticks/radian, and stasis wheel 
 *  ticks/meter.
 */
TickRates_T BaseModel::calculateTickRates( BaseGeometry_T const & geometry ) const
{
  TickRates_T rates;

  rates.ticks_per_meter = calculateTicksPerMeter( geometry.wheel_radius, 
                                                  geometry.wheel_ticks );

  rates.meters_per_tick = calculateMetersPerTick( geometry.wheel_radius, 
                                                  geometry.wheel_ticks );

  rates.ticks_per_radian = calculateTicksPerRadian( geometry.wheel_base, 
                                                    rates.ticks_per_meter );

  rates.radians_per_tick = calculateRadiansPerTick( geometry.wheel_base, 
                                                    rates.meters_per_tick );

  if ( (geometry.stasis_ticks > 0) && (geometry.stasis_radius > 0.0) )
  {
    rates.stasis_ticks_per_meter = calculateTicksPerMeter( geometry.stasis_radius, 
                                                           geometry.stasis_ticks );

    rates.meters_per_stasis_tick = calculateMetersPerTick( geometry.stasis_radius, 
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
double BaseModel::calculateTicksPerMeter( double wheel_radius, uint32_t wheel_ticks) const
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
double BaseModel::calculateMetersPerTick( double wheel_radius, uint32_t wheel_ticks) const
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
double BaseModel::calculateTicksPerRadian(  double wheel_base, double ticks_per_meter ) const
{
  return ticks_per_meter * wheel_base;
}

/** Calculates the amount turned for a difference of 1 tick between the two wheels.
 *  @param wheel_base       The separation between the drive wheels in meters.
 *  @param meters_per_tick  The number of meters traveled in 1 tick of the encoders.
 *  @returns The amount turned for a difference of 1 tick between the two wheels.
 */
double BaseModel::calculateRadiansPerTick(  double wheel_base, double meters_per_tick ) const
{
  return meters_per_tick / wheel_base;
}

/** Returns the change in angle based on difference in distance traveled by each wheel.
 */
double BaseModel::calculateDeltaTheta(  double left_distance,
                                        double right_distance,
                                        double wheel_base,
                                        double correction_factor,
                                        double inv_correction_factor ) const
{
  double cal_theta;
  double ideal_theta = ( (right_distance - left_distance) / wheel_base );

  if ( (correction_factor == 1.0) || (ideal_theta == 0.0) )
  {
    cal_theta = ideal_theta;
  }
  else if ( ideal_theta > 0 )
  {
    cal_theta = ideal_theta * correction_factor;
  }
  else
  {
    if ( inv_correction_factor == 1.0 )
    {
      cal_theta = ideal_theta / correction_factor;
    }
    else
    {
      cal_theta = ideal_theta * inv_correction_factor;
    }
  }

  return cal_theta;
}

/** Returns the linear distance in meters for a given number of encoder ticks.
 */
double BaseModel::calculateDistance(  int32_t ticks, 
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
BaseCorrections_T BaseModel::calculateCorrections( double wheel_ratio ) const
{
  BaseCorrections_T corrections;

  // Apply 1/2 of the ratio to each wheel
  corrections.left_in_right_out = 2.0 / (( 1.0 / wheel_ratio ) + 1.0);
  corrections.right_in_left_out = 2.0 / (1.0 + wheel_ratio);

  // Apply 1/2 the ratio to each direction
  corrections.angle_pos_in_neg_out = corrections.right_in_left_out;
  corrections.angle_neg_in_pos_out = corrections.left_in_right_out;

  return corrections;
}

/** Returns the linear velocity in meters/sec for a given distance and time.
 */
double BaseModel::calculateVelocity(  double distance, double milli_seconds ) const
{
  // distance / (millis / 1000) => (distance * 1000) / millis
  return ( distance * 1000.0 ) / milli_seconds;
}
}
