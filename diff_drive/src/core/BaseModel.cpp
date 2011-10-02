#include <cmath>

#include "BaseModel.hpp"

// Calibrated dead reckoning, as per "A Quick & Easy Guide to Calibrating Your 
// Robot Using UMBMark" at: http://www.dprg.org/articles/2009-02a/
// where _wheel_ratio = Ed
#define LEFT_IN_RIGHT_OUT_CORRECTION ( 2.0 / (( 1.0 / _wheel_ratio ) + 1.0) )
#define RIGHT_IN_LEFT_OUT_CORRECTION ( 2.0 / (1.0 + _wheel_ratio) )
 
namespace diff_drive_core
{
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
          : _wheel_radius(wheel_radius),
            _wheel_ticks(wheel_ticks),
            _wheel_base(wheel_base),
            _wheel_ratio(wheel_ratio),
            _stasis_radius(stasis_radius),
            _stasis_ticks(stasis_ticks),
            _delta_x(0.0),
            _delta_y(0.0),
            _delta_theta(0.0),
            _delta_stasis(0.0),
            _linear_velocity(0.0),
            _angular_velocity(0.0),
            _stasis_velocity(0.0)
{
  CalculateProperties();
}

/** Updates the current model state with new encoder counts.
 *  @param  new_counts  Contains the newly received encoder counts.
 */
void BaseModel::NewEncoderCounts(const diff_drive::EncoderCounts new_counts)
{
  double left_distance;
  double right_distance;
  double average_distance;
  double seconds;

  left_distance = CalculateDistance(  new_counts.left_count, _ticks_per_meter, 
                                      LEFT_IN_RIGHT_OUT_CORRECTION );
  right_distance = CalculateDistance( new_counts.right_count, _ticks_per_meter, 
                                      RIGHT_IN_LEFT_OUT_CORRECTION );
  average_distance = ( left_distance + right_distance ) / 2.0;

  _delta_theta = CalculateDeltaTheta( left_distance, right_distance, _wheel_base );
  _delta_x = CalculateDeltaX( average_distance, _delta_theta );
  _delta_y = CalculateDeltaY( average_distance, _delta_theta );

  // convert milliseconds to seconds
  seconds = new_counts.dt_ms / 1000.0;

  _linear_velocity = CalculateVelocity( average_distance, seconds );
  _angular_velocity = CalculateVelocity( _delta_theta, seconds );

  if ( _stasis_ticks > 0 )
  {
    _delta_stasis = CalculateDistance( new_counts.stasis_count, _stasis_ticks_per_meter );
    _stasis_velocity = CalculateVelocity( _delta_stasis, seconds );
  }
  else
  {
    _delta_stasis = 0.0;
    _stasis_velocity = 0.0;
  }
}

/** Accepts an desired linear and angular velocity and returns the velocities
 *  in ticks/sec. This function does not alter the current model state.
 */
diff_drive::TickVelocity BaseModel::VelocityToTicks(double linear_vel, double angular_vel) const
{
  diff_drive::TickVelocity new_velocity;
  double linear_ticks;
  double angular_ticks;
  double left_corrected;
  double right_corrected;

  linear_ticks = linear_vel * _ticks_per_meter;
  angular_ticks = angular_vel * _ticks_per_radian;
                                            

  left_corrected = ( linear_ticks - (angular_ticks / 2.0) )
                    * RIGHT_IN_LEFT_OUT_CORRECTION;

  right_corrected = ( linear_ticks + ( angular_ticks / 2.0) )
                     * LEFT_IN_RIGHT_OUT_CORRECTION;
                     
  new_velocity.linear_ticks_sec = (int)( (right_corrected + left_corrected) / 2.0 );
  new_velocity.angular_ticks_sec = (int)( right_corrected - left_corrected );

#if 0
  std::cout << "Left Cor: " << left_corrected 
            << " Right Cor: " << right_corrected 
            << " Lin Ticks: " << new_velocity.linear_ticks_sec
            << " Ang Ticks: " << new_velocity.angular_ticks_sec
            << std::endl;
#endif

  return new_velocity;
}

/** Returns the current estimate of the change in X position in meters
 *  based on the last received encoder counts.
 *
 *  Please note that integration is _not_ performed, this value represents 
 *  only the distance traveled during the latest encoder counts.
 */
double BaseModel::GetDeltaX() const
{
  return _delta_x;
}

/** Returns the current estimate of the change in Y position in meters
 *  based on the last received encoder counts.
 *
 *  Please note that integration is _not_ performed, this value represents 
 *  only the distance traveled during the latest encoder counts.
 */
double BaseModel::GetDeltaY() const
{
  return _delta_y;
}

/** Returns the current estimate of the change in heading angle in radians 
 *  based on the last received encoder counts.
 *
 *  Please note that integration is _not_ performed, this value represents 
 *  only the heading change during the latest encoder counts.
 */
double BaseModel::GetDeltaTheta() const
{
  return _delta_theta;
}

/** Returns the current estimate of the change in Stasis position in meters
 *  based on the last received encoder counts.
 *
 *  Please note that integration is _not_ performed, this value represents 
 *  only the distance traveled during the latest encoder counts.
 */
double BaseModel::GetDeltaStasis() const
{
  return _delta_stasis;
}

/** Returns the linear velocity calculated from the latest set of encoder counts,
 *  in meters/sec.
 */
double BaseModel::GetLinearVelocity() const
{
  return _linear_velocity;
}

/** Returns the angular velocity calculated from the latest set of encoder counts,
 *  in radians/sec.
 */
double BaseModel::GetAngularVelocity() const
{
  return _angular_velocity;
}

/** Returns the linear velocity of the stasis wheel (if enabled) calculated from 
 *  the latest set of encoder counts, in meters/sec.
 */
double BaseModel::GetStasisVelocity() const
{
  return _stasis_velocity;
}

/** Returns the drive wheel radius in meters.
 */
double BaseModel::GetWheelRadius() const
{
  return _wheel_radius;
}

/** Sets the drive wheel radius in meters.
 */
void BaseModel::SetWheelRadius(const double wheel_radius)
{
  _wheel_radius = wheel_radius;

  CalculateProperties();
}

/** Returns the drive wheel separation in meters.
 */
double BaseModel::GetWheelBase() const
{
  return _wheel_base;
}

/** Sets the drive wheel separation in meters.
 */
void BaseModel::SetWheelBase(const double wheel_base)
{
  _wheel_base = wheel_base;

  CalculateProperties();
}

/** Returns the drive wheel radius ratio (ie. left radius/ right radius).
 *  
 *  Ideally this value should be 1.0 but it rarely is.
 */
double BaseModel::GetWheelRatio() const
{
  return _wheel_ratio;
}

/** Sets the drive wheel radius ratio (ie. left radius/ right radius).
 *  
 *  Ideally this value should be 1.0 but it rarely is.
 */
void BaseModel::SetWheelRatio(const double wheel_ratio)
{
  _wheel_ratio = wheel_ratio;

  CalculateProperties();
}

/** Return the number of encoder ticks in one full rotation of the drive wheels.
 */
uint16_t BaseModel::GetWheelTicks() const
{
  return _wheel_ticks;
}

/** Sets the number of encoder ticks in one full rotation of the drive wheels.
 */
void BaseModel::SetWheelTicks(const uint16_t wheel_ticks)
{
  _wheel_ticks = wheel_ticks;

  CalculateProperties();
}

/** Returns the radius of the stasis wheel if one is used in meters.
 */
double BaseModel::GetStasisRadius() const
{
  return _stasis_radius;
}

/** Sets the radius of the stasis wheel if one is used in meters.
 */
void BaseModel::SetStasisRadius(const double stasis_radius)
{
  _stasis_radius = stasis_radius;

  CalculateProperties();
}

/** Returns the number of encoder ticks in one full revolution of the stasis
 *  wheel. 
 *
 *  If negative stasis wheel calculations are disabled.
 */
int16_t BaseModel::GetStasisTicks() const
{
  return _stasis_ticks;
}

/** Sets the number of encoder ticks in one full revolution of the stasis
 *  wheel. 
 *
 *  If negative stasis wheel calculations are disabled.
 */
void BaseModel::SetStasisTicks(const int16_t stasis_ticks)
{
  _stasis_ticks = stasis_ticks;

  CalculateProperties();
}

/** Returns the calculated encoder ticks per meter for the drive wheels.
 */
double BaseModel::GetTicksPerMeter() const
{
  return _ticks_per_meter;
}

/** Returns the calculated difference in encoder ticks needed to turn 
 *  one radian.
 */
double BaseModel::GetTicksPerRadian() const
{
  return _ticks_per_radian;
}

/** Returns the calculated encoder ticks per meter for the stasis wheel.
 */
double BaseModel::GetStasisTicksPerMeter() const
{
  return _stasis_ticks_per_meter;
}

/** Calculates drive wheel ticks/meter, ticks/radian, and stasis wheel 
 *  ticks/meter.
 */
void BaseModel::CalculateProperties()
{
  _ticks_per_meter = CalculateTicksPerMeter(_wheel_radius, _wheel_ticks);
  _ticks_per_radian = CalculateTicksPerRadian(_wheel_base, _ticks_per_meter);

  if (_stasis_ticks > 0)
  {
    _stasis_ticks_per_meter = CalculateTicksPerMeter(_stasis_radius, _stasis_ticks);
  }
  else
  {
    _stasis_ticks_per_meter = 0.0;
  }
}

/** Calculates the ticks required to travel 1 meter.
 *  @param wheel_radius The wheel radius in meters.
 *  @param wheel_ticks  The number of encoder ticks in one full revolution.
 *  @returns The numbers of encoder ticks required to travel 1 meter.
 */
double BaseModel::CalculateTicksPerMeter(const double wheel_radius, const uint16_t wheel_ticks) const
{
  double wheel_circumference;

  wheel_circumference = _wheel_radius * 2.0 * M_PI;

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

/** Returns the change in forward distance based on distance traveled by each wheel.
 *
 *  Note: As per Tom Brown of UI Urbana-Champaign delta_theta / 2.0 is used for distance
 *  estimations.
 *
 *  @param average_distance Sum of the left and right drive wheel distances / 2.
 *  @param delta_theta      Estimated change in angle during this traversal.
 *  @returns                The estimated change in position along the X axis.
 */
double BaseModel::CalculateDeltaX(  const double average_distance,
                                    const double delta_theta ) const
{
  return average_distance * cos( delta_theta / 2.0 );
}

/** Returns the change in sideways distance based on distance traveled by each wheel.
 *
 *  Note: As per Tom Brown of UI Urbana-Champaign delta_theta / 2.0 is used for distance
 *  estimations.
 *
 *  @param average_distance Sum of the left and right drive wheel distances / 2.
 *  @param delta_theta      Estimated change in angle during this traversal.
 *  @returns                The estimated change in position along the Y axis.
 */
double BaseModel::CalculateDeltaY(  const double average_distance,
                                    const double delta_theta ) const
{
  return average_distance * sin( delta_theta / 2.0 );
}

/** Returns the linear distance in meters for a given number of encoder ticks.
 */
double BaseModel::CalculateDistance(  const int16_t ticks, 
                                      const double ticks_per_meter,
                                      const double correction_factor ) const
{
  return ((double)ticks / ticks_per_meter) * correction_factor;
}

/** Returns the linear velocity in meters/sec for a given distance and time.
 */
double BaseModel::CalculateVelocity(  const double distance,
                                      const double seconds ) const
{
  return distance / seconds;
}
}
