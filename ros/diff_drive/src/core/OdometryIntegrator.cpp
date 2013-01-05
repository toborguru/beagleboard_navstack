/** @file
 *
 *  Encoder Counts Reader Class File
 *
 *  @details
 *  This class uses @BaseModel class physical attributes to compute distances, 
 *  velocities, and motion state from incoming encoder counts. This class 
 *  optionally supports a stasis wheel, if properly defined in the @BaseModel 
 *  class.
 *  
 *  @author Sawyer Larkin (SJL toborguru)
 *
 *  @copyright GNU Public License Version 2.
 */

#include <cmath>

#include "tf/transform_datatypes.h"

#include "OdometryIntegrator.hpp"

namespace diff_drive_core
{
/** This covariance matrix is used by default with estimated values.
 */
static const double OdometryCovariance[36] = 
{
  1e-9, 1e-9, 0.0,  0.0,  0.0,  1e-9,
  1e-9, 1e-9, 0.0,  0.0,  0.0,  1e-9,
  0.0,  0.0,  1e6,  0.0,  0.0,  0.0,
  0.0,  0.0,  0.0,  1e6,  0.0,  0.0,
  0.0,  0.0,  0.0,  0.0,  1e6,  0.0,
  1e-9, 1e-9, 0.0,  0.0,  0.0,  1e-9
};

/** If we have reason to believe there was significant slip (ie stasis 
 *  wheel does not match the drive wheel) then we reduce our certainties.
 */
static const double OdometryCovarianceLow[36] = 
{
  1e-1, 1e-1, 0.0,  0.0,  0.0,  1e-1,
  1e-1, 1e-1, 0.0,  0.0,  0.0,  1e-1,
  0.0,  0.0,  1e6,  0.0,  0.0,  0.0,
  0.0,  0.0,  0.0,  1e6,  0.0,  0.0,
  0.0,  0.0,  0.0,  0.0,  1e6,  0.0,
  1e-1, 1e-1, 0.0,  0.0,  0.0,  1e-1
};

/** Default constructor.
 */
OdometryIntegrator::OdometryIntegrator()
                   : _average_index(0),
                     _linear_average_total(0.0),
                     _stasis_average_total(0.0),
                     _p_base_model(NULL)
{
  _odometry_listeners.reserve(1);
  _movement_status_listeners.reserve(1);

  _current_position.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

  for (int i = 0; i < AVERAGE_NUM_READINGS; i++)
  {
    _linear_velocities[i] = 0.0;
    _stasis_velocities[i] = 0.0;
  }
}

/** Provides a call-back mechanism for objects interested in receiving 
 *  odometry messages when they are available.
 */
void OdometryIntegrator::Attach(IOdometryListener& odometry_listener) 
{
  _odometry_listeners.push_back(&odometry_listener);
}

/** Provides a call-back mechanism for objects interested in receiving 
 *  movement status messages when they are available.
 */
void OdometryIntegrator::Attach(IMovementStatusListener& movement_status_listener) 
{
  _movement_status_listeners.push_back(&movement_status_listener);
}

/** Sets the BaseModel object to use for ticks to SI conversion.
 */
void OdometryIntegrator::SetBaseModel( const BaseModel& base_model )
{
  _p_base_model = &base_model;
}

/** Callback for IEncoderCountsListener
 */
void OdometryIntegrator::OnEncoderCountsAvailableEvent( const diff_drive::EncoderCounts& encoder_counts )
{
  AddNewCounts( encoder_counts ); 
}

/** This function is used to perform all updates when new counts are ready.
 *
 */
void OdometryIntegrator::AddNewCounts( const diff_drive::EncoderCounts& counts )
{
  _current_position = CalculatePosition( &_velocities, counts, _current_position ); 

  _movement_status = CalculateMovementStatus( _velocities );

  // Set the covariance data
  CalculateCovariance( &_current_position, _movement_status );
  
  NotifyOdometryListeners( _current_position );
  NotifyMovementStatusListeners( _movement_status );
}

/** This function uses the BaseModel class to translate encoder counts into SI
 *  Units and integrates the delta motions into an estimated speed and position.
 *
 */
nav_msgs::Odometry OdometryIntegrator::CalculatePosition(   BaseVelocities_T *p_velocities,
                                                            const diff_drive::EncoderCounts counts, 
                                                            const nav_msgs::Odometry last_position ) 
{
  nav_msgs::Odometry new_position;

  BaseDistance_T delta_position;

  double x;
  double y;
  double theta;
  double dist_theta;
  double old_theta;
  double linear;
  double angular;

  // Run base model here
  if (  (_p_base_model != NULL) && 
        (_p_base_model->GetSetupValid() == true) && 
        (counts.dt_ms > 0) )
  {
    _p_base_model->ConvertCounts( &delta_position, p_velocities, counts );

    old_theta = tf::getYaw(last_position.pose.pose.orientation);

    // Note: As per Tom Brown of UI Urbana-Champaign delta_theta / 2.0 is used for distance
    // estimations.
    dist_theta = old_theta + (delta_position.theta / 2);

    // Integrate the incoming data
    x = last_position.pose.pose.position.x + (delta_position.linear * cos( dist_theta ));
    y = last_position.pose.pose.position.y + (delta_position.linear * sin( dist_theta ));

    // Then the full turn estimate is reported
    theta = old_theta + delta_position.theta;

    linear = p_velocities->linear;
    angular = p_velocities->angular;
  }
  else
  {
    x = 0.0;
    y = 0.0;
    theta = 0.0;
    linear = 0.0;
    angular = 0.0;
  }

#if 0
  std::cout << "X: " << x
            << " Y: " << y
            << " Theta: " << theta
            << std::endl;
#endif

  // Set the Header data
  new_position.header.stamp = counts.reading_time;

  // Set the Pose data
  new_position.pose.pose.position.x = x;
  new_position.pose.pose.position.y = y;
  new_position.pose.pose.position.z = 0.0;

  new_position.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);

  // Set the Twist data
  new_position.twist.twist.linear.x = linear;
  new_position.twist.twist.linear.y = 0.0;
  new_position.twist.twist.linear.z = 0.0;

  new_position.twist.twist.angular.x = 0.0;
  new_position.twist.twist.angular.y = 0.0;
  new_position.twist.twist.angular.z = angular;

  return new_position;
}

/** This function checks all of the available parameters to determine the status
 *  of our movement attempts ie. moving, stalled, or unconfigured.
 *
 */
diff_drive::MovementStatus OdometryIntegrator::CalculateMovementStatus( const BaseVelocities_T  velocities )
{
  float linear_average;
  float stasis_average;
  float abs_linear;
  float abs_stasis;

  diff_drive::MovementStatus movement_status;

  _stasis_window = 0.50; // %
  _stasis_lower_limit = 0.03; // m/s

  if (_p_base_model == NULL) // No base model
  {
    movement_status.motors_state = diff_drive::MovementStatus::SETUP_ERROR;
    movement_status.stasis_wheel_enabled = false;
    movement_status.linear_velocity = 0.0;
    movement_status.linear_velocity_average = 0.0;
    movement_status.stasis_velocity = 0.0;
    movement_status.stasis_velocity_average = 0.0;
  }
  else if ( _p_base_model->GetSetupValid() == false )
  {
    movement_status.motors_state = diff_drive::MovementStatus::SETUP_ERROR;
    movement_status.stasis_wheel_enabled = false;
    movement_status.linear_velocity = 0.0;
    movement_status.linear_velocity_average = 0.0;
    movement_status.stasis_velocity = 0.0;
    movement_status.stasis_velocity_average = 0.0;
  }
  else
  {
    // Copute averages and compare
    _linear_average_total -= _linear_velocities[ _average_index ];
    _linear_velocities[ _average_index ] = velocities.linear;
    _linear_average_total += _linear_velocities[ _average_index ]; 

    _stasis_average_total -= _stasis_velocities[ _average_index ];
    _stasis_velocities[ _average_index ] = velocities.stasis;
    _stasis_average_total += _stasis_velocities[ _average_index ]; 

    linear_average = ldexp( _linear_average_total, -1 * AVERAGE_2N_READINGS );
    stasis_average = ldexp( _stasis_average_total, -1 * AVERAGE_2N_READINGS );

    abs_linear = fabs( linear_average );
    abs_stasis = fabs( stasis_average );

    float lower_limit = abs_linear * ( 1.0 - _stasis_window );
    float upper_limit = abs_linear * ( 1.0 + _stasis_window );

    movement_status.linear_velocity = velocities.linear;
    movement_status.linear_velocity_average = linear_average;
    movement_status.stasis_velocity = velocities.stasis;
    movement_status.stasis_velocity_average = stasis_average;

    _average_index++;
    _average_index %= AVERAGE_NUM_READINGS;

    if ( _p_base_model->GetStasisValid() == false )
    {
      movement_status.motors_state = diff_drive::MovementStatus::CORRECT;
      movement_status.stasis_wheel_enabled = false;
    }
    else // Stasis Wheel Active
    {
      movement_status.stasis_wheel_enabled = true;

      if ( abs_linear > _stasis_lower_limit ) // going fast enough to register stasis wheel movement
      {
        if (abs_stasis > upper_limit)
        {
          movement_status.motors_state = diff_drive::MovementStatus::FREE_WHEELING;
        }
        else if (abs_stasis < lower_limit)
        {
          movement_status.motors_state = diff_drive::MovementStatus::STASIS;
        }
        else
        {
          movement_status.motors_state = diff_drive::MovementStatus::CORRECT;
        }
      }
      else // assume good
      {
        movement_status.motors_state = diff_drive::MovementStatus::CORRECT;
      }
    }
  }

  return movement_status;
}

/** This function computes covariance data based on the movement status state.
 *
 *  If a stasis wheel is in use than the covariance estimates are drastically
 *  reduced if the Linear and Stasis velocities do not match.
 *
 */
void OdometryIntegrator::CalculateCovariance( nav_msgs::Odometry *p_position,
                                                const diff_drive::MovementStatus movement_status )
{
  const double *p_covariance;

  if ( movement_status.motors_state == diff_drive::MovementStatus::CORRECT )
  {
    p_covariance = OdometryCovariance;
  }
  else
  {
    p_covariance = OdometryCovarianceLow;
  } 

  // Set the covariance data
  for (int i = 0; i < 36; i++)
  {
    p_position->pose.covariance[i] = p_covariance[i];
    p_position->twist.covariance[i] = p_covariance[i];
  }
}

/** Calls the callback function for all registered odometry listeners.
 */  
void OdometryIntegrator::NotifyOdometryListeners(const nav_msgs::Odometry& odometry)
{
  for (unsigned int i= 0; i < _odometry_listeners.size(); i++) 
  {
      _odometry_listeners[i]->OnOdometryAvailableEvent(odometry);
  }
}

/** Calls the callback function for all registered movement status listeners.
 */  
void OdometryIntegrator::NotifyMovementStatusListeners(const diff_drive::MovementStatus& movement_status)
{
  for (unsigned int i= 0; i < _movement_status_listeners.size(); i++) 
  {
      _movement_status_listeners[i]->OnMovementStatusAvailableEvent(movement_status);
  }
}
}
