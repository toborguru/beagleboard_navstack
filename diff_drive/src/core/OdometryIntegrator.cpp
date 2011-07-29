/** @file
 *
 *  Encoder Counts Reader Class File
 *
 *  @details
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
                   : _p_base_model(NULL)
{
  _odometry_listeners.reserve(1);

  _current_position.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
}

/** Provides a call-back mechanism for objects interested in receiving 
 *  odometry messages when they are available.
 */
void OdometryIntegrator::Attach(IOdometryListener& odometry_listener) 
{
  _odometry_listeners.push_back(&odometry_listener);
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
  _current_position = AddNewCounts( encoder_counts, _current_position ); 

  NotifyOdometryListeners( _current_position );
}

/** This function uses the BaseModel class to translate encoder counts into SI
 *  Units and integrates the delta motions into an estimated speed and position.
 *
 *  If a stasis wheel is in use than the covariance estimates are drastically
 *  reduced if the Linear and Stasis velocities do not match.
 *
 */
nav_msgs::Odometry OdometryIntegrator::AddNewCounts( const diff_drive::EncoderCounts counts, 
                                                     const nav_msgs::Odometry last_position ) 
{
  nav_msgs::Odometry new_position;

  BaseDistance_T delta_position;
  BaseVelocities_T  velocities;

  const double (*p_covariance)[36];

  double x;
  double y;
  double theta;
  double old_theta;
  double linear;
  double angular;

  // Run base model here
  if (_p_base_model != NULL)
  {
    _p_base_model->ConvertCounts( &delta_position, &velocities, counts );

    old_theta = tf::getYaw(last_position.pose.pose.orientation);

    theta = old_theta + (delta_position.theta / 2);

    // Integrate the incoming data
    x = last_position.pose.pose.position.x + (delta_position.linear * cos( theta ));
    y = last_position.pose.pose.position.y + (delta_position.linear * sin( theta ));
    theta = old_theta + delta_position.theta;
    linear = velocities.linear;
    angular = velocities.angular;

    if ( _p_base_model->GetStasisTicks() > 0 )
    {
      p_covariance = &OdometryCovariance;
    }
    else
    {
      p_covariance = &OdometryCovariance;
    }
  }
  else
  {
    x = 0.0;
    y = 0.0;
    theta = 0.0;
    linear = 0.0;
    angular = 0.0;

    p_covariance = &OdometryCovarianceLow;
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

  // Set the covariance data
  for (int i = 0; i < 36; i++)
  {
    new_position.pose.covariance[i] = *p_covariance[i];
    new_position.twist.covariance[i] = *p_covariance[i];
  }
  
  return new_position;
}

/** Calls the callback function for all registered odometry listeners.
 */  
void OdometryIntegrator::NotifyOdometryListeners(const nav_msgs::Odometry& odometry)
{
  for (int i= 0; i < _odometry_listeners.size(); i++) 
  {
      _odometry_listeners[i]->OnOdometryAvailableEvent(odometry);
  }
}
}
