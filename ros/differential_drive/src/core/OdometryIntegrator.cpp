/** @file
 *
 *  Encoder Counts Reader Class File
 *
 *  @details
 *  This class uses BaseModel class physical attributes to compute distances, 
 *  velocities, and motion state from incoming encoder counts. This class 
 *  optionally supports a stasis wheel, if properly defined in the BaseModel 
 *  class.
 *  
 *  @author Sawyer Larkin (SJL toborguru)
 *
 *  @copyright GNU Public License Version 2.
 */

#include <cmath>

#include "tf/transform_datatypes.h"

#include "OdometryIntegrator.hpp"

namespace differential_drive_core
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
                   : _velocity_allowance(0.10),
                     _velocity_lower_limit(0.05),
                     _average_2n_readings(3),
                     _average_num_readings(0),
                     _p_linear_velocities(NULL),
                     _p_stasis_velocities(NULL),
                     _p_base_model(NULL),
                     _stop_requested(false),
                     _is_running(false)
{
  _odometry_listeners.reserve(1);
  _movement_status_listeners.reserve(1);

  _current_position.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

  _p_data_mutex = new pthread_mutex_t;
  pthread_mutex_init( _p_data_mutex, NULL );

  _p_message_sem = new sem_t;
  sem_init( _p_message_sem, 0, 0 );
  
  setAverage2nReadings( _average_2n_readings );
}

/** Default destructor.
 */
OdometryIntegrator::~OdometryIntegrator()
{
  stopProcessingOdometry();

  if ( _p_linear_velocities != NULL )
  { 
    delete [] _p_linear_velocities;
  }

  if ( _p_stasis_velocities != NULL )
  { 
    delete [] _p_stasis_velocities;
  }

  pthread_mutex_destroy( _p_data_mutex );
  delete _p_data_mutex;

  sem_destroy( _p_message_sem );
  delete _p_message_sem;
}

/** Provides a call-back mechanism for objects interested in receiving 
 *  odometry messages when they are available.
 */
void OdometryIntegrator::attach( IOdometryListener& odometry_listener) 
{
  pthread_mutex_lock( _p_data_mutex );
  _odometry_listeners.push_back(&odometry_listener);
  pthread_mutex_unlock( _p_data_mutex );

  startProcessingOdometry();
}

/** Allows a listener to stop receiving call-backs. If this is the last listener
 *  the class will automatically call unsubscribe.
 */
void OdometryIntegrator::detach( IOdometryListener& odometry_listener )
{
  pthread_mutex_lock( _p_data_mutex );

  // Using the remove-erase idiom
  std::vector<IOdometryListener*>& vec = _odometry_listeners; // use shorter name
  vec.erase( std::remove(vec.begin(), vec.end(), &odometry_listener), vec.end() );
  
  pthread_mutex_unlock( _p_data_mutex );

  if ( (_odometry_listeners.size() == 0) && (_movement_status_listeners.size() == 0) )
  {
    stopProcessingOdometry();
  }
}

/** Provides a call-back mechanism for objects interested in receiving 
 *  movement status messages when they are available.
 */
void OdometryIntegrator::attach(IMovementStatusListener& movement_status_listener) 
{
  pthread_mutex_lock( _p_data_mutex );
  _movement_status_listeners.push_back(&movement_status_listener);
  pthread_mutex_unlock( _p_data_mutex );

  startProcessingOdometry();
}

/** Allows a listener to stop receiving call-backs. If this is the last listener
 *  the class will automatically call unsubscribe.
 */
void OdometryIntegrator::detach( IMovementStatusListener& movement_status_listener )
{
  pthread_mutex_lock( _p_data_mutex );

  // Using the remove-erase idiom
  std::vector<IMovementStatusListener*>& vec = _movement_status_listeners; // use shorter name
  vec.erase( std::remove(vec.begin(), vec.end(), &movement_status_listener), vec.end() );

  pthread_mutex_unlock( _p_data_mutex );

  if ( (_odometry_listeners.size() == 0) && (_movement_status_listeners.size() == 0) )
  {
    stopProcessingOdometry();
  }
}

/** sets the BaseModel object to use for ticks to SI conversion.
 */
void OdometryIntegrator::setBaseModel( BaseModel const & base_model )
{
  pthread_mutex_lock( _p_data_mutex );
  _p_base_model = &base_model;
  pthread_mutex_unlock( _p_data_mutex );
}

/** Callback for IEncoderCountsListener
 */
void OdometryIntegrator::onEncoderCountsAvailableEvent( differential_drive::EncoderCounts const & encoder_counts )
{
  const differential_drive::EncoderCounts* p_new_message;

  p_new_message = new differential_drive::EncoderCounts( encoder_counts );

#if 0
  std::cout << "Copy left: " << p_new_message->left_count
            << " right: " << p_new_message->right_count
            << std::endl;
#endif

  pthread_mutex_lock( _p_data_mutex );

  // Add our local copy of the message
  _encoder_counts_messages.push( p_new_message );

  pthread_mutex_unlock( _p_data_mutex );

  // Send signal
  sem_post( _p_message_sem );
}

/** Returns actual number of velocity readings that will be averaged prior to
 *  comparison.
 */
unsigned int OdometryIntegrator::getAverageNumReadings() const
{
  return _average_num_readings;
}

/** sets the size of the buffers used to averages the velocity readings.
 *  Because the buffers need to be 2^n in size, this function will choose 
 *  the largest 2^n number that will fit inside. Ex: 10=>8, 100=>64.
 *
 *  @returns true if @c average_num_readings is between 0 and MAX, and the internal data member is updated.
 */
bool OdometryIntegrator::setAverageNumReadings( int average_num_readings )
{
  bool valid;
  unsigned int average_2n = MAX_2N_AVERAGES;

  if ( average_num_readings <= 0 )
  {
    valid = false;
  }
  else
  {
    for ( unsigned int i = 0; i <= MAX_2N_AVERAGES; ++i )
    {
      if ( average_num_readings < ldexp(1.0, i) )
      {
        average_2n = i - 1;
        break; 
      }
    }

    valid = setAverage2nReadings( average_2n );
  }

  return valid;
}

/** Returns average_2n_readings in the eq: 2^average_2n_readings velocity 
 *  readings will be averaged before comparison.
 */
unsigned int OdometryIntegrator::getAverage2nReadings() const
{
  return _average_2n_readings;
}

/** 2^average_2n_readings velocity readings will be averaged before comparison.
 *
 *  @returns true if @c average_2n_readings is between 0 and MAX_2N_AVERAGES, and the internal data member is updated.
 */
bool OdometryIntegrator::setAverage2nReadings( int average_2n_readings )
{
  bool valid;

  unsigned int new_num_readings;

  if ( average_2n_readings < 0 )
  {
    valid = false;
  }
  else
  {
    // Max 64K readings buffer...
    if ( average_2n_readings > MAX_2N_AVERAGES )
    {
      valid = false;
    }
    else
    {
      valid = true;

      pthread_mutex_lock( _p_data_mutex );
      _average_2n_readings = average_2n_readings;
      new_num_readings = ldexp( 1.0, _average_2n_readings );

      if ( new_num_readings != _average_num_readings )
      {
        if ( _p_linear_velocities != NULL )
        { 
          delete [] _p_linear_velocities;
        }

        if ( _p_stasis_velocities != NULL )
        { 
          delete [] _p_stasis_velocities;
        }

        _p_linear_velocities = new double [ new_num_readings ];
        _p_stasis_velocities = new double [ new_num_readings ];
        _average_num_readings = new_num_readings;
      }

      // Clear averaging arrays
      for ( unsigned int i = 0; i < _average_num_readings; ++i )
      {
        _p_linear_velocities[ i ] = 0.0;
        _p_stasis_velocities[ i ] = 0.0;
      }

      // Reset averaging state
      _average_index = 0;
      _average_index_mask = _average_num_readings - 1;
      _num_readings_read = 0;
      _linear_average_total = 0.0;
      _stasis_average_total = 0.0;

      pthread_mutex_unlock( _p_data_mutex );
    }
  }

  return valid;
}

/** Access function.
 */
double OdometryIntegrator::getVelocityMatchPercentage() const
{
  return ( _velocity_allowance * 100.0 );
}

/** Access function.
 *
 *  @returns true if @p percentage is between 0 and 100 and the internal data member is updated. 
 */
bool OdometryIntegrator::setVelocityMatchPercentage( double percentage )
{
  if ( (percentage > 0.0) && (percentage <= 100.0) )
  {
    percentage = percentage / 100.0;

    pthread_mutex_lock( _p_data_mutex );
    _velocity_allowance = percentage;
    pthread_mutex_unlock( _p_data_mutex );

    return true;
  }

  return false;
}

/** Access function.
 */
double OdometryIntegrator::getVelocityLowerLimit() const
{
  return _velocity_lower_limit;
}

/** Access function.
 *  
 *  @returns true if @c velocity_limit is no negative, and the internal data memeber is updated.
 */
bool OdometryIntegrator::setVelocityLowerLimit( double velocity_limit )
{
  if ( velocity_limit >= 0.0 )
  {
    pthread_mutex_lock( _p_data_mutex );
    _velocity_lower_limit = velocity_limit;
    pthread_mutex_unlock( _p_data_mutex );

    return true;
  }

  return false;
}

/** This function is used to perform all updates when new counts are ready.
 *
 */
void OdometryIntegrator::addNewCounts( differential_drive::EncoderCounts const & counts )
{
  BaseVelocities_T  velocities;
  nav_msgs::Odometry new_position;
  nav_msgs::Odometry old_position;
  differential_drive::MovementStatus new_movement_status;

  assert ( _p_base_model != NULL );

  pthread_mutex_lock( _p_data_mutex );
  old_position = _current_position;
  velocities = _velocities;
  pthread_mutex_unlock( _p_data_mutex );

  new_position = calculatePosition( &velocities, counts, old_position, *_p_base_model ); 
  new_movement_status = calculateMovementStatus( velocities, *_p_base_model );

  // set the covariance data
  calculateCovariance( &new_position, new_movement_status );

  pthread_mutex_lock( _p_data_mutex );
  _current_position = new_position;
  _velocities = velocities;
  pthread_mutex_unlock( _p_data_mutex );

  notifyOdometryListeners( new_position );
  notifyMovementStatusListeners( new_movement_status );
}

/** This function uses the BaseModel class to translate encoder counts into SI
 *  Units and integrates the delta motions into an estimated speed and position.
 *
 */
nav_msgs::Odometry OdometryIntegrator::calculatePosition(   BaseVelocities_T* p_velocities,
                                                            differential_drive::EncoderCounts const & counts, 
                                                            nav_msgs::Odometry const & last_position,
                                                            BaseModel const & base_model ) const
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
  if (  (base_model.getSetupValid() == true) && 
        (counts.dt_ms > 0) )
  {
    base_model.convertCounts( &delta_position, p_velocities, counts );

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

  // set the Header data
  new_position.header.stamp = counts.reading_time;

  // set the Pose data
  new_position.pose.pose.position.x = x;
  new_position.pose.pose.position.y = y;
  new_position.pose.pose.position.z = 0.0;

  new_position.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);

  // set the Twist data
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
differential_drive::MovementStatus OdometryIntegrator::calculateMovementStatus( BaseVelocities_T const & velocities,
                                                                                BaseModel const & base_model )
{
  double linear_average;
  double stasis_average;
  double abs_linear;
  double abs_stasis;

  differential_drive::MovementStatus movement_status;

  if ( base_model.getSetupValid() == false )
  {
    movement_status.motors_state = differential_drive::MovementStatus::SETUP_ERROR;
    movement_status.stasis_wheel_enabled = false;
    movement_status.linear_velocity = 0.0;
    movement_status.linear_velocity_average = 0.0;
    movement_status.stasis_velocity = 0.0;
    movement_status.stasis_velocity_average = 0.0;
  }
  else
  {
    pthread_mutex_lock( _p_data_mutex );

    // Compute averages and compare
    _linear_average_total -= _p_linear_velocities[ _average_index ];
    _p_linear_velocities[ _average_index ] = velocities.linear;
    _linear_average_total += _p_linear_velocities[ _average_index ]; 

    _stasis_average_total -= _p_stasis_velocities[ _average_index ];
    _p_stasis_velocities[ _average_index ] = velocities.stasis;
    _stasis_average_total += _p_stasis_velocities[ _average_index ]; 
    
    // Correctly compute the average velocity
    if ( _num_readings_read < _average_num_readings )
    {
      // Account for the reading that called this function
      // I could add this outside the if and prevent run-away in the else 
      // block but I figure 1 extra division loop is better than two 
      // assignments during every loop of normal operation
      ++_num_readings_read;

      linear_average = _linear_average_total / _num_readings_read;
      stasis_average = _stasis_average_total / _num_readings_read;
    }
    else
    {
      linear_average = ldexp( _linear_average_total, -1 * _average_2n_readings );
      stasis_average = ldexp( _stasis_average_total, -1 * _average_2n_readings );
    }

    abs_linear = fabs( linear_average );
    abs_stasis = fabs( stasis_average );

    double lower_limit = abs_linear * ( 1.0 - _velocity_allowance );
    double upper_limit = abs_linear * ( 1.0 + _velocity_allowance );

    movement_status.linear_velocity = velocities.linear;
    movement_status.linear_velocity_average = linear_average;
    movement_status.stasis_velocity = velocities.stasis;
    movement_status.stasis_velocity_average = stasis_average;

    ++_average_index;
    _average_index &= _average_index_mask;

    if ( base_model.getStasisValid() == false )
    {
      movement_status.motors_state = differential_drive::MovementStatus::CORRECT;
      movement_status.stasis_wheel_enabled = false;
    }
    else // Stasis Wheel Active
    {
      movement_status.stasis_wheel_enabled = true;
      
      // going fast enough to register stasis wheel movement
      if ( (abs_linear > _velocity_lower_limit) || (abs_stasis > _velocity_lower_limit) )
      {
        if (abs_stasis > upper_limit)
        {
          movement_status.motors_state = differential_drive::MovementStatus::FREE_WHEELING;
        }
        else if (abs_stasis < lower_limit)
        {
          movement_status.motors_state = differential_drive::MovementStatus::STASIS;
        }
        else
        {
          movement_status.motors_state = differential_drive::MovementStatus::CORRECT;
        }
      }
      else // assume good
      {
        movement_status.motors_state = differential_drive::MovementStatus::CORRECT;
      }
    }

    pthread_mutex_unlock( _p_data_mutex );
  }

  return movement_status;
}

/** This function computes covariance data based on the movement status state.
 *
 *  If a stasis wheel is in use than the covariance estimates are drastically
 *  reduced if the Linear and Stasis velocities do not match.
 *
 */
void OdometryIntegrator::calculateCovariance( nav_msgs::Odometry *p_position,
                                              differential_drive::MovementStatus const & movement_status ) const
{
  const double *p_covariance;

  if ( movement_status.motors_state == differential_drive::MovementStatus::CORRECT )
  {
    p_covariance = OdometryCovariance;
  }
  else
  {
    p_covariance = OdometryCovarianceLow;
  } 

  // set the covariance data
  for (int i = 0; i < 36; ++i)
  {
    p_position->pose.covariance[i] = p_covariance[i];
    p_position->twist.covariance[i] = p_covariance[i];
  }
}

/** Calls the callback function for all registered odometry listeners.
 */  
void OdometryIntegrator::notifyOdometryListeners( nav_msgs::Odometry const & odometry) const
{
  pthread_mutex_lock( _p_data_mutex );

  for (unsigned int i= 0; i < _odometry_listeners.size(); ++i) 
  {
    _odometry_listeners[i]->OnOdometryAvailableEvent(odometry);
  }

  pthread_mutex_unlock( _p_data_mutex );
}

/** Calls the callback function for all registered movement status listeners.
 */  
void OdometryIntegrator::notifyMovementStatusListeners( differential_drive::MovementStatus const & movement_status) const
{
  pthread_mutex_lock( _p_data_mutex );

  for (unsigned int i= 0; i < _movement_status_listeners.size(); ++i) 
  {
    _movement_status_listeners[i]->OnMovementStatusAvailableEvent(movement_status);
  }

  pthread_mutex_unlock( _p_data_mutex );
}

/** Worker thread
 */
void OdometryIntegrator::processOdometry()
{
  const differential_drive::EncoderCounts* p_message;
  
  while ( ! _stop_requested )
  {
    while ( ! _encoder_counts_messages.empty() )
    {
      pthread_mutex_lock( _p_data_mutex );
        
      p_message = _encoder_counts_messages.front();
      _encoder_counts_messages.pop();

      pthread_mutex_unlock( _p_data_mutex );

      addNewCounts( *p_message ); 

      delete p_message;
    }

    // Wait for message signal
    sem_wait( _p_message_sem );
  }
}

/** Starts worker thread.
 */
void OdometryIntegrator::startProcessingOdometry()
{
  pthread_mutex_lock( _p_data_mutex );

  if ( ! _is_running ) 
  {
    _is_running = true;
    _stop_requested = false;

    pthread_mutex_unlock( _p_data_mutex );

    // Spawn async thread 
    pthread_create(&_thread, 0, processOdometryFunction, this);

    pthread_mutex_lock( _p_data_mutex );
  }

  pthread_mutex_unlock( _p_data_mutex );
}

/** Stops worker thread.
 */
void OdometryIntegrator::stopProcessingOdometry()
{
  pthread_mutex_lock( _p_data_mutex );

  if ( _is_running ) 
  {
    _is_running = false;
    _stop_requested = true;

    pthread_mutex_unlock( _p_data_mutex );

    // Wake thread up
    sem_post( _p_message_sem );

    // Wait to return until _thread has completed
    pthread_join(_thread, 0);

    pthread_mutex_lock( _p_data_mutex );
  }

  pthread_mutex_unlock( _p_data_mutex );
}
}
