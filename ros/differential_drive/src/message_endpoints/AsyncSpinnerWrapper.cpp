/** @file
 *  Wrapper which allows multiple instances of this class to start and stop a
 *  single AsyncSpinner instance. 
 *
 *  This allows individual threads (instances) to call Stop() without affecting 
 *  other threads or needing central object pass-in. When every started instance 
 *  has been stopped or destroyed the Async spinner will be stopped.
 * 
 *  Note that if the ROS entity (generally an ros::Subscriber) still exists the 
 *  call backs will continue to occur, even if they called Stop() on their 
 *  instance of this wrapper.
 *
 */
 
#include "AsyncSpinnerWrapper.hpp"

namespace differential_drive_message_endpoints
{

// Static object initialization
ros::AsyncSpinner*  AsyncSpinnerWrapper::_p_spinner = NULL;
pthread_mutex_t*    AsyncSpinnerWrapper::_p_lock_mutex = NULL;
int                 AsyncSpinnerWrapper::_num_started = 0;
int                 AsyncSpinnerWrapper::_num_instances = 0;
bool                AsyncSpinnerWrapper::_is_spinning = false;

/** Default Constructor
 */
AsyncSpinnerWrapper::AsyncSpinnerWrapper()
                    : _this_started(false)
{
  if ( _p_lock_mutex == NULL )
  {
    _p_lock_mutex = new pthread_mutex_t;
    pthread_mutex_init( _p_lock_mutex, NULL );

    // This may be paranoid ;)
    pthread_mutex_lock( _p_lock_mutex );

    _p_spinner = new ros::AsyncSpinner(0);
    _num_instances = 1;
    _num_started = 0;
    _is_spinning = false;

    pthread_mutex_unlock( _p_lock_mutex );
  }
  else
  {
    pthread_mutex_lock( _p_lock_mutex );

    // Register ourselves so we can correctly handle the mutex destruction
    _num_instances++;
    
    pthread_mutex_unlock( _p_lock_mutex );
  }
}

/** Copy Constructor
 */
AsyncSpinnerWrapper::AsyncSpinnerWrapper( AsyncSpinnerWrapper& orig_wrapper )
                    : _this_started(false)
{
    pthread_mutex_lock( _p_lock_mutex );

    // Register ourselves so we can correctly handle the mutex destruction
    _num_instances++;
    
    pthread_mutex_unlock( _p_lock_mutex );
}

/** Destructor
 */
AsyncSpinnerWrapper::~AsyncSpinnerWrapper()
{
  // Make sure our instance has "clocked out"
  Stop();

  pthread_mutex_lock( _p_lock_mutex );

  // Free the mutex if we are the last object 
  if ( _num_instances <= 0 )
  {
    pthread_mutex_unlock( _p_lock_mutex );

    pthread_mutex_destroy( _p_lock_mutex );
    delete _p_lock_mutex;
    _p_lock_mutex = NULL;

    delete _p_spinner;
    _p_spinner = NULL;
  }

  pthread_mutex_unlock( _p_lock_mutex );
}

/** Start the spinner.
 */
void AsyncSpinnerWrapper::Start()
{
  pthread_mutex_lock( _p_lock_mutex );
 
  if ( _is_spinning == false )
  {
    _is_spinning = true;
    _p_spinner->start();
  }

  if ( !_this_started )
  {
    _num_started++;
  }

  _this_started = true;

  pthread_mutex_unlock( _p_lock_mutex );
}

/** Stop the spinner.
 *
 *  This function tells the wrapper that this instance no longer needs updates.
 *  The AsyncSpinner will only be stopped if all instances have called Stop, or 
 *  been destructed. Note that if the ROS entity (generally an ros::Subscriber) 
 *  still exists the call backs will continue to occur.
 */
void AsyncSpinnerWrapper::Stop()
{
  pthread_mutex_lock( _p_lock_mutex );
  
  if ( _this_started )
  {
    _num_started--;
  }

  _this_started = false;

  if ( _num_started <= 0 )
  {
    _p_spinner->stop();
    _is_spinning = false;
  }

  pthread_mutex_unlock( _p_lock_mutex );
}

/** Access function, returns the number of AsyncSpinnerWrapper instances that 
 *  have requested a start.
 */
int AsyncSpinnerWrapper::GetNumStarted() const
{
  return _num_started;
}

/** Access function, returns the number of AsyncSpinnerWrapper instances.
 * 
 *  Mostly for debugging.
 */
int AsyncSpinnerWrapper::GetNumInstances() const
{
  return _num_instances;
}
}
