// AsyncSpinnerWrapper.hpp
 
#ifndef GUARD_AsyncSpinnerWrapper
#define GUARD_AsyncSpinnerWrapper
 
#include <ros/ros.h>
#include <pthread.h>
 
namespace differential_drive_message_endpoints
{
class AsyncSpinnerWrapper
{
public:
  AsyncSpinnerWrapper();
  AsyncSpinnerWrapper( AsyncSpinnerWrapper& orig_wrapper );

  ~AsyncSpinnerWrapper();

  void Start();
  void Stop();

  int GetNumStarted() const;
  int GetNumInstances() const;

private:
  // Disable the assignment operator because it doesn't make sense, and the 
  // implementation was 'do nothing' anyway. Do not give this a body.
  AsyncSpinnerWrapper& operator= ( const AsyncSpinnerWrapper& );

  // Static Variables
  static      ros::AsyncSpinner *_p_spinner;
  static      pthread_mutex_t  *_p_lock_mutex;
  static int  _num_started;
  static int  _num_instances;
  static bool _is_spinning;

  // Member Variables
  bool _this_started;
};
}
 
#endif /* GUARD_AsyncSpinnerWrapper */
