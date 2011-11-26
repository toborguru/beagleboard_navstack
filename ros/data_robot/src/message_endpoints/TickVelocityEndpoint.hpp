// TickVelocityI2CEndpoint.hpp
 
#ifndef GUARD_TickVelocityEndpoint
#define GUARD_TickVelocityEndpoint
 
#include <ros/ros.h>
#include <pthread.h>
#include <vector>

#include "ITickVelocityEndpoint.hpp"
 
namespace data_robot_message_endpoints
{
class TickVelocityEndpoint : public data_robot_core::ITickVelocityEndpoint
{ 
public:
  TickVelocityEndpoint();
  ~TickVelocityEndpoint();

  void Subscribe();
  void Unsubscribe();
  void Attach( data_robot_core::ITickVelocityListener& tick_velocity_listener );

  void NewTickVelocityReceived( const diff_drive::TickVelocity& tick_velocity );

private:
  void ReceiveTickVelocityMessages();

  void NotifyTickVelocityListeners( const diff_drive::TickVelocity& tick_velocity );

  std::vector<data_robot_core::ITickVelocityListener*> _tick_velocity_listeners;

  // Create handle to node
  ros::NodeHandle _tick_velocity_node;

  ros::Subscriber _tick_velocity_subscriber;

  // Basic threading support as suggested by Jeremy Friesner at
  // http://stackoverflow.com/questions/1151582/pthread-function-from-a-class
  volatile bool _stop_requested;
  volatile bool _running;

  pthread_t _thread;

  static void * ReceiveTickVelocityMessagesFunction(void * This) 
  {
    ((TickVelocityEndpoint*)This)->ReceiveTickVelocityMessages();
    return 0;
  }
};
}
 
#endif /* GUARD_TickVelocityEndpoint */
