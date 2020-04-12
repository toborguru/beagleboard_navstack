// TickVelocityI2CEndpoint.hpp
 
#ifndef GUARD_TickVelocitySubscriberEndpoint
#define GUARD_TickVelocitySubscriberEndpoint
 
#include <ros/ros.h>
#include <pthread.h>
#include <vector>

#include "ILogPublisherEndpoint.hpp"

#include "ITickVelocitySubscriberEndpoint.hpp"
 
namespace data_robot_message_endpoints
{
class TickVelocitySubscriberEndpoint : public data_robot_core::ITickVelocitySubscriberEndpoint
{ 
public:
  TickVelocitySubscriberEndpoint();
  TickVelocitySubscriberEndpoint( boost::shared_ptr<data_robot_core::ILogPublisherEndpoint> log_endpoint );
  ~TickVelocitySubscriberEndpoint();

  void Subscribe();
  void Unsubscribe();
  void Attach( data_robot_core::ITickVelocityListener& tick_velocity_listener );

  void NewTickVelocityReceived( const diff_drive_calibrated::TickVelocity& tick_velocity );

private:
  void ReceiveTickVelocityMessages();

  void NotifyTickVelocityListeners( const diff_drive_calibrated::TickVelocity& tick_velocity );

  void Log( data_robot_core::LogLevel_T level, data_robot_core::LogFlags_T flags, const std::string& log_message );

  boost::shared_ptr<data_robot_core::ILogPublisherEndpoint> _p_log_endpoint;

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
    ((TickVelocitySubscriberEndpoint*)This)->ReceiveTickVelocityMessages();
    return 0;
  }
};
}
 
#endif /* GUARD_TickVelocitySubscriberEndpoint */
