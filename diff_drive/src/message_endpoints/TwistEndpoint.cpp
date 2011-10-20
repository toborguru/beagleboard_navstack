// TwistEndpoint.cpp
 
#include <fcntl.h>
#include <ros/ros.h>

#include "TwistEndpoint.hpp"

using namespace diff_drive_core;
 
namespace diff_drive_message_endpoints
{
TwistEndpoint::TwistEndpoint()    
              : _stopRequested(false), 
                _running(false) 
{
  _twist_listeners.reserve(1);
}

TwistEndpoint::~TwistEndpoint()
{
}

void TwistEndpoint::Subscribe()
{
  if (! _running) 
  {
    _running = true;
    _stopRequested = false;
    // Spawn async thread for reading laser scans
    pthread_create(&_thread, 0, ReceiveTwistMessagesFunction, this);
  }
}

void TwistEndpoint::Unsubscribe()
{
  if (_running) 
  {
    _running = false;
    _stopRequested = true;

    // Wait to return until _thread has completed
    pthread_join(_thread, 0);
  }
}

void TwistEndpoint::Attach( ITwistListener& twist_listener )
{
  _twist_listeners.push_back(&twist_listener);
}

void TwistEndpoint::TwistReceivedCallback( const geometry_msgs::Twist& twist )
{
  NotifyTwistListeners( twist );
}

void TwistEndpoint::ReceiveTwistMessages()
{
  ros::Subscriber twist_subscriber = _twist_node.subscribe( "~cmd_vel", 
                                                            1, 
                                                            &TwistEndpoint::TwistReceivedCallback,
                                                            this );

  ros::Rate r(100); // 100 hz

  while (!_stopRequested && ros::ok()) 
  {
    ros::spinOnce();
    r.sleep();
  }

  _running = false;
}

void TwistEndpoint::NotifyTwistListeners( const geometry_msgs::Twist& twist )
{
  for (int i= 0; i < _twist_listeners.size(); i++)
  {
    _twist_listeners[i]->OnTwistAvailableEvent( twist );
  }
}
}
