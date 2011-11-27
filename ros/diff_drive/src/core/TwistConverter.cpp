/** @file
 *
 *  Twist Reader Class File
 *
 *  @details
 *  
 *  @author Sawyer Larkin (SJL toborguru)
 *
 *  @copyright GNU Public License Version 2.
 */

#include "TwistConverter.hpp"

namespace diff_drive_core
{
/** Default constructor.
 */
TwistConverter::TwistConverter()
           : _p_base_model(NULL)
{
  _tick_velocity_listeners.reserve(1);
}

/** Provides a call-back mechanism for objects interested in receiving 
 *  tick_velocity messages when they are available.
 */
void TwistConverter::Attach(ITickVelocityListener& tick_velocity_listener) 
{
  _tick_velocity_listeners.push_back(&tick_velocity_listener);
}

/** Sets the BaseModel object to use for SI to ticks conversion.
 */
void TwistConverter::SetBaseModel( const BaseModel& base_model )
{
  _p_base_model = &base_model;
}

/** Callback for ITwistListener
 */
void TwistConverter::OnTwistAvailableEvent( const geometry_msgs::Twist& twist )
{
  diff_drive::TickVelocity tick_velocity;
  tick_velocity = ConvertTwist( twist ); 

  NotifyTickVelocityListeners( tick_velocity );
}

/** This function uses the BaseModel class to translate SI Units into encoder
 *  ticks.
 */
diff_drive::TickVelocity TwistConverter::ConvertTwist( const geometry_msgs::Twist twist )
{
  diff_drive::TickVelocity ticks;

  if ( _p_base_model != NULL )
  {
    ticks = _p_base_model->VelocityToTicks( twist.linear.x, twist.angular.z );
  }
  else
  {
    ticks.linear_ticks_sec = 0;
    ticks.angular_ticks_sec = 0;
  }

  return ticks;
}

/** Calls the callback function for all registered tick_velocity listeners.
 */  
void TwistConverter::NotifyTickVelocityListeners(const diff_drive::TickVelocity& tick_velocity)
{
  for (int i= 0; i < _tick_velocity_listeners.size(); i++) 
  {
      _tick_velocity_listeners[i]->OnTickVelocityAvailableEvent(tick_velocity);
  }
}
}