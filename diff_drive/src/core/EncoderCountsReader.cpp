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
 
#include "EncoderCountsReader.hpp"

namespace diff_drive_core
{
/** Default constructor.
 */
EncoderCountsReader::EncoderCountsReader()
{
  _odometryListeners.reserve(1);
}

/** Constructor which automatically connects to the endpoint 
 *  @p encoderEndpoint.
 */
EncoderCountsReader::EncoderCountsReader( IEncoderCountsEndpoint& encoderEndpoint ) 
{
  _odometryListeners.reserve(1);
  Connect( encoderEndpoint );
}

EncoderCountsReader::~EncoderCountsReader()
{
  _p_encoderCountsEndpoint->Unsubscribe();
}

/** Provides a call-back mechanism for objects interested in receiving 
 *  odometry messages when they are available.
 */
void EncoderCountsReader::Attach(IOdometryListener& odometryListener) 
{
  _odometryListeners.push_back(&odometryListener);
}

/** Connect a message endpoint to receive encoder count messages.
*/
void EncoderCountsReader::Connect( IEncoderCountsEndpoint& encoderEndpoint )
{
  _p_encoderCountsEndpoint = &encoderEndpoint;
  _p_encoderCountsEndpoint->Attach(*this);
  _p_encoderCountsEndpoint->Subscribe();
}

/** Callback for IEncoderCountsListener
*/
void EncoderCountsReader::OnEncoderCountsAvailableEvent( const diff_drive::EncoderCounts& encoderCounts )
{
  _currentPosition = CountsReceived( encoderCounts, _currentPosition ); 

  NotifyOdometryListeners( _currentPosition );
}

/** Updates */
nav_msgs::Odometry EncoderCountsReader::CountsReceived( const diff_drive::EncoderCounts counts, 
                                                        const nav_msgs::Odometry lastPosition ) 
{
  int i = 0;

    nav_msgs::Odometry newPosition;

    // Run base model here

    // Just set some data
    newPosition.pose.pose.position.x = (float)( counts.left_count );
    newPosition.pose.pose.position.y = (float)( counts.right_count );

    return newPosition;
}

/** Calls the callback function for all registered odometry listeners.
 */  
void EncoderCountsReader::NotifyOdometryListeners(const nav_msgs::Odometry& odometry)
{
  for (int i= 0; i < _odometryListeners.size(); i++) 
  {
      _odometryListeners[i]->OnOdometryAvailableEvent(odometry);
  }
}
}
