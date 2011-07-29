/* Unit test for EncoderCountsReader class.
 *
 * @author Sawyer Larkin (SJL toborguru)
 */

#include <gtest/gtest.h>
#include "EncoderCountsReader.hpp"
#include "IOdometryListener.hpp"
#include "IEncoderCountsEndpoint.hpp"
#include "nav_msgs/Odometry.h"
 
using namespace diff_drive_core;
 
namespace diff_drive_core_test_core
{
// Will be used by unit test to check odometry
struct OdometryReceiver : public diff_drive_core::IOdometryListener
{
  OdometryReceiver()
    : _countOfOdometrysReceived(0) 
  { }

  int _countOfOdometrysReceived;

  void OnOdometryAvailableEvent(const nav_msgs::Odometry& odometry)
  {
    _countOfOdometrysReceived++;

    // Output the laser scan seq number to the terminal; this isn't the
    // unit test, but merely a helpful means to show what's going on.
    std::cout << "Odometry sent to OdometryReceiver with x, y of: " 
              << odometry.pose.pose.position.x 
              << odometry.pose.pose.position.y 
              << std::endl;
  }
};

// Will be used by the unit test to produce encoder ticks
struct EncoderCountsGenerator : public diff_drive_core::IEncoderCountsEndpoint
{
  EncoderCountsGenerator() 
    : _subscribed(false)
  {}

  void AddTicks( const diff_drive::EncoderCounts encoderCounts )
  {
    for (unsigned int i= 0; i < _encoderCountsListeners.size(); i++) 
    {
      _encoderCountsListeners[i]->OnEncoderCountsAvailableEvent(encoderCounts);
    }
  }

  void Subscribe()
  {
    _subscribed = true;
  }
  
  void Unsubscribe()
  {
    _subscribed = false;
  }
  
  void Attach( IEncoderCountsListener& encoderCountsListener )
  {
    _encoderCountsListeners.push_back(&encoderCountsListener);
  }

  bool _subscribed;
  std::vector<IEncoderCountsListener*> _encoderCountsListeners;
};

// Define the unit test to verify ability to listen for Ticks and generate Odometry
TEST( EncoderCountsReaderTests, canSendCountsAndReceiveOdometry ) 
{
  // Establish Context
  EncoderCountsReader encoderCountsReader;
  EncoderCountsGenerator countGenerator;
  OdometryReceiver odometryReceiver;

  diff_drive::EncoderCounts new_counts;

  // Wire up the reader to the handler of laser scan reports
  encoderCountsReader.Attach(odometryReceiver);
  encoderCountsReader.Connect(countGenerator);

  // Act
  new_counts.left_count = 5;
  new_counts.right_count = 2;
  new_counts.dt_ms = 10;
  countGenerator.AddTicks(new_counts);
  countGenerator.AddTicks(new_counts);
  countGenerator.AddTicks(new_counts);
  countGenerator.AddTicks(new_counts);

  // Assert

  // Check the results of adding four counts
  EXPECT_TRUE(odometryReceiver._countOfOdometrysReceived == 4);
}
}
