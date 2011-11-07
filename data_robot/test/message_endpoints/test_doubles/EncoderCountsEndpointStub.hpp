// EncoderCountsEndpointStub.hpp
 
#ifndef GUARD_EncoderCountsEndpointStub
#define GUARD_EncoderCountsEndpointStub
 
#include "diff_drive/EncoderCounts.h"
#include "IEncoderCountsEndpoint.hpp"
 
namespace data_robot_test_message_endpoints_test_doubles
{
class EncoderCountsEndpointStub : public data_robot_core::IEncoderCountsEndpoint
{ 
public:
  EncoderCountsEndpointStub()
    : _count_of_encoder_counts_published(0),
      _left(0),
      _right(0)
  { }

  int _count_of_encoder_counts_published;
  int _left;
  int _right;

  void Publish(const diff_drive::EncoderCounts& encoder_counts)
  {
    _count_of_encoder_counts_published++;

    _left = encoder_counts.left_count;
    _right = encoder_counts.right_count;

#if 0
    // Output the read values to the terminal; this isn't the
    // unit test, but merely a helpful means to show what's going on.
    std::cout << "EncoderCounts published on EncoderCountsEndpoint with L: " 
              << _left
              << ", R: "
              << _right
              << std::endl;
#endif
  }
};
}
 
#endif /* GUARD_EncoderCountsEndpointStub */
