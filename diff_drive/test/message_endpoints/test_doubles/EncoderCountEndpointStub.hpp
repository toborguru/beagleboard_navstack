// EncoderCountEndpointStub.hpp
 
#ifndef GUARD_EncoderCountEndpointStub
#define GUARD_EncoderCountEndpointStub
 
#include "IEncoderCountEndpoint.hpp"
 
namespace diff_drive_test_message_endpoints_test_doubles
{
  class EncoderCountEndpointStub : public diff_drive_core::IEncoderCountEndpoint
  { 
    public:
      EncoderCountEndpointStub()
        : countOfEncoderCountsPublished(0) { }

      diff_drive_core::EncoderCount_t RequestEncoderCounts() { 
        diff_drive_core::EncoderCount_t counts;

        countOfEncoderCountsPublished++;

        counts.left_count = countOfEncoderCountsPublished;
        counts.right_count -= countOfEncoderCountsPublished;
 
        // Output the laser scan seq number to the terminal; this isn't the
        // unit test, but merely a helpful means to show what's going on.
        std::cout << "EncoderCount endpoint stub sent to EncoderCountReceiver with an L, R of: " 
                  << counts.left_count << ", " << counts.right_count << std::endl;

        return counts;
      };
 
      // Extend IEncoderCountEndpoint for unit testing needs.
      // May be modified by const functions but maintains logical constness [Meyers, 2005, Item 3].
      mutable int countOfEncoderCountsPublished;
  };
}
 
#endif /* GUARD_EncoderCountEndpointStub */
