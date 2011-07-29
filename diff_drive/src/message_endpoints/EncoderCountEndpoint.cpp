// EncoderCountEndpoint.cpp
 
#include <fcntl.h>
#include <ros/ros.h>

#include "EncoderCountEndpoint.hpp"
#include "i2c-api.h"
 
namespace diff_drive_message_endpoints
{
  EncoderCountEndpoint::EncoderCountEndpoint()
  {
  }
 
  EncoderCountEndpoint::~EncoderCountEndpoint()
  {
  }
 
  diff_drive_core::EncoderCount_t EncoderCountEndpoint::RequestEncoderCounts() 
  {
    diff_drive_core::EncoderCount_t counts;

    uint8_t addr = 4;
    uint8_t dataByte[ _MAX_DATA_LEN ];
    int     rc;
    int     byteCount = 8;

    if (( rc = I2cReadBytes( _i2cDev, addr, dataByte, byteCount )) != 0 )
    {
        printf("Error reading ");
        return counts;
    }

    counts.left_count = *(int16_t*)&dataByte[0];
    counts.right_count = *(int16_t*)&dataByte[4];

    ROS_INFO("Published encoderCount to encoderCount topic with L, R of: %d, %d", 
             counts.left_count, counts.right_count );

    return counts;
  };
}
