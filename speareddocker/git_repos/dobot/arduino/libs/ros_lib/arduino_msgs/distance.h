#ifndef _ROS_arduino_msgs_distance_h
#define _ROS_arduino_msgs_distance_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace arduino_msgs
{

  class distance : public ros::Msg
  {
    public:
      typedef uint8_t _dist_type;
      _dist_type dist;
      typedef uint8_t _status_type;
      _status_type status;

    distance():
      dist(0),
      status(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->dist >> (8 * 0)) & 0xFF;
      offset += sizeof(this->dist);
      *(outbuffer + offset + 0) = (this->status >> (8 * 0)) & 0xFF;
      offset += sizeof(this->status);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->dist =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->dist);
      this->status =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->status);
     return offset;
    }

    const char * getType(){ return "arduino_msgs/distance"; };
    const char * getMD5(){ return "499b7bc372c08ba6b8e13b240e70712d"; };

  };

}
#endif