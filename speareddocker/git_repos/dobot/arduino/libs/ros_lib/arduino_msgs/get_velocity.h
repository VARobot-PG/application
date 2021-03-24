#ifndef _ROS_arduino_msgs_get_velocity_h
#define _ROS_arduino_msgs_get_velocity_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace arduino_msgs
{

  class get_velocity : public ros::Msg
  {
    public:
      typedef int8_t _velocity_type;
      _velocity_type velocity;

    get_velocity():
      velocity(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_velocity;
      u_velocity.real = this->velocity;
      *(outbuffer + offset + 0) = (u_velocity.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->velocity);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_velocity;
      u_velocity.base = 0;
      u_velocity.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->velocity = u_velocity.real;
      offset += sizeof(this->velocity);
     return offset;
    }

    const char * getType(){ return "arduino_msgs/get_velocity"; };
    const char * getMD5(){ return "3dafcaf7789e3ecdfbe8f6970942cec9"; };

  };

}
#endif