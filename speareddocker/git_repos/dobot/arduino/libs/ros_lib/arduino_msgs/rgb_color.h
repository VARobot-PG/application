#ifndef _ROS_arduino_msgs_rgb_color_h
#define _ROS_arduino_msgs_rgb_color_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace arduino_msgs
{

  class rgb_color : public ros::Msg
  {
    public:
      typedef uint8_t _r_type;
      _r_type r;
      typedef uint8_t _g_type;
      _g_type g;
      typedef uint8_t _b_type;
      _b_type b;
      typedef uint16_t _c_type;
      _c_type c;

    rgb_color():
      r(0),
      g(0),
      b(0),
      c(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->r >> (8 * 0)) & 0xFF;
      offset += sizeof(this->r);
      *(outbuffer + offset + 0) = (this->g >> (8 * 0)) & 0xFF;
      offset += sizeof(this->g);
      *(outbuffer + offset + 0) = (this->b >> (8 * 0)) & 0xFF;
      offset += sizeof(this->b);
      *(outbuffer + offset + 0) = (this->c >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->c >> (8 * 1)) & 0xFF;
      offset += sizeof(this->c);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->r =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->r);
      this->g =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->g);
      this->b =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->b);
      this->c =  ((uint16_t) (*(inbuffer + offset)));
      this->c |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->c);
     return offset;
    }

    const char * getType(){ return "arduino_msgs/rgb_color"; };
    const char * getMD5(){ return "6ee6e880a482498932b984bababcbe84"; };

  };

}
#endif