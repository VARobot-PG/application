#ifndef _ROS_SERVICE_rfiduid_h
#define _ROS_SERVICE_rfiduid_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace arduino_msgs
{

static const char RFIDUID[] = "arduino_msgs/rfiduid";

  class rfiduidRequest : public ros::Msg
  {
    public:

    rfiduidRequest()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return RFIDUID; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class rfiduidResponse : public ros::Msg
  {
    public:
      typedef uint8_t _b0_type;
      _b0_type b0;
      typedef uint8_t _b1_type;
      _b1_type b1;
      typedef uint8_t _b2_type;
      _b2_type b2;
      typedef uint8_t _b3_type;
      _b3_type b3;

    rfiduidResponse():
      b0(0),
      b1(0),
      b2(0),
      b3(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->b0 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->b0);
      *(outbuffer + offset + 0) = (this->b1 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->b1);
      *(outbuffer + offset + 0) = (this->b2 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->b2);
      *(outbuffer + offset + 0) = (this->b3 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->b3);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->b0 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->b0);
      this->b1 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->b1);
      this->b2 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->b2);
      this->b3 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->b3);
     return offset;
    }

    const char * getType(){ return RFIDUID; };
    const char * getMD5(){ return "9190170cf9d2886eca90f9741927c5a3"; };

  };

  class rfiduid {
    public:
    typedef rfiduidRequest Request;
    typedef rfiduidResponse Response;
  };

}
#endif
