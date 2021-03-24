#ifndef _ROS_SERVICE_GetIOMultiplexing_h
#define _ROS_SERVICE_GetIOMultiplexing_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dobot_msgs
{

static const char GETIOMULTIPLEXING[] = "dobot_msgs/GetIOMultiplexing";

  class GetIOMultiplexingRequest : public ros::Msg
  {
    public:
      typedef uint8_t _address_type;
      _address_type address;

    GetIOMultiplexingRequest():
      address(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->address >> (8 * 0)) & 0xFF;
      offset += sizeof(this->address);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->address =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->address);
     return offset;
    }

    const char * getType(){ return GETIOMULTIPLEXING; };
    const char * getMD5(){ return "972132462544b1029bf37f19a88e11c4"; };

  };

  class GetIOMultiplexingResponse : public ros::Msg
  {
    public:
      typedef int32_t _result_type;
      _result_type result;
      typedef uint8_t _multiplex_type;
      _multiplex_type multiplex;

    GetIOMultiplexingResponse():
      result(0),
      multiplex(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_result;
      u_result.real = this->result;
      *(outbuffer + offset + 0) = (u_result.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_result.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_result.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_result.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->result);
      *(outbuffer + offset + 0) = (this->multiplex >> (8 * 0)) & 0xFF;
      offset += sizeof(this->multiplex);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_result;
      u_result.base = 0;
      u_result.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_result.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_result.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_result.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->result = u_result.real;
      offset += sizeof(this->result);
      this->multiplex =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->multiplex);
     return offset;
    }

    const char * getType(){ return GETIOMULTIPLEXING; };
    const char * getMD5(){ return "61bdcf53e1dc8779bd7c79b4dc9c2aa4"; };

  };

  class GetIOMultiplexing {
    public:
    typedef GetIOMultiplexingRequest Request;
    typedef GetIOMultiplexingResponse Response;
  };

}
#endif
