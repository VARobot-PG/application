#ifndef _ROS_SERVICE_SetConveyorbeltSpeed_h
#define _ROS_SERVICE_SetConveyorbeltSpeed_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace arduino_msgs
{

static const char SETCONVEYORBELTSPEED[] = "arduino_msgs/SetConveyorbeltSpeed";

  class SetConveyorbeltSpeedRequest : public ros::Msg
  {
    public:
      typedef int8_t _speed_type;
      _speed_type speed;

    SetConveyorbeltSpeedRequest():
      speed(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_speed;
      u_speed.real = this->speed;
      *(outbuffer + offset + 0) = (u_speed.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->speed);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_speed;
      u_speed.base = 0;
      u_speed.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->speed = u_speed.real;
      offset += sizeof(this->speed);
     return offset;
    }

    const char * getType(){ return SETCONVEYORBELTSPEED; };
    const char * getMD5(){ return "1f1b16c0f00410da01ce6cbe511e4fcc"; };

  };

  class SetConveyorbeltSpeedResponse : public ros::Msg
  {
    public:
      typedef int32_t _result_type;
      _result_type result;

    SetConveyorbeltSpeedResponse():
      result(0)
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
     return offset;
    }

    const char * getType(){ return SETCONVEYORBELTSPEED; };
    const char * getMD5(){ return "034a8e20d6a306665e3a5b340fab3f09"; };

  };

  class SetConveyorbeltSpeed {
    public:
    typedef SetConveyorbeltSpeedRequest Request;
    typedef SetConveyorbeltSpeedResponse Response;
  };

}
#endif
