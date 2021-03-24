#ifndef _ROS_SERVICE_GetIOPWM_h
#define _ROS_SERVICE_GetIOPWM_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dobot_msgs
{

static const char GETIOPWM[] = "dobot_msgs/GetIOPWM";

  class GetIOPWMRequest : public ros::Msg
  {
    public:
      typedef uint8_t _address_type;
      _address_type address;

    GetIOPWMRequest():
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

    const char * getType(){ return GETIOPWM; };
    const char * getMD5(){ return "972132462544b1029bf37f19a88e11c4"; };

  };

  class GetIOPWMResponse : public ros::Msg
  {
    public:
      typedef int32_t _result_type;
      _result_type result;
      typedef float _frequency_type;
      _frequency_type frequency;
      typedef float _dutyCycle_type;
      _dutyCycle_type dutyCycle;

    GetIOPWMResponse():
      result(0),
      frequency(0),
      dutyCycle(0)
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
      union {
        float real;
        uint32_t base;
      } u_frequency;
      u_frequency.real = this->frequency;
      *(outbuffer + offset + 0) = (u_frequency.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_frequency.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_frequency.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_frequency.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->frequency);
      union {
        float real;
        uint32_t base;
      } u_dutyCycle;
      u_dutyCycle.real = this->dutyCycle;
      *(outbuffer + offset + 0) = (u_dutyCycle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dutyCycle.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_dutyCycle.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_dutyCycle.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dutyCycle);
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
      union {
        float real;
        uint32_t base;
      } u_frequency;
      u_frequency.base = 0;
      u_frequency.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_frequency.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_frequency.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_frequency.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->frequency = u_frequency.real;
      offset += sizeof(this->frequency);
      union {
        float real;
        uint32_t base;
      } u_dutyCycle;
      u_dutyCycle.base = 0;
      u_dutyCycle.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dutyCycle.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_dutyCycle.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_dutyCycle.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->dutyCycle = u_dutyCycle.real;
      offset += sizeof(this->dutyCycle);
     return offset;
    }

    const char * getType(){ return GETIOPWM; };
    const char * getMD5(){ return "17a6ba166d1125f7c7225e13ea02a8e3"; };

  };

  class GetIOPWM {
    public:
    typedef GetIOPWMRequest Request;
    typedef GetIOPWMResponse Response;
  };

}
#endif
