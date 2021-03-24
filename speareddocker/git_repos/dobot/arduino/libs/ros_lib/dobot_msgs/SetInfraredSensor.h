#ifndef _ROS_SERVICE_SetInfraredSensor_h
#define _ROS_SERVICE_SetInfraredSensor_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dobot_msgs
{

static const char SETINFRAREDSENSOR[] = "dobot_msgs/SetInfraredSensor";

  class SetInfraredSensorRequest : public ros::Msg
  {
    public:
      typedef bool _enable_type;
      _enable_type enable;
      typedef uint8_t _infraredPort_type;
      _infraredPort_type infraredPort;

    SetInfraredSensorRequest():
      enable(0),
      infraredPort(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_enable;
      u_enable.real = this->enable;
      *(outbuffer + offset + 0) = (u_enable.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->enable);
      *(outbuffer + offset + 0) = (this->infraredPort >> (8 * 0)) & 0xFF;
      offset += sizeof(this->infraredPort);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_enable;
      u_enable.base = 0;
      u_enable.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->enable = u_enable.real;
      offset += sizeof(this->enable);
      this->infraredPort =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->infraredPort);
     return offset;
    }

    const char * getType(){ return SETINFRAREDSENSOR; };
    const char * getMD5(){ return "e5efc496c951f4fefcbb56345c0e5446"; };

  };

  class SetInfraredSensorResponse : public ros::Msg
  {
    public:
      typedef int32_t _result_type;
      _result_type result;

    SetInfraredSensorResponse():
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

    const char * getType(){ return SETINFRAREDSENSOR; };
    const char * getMD5(){ return "034a8e20d6a306665e3a5b340fab3f09"; };

  };

  class SetInfraredSensor {
    public:
    typedef SetInfraredSensorRequest Request;
    typedef SetInfraredSensorResponse Response;
  };

}
#endif
