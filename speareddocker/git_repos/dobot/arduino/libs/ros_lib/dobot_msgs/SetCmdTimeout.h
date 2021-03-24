#ifndef _ROS_SERVICE_SetCmdTimeout_h
#define _ROS_SERVICE_SetCmdTimeout_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dobot_msgs
{

static const char SETCMDTIMEOUT[] = "dobot_msgs/SetCmdTimeout";

  class SetCmdTimeoutRequest : public ros::Msg
  {
    public:
      typedef uint32_t _timeout_type;
      _timeout_type timeout;

    SetCmdTimeoutRequest():
      timeout(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->timeout >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->timeout >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->timeout >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->timeout >> (8 * 3)) & 0xFF;
      offset += sizeof(this->timeout);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->timeout =  ((uint32_t) (*(inbuffer + offset)));
      this->timeout |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->timeout |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->timeout |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->timeout);
     return offset;
    }

    const char * getType(){ return SETCMDTIMEOUT; };
    const char * getMD5(){ return "971faa53b2e2e970811208c17e1880c5"; };

  };

  class SetCmdTimeoutResponse : public ros::Msg
  {
    public:
      typedef int32_t _result_type;
      _result_type result;

    SetCmdTimeoutResponse():
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

    const char * getType(){ return SETCMDTIMEOUT; };
    const char * getMD5(){ return "034a8e20d6a306665e3a5b340fab3f09"; };

  };

  class SetCmdTimeout {
    public:
    typedef SetCmdTimeoutRequest Request;
    typedef SetCmdTimeoutResponse Response;
  };

}
#endif
