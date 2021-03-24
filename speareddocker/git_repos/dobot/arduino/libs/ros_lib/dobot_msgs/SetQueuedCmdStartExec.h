#ifndef _ROS_SERVICE_SetQueuedCmdStartExec_h
#define _ROS_SERVICE_SetQueuedCmdStartExec_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dobot_msgs
{

static const char SETQUEUEDCMDSTARTEXEC[] = "dobot_msgs/SetQueuedCmdStartExec";

  class SetQueuedCmdStartExecRequest : public ros::Msg
  {
    public:

    SetQueuedCmdStartExecRequest()
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

    const char * getType(){ return SETQUEUEDCMDSTARTEXEC; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class SetQueuedCmdStartExecResponse : public ros::Msg
  {
    public:
      typedef int32_t _result_type;
      _result_type result;

    SetQueuedCmdStartExecResponse():
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

    const char * getType(){ return SETQUEUEDCMDSTARTEXEC; };
    const char * getMD5(){ return "034a8e20d6a306665e3a5b340fab3f09"; };

  };

  class SetQueuedCmdStartExec {
    public:
    typedef SetQueuedCmdStartExecRequest Request;
    typedef SetQueuedCmdStartExecResponse Response;
  };

}
#endif
