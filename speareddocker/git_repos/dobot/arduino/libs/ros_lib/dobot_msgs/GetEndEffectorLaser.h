#ifndef _ROS_SERVICE_GetEndEffectorLaser_h
#define _ROS_SERVICE_GetEndEffectorLaser_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dobot_msgs
{

static const char GETENDEFFECTORLASER[] = "dobot_msgs/GetEndEffectorLaser";

  class GetEndEffectorLaserRequest : public ros::Msg
  {
    public:

    GetEndEffectorLaserRequest()
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

    const char * getType(){ return GETENDEFFECTORLASER; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class GetEndEffectorLaserResponse : public ros::Msg
  {
    public:
      typedef int32_t _result_type;
      _result_type result;
      typedef uint8_t _enableCtrl_type;
      _enableCtrl_type enableCtrl;
      typedef uint8_t _on_type;
      _on_type on;

    GetEndEffectorLaserResponse():
      result(0),
      enableCtrl(0),
      on(0)
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
      *(outbuffer + offset + 0) = (this->enableCtrl >> (8 * 0)) & 0xFF;
      offset += sizeof(this->enableCtrl);
      *(outbuffer + offset + 0) = (this->on >> (8 * 0)) & 0xFF;
      offset += sizeof(this->on);
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
      this->enableCtrl =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->enableCtrl);
      this->on =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->on);
     return offset;
    }

    const char * getType(){ return GETENDEFFECTORLASER; };
    const char * getMD5(){ return "a8f296ec06b91031fd6d56c18b2ea1e9"; };

  };

  class GetEndEffectorLaser {
    public:
    typedef GetEndEffectorLaserRequest Request;
    typedef GetEndEffectorLaserResponse Response;
  };

}
#endif
