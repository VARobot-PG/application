#ifndef _ROS_SERVICE_GetEndEffectorGripper_h
#define _ROS_SERVICE_GetEndEffectorGripper_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dobot_msgs
{

static const char GETENDEFFECTORGRIPPER[] = "dobot_msgs/GetEndEffectorGripper";

  class GetEndEffectorGripperRequest : public ros::Msg
  {
    public:

    GetEndEffectorGripperRequest()
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

    const char * getType(){ return GETENDEFFECTORGRIPPER; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class GetEndEffectorGripperResponse : public ros::Msg
  {
    public:
      typedef int32_t _result_type;
      _result_type result;
      typedef uint8_t _enableCtrl_type;
      _enableCtrl_type enableCtrl;
      typedef uint8_t _grip_type;
      _grip_type grip;

    GetEndEffectorGripperResponse():
      result(0),
      enableCtrl(0),
      grip(0)
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
      *(outbuffer + offset + 0) = (this->grip >> (8 * 0)) & 0xFF;
      offset += sizeof(this->grip);
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
      this->grip =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->grip);
     return offset;
    }

    const char * getType(){ return GETENDEFFECTORGRIPPER; };
    const char * getMD5(){ return "4185902cfc52c9751c7305887881349c"; };

  };

  class GetEndEffectorGripper {
    public:
    typedef GetEndEffectorGripperRequest Request;
    typedef GetEndEffectorGripperResponse Response;
  };

}
#endif
