#ifndef _ROS_SERVICE_GetPoseL_h
#define _ROS_SERVICE_GetPoseL_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dobot_msgs
{

static const char GETPOSEL[] = "dobot_msgs/GetPoseL";

  class GetPoseLRequest : public ros::Msg
  {
    public:

    GetPoseLRequest()
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

    const char * getType(){ return GETPOSEL; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class GetPoseLResponse : public ros::Msg
  {
    public:
      typedef int32_t _result_type;
      _result_type result;
      typedef float _l_type;
      _l_type l;

    GetPoseLResponse():
      result(0),
      l(0)
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
      } u_l;
      u_l.real = this->l;
      *(outbuffer + offset + 0) = (u_l.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_l.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_l.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_l.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->l);
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
      } u_l;
      u_l.base = 0;
      u_l.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_l.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_l.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_l.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->l = u_l.real;
      offset += sizeof(this->l);
     return offset;
    }

    const char * getType(){ return GETPOSEL; };
    const char * getMD5(){ return "8ea0ec2119828b8ffb2dfc7a5b8bf786"; };

  };

  class GetPoseL {
    public:
    typedef GetPoseLRequest Request;
    typedef GetPoseLResponse Response;
  };

}
#endif
