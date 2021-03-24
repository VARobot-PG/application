#ifndef _ROS_SERVICE_GetARCParams_h
#define _ROS_SERVICE_GetARCParams_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dobot_msgs
{

static const char GETARCPARAMS[] = "dobot_msgs/GetARCParams";

  class GetARCParamsRequest : public ros::Msg
  {
    public:

    GetARCParamsRequest()
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

    const char * getType(){ return GETARCPARAMS; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class GetARCParamsResponse : public ros::Msg
  {
    public:
      typedef int32_t _result_type;
      _result_type result;
      typedef float _xyzVelocity_type;
      _xyzVelocity_type xyzVelocity;
      typedef float _rVelocity_type;
      _rVelocity_type rVelocity;
      typedef float _xyzAcceleration_type;
      _xyzAcceleration_type xyzAcceleration;
      typedef float _rAcceleration_type;
      _rAcceleration_type rAcceleration;

    GetARCParamsResponse():
      result(0),
      xyzVelocity(0),
      rVelocity(0),
      xyzAcceleration(0),
      rAcceleration(0)
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
      } u_xyzVelocity;
      u_xyzVelocity.real = this->xyzVelocity;
      *(outbuffer + offset + 0) = (u_xyzVelocity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_xyzVelocity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_xyzVelocity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_xyzVelocity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->xyzVelocity);
      union {
        float real;
        uint32_t base;
      } u_rVelocity;
      u_rVelocity.real = this->rVelocity;
      *(outbuffer + offset + 0) = (u_rVelocity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rVelocity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rVelocity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rVelocity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rVelocity);
      union {
        float real;
        uint32_t base;
      } u_xyzAcceleration;
      u_xyzAcceleration.real = this->xyzAcceleration;
      *(outbuffer + offset + 0) = (u_xyzAcceleration.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_xyzAcceleration.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_xyzAcceleration.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_xyzAcceleration.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->xyzAcceleration);
      union {
        float real;
        uint32_t base;
      } u_rAcceleration;
      u_rAcceleration.real = this->rAcceleration;
      *(outbuffer + offset + 0) = (u_rAcceleration.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rAcceleration.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rAcceleration.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rAcceleration.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rAcceleration);
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
      } u_xyzVelocity;
      u_xyzVelocity.base = 0;
      u_xyzVelocity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_xyzVelocity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_xyzVelocity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_xyzVelocity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->xyzVelocity = u_xyzVelocity.real;
      offset += sizeof(this->xyzVelocity);
      union {
        float real;
        uint32_t base;
      } u_rVelocity;
      u_rVelocity.base = 0;
      u_rVelocity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rVelocity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rVelocity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rVelocity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->rVelocity = u_rVelocity.real;
      offset += sizeof(this->rVelocity);
      union {
        float real;
        uint32_t base;
      } u_xyzAcceleration;
      u_xyzAcceleration.base = 0;
      u_xyzAcceleration.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_xyzAcceleration.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_xyzAcceleration.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_xyzAcceleration.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->xyzAcceleration = u_xyzAcceleration.real;
      offset += sizeof(this->xyzAcceleration);
      union {
        float real;
        uint32_t base;
      } u_rAcceleration;
      u_rAcceleration.base = 0;
      u_rAcceleration.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rAcceleration.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rAcceleration.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rAcceleration.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->rAcceleration = u_rAcceleration.real;
      offset += sizeof(this->rAcceleration);
     return offset;
    }

    const char * getType(){ return GETARCPARAMS; };
    const char * getMD5(){ return "886db9a7d126004f299ba6b6878cb966"; };

  };

  class GetARCParams {
    public:
    typedef GetARCParamsRequest Request;
    typedef GetARCParamsResponse Response;
  };

}
#endif
