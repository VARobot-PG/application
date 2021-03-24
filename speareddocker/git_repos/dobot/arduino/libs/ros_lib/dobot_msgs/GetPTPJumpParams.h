#ifndef _ROS_SERVICE_GetPTPJumpParams_h
#define _ROS_SERVICE_GetPTPJumpParams_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dobot_msgs
{

static const char GETPTPJUMPPARAMS[] = "dobot_msgs/GetPTPJumpParams";

  class GetPTPJumpParamsRequest : public ros::Msg
  {
    public:

    GetPTPJumpParamsRequest()
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

    const char * getType(){ return GETPTPJUMPPARAMS; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class GetPTPJumpParamsResponse : public ros::Msg
  {
    public:
      typedef int32_t _result_type;
      _result_type result;
      typedef float _jumpHeight_type;
      _jumpHeight_type jumpHeight;
      typedef float _zLimit_type;
      _zLimit_type zLimit;

    GetPTPJumpParamsResponse():
      result(0),
      jumpHeight(0),
      zLimit(0)
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
      } u_jumpHeight;
      u_jumpHeight.real = this->jumpHeight;
      *(outbuffer + offset + 0) = (u_jumpHeight.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_jumpHeight.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_jumpHeight.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_jumpHeight.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->jumpHeight);
      union {
        float real;
        uint32_t base;
      } u_zLimit;
      u_zLimit.real = this->zLimit;
      *(outbuffer + offset + 0) = (u_zLimit.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_zLimit.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_zLimit.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_zLimit.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->zLimit);
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
      } u_jumpHeight;
      u_jumpHeight.base = 0;
      u_jumpHeight.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_jumpHeight.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_jumpHeight.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_jumpHeight.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->jumpHeight = u_jumpHeight.real;
      offset += sizeof(this->jumpHeight);
      union {
        float real;
        uint32_t base;
      } u_zLimit;
      u_zLimit.base = 0;
      u_zLimit.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_zLimit.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_zLimit.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_zLimit.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->zLimit = u_zLimit.real;
      offset += sizeof(this->zLimit);
     return offset;
    }

    const char * getType(){ return GETPTPJUMPPARAMS; };
    const char * getMD5(){ return "c789ca108051f9c81e48ec0ed44d3ab7"; };

  };

  class GetPTPJumpParams {
    public:
    typedef GetPTPJumpParamsRequest Request;
    typedef GetPTPJumpParamsResponse Response;
  };

}
#endif
