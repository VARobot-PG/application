#ifndef _ROS_SERVICE_GetEndEffectorParams_h
#define _ROS_SERVICE_GetEndEffectorParams_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dobot_msgs
{

static const char GETENDEFFECTORPARAMS[] = "dobot_msgs/GetEndEffectorParams";

  class GetEndEffectorParamsRequest : public ros::Msg
  {
    public:

    GetEndEffectorParamsRequest()
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

    const char * getType(){ return GETENDEFFECTORPARAMS; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class GetEndEffectorParamsResponse : public ros::Msg
  {
    public:
      typedef int32_t _result_type;
      _result_type result;
      typedef float _xBias_type;
      _xBias_type xBias;
      typedef float _yBias_type;
      _yBias_type yBias;
      typedef float _zBias_type;
      _zBias_type zBias;

    GetEndEffectorParamsResponse():
      result(0),
      xBias(0),
      yBias(0),
      zBias(0)
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
      } u_xBias;
      u_xBias.real = this->xBias;
      *(outbuffer + offset + 0) = (u_xBias.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_xBias.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_xBias.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_xBias.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->xBias);
      union {
        float real;
        uint32_t base;
      } u_yBias;
      u_yBias.real = this->yBias;
      *(outbuffer + offset + 0) = (u_yBias.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_yBias.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_yBias.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_yBias.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->yBias);
      union {
        float real;
        uint32_t base;
      } u_zBias;
      u_zBias.real = this->zBias;
      *(outbuffer + offset + 0) = (u_zBias.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_zBias.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_zBias.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_zBias.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->zBias);
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
      } u_xBias;
      u_xBias.base = 0;
      u_xBias.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_xBias.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_xBias.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_xBias.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->xBias = u_xBias.real;
      offset += sizeof(this->xBias);
      union {
        float real;
        uint32_t base;
      } u_yBias;
      u_yBias.base = 0;
      u_yBias.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_yBias.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_yBias.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_yBias.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->yBias = u_yBias.real;
      offset += sizeof(this->yBias);
      union {
        float real;
        uint32_t base;
      } u_zBias;
      u_zBias.base = 0;
      u_zBias.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_zBias.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_zBias.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_zBias.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->zBias = u_zBias.real;
      offset += sizeof(this->zBias);
     return offset;
    }

    const char * getType(){ return GETENDEFFECTORPARAMS; };
    const char * getMD5(){ return "a07055f2964cf165e2fb447024254f9f"; };

  };

  class GetEndEffectorParams {
    public:
    typedef GetEndEffectorParamsRequest Request;
    typedef GetEndEffectorParamsResponse Response;
  };

}
#endif
