#ifndef _ROS_SERVICE_GetPTPCommonParams_h
#define _ROS_SERVICE_GetPTPCommonParams_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dobot_msgs
{

static const char GETPTPCOMMONPARAMS[] = "dobot_msgs/GetPTPCommonParams";

  class GetPTPCommonParamsRequest : public ros::Msg
  {
    public:

    GetPTPCommonParamsRequest()
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

    const char * getType(){ return GETPTPCOMMONPARAMS; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class GetPTPCommonParamsResponse : public ros::Msg
  {
    public:
      typedef int32_t _result_type;
      _result_type result;
      typedef float _velocityRatio_type;
      _velocityRatio_type velocityRatio;
      typedef float _accelerationRatio_type;
      _accelerationRatio_type accelerationRatio;

    GetPTPCommonParamsResponse():
      result(0),
      velocityRatio(0),
      accelerationRatio(0)
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
      } u_velocityRatio;
      u_velocityRatio.real = this->velocityRatio;
      *(outbuffer + offset + 0) = (u_velocityRatio.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_velocityRatio.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_velocityRatio.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_velocityRatio.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->velocityRatio);
      union {
        float real;
        uint32_t base;
      } u_accelerationRatio;
      u_accelerationRatio.real = this->accelerationRatio;
      *(outbuffer + offset + 0) = (u_accelerationRatio.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_accelerationRatio.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_accelerationRatio.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_accelerationRatio.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->accelerationRatio);
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
      } u_velocityRatio;
      u_velocityRatio.base = 0;
      u_velocityRatio.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_velocityRatio.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_velocityRatio.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_velocityRatio.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->velocityRatio = u_velocityRatio.real;
      offset += sizeof(this->velocityRatio);
      union {
        float real;
        uint32_t base;
      } u_accelerationRatio;
      u_accelerationRatio.base = 0;
      u_accelerationRatio.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_accelerationRatio.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_accelerationRatio.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_accelerationRatio.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->accelerationRatio = u_accelerationRatio.real;
      offset += sizeof(this->accelerationRatio);
     return offset;
    }

    const char * getType(){ return GETPTPCOMMONPARAMS; };
    const char * getMD5(){ return "ccb42558fae0625dedee5118e81add9a"; };

  };

  class GetPTPCommonParams {
    public:
    typedef GetPTPCommonParamsRequest Request;
    typedef GetPTPCommonParamsResponse Response;
  };

}
#endif
