#ifndef _ROS_SERVICE_GetCPParams_h
#define _ROS_SERVICE_GetCPParams_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dobot_msgs
{

static const char GETCPPARAMS[] = "dobot_msgs/GetCPParams";

  class GetCPParamsRequest : public ros::Msg
  {
    public:

    GetCPParamsRequest()
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

    const char * getType(){ return GETCPPARAMS; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class GetCPParamsResponse : public ros::Msg
  {
    public:
      typedef int32_t _result_type;
      _result_type result;
      typedef float _planAcc_type;
      _planAcc_type planAcc;
      typedef float _junctionVel_type;
      _junctionVel_type junctionVel;
      typedef float _acc_type;
      _acc_type acc;
      typedef uint8_t _realTimeTrack_type;
      _realTimeTrack_type realTimeTrack;

    GetCPParamsResponse():
      result(0),
      planAcc(0),
      junctionVel(0),
      acc(0),
      realTimeTrack(0)
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
      } u_planAcc;
      u_planAcc.real = this->planAcc;
      *(outbuffer + offset + 0) = (u_planAcc.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_planAcc.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_planAcc.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_planAcc.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->planAcc);
      union {
        float real;
        uint32_t base;
      } u_junctionVel;
      u_junctionVel.real = this->junctionVel;
      *(outbuffer + offset + 0) = (u_junctionVel.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_junctionVel.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_junctionVel.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_junctionVel.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->junctionVel);
      union {
        float real;
        uint32_t base;
      } u_acc;
      u_acc.real = this->acc;
      *(outbuffer + offset + 0) = (u_acc.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_acc.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_acc.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_acc.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->acc);
      *(outbuffer + offset + 0) = (this->realTimeTrack >> (8 * 0)) & 0xFF;
      offset += sizeof(this->realTimeTrack);
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
      } u_planAcc;
      u_planAcc.base = 0;
      u_planAcc.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_planAcc.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_planAcc.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_planAcc.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->planAcc = u_planAcc.real;
      offset += sizeof(this->planAcc);
      union {
        float real;
        uint32_t base;
      } u_junctionVel;
      u_junctionVel.base = 0;
      u_junctionVel.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_junctionVel.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_junctionVel.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_junctionVel.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->junctionVel = u_junctionVel.real;
      offset += sizeof(this->junctionVel);
      union {
        float real;
        uint32_t base;
      } u_acc;
      u_acc.base = 0;
      u_acc.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_acc.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_acc.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_acc.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->acc = u_acc.real;
      offset += sizeof(this->acc);
      this->realTimeTrack =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->realTimeTrack);
     return offset;
    }

    const char * getType(){ return GETCPPARAMS; };
    const char * getMD5(){ return "85dff81a44afbb98d15e48705b4ea806"; };

  };

  class GetCPParams {
    public:
    typedef GetCPParamsRequest Request;
    typedef GetCPParamsResponse Response;
  };

}
#endif
