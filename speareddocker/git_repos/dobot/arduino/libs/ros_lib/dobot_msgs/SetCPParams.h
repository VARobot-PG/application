#ifndef _ROS_SERVICE_SetCPParams_h
#define _ROS_SERVICE_SetCPParams_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dobot_msgs
{

static const char SETCPPARAMS[] = "dobot_msgs/SetCPParams";

  class SetCPParamsRequest : public ros::Msg
  {
    public:
      typedef float _planAcc_type;
      _planAcc_type planAcc;
      typedef float _junctionVel_type;
      _junctionVel_type junctionVel;
      typedef float _acc_type;
      _acc_type acc;
      typedef uint8_t _realTimeTrack_type;
      _realTimeTrack_type realTimeTrack;
      typedef bool _isQueued_type;
      _isQueued_type isQueued;

    SetCPParamsRequest():
      planAcc(0),
      junctionVel(0),
      acc(0),
      realTimeTrack(0),
      isQueued(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
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
      union {
        bool real;
        uint8_t base;
      } u_isQueued;
      u_isQueued.real = this->isQueued;
      *(outbuffer + offset + 0) = (u_isQueued.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->isQueued);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
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
      union {
        bool real;
        uint8_t base;
      } u_isQueued;
      u_isQueued.base = 0;
      u_isQueued.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->isQueued = u_isQueued.real;
      offset += sizeof(this->isQueued);
     return offset;
    }

    const char * getType(){ return SETCPPARAMS; };
    const char * getMD5(){ return "554ea4f3eb746a3d6db4a5ca9e210a01"; };

  };

  class SetCPParamsResponse : public ros::Msg
  {
    public:
      typedef int32_t _result_type;
      _result_type result;
      typedef uint64_t _queuedCmdIndex_type;
      _queuedCmdIndex_type queuedCmdIndex;

    SetCPParamsResponse():
      result(0),
      queuedCmdIndex(0)
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
        uint64_t real;
        uint32_t base;
      } u_queuedCmdIndex;
      u_queuedCmdIndex.real = this->queuedCmdIndex;
      *(outbuffer + offset + 0) = (u_queuedCmdIndex.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_queuedCmdIndex.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_queuedCmdIndex.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_queuedCmdIndex.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->queuedCmdIndex);
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
        uint64_t real;
        uint32_t base;
      } u_queuedCmdIndex;
      u_queuedCmdIndex.base = 0;
      u_queuedCmdIndex.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_queuedCmdIndex.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_queuedCmdIndex.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_queuedCmdIndex.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->queuedCmdIndex = u_queuedCmdIndex.real;
      offset += sizeof(this->queuedCmdIndex);
     return offset;
    }

    const char * getType(){ return SETCPPARAMS; };
    const char * getMD5(){ return "cbf7b461449133eb5dd6ebbd8d38dedc"; };

  };

  class SetCPParams {
    public:
    typedef SetCPParamsRequest Request;
    typedef SetCPParamsResponse Response;
  };

}
#endif
