#ifndef _ROS_SERVICE_SetPTPJumpParams_h
#define _ROS_SERVICE_SetPTPJumpParams_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dobot_msgs
{

static const char SETPTPJUMPPARAMS[] = "dobot_msgs/SetPTPJumpParams";

  class SetPTPJumpParamsRequest : public ros::Msg
  {
    public:
      typedef float _jumpHeight_type;
      _jumpHeight_type jumpHeight;
      typedef float _zLimit_type;
      _zLimit_type zLimit;
      typedef bool _isQueued_type;
      _isQueued_type isQueued;

    SetPTPJumpParamsRequest():
      jumpHeight(0),
      zLimit(0),
      isQueued(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
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

    const char * getType(){ return SETPTPJUMPPARAMS; };
    const char * getMD5(){ return "c706ca9844b16def2443e216338d8d27"; };

  };

  class SetPTPJumpParamsResponse : public ros::Msg
  {
    public:
      typedef int32_t _result_type;
      _result_type result;
      typedef uint64_t _queuedCmdIndex_type;
      _queuedCmdIndex_type queuedCmdIndex;

    SetPTPJumpParamsResponse():
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

    const char * getType(){ return SETPTPJUMPPARAMS; };
    const char * getMD5(){ return "cbf7b461449133eb5dd6ebbd8d38dedc"; };

  };

  class SetPTPJumpParams {
    public:
    typedef SetPTPJumpParamsRequest Request;
    typedef SetPTPJumpParamsResponse Response;
  };

}
#endif
