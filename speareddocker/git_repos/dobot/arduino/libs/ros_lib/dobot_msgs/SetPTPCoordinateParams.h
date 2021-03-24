#ifndef _ROS_SERVICE_SetPTPCoordinateParams_h
#define _ROS_SERVICE_SetPTPCoordinateParams_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dobot_msgs
{

static const char SETPTPCOORDINATEPARAMS[] = "dobot_msgs/SetPTPCoordinateParams";

  class SetPTPCoordinateParamsRequest : public ros::Msg
  {
    public:
      typedef float _xyzVelocity_type;
      _xyzVelocity_type xyzVelocity;
      typedef float _rVelocity_type;
      _rVelocity_type rVelocity;
      typedef float _xyzAcceleration_type;
      _xyzAcceleration_type xyzAcceleration;
      typedef float _rAcceleration_type;
      _rAcceleration_type rAcceleration;
      typedef bool _isQueued_type;
      _isQueued_type isQueued;

    SetPTPCoordinateParamsRequest():
      xyzVelocity(0),
      rVelocity(0),
      xyzAcceleration(0),
      rAcceleration(0),
      isQueued(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
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

    const char * getType(){ return SETPTPCOORDINATEPARAMS; };
    const char * getMD5(){ return "76117133666c08ba06a7aa5aebab19f5"; };

  };

  class SetPTPCoordinateParamsResponse : public ros::Msg
  {
    public:
      typedef int32_t _result_type;
      _result_type result;
      typedef uint64_t _queuedCmdIndex_type;
      _queuedCmdIndex_type queuedCmdIndex;

    SetPTPCoordinateParamsResponse():
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

    const char * getType(){ return SETPTPCOORDINATEPARAMS; };
    const char * getMD5(){ return "cbf7b461449133eb5dd6ebbd8d38dedc"; };

  };

  class SetPTPCoordinateParams {
    public:
    typedef SetPTPCoordinateParamsRequest Request;
    typedef SetPTPCoordinateParamsResponse Response;
  };

}
#endif
