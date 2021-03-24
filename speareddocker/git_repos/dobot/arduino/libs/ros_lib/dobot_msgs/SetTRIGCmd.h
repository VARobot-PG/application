#ifndef _ROS_SERVICE_SetTRIGCmd_h
#define _ROS_SERVICE_SetTRIGCmd_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dobot_msgs
{

static const char SETTRIGCMD[] = "dobot_msgs/SetTRIGCmd";

  class SetTRIGCmdRequest : public ros::Msg
  {
    public:
      typedef uint8_t _address_type;
      _address_type address;
      typedef uint8_t _mode_type;
      _mode_type mode;
      typedef uint8_t _condition_type;
      _condition_type condition;
      typedef uint16_t _threshold_type;
      _threshold_type threshold;

    SetTRIGCmdRequest():
      address(0),
      mode(0),
      condition(0),
      threshold(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->address >> (8 * 0)) & 0xFF;
      offset += sizeof(this->address);
      *(outbuffer + offset + 0) = (this->mode >> (8 * 0)) & 0xFF;
      offset += sizeof(this->mode);
      *(outbuffer + offset + 0) = (this->condition >> (8 * 0)) & 0xFF;
      offset += sizeof(this->condition);
      *(outbuffer + offset + 0) = (this->threshold >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->threshold >> (8 * 1)) & 0xFF;
      offset += sizeof(this->threshold);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->address =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->address);
      this->mode =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->mode);
      this->condition =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->condition);
      this->threshold =  ((uint16_t) (*(inbuffer + offset)));
      this->threshold |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->threshold);
     return offset;
    }

    const char * getType(){ return SETTRIGCMD; };
    const char * getMD5(){ return "f69ad7dea317db8be9d35dae66b39246"; };

  };

  class SetTRIGCmdResponse : public ros::Msg
  {
    public:
      typedef int32_t _result_type;
      _result_type result;
      typedef uint64_t _queuedCmdIndex_type;
      _queuedCmdIndex_type queuedCmdIndex;

    SetTRIGCmdResponse():
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

    const char * getType(){ return SETTRIGCMD; };
    const char * getMD5(){ return "cbf7b461449133eb5dd6ebbd8d38dedc"; };

  };

  class SetTRIGCmd {
    public:
    typedef SetTRIGCmdRequest Request;
    typedef SetTRIGCmdResponse Response;
  };

}
#endif
