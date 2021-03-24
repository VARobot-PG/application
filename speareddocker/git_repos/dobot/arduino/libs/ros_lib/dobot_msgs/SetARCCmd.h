#ifndef _ROS_SERVICE_SetARCCmd_h
#define _ROS_SERVICE_SetARCCmd_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dobot_msgs
{

static const char SETARCCMD[] = "dobot_msgs/SetARCCmd";

  class SetARCCmdRequest : public ros::Msg
  {
    public:
      typedef float _x1_type;
      _x1_type x1;
      typedef float _y1_type;
      _y1_type y1;
      typedef float _z1_type;
      _z1_type z1;
      typedef float _r1_type;
      _r1_type r1;
      typedef float _x2_type;
      _x2_type x2;
      typedef float _y2_type;
      _y2_type y2;
      typedef float _z2_type;
      _z2_type z2;
      typedef float _r2_type;
      _r2_type r2;

    SetARCCmdRequest():
      x1(0),
      y1(0),
      z1(0),
      r1(0),
      x2(0),
      y2(0),
      z2(0),
      r2(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_x1;
      u_x1.real = this->x1;
      *(outbuffer + offset + 0) = (u_x1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_x1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_x1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_x1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->x1);
      union {
        float real;
        uint32_t base;
      } u_y1;
      u_y1.real = this->y1;
      *(outbuffer + offset + 0) = (u_y1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_y1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_y1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_y1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->y1);
      union {
        float real;
        uint32_t base;
      } u_z1;
      u_z1.real = this->z1;
      *(outbuffer + offset + 0) = (u_z1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_z1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_z1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_z1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->z1);
      union {
        float real;
        uint32_t base;
      } u_r1;
      u_r1.real = this->r1;
      *(outbuffer + offset + 0) = (u_r1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_r1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_r1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_r1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->r1);
      union {
        float real;
        uint32_t base;
      } u_x2;
      u_x2.real = this->x2;
      *(outbuffer + offset + 0) = (u_x2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_x2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_x2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_x2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->x2);
      union {
        float real;
        uint32_t base;
      } u_y2;
      u_y2.real = this->y2;
      *(outbuffer + offset + 0) = (u_y2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_y2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_y2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_y2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->y2);
      union {
        float real;
        uint32_t base;
      } u_z2;
      u_z2.real = this->z2;
      *(outbuffer + offset + 0) = (u_z2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_z2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_z2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_z2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->z2);
      union {
        float real;
        uint32_t base;
      } u_r2;
      u_r2.real = this->r2;
      *(outbuffer + offset + 0) = (u_r2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_r2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_r2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_r2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->r2);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_x1;
      u_x1.base = 0;
      u_x1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_x1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_x1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_x1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->x1 = u_x1.real;
      offset += sizeof(this->x1);
      union {
        float real;
        uint32_t base;
      } u_y1;
      u_y1.base = 0;
      u_y1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_y1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_y1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_y1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->y1 = u_y1.real;
      offset += sizeof(this->y1);
      union {
        float real;
        uint32_t base;
      } u_z1;
      u_z1.base = 0;
      u_z1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_z1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_z1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_z1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->z1 = u_z1.real;
      offset += sizeof(this->z1);
      union {
        float real;
        uint32_t base;
      } u_r1;
      u_r1.base = 0;
      u_r1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_r1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_r1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_r1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->r1 = u_r1.real;
      offset += sizeof(this->r1);
      union {
        float real;
        uint32_t base;
      } u_x2;
      u_x2.base = 0;
      u_x2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_x2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_x2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_x2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->x2 = u_x2.real;
      offset += sizeof(this->x2);
      union {
        float real;
        uint32_t base;
      } u_y2;
      u_y2.base = 0;
      u_y2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_y2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_y2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_y2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->y2 = u_y2.real;
      offset += sizeof(this->y2);
      union {
        float real;
        uint32_t base;
      } u_z2;
      u_z2.base = 0;
      u_z2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_z2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_z2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_z2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->z2 = u_z2.real;
      offset += sizeof(this->z2);
      union {
        float real;
        uint32_t base;
      } u_r2;
      u_r2.base = 0;
      u_r2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_r2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_r2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_r2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->r2 = u_r2.real;
      offset += sizeof(this->r2);
     return offset;
    }

    const char * getType(){ return SETARCCMD; };
    const char * getMD5(){ return "a21ff454d6d7cacb84b15fe1f9c70626"; };

  };

  class SetARCCmdResponse : public ros::Msg
  {
    public:
      typedef int32_t _result_type;
      _result_type result;
      typedef uint64_t _queuedCmdIndex_type;
      _queuedCmdIndex_type queuedCmdIndex;

    SetARCCmdResponse():
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

    const char * getType(){ return SETARCCMD; };
    const char * getMD5(){ return "cbf7b461449133eb5dd6ebbd8d38dedc"; };

  };

  class SetARCCmd {
    public:
    typedef SetARCCmdRequest Request;
    typedef SetARCCmdResponse Response;
  };

}
#endif
