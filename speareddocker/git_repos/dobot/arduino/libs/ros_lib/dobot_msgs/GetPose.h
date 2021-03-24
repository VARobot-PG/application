#ifndef _ROS_SERVICE_GetPose_h
#define _ROS_SERVICE_GetPose_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dobot_msgs
{

static const char GETPOSE[] = "dobot_msgs/GetPose";

  class GetPoseRequest : public ros::Msg
  {
    public:

    GetPoseRequest()
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

    const char * getType(){ return GETPOSE; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class GetPoseResponse : public ros::Msg
  {
    public:
      typedef int32_t _result_type;
      _result_type result;
      typedef float _x_type;
      _x_type x;
      typedef float _y_type;
      _y_type y;
      typedef float _z_type;
      _z_type z;
      typedef float _r_type;
      _r_type r;
      uint32_t jointAngle_length;
      typedef float _jointAngle_type;
      _jointAngle_type st_jointAngle;
      _jointAngle_type * jointAngle;

    GetPoseResponse():
      result(0),
      x(0),
      y(0),
      z(0),
      r(0),
      jointAngle_length(0), jointAngle(NULL)
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
      } u_x;
      u_x.real = this->x;
      *(outbuffer + offset + 0) = (u_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->x);
      union {
        float real;
        uint32_t base;
      } u_y;
      u_y.real = this->y;
      *(outbuffer + offset + 0) = (u_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->y);
      union {
        float real;
        uint32_t base;
      } u_z;
      u_z.real = this->z;
      *(outbuffer + offset + 0) = (u_z.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_z.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_z.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_z.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->z);
      union {
        float real;
        uint32_t base;
      } u_r;
      u_r.real = this->r;
      *(outbuffer + offset + 0) = (u_r.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_r.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_r.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_r.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->r);
      *(outbuffer + offset + 0) = (this->jointAngle_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->jointAngle_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->jointAngle_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->jointAngle_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->jointAngle_length);
      for( uint32_t i = 0; i < jointAngle_length; i++){
      union {
        float real;
        uint32_t base;
      } u_jointAnglei;
      u_jointAnglei.real = this->jointAngle[i];
      *(outbuffer + offset + 0) = (u_jointAnglei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_jointAnglei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_jointAnglei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_jointAnglei.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->jointAngle[i]);
      }
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
      } u_x;
      u_x.base = 0;
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->x = u_x.real;
      offset += sizeof(this->x);
      union {
        float real;
        uint32_t base;
      } u_y;
      u_y.base = 0;
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->y = u_y.real;
      offset += sizeof(this->y);
      union {
        float real;
        uint32_t base;
      } u_z;
      u_z.base = 0;
      u_z.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_z.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_z.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_z.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->z = u_z.real;
      offset += sizeof(this->z);
      union {
        float real;
        uint32_t base;
      } u_r;
      u_r.base = 0;
      u_r.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_r.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_r.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_r.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->r = u_r.real;
      offset += sizeof(this->r);
      uint32_t jointAngle_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      jointAngle_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      jointAngle_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      jointAngle_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->jointAngle_length);
      if(jointAngle_lengthT > jointAngle_length)
        this->jointAngle = (float*)realloc(this->jointAngle, jointAngle_lengthT * sizeof(float));
      jointAngle_length = jointAngle_lengthT;
      for( uint32_t i = 0; i < jointAngle_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_jointAngle;
      u_st_jointAngle.base = 0;
      u_st_jointAngle.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_jointAngle.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_jointAngle.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_jointAngle.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_jointAngle = u_st_jointAngle.real;
      offset += sizeof(this->st_jointAngle);
        memcpy( &(this->jointAngle[i]), &(this->st_jointAngle), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return GETPOSE; };
    const char * getMD5(){ return "d4de75e47bb9a51cbb33e89d50edf04f"; };

  };

  class GetPose {
    public:
    typedef GetPoseRequest Request;
    typedef GetPoseResponse Response;
  };

}
#endif
