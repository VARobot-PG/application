#ifndef _ROS_SERVICE_AddFrame_h
#define _ROS_SERVICE_AddFrame_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace detection_msgs
{

static const char ADDFRAME[] = "detection_msgs/AddFrame";

  class AddFrameRequest : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;
      typedef float _x_type;
      _x_type x;
      typedef float _y_type;
      _y_type y;
      typedef float _z_type;
      _z_type z;
      typedef bool _useXYZ_type;
      _useXYZ_type useXYZ;
      typedef float _roll_type;
      _roll_type roll;
      typedef float _pitch_type;
      _pitch_type pitch;
      typedef float _yaw_type;
      _yaw_type yaw;
      typedef bool _useRotation_type;
      _useRotation_type useRotation;

    AddFrameRequest():
      name(""),
      x(0),
      y(0),
      z(0),
      useXYZ(0),
      roll(0),
      pitch(0),
      yaw(0),
      useRotation(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_name = strlen(this->name);
      varToArr(outbuffer + offset, length_name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      offset += serializeAvrFloat64(outbuffer + offset, this->x);
      offset += serializeAvrFloat64(outbuffer + offset, this->y);
      offset += serializeAvrFloat64(outbuffer + offset, this->z);
      union {
        bool real;
        uint8_t base;
      } u_useXYZ;
      u_useXYZ.real = this->useXYZ;
      *(outbuffer + offset + 0) = (u_useXYZ.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->useXYZ);
      offset += serializeAvrFloat64(outbuffer + offset, this->roll);
      offset += serializeAvrFloat64(outbuffer + offset, this->pitch);
      offset += serializeAvrFloat64(outbuffer + offset, this->yaw);
      union {
        bool real;
        uint8_t base;
      } u_useRotation;
      u_useRotation.real = this->useRotation;
      *(outbuffer + offset + 0) = (u_useRotation.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->useRotation);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_name;
      arrToVar(length_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->x));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->y));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->z));
      union {
        bool real;
        uint8_t base;
      } u_useXYZ;
      u_useXYZ.base = 0;
      u_useXYZ.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->useXYZ = u_useXYZ.real;
      offset += sizeof(this->useXYZ);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->roll));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->pitch));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->yaw));
      union {
        bool real;
        uint8_t base;
      } u_useRotation;
      u_useRotation.base = 0;
      u_useRotation.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->useRotation = u_useRotation.real;
      offset += sizeof(this->useRotation);
     return offset;
    }

    const char * getType(){ return ADDFRAME; };
    const char * getMD5(){ return "d63c21054b3b1e0b1024b94670463b46"; };

  };

  class AddFrameResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    AddFrameResponse():
      success(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
     return offset;
    }

    const char * getType(){ return ADDFRAME; };
    const char * getMD5(){ return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class AddFrame {
    public:
    typedef AddFrameRequest Request;
    typedef AddFrameResponse Response;
  };

}
#endif
