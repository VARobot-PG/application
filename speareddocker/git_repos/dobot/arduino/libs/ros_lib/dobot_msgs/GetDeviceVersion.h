#ifndef _ROS_SERVICE_GetDeviceVersion_h
#define _ROS_SERVICE_GetDeviceVersion_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dobot_msgs
{

static const char GETDEVICEVERSION[] = "dobot_msgs/GetDeviceVersion";

  class GetDeviceVersionRequest : public ros::Msg
  {
    public:

    GetDeviceVersionRequest()
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

    const char * getType(){ return GETDEVICEVERSION; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class GetDeviceVersionResponse : public ros::Msg
  {
    public:
      typedef int32_t _result_type;
      _result_type result;
      typedef uint8_t _majorVersion_type;
      _majorVersion_type majorVersion;
      typedef uint8_t _minorVersion_type;
      _minorVersion_type minorVersion;
      typedef uint8_t _revision_type;
      _revision_type revision;

    GetDeviceVersionResponse():
      result(0),
      majorVersion(0),
      minorVersion(0),
      revision(0)
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
      *(outbuffer + offset + 0) = (this->majorVersion >> (8 * 0)) & 0xFF;
      offset += sizeof(this->majorVersion);
      *(outbuffer + offset + 0) = (this->minorVersion >> (8 * 0)) & 0xFF;
      offset += sizeof(this->minorVersion);
      *(outbuffer + offset + 0) = (this->revision >> (8 * 0)) & 0xFF;
      offset += sizeof(this->revision);
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
      this->majorVersion =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->majorVersion);
      this->minorVersion =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->minorVersion);
      this->revision =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->revision);
     return offset;
    }

    const char * getType(){ return GETDEVICEVERSION; };
    const char * getMD5(){ return "c3c3f825efd26e7d40dd5c75956d6244"; };

  };

  class GetDeviceVersion {
    public:
    typedef GetDeviceVersionRequest Request;
    typedef GetDeviceVersionResponse Response;
  };

}
#endif
