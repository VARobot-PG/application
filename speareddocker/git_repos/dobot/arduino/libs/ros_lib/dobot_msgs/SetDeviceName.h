#ifndef _ROS_SERVICE_SetDeviceName_h
#define _ROS_SERVICE_SetDeviceName_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/String.h"

namespace dobot_msgs
{

static const char SETDEVICENAME[] = "dobot_msgs/SetDeviceName";

  class SetDeviceNameRequest : public ros::Msg
  {
    public:
      typedef std_msgs::String _deviceName_type;
      _deviceName_type deviceName;

    SetDeviceNameRequest():
      deviceName()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->deviceName.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->deviceName.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return SETDEVICENAME; };
    const char * getMD5(){ return "5cfd0d17902de8430ea45b9b5c531010"; };

  };

  class SetDeviceNameResponse : public ros::Msg
  {
    public:
      typedef int32_t _result_type;
      _result_type result;

    SetDeviceNameResponse():
      result(0)
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
     return offset;
    }

    const char * getType(){ return SETDEVICENAME; };
    const char * getMD5(){ return "034a8e20d6a306665e3a5b340fab3f09"; };

  };

  class SetDeviceName {
    public:
    typedef SetDeviceNameRequest Request;
    typedef SetDeviceNameResponse Response;
  };

}
#endif
