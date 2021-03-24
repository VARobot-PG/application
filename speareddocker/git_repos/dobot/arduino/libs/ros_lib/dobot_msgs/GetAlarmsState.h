#ifndef _ROS_SERVICE_GetAlarmsState_h
#define _ROS_SERVICE_GetAlarmsState_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dobot_msgs
{

static const char GETALARMSSTATE[] = "dobot_msgs/GetAlarmsState";

  class GetAlarmsStateRequest : public ros::Msg
  {
    public:

    GetAlarmsStateRequest()
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

    const char * getType(){ return GETALARMSSTATE; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class GetAlarmsStateResponse : public ros::Msg
  {
    public:
      typedef int32_t _result_type;
      _result_type result;
      uint32_t alarmsState_length;
      typedef uint8_t _alarmsState_type;
      _alarmsState_type st_alarmsState;
      _alarmsState_type * alarmsState;

    GetAlarmsStateResponse():
      result(0),
      alarmsState_length(0), alarmsState(NULL)
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
      *(outbuffer + offset + 0) = (this->alarmsState_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->alarmsState_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->alarmsState_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->alarmsState_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->alarmsState_length);
      for( uint32_t i = 0; i < alarmsState_length; i++){
      *(outbuffer + offset + 0) = (this->alarmsState[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->alarmsState[i]);
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
      uint32_t alarmsState_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      alarmsState_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      alarmsState_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      alarmsState_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->alarmsState_length);
      if(alarmsState_lengthT > alarmsState_length)
        this->alarmsState = (uint8_t*)realloc(this->alarmsState, alarmsState_lengthT * sizeof(uint8_t));
      alarmsState_length = alarmsState_lengthT;
      for( uint32_t i = 0; i < alarmsState_length; i++){
      this->st_alarmsState =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->st_alarmsState);
        memcpy( &(this->alarmsState[i]), &(this->st_alarmsState), sizeof(uint8_t));
      }
     return offset;
    }

    const char * getType(){ return GETALARMSSTATE; };
    const char * getMD5(){ return "471cc92db011a752d21793dfa031a894"; };

  };

  class GetAlarmsState {
    public:
    typedef GetAlarmsStateRequest Request;
    typedef GetAlarmsStateResponse Response;
  };

}
#endif
