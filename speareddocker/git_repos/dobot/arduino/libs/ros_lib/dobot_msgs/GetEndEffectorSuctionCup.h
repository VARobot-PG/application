#ifndef _ROS_SERVICE_GetEndEffectorSuctionCup_h
#define _ROS_SERVICE_GetEndEffectorSuctionCup_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dobot_msgs
{

static const char GETENDEFFECTORSUCTIONCUP[] = "dobot_msgs/GetEndEffectorSuctionCup";

  class GetEndEffectorSuctionCupRequest : public ros::Msg
  {
    public:

    GetEndEffectorSuctionCupRequest()
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

    const char * getType(){ return GETENDEFFECTORSUCTIONCUP; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class GetEndEffectorSuctionCupResponse : public ros::Msg
  {
    public:
      typedef int32_t _result_type;
      _result_type result;
      typedef uint8_t _enableCtrl_type;
      _enableCtrl_type enableCtrl;
      typedef uint8_t _suck_type;
      _suck_type suck;

    GetEndEffectorSuctionCupResponse():
      result(0),
      enableCtrl(0),
      suck(0)
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
      *(outbuffer + offset + 0) = (this->enableCtrl >> (8 * 0)) & 0xFF;
      offset += sizeof(this->enableCtrl);
      *(outbuffer + offset + 0) = (this->suck >> (8 * 0)) & 0xFF;
      offset += sizeof(this->suck);
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
      this->enableCtrl =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->enableCtrl);
      this->suck =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->suck);
     return offset;
    }

    const char * getType(){ return GETENDEFFECTORSUCTIONCUP; };
    const char * getMD5(){ return "4855d73076b6df7c6c4785878f4cef46"; };

  };

  class GetEndEffectorSuctionCup {
    public:
    typedef GetEndEffectorSuctionCupRequest Request;
    typedef GetEndEffectorSuctionCupResponse Response;
  };

}
#endif
