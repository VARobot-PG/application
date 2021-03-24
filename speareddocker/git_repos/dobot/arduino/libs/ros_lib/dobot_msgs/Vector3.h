#ifndef _ROS_SERVICE_Vector3_h
#define _ROS_SERVICE_Vector3_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dobot_msgs
{

static const char VECTOR3[] = "dobot_msgs/Vector3";

  class Vector3Request : public ros::Msg
  {
    public:
      typedef float _xIn_type;
      _xIn_type xIn;
      typedef float _yIn_type;
      _yIn_type yIn;
      typedef float _zIn_type;
      _zIn_type zIn;

    Vector3Request():
      xIn(0),
      yIn(0),
      zIn(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->xIn);
      offset += serializeAvrFloat64(outbuffer + offset, this->yIn);
      offset += serializeAvrFloat64(outbuffer + offset, this->zIn);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->xIn));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->yIn));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->zIn));
     return offset;
    }

    const char * getType(){ return VECTOR3; };
    const char * getMD5(){ return "a93128747e307b5c676a35cf063fa4ac"; };

  };

  class Vector3Response : public ros::Msg
  {
    public:
      typedef float _xOut_type;
      _xOut_type xOut;
      typedef float _yOut_type;
      _yOut_type yOut;
      typedef float _zOut_type;
      _zOut_type zOut;

    Vector3Response():
      xOut(0),
      yOut(0),
      zOut(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->xOut);
      offset += serializeAvrFloat64(outbuffer + offset, this->yOut);
      offset += serializeAvrFloat64(outbuffer + offset, this->zOut);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->xOut));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->yOut));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->zOut));
     return offset;
    }

    const char * getType(){ return VECTOR3; };
    const char * getMD5(){ return "f666f3c5d5200493959a2bc683b81628"; };

  };

  class Vector3 {
    public:
    typedef Vector3Request Request;
    typedef Vector3Response Response;
  };

}
#endif
