#ifndef _ROS_dobot_msgs_block_rotation_h
#define _ROS_dobot_msgs_block_rotation_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dobot_msgs
{

  class block_rotation : public ros::Msg
  {
    public:
      typedef uint8_t _min_dist_l_type;
      _min_dist_l_type min_dist_l;
      typedef uint8_t _min_dist_r_type;
      _min_dist_r_type min_dist_r;
      typedef uint8_t _max_dist_l_type;
      _max_dist_l_type max_dist_l;
      typedef uint8_t _max_dist_r_type;
      _max_dist_r_type max_dist_r;
      typedef float _rotation_type;
      _rotation_type rotation;
      typedef bool _new_measurement_type;
      _new_measurement_type new_measurement;

    block_rotation():
      min_dist_l(0),
      min_dist_r(0),
      max_dist_l(0),
      max_dist_r(0),
      rotation(0),
      new_measurement(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->min_dist_l >> (8 * 0)) & 0xFF;
      offset += sizeof(this->min_dist_l);
      *(outbuffer + offset + 0) = (this->min_dist_r >> (8 * 0)) & 0xFF;
      offset += sizeof(this->min_dist_r);
      *(outbuffer + offset + 0) = (this->max_dist_l >> (8 * 0)) & 0xFF;
      offset += sizeof(this->max_dist_l);
      *(outbuffer + offset + 0) = (this->max_dist_r >> (8 * 0)) & 0xFF;
      offset += sizeof(this->max_dist_r);
      union {
        float real;
        uint32_t base;
      } u_rotation;
      u_rotation.real = this->rotation;
      *(outbuffer + offset + 0) = (u_rotation.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rotation.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rotation.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rotation.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rotation);
      union {
        bool real;
        uint8_t base;
      } u_new_measurement;
      u_new_measurement.real = this->new_measurement;
      *(outbuffer + offset + 0) = (u_new_measurement.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->new_measurement);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->min_dist_l =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->min_dist_l);
      this->min_dist_r =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->min_dist_r);
      this->max_dist_l =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->max_dist_l);
      this->max_dist_r =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->max_dist_r);
      union {
        float real;
        uint32_t base;
      } u_rotation;
      u_rotation.base = 0;
      u_rotation.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rotation.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rotation.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rotation.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->rotation = u_rotation.real;
      offset += sizeof(this->rotation);
      union {
        bool real;
        uint8_t base;
      } u_new_measurement;
      u_new_measurement.base = 0;
      u_new_measurement.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->new_measurement = u_new_measurement.real;
      offset += sizeof(this->new_measurement);
     return offset;
    }

    const char * getType(){ return "dobot_msgs/block_rotation"; };
    const char * getMD5(){ return "62b6156ab0855ad043e9190bdd4a3789"; };

  };

}
#endif