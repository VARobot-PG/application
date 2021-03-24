#ifndef _ROS_detection_msgs_DetectedObject_h
#define _ROS_detection_msgs_DetectedObject_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Point.h"
#include "ros/time.h"

namespace detection_msgs
{

  class DetectedObject : public ros::Msg
  {
    public:
      typedef geometry_msgs::Point _graspPoint_type;
      _graspPoint_type graspPoint;
      typedef geometry_msgs::Point _startPoint_type;
      _startPoint_type startPoint;
      typedef geometry_msgs::Point _endPoint_type;
      _endPoint_type endPoint;
      uint32_t color_length;
      typedef float _color_type;
      _color_type st_color;
      _color_type * color;
      typedef const char* _type_type;
      _type_type type;
      typedef float _probability_type;
      _probability_type probability;
      typedef ros::Time _timestamp_type;
      _timestamp_type timestamp;

    DetectedObject():
      graspPoint(),
      startPoint(),
      endPoint(),
      color_length(0), color(NULL),
      type(""),
      probability(0),
      timestamp()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->graspPoint.serialize(outbuffer + offset);
      offset += this->startPoint.serialize(outbuffer + offset);
      offset += this->endPoint.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->color_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->color_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->color_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->color_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->color_length);
      for( uint32_t i = 0; i < color_length; i++){
      union {
        float real;
        uint32_t base;
      } u_colori;
      u_colori.real = this->color[i];
      *(outbuffer + offset + 0) = (u_colori.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_colori.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_colori.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_colori.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->color[i]);
      }
      uint32_t length_type = strlen(this->type);
      varToArr(outbuffer + offset, length_type);
      offset += 4;
      memcpy(outbuffer + offset, this->type, length_type);
      offset += length_type;
      offset += serializeAvrFloat64(outbuffer + offset, this->probability);
      *(outbuffer + offset + 0) = (this->timestamp.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->timestamp.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->timestamp.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->timestamp.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->timestamp.sec);
      *(outbuffer + offset + 0) = (this->timestamp.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->timestamp.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->timestamp.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->timestamp.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->timestamp.nsec);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->graspPoint.deserialize(inbuffer + offset);
      offset += this->startPoint.deserialize(inbuffer + offset);
      offset += this->endPoint.deserialize(inbuffer + offset);
      uint32_t color_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      color_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      color_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      color_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->color_length);
      if(color_lengthT > color_length)
        this->color = (float*)realloc(this->color, color_lengthT * sizeof(float));
      color_length = color_lengthT;
      for( uint32_t i = 0; i < color_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_color;
      u_st_color.base = 0;
      u_st_color.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_color.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_color.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_color.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_color = u_st_color.real;
      offset += sizeof(this->st_color);
        memcpy( &(this->color[i]), &(this->st_color), sizeof(float));
      }
      uint32_t length_type;
      arrToVar(length_type, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_type; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_type-1]=0;
      this->type = (char *)(inbuffer + offset-1);
      offset += length_type;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->probability));
      this->timestamp.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->timestamp.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->timestamp.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->timestamp.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->timestamp.sec);
      this->timestamp.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->timestamp.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->timestamp.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->timestamp.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->timestamp.nsec);
     return offset;
    }

    const char * getType(){ return "detection_msgs/DetectedObject"; };
    const char * getMD5(){ return "6db0198d8c68aab34746ef6c59f50498"; };

  };

}
#endif