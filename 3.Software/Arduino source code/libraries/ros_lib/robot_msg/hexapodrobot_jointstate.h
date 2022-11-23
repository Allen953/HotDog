#ifndef _ROS_robot_msg_hexapodrobot_jointstate_h
#define _ROS_robot_msg_hexapodrobot_jointstate_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace robot_msg
{

  class hexapodrobot_jointstate : public ros::Msg
  {
    public:
      float position[18];

    hexapodrobot_jointstate():
      position()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      for( uint32_t i = 0; i < 18; i++){
      union {
        float real;
        uint32_t base;
      } u_positioni;
      u_positioni.real = this->position[i];
      *(outbuffer + offset + 0) = (u_positioni.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_positioni.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_positioni.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_positioni.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->position[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      for( uint32_t i = 0; i < 18; i++){
      union {
        float real;
        uint32_t base;
      } u_positioni;
      u_positioni.base = 0;
      u_positioni.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_positioni.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_positioni.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_positioni.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->position[i] = u_positioni.real;
      offset += sizeof(this->position[i]);
      }
     return offset;
    }

    const char * getType(){ return "robot_msg/hexapodrobot_jointstate"; };
    const char * getMD5(){ return "4d7fde1b9fa4856fefe74c9439077324"; };

  };

}
#endif