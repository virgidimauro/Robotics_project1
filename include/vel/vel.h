#ifndef VEL_H
#define VEL_H

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/TwistStamped.h"

class vel  //header of the class
{
  private: 
    ros::NodeHandle Handle;

    /* ROS topics */
    ros::Subscriber sub;
    ros::Publisher pub;

    /* ROS topic callbacks */
    void inputMsg_Callback(const sensor_msgs::JointState::ConstPtr& wheels_msg);
     
    /*auxiliary functions*/
    void vel_computation(void);
    void publish(void);
    
    
    /* Node state variables */
    
    double loop_rate;
    ros::Time current_time, past_time;
    
    double current_pos[4];
    double past_pos[4];
    double vel_xyz[3];
    double l, w, r, T, N;
    

  public:
  void Data(void);
  void RunPeriod(void);
  void Stop(void);

};

#endif