#ifndef INVERSEVEL_H
#define INVERSEVEL_H

#include "ros/ros.h"

#include "geometry_msgs/TwistStamped.h"
#include "Robotics_project1/w_rpm.h"


#define NAME_OF_THIS_NODE "inversevel"


class inversevel  //header of the class
{
  private: 
    ros::NodeHandle Handle;

    /* ROS topics */
    ros::Subscriber input_sub;
    ros::Publisher output_pub;

    /* ROS topic callbacks */
    void input_MsgCallback(const geometry_msgs::TwistStamped::ConstPtr& cmd_vel);
     
    /*auxiliary functions*/
    void inversevel_computation(void);
    void publish(void);
    
    
    /* Node state variables */
    
    double vel_x, vel_y, ome;
    double ome_fl, ome_fr, ome_rl, ome_rr;
    double l, w, r, T, N;
    

  public:
    void Data(void);
    
    void RunPeriod(void);
    
    void Stop(void);

};

#endif