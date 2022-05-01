#ifndef VEL_H
#define VEL_H

#include "ros/ros.h"
#include "std.msgs/Float64MultiArray.h"
#include "std.msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "nav_msgs/vel.h"

#define NODE_NAME "vel"

class vel {
private:
	ros::NodeHandle Handle;
	ros::Subscriber input_sub;
	ros::Publisher output_pub;

	void inputMsg_Callback(const sensor_msgs::JointState::ConstPtr& wheels_msg);
	void vel_computation(void);
	void publish(void);

	ros::Time current_time,past_time;

	double vel[3];
	double current_pos[4], past_pos[4];

public:
	void Data(void);
	void RunPeriod(void);
	void Stop(void);

};

#endif

/*NUOVOOOOOOO
#ifndef VELOCITY_H
#define VELOCITY_H

#include "ros/ros.h"

#include "sensor_msgs/JointState.h"
#include "geometry_msgs/TwistStamped.h"


#define NAME_OF_THIS_NODE "velocity"


class velocity  //header of the class
{
  private: 
    ros::NodeHandle Handle;

    /* ROS topics */
    ros::Subscriber input_subscriber;
    ros::Publisher output_publisher;

    /* ROS topic callbacks */
    void input_MessageCallback(const sensor_msgs::JointState::ConstPtr& wheel_state);
     
    /*auxiliary functions*/
    void compute_velocity(void);
    void publish(void);
    
    
    /* Node state variables */
    
    double LoopRate;
    ros::Time current_time, past_time;
    
    double position_curr[4], position_past[4];
    double vel[3];
    double l, w, r, T, N;
    

  public:
    void Prepare(void);
    
    void RunPeriodically(void);
    
    void Shutdown(void);

};

#endif */