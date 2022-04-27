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