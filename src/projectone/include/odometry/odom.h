#ifndef ODOM_H
#define ODOM_H

#include "ros/ros.h"
#include "std.msgs/Float64MultiArray.h"
#include "std.msgs/Float64.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/odom.h"

#include "Robotics_project1/Reset_Odometry.h"
#include <dynamic_reconfigure/server.h>
#include <Robotics_project1/integration_methodsConfig.h>

#define NODE_NAME "odom"

class odom {
private:
	ros::NodeHandle Handle;
	ros::Subscriber input_sub;
	ros::Publisher output_pub;

	/*Ros service*/
	ros::ServiceServer server;
	/*dyn reconfig server*/
	dynamic_reconfigure::Server<Robotics_project1::integration_methodsConfig> dynServer;
	/*parameters from Ros parameter server*/
	double loopRate;
	/*Ros topic callbacks*/
	void inputMsg_Callback(const geometry_msgs::TwistStamped::ConstPtr& cmd_vel);
	/*Ros service callbacks*/
	bool odomReset_Callback(Robotics_project1::Reset_Odometry::Request& request,Robotics_project1::Reset_Odometry::Response& response);
	/*dyn reconfig callback*/
	void reconfig_Callback(Robotics_project1::integration_methodsConfig &config, uint32_t level);

	void integration(void);
	void publish(void);

	ros::Time current_time,past_time;

	int wayofintegrating;
	double x,y,theta;
	double vel_x,vel_y, ome;

public:
	void Data(void);
	void RunPeriod(void);
	void Stop(void);

};

#endif