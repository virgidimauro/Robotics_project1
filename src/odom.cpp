#include "odom/odom.h"

#include <dynamic_reconfigure/server.h>
#include <Robotics_project1/integration_methodsConfig.h>

void vel::Data(void)
{

	/*ROS topics */
	this->input_sub = this->Handle.subscriber("/cmd_vel", 1, &odom::input_Msg_Callback, this)
	this->output_pub = this->Handle.advertise<nav_msgs::odom>("/odom", 1);

	/*Ros services*/
	this->server = this->Handle.advertiseService("Reset_Odometry", &odom::odomReset_Callback, this);

	/*dynamic reconfigure*/
	/*dynamic_reconfigure::Server<pub_sub::parametersConfig> dynServer; DA CAPIRE SE È DA AGIUNGERE*/
	dynamic_reconfigure::Server<Robotics_project1::integration_methodsConfig>::CallbackType f;
	f = boost::bind(&odom::reconfig_Callback, this, _1, _2);
	dynServer.setCallback(f);

	/* Initialize node state */
	this->current_time = ros::Time::now();
	this->past_time = ros::Time::now();

	this->x = 0.0;
	this->y = 0.0;
	this->theta = 0.0;

	this->vel_x = 0.0;
	this->vel_y = 0.0;
	this->omega = 0.0;

	this->integration_method = 0;

	ROS_INFO("Node %s ready to run", ros::this_node::getName().c_str());
}

void odom::RunPeriod(void)
{
	ROSI_INFO("Node %s running.", ros::this_node::getName().c_str());

	//Wait other nodes start
	sleep (1.0);

	ros::spin();
}

void odom::Stop(void){
	ROS_INFO("Node %s is closing", ros::this_node::getName().c_str());
}

void odom::inputMsg_Callback(const sensor_msgs::JointState::ConstPtr& cmd_vel) {
	this->vel_x=cmd_vel-> twist.linear.x;
	this->vel_y=cmd_vel-> twist.linear.y;
	this->ome=cmd_vel-> twist.angular.z;

	integration();
}

bool odom::odomReset_Callback(Robotics_project1::Reset_Odometry::Request& request,Robotics_project1::Reset_Odometry::Response& response){
	response.x=this->x;
	response.y=this->y;
	response.theta=this->theta;

	this->x=request.x;
	this->y=request.y;
	this->theta=request.theta;

	ROS_INFO("Requested to reset the pose of the odometry to [%f %f %f]. Responded with the last pose [%f %f %f].", 
			(double)request.x, (double)request.y, (double)request.theta, (double)response.x, (double)response.y, (double)response.theta);
	return true;
}

void odom::reconfig_Callback(Robotics_project1::integration_methodsConfig &config,uint32_t level){
	ROS_INFO("Reconfigure request: %d - level %d", 
		integration_methodsConfig,level);
	this->integration_method=config.integration_method;
}


void odom::integration(void){
	this->current_time=ros::Time::now();
	double dt=(this->current_time-this->past_time).toSec();
	double delta_x, delta_y, delta_theta;

	switch (this->integration_method) {
		case 0:
			delta_x = vel_x*dt*std::cos(theta) - vel_y*dt*std::sin(theta);
			delta_y = vel_x*dt*std::sin(theta) + vel_y*dt*std::cos(theta);
			delta_theta = omega*dt;
			break;

		case 1:
			delta_x = vel_x*dt*std::cos(theta+omega*dt/2) - vel_y*dt*std::sin(theta+omega*dt/2);
			delta_y = vel_x*dt*std::sin(theta+omega*dt/2) - vel_y*dt*std::cos(theta+omega*dt/2);
			delta_theta = omega*dt;
			break;

	}

	this->x += delta_x;
	this->y += delta_y;
	this->theta += delta_theta;

	this->past_time = this->current_time;

	ROS_INFO("Suppose pose is [%f,%f,%f], integrated whit method %d", (double)this->x, (double)this->y, (double)this->theta, this->integration_method);
}

void odom::publish(void){
	nav_msgs::Odometry odom_msgs;


	output_pub.publish(odom_msgs);
}