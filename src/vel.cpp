#include "vel/vel.h"
void vel::Data(void)
{

	/*ROS topics */
	this->input_sub = this->Handle.subscriber("/wheels_msg", 1000, &vel::input_Msg_Callback, this)
	this->output_pub = this->Handle.advertise<nav_msgs::vel>("/vel", 1000);

	/* Initialize node state */
	this->current_time = ros::Time::now();
	this->past_time = ros::Time::now();

	this->position_curr[] = 0.0;
	this->position_past[] = 0.0;
	double r = 0.07,lx = 0.2, ly = 0.169, T = 5, N = 42;

	ROSI_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}

void vel::RunPeriod(void)
{
	ROSI_INFO("Node %s running.", ros::this_node::getName().c_str());

	//Wait other nodes start
	sleep (1.0);

	ros::Rate r(10);

	while (ros::ok()){
		ros::spinOnce();
		r.sleep();
	}
}

void vel::Stop(void){
	ROS_INFO("Node %s is closing", ros::this_node::getName().c_str());
}

void vel::inputMsg_Callback(const sensor_msgs::JointState::ConstPtr& wheels_msg) {
	this->current_pos[]=wheels_msg-> pos[];

	vel_computation();
}

void vel::vel_computation(void){
	this->current_time=ros::Time::now();
	double dt=(this->current_time-this->past_time).toSec();
	double tick[]=this->current_pos[]-this->past_pos[];
	double wheel_vel[4];

	wheel_vel[]=tick[]*2*3,14159/dt/N/T;
	this->vel[]=r/4*[1 1 1 1; -1 1 1 -1; -1/(l+w) 1/(l+w) -1/(l+w) 1/(l+w)]*wheel_vel[];

	this->past_time=this->current_time;
	this->past_pos=this->current_pos;
}

void vel::publish(void){
	geometry_msgs::TwistStamped cmd_vel;


	output_pub.publish(cmd_vel);
}