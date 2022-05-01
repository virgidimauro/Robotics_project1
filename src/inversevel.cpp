#include "vel/inversevel.h"

void inversevel::Data(void){

	/*recover parameters from Ros parameter server*/
	std::string name;
	std::string shortname="OmnidirectionalRobot";

	name= shortname+"/l";
	if (false==Handle.getParam(name,l))
		ROS_ERROR("Node %s couldn't recover parameter %s",ros::this_node::getName().c_str(),name.c_str());

	name= shortname+"/w";
	if (false==Handle.getParam(name,w))
		ROS_ERROR("Node %s couldn't recover parameter %s",ros::this_node::getName().c_str(),name.c_str());

	name= shortname+"/r";
	if (false==Handle.getParam(name,r))
		ROS_ERROR("Node %s couldn't recover parameter %s",ros::this_node::getName().c_str(),name.c_str());

	name= shortname+"/T";
	if (false==Handle.getParam(name,T))
		ROS_ERROR("Node %s couldn't recover parameter %s",ros::this_node::getName().c_str(),name.c_str());

	name= shortname+"/N";
	if (false==Handle.getParam(name,N))
		ROS_ERROR("Node %s couldn't recover parameter %s",ros::this_node::getName().c_str(),name.c_str());


	/*ROS topics */
	this->input_sub = this->Handle.subscribe("/cmd_vel", 1, &inversevel::inputMsg_Callback, this)
	this->output_pub = this->Handle.advertise<Robotics_project1::w_rpm>("/wheels_rpm", 1); /*NON CAPISCO QUEL ROBOTICS_...*/

	/* Initialize node state */
	this->vel_x=0;
	this->vel_y=0;
	this->ome=0;
	this->ome_fl=0;
	this->ome_fr=0;
	this->ome_rl=0;
	this->ome_rr=0;
	
	ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
};

void inversevel::RunPeriod(void){
	ROS_INFO("Node %s running.", ros::this_node::getName().c_str());

	//Wait other nodes to start
	sleep (1.0);
	ros::spin(); /*anche questa NEW*/

};

void inversevel::Stop(void){
	ROS_INFO("Node %s is closing", ros::this_node::getName().c_str());
};


void inversevel::inputMsg_Callback(const geometry_msgs::TwistStamped::ConstPtr& cmd_vel) {
	/*reads msg and stores info*/
	this->vel_x=cmd_vel->twist.linear.x;
	this->vel_y=cmd_vel->twist.linear.y;
	this->ome=cmd_vel->twist.angular.z;

	inversevel::inversevel_computation();
	inversevel::publish();
};

void inversevel::inversevel_computation(void){
	this->ome_fl=T/r*(vel_x-vel_y-ome*(l+w));
	this->ome_fr=T/r*(vel_x+vel_y+ome*(l+w));
	this->ome_rl=T/r*(vel_x+vel_y-ome*(l+w));;
	this->ome_rr=T/r*(vel_x-vel_y+ome*(l+w));
	
	ROS_INFO("estimated wheels' velocities Wfl,Wfr,Wrl,Wrrare [%f, %f, %f, %f]", (double)this->ome_fl, (double)this->ome_fr, (double)this->ome_rl, (double)this->ome_rr);
};

void inversevel::publish(void){

	Robotics_project1:w_rpm wheels_rpm;

	wheels_rpm.rpm_fl=this->ome_fl;
	wheels_rpm.rpm_fr=this->ome_fr;
	wheels_rpm.rpm_rl=this->ome_rl;
	wheels_rpm.rpm_rr=this->ome_rr;

	output_pub.publish(wheels_rpm);
};