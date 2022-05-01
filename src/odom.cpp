#include "ros/ros.h"
#include "std.msgs/Float64MultiArray.h"
#include "std.msgs/Float64.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"
#include "Robotics_project1/Reset_Odometry.h"
#include <dynamic_reconfigure/server.h>
#include <Robotics_project1/integration_methodsConfig.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

//CONTROLARE BROADCAST PUBLISH

class odom {
private:
	ros::NodeHandle n;
	/*Ros topics*/
	ros::Subscriber sub;
	ros::Publisher pub;

	/*Ros service*/
	ros::ServiceServer server;
	/*dyn reconfig server*/
	dynamic_reconfigure::Server<Robotics_project1::integration_methodsConfig> dynServer;
	/*tf broadcaster*/
	tf2_ros::TransformBroadcaster broadc_odom;
	/*parameters from Ros parameter server*/
	double loop_rate;
	/*Ros topic callbacks*/
	void inputMsg_Callback(const geometry_msgs::TwistedStamped::ConstPtr& cmd_vel) {
		this->current_time = cmd_vel->header.stamp;

		integration(); /*c'era odom::*/
		publish(); /*c'era odom::*/

		this->vel_x=cmd_vel-> twist.linear.x;
		this->vel_y=cmd_vel-> twist.linear.y;
		this->ome=cmd_vel-> twist.angular.z;

		this->past_time = this->current_time;
	};

	/*Ros service callbacks*/
	bool odomReset_Callback(Robotics_project1::Reset_Odometry::Request& request,Robotics_project1::Reset_Odometry::Response& response){
		response.x=this->x;
		response.y=this->y;
		response.theta=this->theta;

		this->x=request.x;
		this->y=request.y;
		this->theta=request.theta;

		ROS_INFO("Requested to reset the pose of the odometry to [%f %f %f]. Responded with the last pose [%f %f %f].", 
				(double)request.x, (double)request.y, (double)request.theta, (double)response.x, (double)response.y, (double)response.theta);
		return true;
	};

	/*dyn reconfig callback*/
	void reconfig_Callback(Robotics_project1::integration_methodsConfig &config,uint32_t level){
		ROS_INFO("Requested to reconfigure: %d - level %d", integration_methodsConfig,level);
		this->int_choice=config.int_choice;
	};

	void integration(void){
		double dt=(this->current_time-this->past_time).toSec();
		double delta_x, delta_y, delta_theta;

		switch (this->int_choice) {
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

		ROS_INFO("Estimated pose is [%f,%f,%f], integrated with method %d", (double)this->x, (double)this->y, (double)this->theta, this->int_choice);
	};

	void publish(void){
	     //We create a quaternion based on the the yaw of the robot
	     tf2::Quaternion q;
	     q.setRPY(0,0,this->theta);

	     //first, we'll publish the transform over tf
	     geometry_msgs::TransformStamped transformation;
	     transformation.header.stamp = this->current_time;
	     transformation.header.frame_id = "odom";
	     transformation.child_frame_id = "base_link";

	     transformation.transform.translation.x = this->x;
	     transformation.transform.translation.y = this->y;
	     transformation.transform.translation.z = 0.0;
	     transformation.transform.rotation.x = q.x();
	     transformation.transform.rotation.y = q.y();
	     transformation.transform.rotation.z = q.z();
	     transformation.transform.rotation.w = q.w();

	     //send the transform
	     odom_broadcaster.sendTransform(transformation);

	     //next, we'll publish the odometry message over ROS
	     nav_msgs::Odometry odom;
	     odom.header.stamp = this->current_time;
	     odom.header.frame_id = "odom";

	     //set the position
	     odom.pose.pose.position.x = this->x;
	     odom.pose.pose.position.y = this->y;
	     odom.pose.pose.position.z = 0.0;
	     odom.pose.pose.orientation = transformation.transform.rotation;

	     //set the velocity
	     odom.child_frame_id = "base_link";
	     odom.twist.twist.linear.x = this->vel_x;
	     odom.twist.twist.linear.y = this->vel_y;
	     odom.twist.twist.linear.z = 0.0;
	     odom.twist.twist.angular.x = 0.0;
	     odom.twist.twist.angular.y = 0.0;
	     odom.twist.twist.angular.z = this->ome;

	     //publish the message
	     pub.publish(odom);
	    
	};

	ros::Time current_time,past_time;

	int int_choice;
	double x,y,theta;
	double vel_x,vel_y, ome;

public:
	void Data(void){

		/*ROS topics */
		this->sub = this->n.subscribe("/cmd_vel", 1, &odom::input_Msg_Callback, this)
		this->pub = this->n.advertise<nav_msgs::Odometry>("/odom", 1);

		/*Ros services*/
		this->server = this->n.advertiseService("Reset_Odometry", &odom::odomReset_Callback, this);

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

		this->int_choice = 0;

		ROS_INFO("Node %s ready to run", ros::this_node::getName().c_str());
	};
	void RunPeriod(void){
		ROSI_INFO("Node %s running.", ros::this_node::getName().c_str());

		//Wait other nodes start
		sleep (1.0);

		ros::spin();
	};

	void Stop(void){
		ROS_INFO("Node %s is closing", ros::this_node::getName().c_str());
	};
};

int main(int argc, char **argv){
	ros::init(argc, argv, "odom");

	odom odom_node;

	//odom_node.Data();

	//odom_node.RunPeriod();

	//odom_node.Stop();

	return(0);
}