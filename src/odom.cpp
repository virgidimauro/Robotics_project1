#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"
#include "Robotics_project1/Reset_odom.h"
#include <dynamic_reconfigure/server.h>
#include <Robotics_project1/integration_methodsConfig.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

/* 		
		This node estimates the pose of the robot from the TwistStamped messages published in /cmd_vel, 
        and publishes /odom (nav_msgs/Odometry messages).
        It also publishes the transformation between the odom and base_link reference systems.
        It offers a service to reset the pose to a given pose (x,y,theta).
        The odometry is calculated using either Euler or Runge-Kutta integration method, depending on the 
        parameter int_choice which can be set with dynamic reconfigure. The default method is Euler.
*/

class odom { //header of the class
private:
	ros::NodeHandle n;

	/*Ros topics*/
	ros::Subscriber sub;
	ros::Publisher pub; //publishes Odometry messages

	/*Ros service*/
	ros::ServiceServer server;

	/*dyn reconfig server*/
	dynamic_reconfigure::Server<Robotics_project1::integration_methodsConfig> dynServer;

	/*tf broadcaster*/
	tf2_ros::TransformBroadcaster broadc_odom;


	ros::Time next_time,previous_time;

	int int_choice;
	double x,y,theta;
	double vel_x,vel_y, ome;

	/*Ros topic callbacks*/
	void inputMsg_Callback(const geometry_msgs::TwistStamped::ConstPtr& cmd_vel) {
		this->next_time = cmd_vel->header.stamp;
		integration();
		publish();
		this->vel_x=cmd_vel-> twist.linear.x;
		this->vel_y=cmd_vel-> twist.linear.y;
		this->ome=cmd_vel-> twist.angular.z;
		this->previous_time = this->next_time;
	};

	/*Ros service callbacks*/
	bool odomReset_Callback(Robotics_project1::Reset_odom::Request& request,Robotics_project1::Reset_odom::Response& response){

		//callback of the service Reset_odom: forces the robot pose to (x,y,theta) chosen by the caller of the service

		response.x=this->x;
		response.y=this->y;
		response.theta=this->theta;
		this->x=request.x;request
		this->y=request.y;
		this->theta=request.theta;
		//ROS_INFO("Requested to reset the pose of the odometry to [%f %f %f]. Responded with the last pose [%f %f %f].", 
				//(double)request.x, (double)request.y, (double)request.theta, (double)response.x, (double)response.y, (double)response.theta);
		return true;
	};

	/*dyn reconfig callback*/
	void reconfig_Callback(Robotics_project1::integration_methodsConfig &config,uint32_t level){

		//callback of the dynamic reconfigure server to set the parameter int_choice to the desired value: 
		// 0 for Euler and 1 for Rugge-Kutta integration method

		//ROS_INFO("Requested to reconfigure: %d - level %d", config.int_choice,level);
		this->int_choice=config.int_choice;
	};

	void integration(void){

		//computation of Euler and Rugge-Kutta integration methods

		double dt=(this->next_time-this->previous_time).toSec();
		double delta_x, delta_y, delta_theta;
		switch (this->int_choice) {
			case 0:
				delta_x = vel_x*dt*std::cos(theta) - vel_y*dt*std::sin(theta);
				delta_y = vel_x*dt*std::sin(theta) + vel_y*dt*std::cos(theta);
				delta_theta = ome*dt;
				break;

			case 1:
				delta_x = vel_x*dt*std::cos(theta+ome*dt/2) - vel_y*dt*std::sin(theta+ome*dt/2);
				delta_y = vel_x*dt*std::sin(theta+ome*dt/2) - vel_y*dt*std::cos(theta+ome*dt/2);
				delta_theta = ome*dt;
				break;
		}

		this->x += delta_x;
		this->y += delta_y;
		this->theta += delta_theta;
		this->previous_time = this->next_time;
		//ROS_INFO("Estimated pose is [%f,%f,%f], integrated with method %d, where 0 is Euler and 1 is Rugge-Kutta", (double)this->x, (double)this->y, (double)this->theta, this->int_choice);
	};

	void publish(void){
		//transformation from odom reference frame to the base-link reference frame
	     //Quaternion based on the the yaw of the robot
	     tf2::Quaternion q;
	     q.setRPY(0,0,this->theta);

	     //Publishing the transform over tf
	     geometry_msgs::TransformStamped transformation;
	     transformation.header.stamp = this->next_time;
	     transformation.header.frame_id = "odom";
	     transformation.child_frame_id = "base_link";

	     transformation.transform.translation.x = this->x;
	     transformation.transform.translation.y = this->y;
	     transformation.transform.translation.z = 0.0;
	     transformation.transform.rotation.x = q.x();
	     transformation.transform.rotation.y = q.y();
	     transformation.transform.rotation.z = q.z();
	     transformation.transform.rotation.w = q.w();

	     //Sending the transform
	     broadc_odom.sendTransform(transformation);

	     //Publishing the odometry message, needed for rviz
	     nav_msgs::Odometry odom;
	     odom.header.stamp = this->next_time;
	     odom.header.frame_id = "odom";

	     //Setting the pose
	     odom.pose.pose.position.x = this->x;
	     odom.pose.pose.position.y = this->y;
	     odom.pose.pose.position.z = 0.0;
	     odom.pose.pose.orientation = transformation.transform.rotation;

	     //Setting the velocity
	     odom.child_frame_id = "base_link";
	     odom.twist.twist.linear.x = this->vel_x;
	     odom.twist.twist.linear.y = this->vel_y;
	     odom.twist.twist.linear.z = 0.0;
	     odom.twist.twist.angular.x = 0.0;
	     odom.twist.twist.angular.y = 0.0;
	     odom.twist.twist.angular.z = this->ome;

	     //Publishing the message to odom
	     pub.publish(odom);
	};

public:
	void main_function(void){

		//Recover parameters from Ros parameter server (inital_pose.yaml)
		std::string name;
		std::string shortname="OmnidirectionalRobot/initial_pose";

		name= shortname+"/x";
		//if (false==n.getParam(name,x))
		//	ROS_ERROR("Node %s couldn't recover parameter %s",ros::this_node::getName().c_str(),name.c_str());

		name= shortname+"/y";
		//if (false==n.getParam(name,y))
		//	ROS_ERROR("Node %s couldn't recover parameter %s",ros::this_node::getName().c_str(),name.c_str());

		name= shortname+"/theta";
		//if (false==n.getParam(name,theta))
		//	ROS_ERROR("Node %s couldn't recover parameter %s",ros::this_node::getName().c_str(),name.c_str());

		/*ROS topics */
		this->sub = this->n.subscribe("/cmd_vel", 10, &odom::inputMsg_Callback, this);
		this->pub = this->n.advertise<nav_msgs::Odometry>("/odom", 1);

		/*Ros services*/
		this->server = this->n.advertiseService("Reset_odom", &odom::odomReset_Callback, this);

		/*dynamic reconfigure*/
		dynamic_reconfigure::Server<Robotics_project1::integration_methodsConfig>::CallbackType f;
		f = boost::bind(&odom::reconfig_Callback, this, _1, _2);
		dynServer.setCallback(f);

		/* Initialize node state */
		this->next_time = ros::Time::now();
		this->previous_time = ros::Time::now();

		this->vel_x = 0.0;
		this->vel_y = 0.0;
		this->ome = 0.0;

		this->int_choice = 0;

		//ROS_INFO("Node %s ready to run", ros::this_node::getName().c_str());
	};
};

int main(int argc, char **argv){
	ros::init(argc, argv, "odom");
	odom odom_node;
	odom_node.main_function();
	//Wait other nodes start
	sleep (1.0);
	ros::spin();
	return(0);
} 