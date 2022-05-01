#ifndef ODOM_H
#define ODOM_H

#include "ros/ros.h"
#include "std.msgs/Float64MultiArray.h"
#include "std.msgs/Float64.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/odom.h" /*NON SO SE VA Odometri.h */

#include "Robotics_project1/Reset_Odometry.h"
#include <dynamic_reconfigure/server.h>
#include <Robotics_project1/integration_methodsConfig.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#define NODE_NAME "odom"

class odom {
private:
	ros::NodeHandle Handle;
	/*Ros topics*/
	ros::Subscriber input_sub;
	ros::Publisher output_pub;

	/*Ros service*/
	ros::ServiceServer server;
	/*dyn reconfig server*/
	dynamic_reconfigure::Server<Robotics_project1::integration_methodsConfig> dynServer;
	/*tf broadcaster*/
	tf2_ros:TransformBroadcaster broadc_odom;
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

	int integration_method;
	double x,y,theta;
	double vel_x,vel_y, ome;

public:
	void Data(void);
	void RunPeriod(void);
	void Stop(void);

};

#endif

/*nuovooo
#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "ros/ros.h"

#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "project1/Reset_Odometry.h"
#include <dynamic_reconfigure/server.h>
#include <project1/integration_methodsConfig.h>


#define NAME_OF_THIS_NODE "odometer"


class odometer  //header of the class
{
  private: 
    ros::NodeHandle Handle;

    /* ROS topics */
    ros::Subscriber input_subscriber;
    ros::Publisher output_publisher;
    
    /* Ros service*/
    ros::ServiceServer server;

    /* Dynamic reconfigure server*/
    dynamic_reconfigure::Server<project1::integration_methodsConfig> dynServer;

    /* TF Broadcaster */
    tf2_ros::TransformBroadcaster odom_broadcaster;


    /* Parameters from ROS parameter server */
    double loopRate;
    

    /* ROS topic callbacks */
    void input_MessageCallback(const geometry_msgs::TwistStamped::ConstPtr& cmd_vel);
    
    /* ROS service callbacks */
    bool reset_callback(project1::Reset_Odometry::Request& req,
    project1::Reset_Odometry::Response& res);
    
    /* dynamics reconfigure callback*/
    void int_method_callback(project1::integration_methodsConfig &config, uint32_t level);
     
    /*auxiliary functions*/
    void integrate(void);
    void publish(void);
    
    
    /* Node state variables */
    ros::Time current_time, past_time;
    
    int integration_method;
    
    double x, y, theta;
    double vel_x, vel_y, omega;
  
    

  public:
    void Prepare(void);
    
    void RunPeriodically(void);
    
    void Shutdown(void);

};

#endif */