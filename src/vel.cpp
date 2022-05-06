#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/TwistStamped.h"
#include <dynamic_reconfigure/server.h>
#include <Robotics_project1/parameters_calibrationConfig.h>

/* 		
		This node estimates the linear and the angular velocity of the robot and publishes them in the 
		topic /cmd_vel as TwistStamped messages. 
       	It listens to the /wheel_states topic published by the bags (sensor_msgs/JointState messages).
*/

class vel  { //header of the class
	private: 

    ros::NodeHandle n;

    /* ROS topics */
    ros::Subscriber sub;
    ros::Publisher pub;

    /*dyn reconfig server*/
	dynamic_reconfigure::Server<Robotics_project1::parameters_calibrationConfig> dynServer;

	/* Node state variables */
	ros::Time next_time, previous_time;
    
    double next_pos[4];
    double previous_pos[4];
    double vel_xyz[3];
    double l, w, r, N, T;

    /* ROS topic callbacks */
    void inputMsg_Callback(const sensor_msgs::JointState::ConstPtr& wheels_msg) {
		/*reads msg and stores info for each wheel*/
		for(int i=0;i<4;i++){
			this->next_pos[i] = wheels_msg->position[i];
		};
	};

	void parameters_Callback(Robotics_project1::parameters_calibrationConfig &config,uint32_t level){

		//callback of the dynamic reconfigure server to set the parameters of the robot

		//ROS_INFO("Requested to reconfigure: l=%f, w=%f, r=%f, T=%d, N=%d - Level %d", config.l, config.w, config.r, config.T, config.N, level);
	    this->l = config.l;
	    this->w = config.w;
	    this->r = config.r;
	    this->T = config.T;
	    this->N = config.N;
	};
 
    /*auxiliary functions*/
    void vel_computation(void){

		//computation of the velocity of the wheels from the encoder ticks and then the velocity of the robot

		this->next_time=ros::Time::now();
		double dt=(this->next_time - this->previous_time).toSec();
		double tick[4];
		for(int j=0;j<4;j++){
			tick[j]=this->next_pos[j]-this->previous_pos[j];
		};
		double wheel_vel[4];
		double matrix_fromwheelveltovel_xyz[3][4]={{r/4, r/4, r/4, r/4},{-r/4,r/4,r/4,-r/4},{(-r/4)/(l+w),(r/4)/(l+w),-(r/4)/(l+w),(r/4)/(l+w)}}; 
		for(int i=0;i<3;i++)
			vel_xyz[i]=0;
		for(int i=0;i<3;i++){
			for(int j=0;j<4;j++){
				wheel_vel[j]=tick[j]/dt*2*3.14159/N/T;
				this->vel_xyz[i]=this->vel_xyz[i]+matrix_fromwheelveltovel_xyz[i][j]*wheel_vel[j];
			};
		};
		//ROS_INFO("supposed velocity is [%f,%f,%f]", (double)this->vel_xyz[0], (double)this->vel_xyz[1], (double)this->vel_xyz[2]);

	    this->previous_time = this->next_time;

	    for(int j = 0; j<4; j++){
	        this->previous_pos[j] = this->next_pos[j];
	    }; //for each wheel
	};	
	
    void publish(void){
		geometry_msgs::TwistStamped cmd_vel;

		cmd_vel.header.stamp=this->next_time;
		cmd_vel.header.frame_id="robot";
		cmd_vel.twist.linear.x=this->vel_xyz[0];
		cmd_vel.twist.linear.y=this->vel_xyz[1];
		cmd_vel.twist.linear.z=0.0;
		cmd_vel.twist.angular.x=0.0;
		cmd_vel.twist.angular.y=0.0;
		cmd_vel.twist.angular.z=this->vel_xyz[2];

		//Publishing v and omega as topic cmd_vel

		pub.publish(cmd_vel);
	};
    

  public:
	void main_function(void){

		/*ROS topics */
		this->sub = this->n.subscribe("/wheel_states", 1, &vel::inputMsg_Callback, this);
		this->pub = this->n.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 1);

		/*dynamic reconfigure*/
		dynamic_reconfigure::Server<Robotics_project1::parameters_calibrationConfig>::CallbackType f;
		f = boost::bind(&vel::parameters_Callback, this, _1, _2);
		dynServer.setCallback(f);

		/* Initialize node state */
		this->next_time = ros::Time::now();
		this->previous_time = ros::Time::now();

		for(int j=0;j<4;j++){
			this->vel_xyz[j]=0;
		}; //for each wheel

		for(int j=0;j<4;j++){
			this->next_pos[j] = 0.0;
			this->previous_pos[j] = 0.0;
		}; //for each wheel

		ros::Rate loop_rate(10);
		//Waiting other nodes to start
		sleep (1.0);

		//Initialize ticks
		ros::spinOnce();
		for(int j=0;j<4;j++)
			this->previous_pos[j] = this->next_pos[j];
		sleep(0.5);

		while (ros::ok()){
			ros::spinOnce();
			vel_computation();
			publish();
			loop_rate.sleep();
		};
	};
};
      
int main (int argc, char **argv) {
	ros::init(argc,argv, "vel");
	vel vel_node;
	vel_node.main_function();
	return (0);
}