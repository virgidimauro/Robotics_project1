#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "Robotics_project1/w_rpm.h"
#include <dynamic_reconfigure/server.h>
#include <Robotics_project1/parameters_calibrationConfig.h>

/* 		
		This node estimates the velocities of the wheels given v_x, v_y and omega, subscribing to /cmd_vel.
		It publishes them as custom message on the topic /wheels_rpm. 
*/

class inversevel  //header of the class
{
  private: 
    ros::NodeHandle n;

    /* ROS topics */
    ros::Subscriber sub;
    ros::Publisher pub;

    /*dyn reconfig server*/
		dynamic_reconfigure::Server<Robotics_project1::parameters_calibrationConfig> dynServer;


		/* Node state variables */
    double vel_x, vel_y, ome;
    double ome_fl, ome_fr, ome_rl, ome_rr;
    double l, w, r, T, N;


    /* ROS topic callbacks */
    void inputMsg_Callback(const geometry_msgs::TwistStamped::ConstPtr& cmd_vel) {
			/*reads msg and stores info*/
			this->vel_x = cmd_vel->twist.linear.x;
			this->vel_y = cmd_vel->twist.linear.y;
			this->ome = cmd_vel->twist.angular.z;
			inversevel_computation();
			publish();
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
    void inversevel_computation(void){

    //computation of the velocity of the wheels in RPM from the velocity of the robot

			this->ome_fl=T/r*(vel_x-vel_y-ome*(l+w))*60/(2*3.14159);
			this->ome_fr=T/r*(vel_x+vel_y+ome*(l+w))*60/(2*3.14159);
			this->ome_rl=T/r*(vel_x+vel_y-ome*(l+w))*60/(2*3.14159);
			this->ome_rr=T/r*(vel_x-vel_y+ome*(l+w))*60/(2*3.14159);
			
			//ROS_INFO("estimated wheels' velocities Wfl,Wfr,Wrl,Wrr are [%f, %f, %f, %f]", (double)this->ome_fl, (double)this->ome_fr, (double)this->ome_rl, (double)this->ome_rr);
		};

    void publish(void){

		//Publishing wheels' velocities on topic wheels_rpm

			Robotics_project1::w_rpm wheels_rpm;

			wheels_rpm.rpm_fl=this->ome_fl;
			wheels_rpm.rpm_fr=this->ome_fr;
			wheels_rpm.rpm_rl=this->ome_rl;
			wheels_rpm.rpm_rr=this->ome_rr;

			pub.publish(wheels_rpm);
		};
    

  public:
    void main_function(void){

			/*ROS topics */
			this->sub = this->n.subscribe("/cmd_vel", 1, &inversevel::inputMsg_Callback, this);
			this->pub = this->n.advertise<Robotics_project1::w_rpm>("/wheels_rpm", 1);

			/*dynamic reconfigure*/
			dynamic_reconfigure::Server<Robotics_project1::parameters_calibrationConfig>::CallbackType f;
			f = boost::bind(&inversevel::parameters_Callback, this, _1, _2);
			dynServer.setCallback(f);

			/* Initialize node state */
			this->vel_x=0;
			this->vel_y=0;
			this->ome=0;
			this->ome_fl=0;
			this->ome_fr=0;
			this->ome_rl=0;
			this->ome_rr=0;
			
		};
};

int main (int argc, char **argv) {
	ros::init(argc,argv,"inversevel");
	inversevel inversevel_node;
	inversevel_node.main_function();
	//Wait other nodes to start
	sleep (1.0);
	ros::spin(); 
	return (0);
}