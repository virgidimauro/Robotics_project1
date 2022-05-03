#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "Robotics_project1/w_rpm.h"
#include <dynamic_reconfigure/server.h>
#include <Robotics_project1/calibrationConfig.h>

class inversevel  //header of the class
{
  private: 
    ros::NodeHandle n;

    /* ROS topics */
    ros::Subscriber sub;
    ros::Publisher pub;
    /*dyn reconfig server*/
		dynamic_reconfigure::Server<Robotics_project1::calibrationConfig> dynServer;

    /* ROS topic callbacks */
    void inputMsg_Callback(const geometry_msgs::TwistStamped::ConstPtr& cmd_vel) {
			/*reads msg and stores info*/
			this->vel_x = cmd_vel->twist.linear.x;
			this->vel_y = cmd_vel->twist.linear.y;
			this->ome = cmd_vel->twist.angular.z;

			inversevel::inversevel_computation();
			inversevel::publish();
		};

    void parameters_Callback(Robotics_project1::calibrationConfig &config,uint32_t level){
			ROS_INFO("Requested to reconfigure: l=%f, w=%f, r=%f, T=%d, N=%d - Level %d", config.l, config.w, config.r, config.T, config.N, level);
		    
		    this->l = config.l;
		    this->w = config.w;
		    this->r = config.r;
		    this->T = config.T;
		    this->N = config.N;
		};

    /*auxiliary functions*/
    void inversevel_computation(void){
			this->ome_fl=T/r*(vel_x-vel_y-ome*(l+w));
			this->ome_fr=T/r*(vel_x+vel_y+ome*(l+w));
			this->ome_rl=T/r*(vel_x+vel_y-ome*(l+w));;
			this->ome_rr=T/r*(vel_x-vel_y+ome*(l+w));
			
			ROS_INFO("estimated wheels' velocities Wfl,Wfr,Wrl,Wrr are [%f, %f, %f, %f]", (double)this->ome_fl, (double)this->ome_fr, (double)this->ome_rl, (double)this->ome_rr);
		};
    void publish(void){

			Robotics_project1::w_rpm wheels_rpm;

			wheels_rpm.rpm_fl=this->ome_fl;
			wheels_rpm.rpm_fr=this->ome_fr;
			wheels_rpm.rpm_rl=this->ome_rl;
			wheels_rpm.rpm_rr=this->ome_rr;

			pub.publish(wheels_rpm);
		};
	    
    
    /* Node state variables */
    
    double vel_x, vel_y, ome;
    double ome_fl, ome_fr, ome_rl, ome_rr;
    int T,N; //or they might be double CHECK!!!!
    double l, w, r;
    

  public:
    void Data(void){

			/*recover parameters from Ros parameter server
			std::string name;
			std::string shortname="OmnidirectionalRobot";

			name= shortname+"/l";
			if (false==n.getParam(name,l))
				ROS_ERROR("Node %s couldn't recover parameter %s",ros::this_node::getName().c_str(),name.c_str());

			name= shortname+"/w";
			if (false==n.getParam(name,w))
				ROS_ERROR("Node %s couldn't recover parameter %s",ros::this_node::getName().c_str(),name.c_str());

			name= shortname+"/r";
			if (false==n.getParam(name,r))
				ROS_ERROR("Node %s couldn't recover parameter %s",ros::this_node::getName().c_str(),name.c_str());

			name= shortname+"/T";
			if (false==n.getParam(name,T))
				ROS_ERROR("Node %s couldn't recover parameter %s",ros::this_node::getName().c_str(),name.c_str());

			name= shortname+"/N";
			if (false==n.getParam(name,N))
				ROS_ERROR("Node %s couldn't recover parameter %s",ros::this_node::getName().c_str(),name.c_str()); */


			/*ROS topics */
			this->sub = this->n.subscribe("/cmd_vel", 1, &inversevel::inputMsg_Callback, this);
			this->pub = this->n.advertise<Robotics_project1::w_rpm>("/wheels_rpm", 1);

			/*dynamic reconfigure*/
			dynamic_reconfigure::Server<Robotics_project1::calibrationConfig>::CallbackType f;
			f = boost::bind(&vel::parameters_Callback, this, _1, _2);
			dynServer.setCallback(f);

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
    void RunPeriod(void){
			ROS_INFO("Node %s running.", ros::this_node::getName().c_str());

			//Wait other nodes to start
			sleep (1.0);
			ros::spin(); 

		};
    void Stop(void){
			ROS_INFO("Node %s is closing", ros::this_node::getName().c_str());
		};

};

int main (int argc, char **argv) {
	ros::init(argc,argv,"inversevel");
	inversevel inversevel_node;
	inversevel_node.Data();
	inversevel_node.RunPeriod();
	inversevel_node.Stop();

	return (0);
}