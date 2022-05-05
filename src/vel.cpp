#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/TwistStamped.h"
#include <dynamic_reconfigure/server.h>
#include <Robotics_project1/parameters_calibrationConfig.h>

class vel  { //header of the class
	private: 
    ros::NodeHandle n;

    /* ROS topics */
    ros::Subscriber sub;
    ros::Publisher pub;
    /*dyn reconfig server*/
	dynamic_reconfigure::Server<Robotics_project1::parameters_calibrationConfig> dynServer;

    /* ROS topic callbacks */
    void inputMsg_Callback(const sensor_msgs::JointState::ConstPtr& wheels_msg) {
		/*reads msg and stores info*/
		for(int j=0;j<4;j++){
			this->next_pos[j]=wheels_msg-> position[j];
		};
	};

	void parameters_Callback(Robotics_project1::parameters_calibrationConfig &config,uint32_t level){
		ROS_INFO("Requested to reconfigure: l=%f, w=%f, r=%f, T=%d, N=%d - Level %d", config.l, config.w, config.r, config.T, config.N, level);
	    
	    this->l = config.l;
	    this->w = config.w;
	    this->r = config.r;
	    this->T = config.T;
	    this->N = config.N;
	};
 
    /*auxiliary functions*/
    void vel_computation(void){
		this->next_time=ros::Time::now();
		double dt=(this->next_time-this->previous_time).toSec();
		/*double tick[]=this->next_pos[]-this->previous_pos[]; VECCHIA VERSIONE IN ALTERNATIVA A RIGHE 97-100*/
		double tick[4];
		for(int j=0;j<4;j++){
			tick[j]=this->next_pos[j]-this->previous_pos[j];
		};
		double wheel_vel[4];

		/*wheel_vel[]=tick[]*2*3,14159/dt/N/T;
		this->vel[]=r/4*[1 1 1 1; -1 1 1 -1; -1/(l+w) 1/(l+w) -1/(l+w) 1/(l+w)]*wheel_vel[];

		this->previous_time=this->next_time; /*questa ora è riga 120
		this->previous_pos=this->next_pos; VECCHIA VERSIONE di 109-117 MA SCRITTURA MAT SBAGLIATA*/

		double matrix_fromwheelveltovel_xyz[3][4]={{r/4, r/4, r/4, r/4},{-r/4,r/4,r/4,-r/4},{(-r/4)/(l+w),(r/4)/(l+w),-(r/4)/(l+w),(r/4)/(l+w)}}; /*CHECK IF OK EVEN IF r/4 out of parentheses*/
		for(int i=0;i<3;i++)
			vel_xyz[i]=0;
		for(int i=0;i<3;i++){
			for(int j=0;j<4;j++){
				wheel_vel[j]=tick[j]*2*3,14159/dt/N/T;
				this->vel_xyz[i]=this->vel_xyz[i]+matrix_fromwheelveltovel_xyz[i][j]*wheel_vel[j];
			};
		};
		ROS_INFO("supposed velocity is [%f,%f,%f]", (double)this->vel_xyz[0], (double)this->vel_xyz[1], (double)this->vel_xyz[2]);

	    this->previous_time = this->next_time;

	    for(int j = 0; j<4; j++){
	        this->previous_pos[j] = this->next_pos[j];
	    };
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

		pub.publish(cmd_vel);
	};
    
    
    /* Node state variables */
    ros::Time next_time, previous_time;
    
    double next_pos[4];
    double previous_pos[4];
    double vel_xyz[3];
    int T, N; //or they might be double CHECK!!!!
    double l, w, r;
    

  public:
	void Data(void){

		/*recover parameters from Ros parameter server
		std::string name;
		std::string shortname="OmnidirectionalRobot";

		name= ros::this_node::getName()+"/loop_rate";
		if (false==n.getParam(name,loop_rate))
		 	ROS_ERROR("Node %s couldn't recover parameter %s",ros::this_node::getName().c_str(),name.c_str());

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
		this->sub = this->n.subscribe("/wheel_states", 1, &vel::inputMsg_Callback, this);
		this->pub = this->n.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 1);

		/*dynamic reconfigure*/
		dynamic_reconfigure::Server<Robotics_project1::parameters_calibrationConfig>::CallbackType f;
		f = boost::bind(&vel::parameters_Callback, this, _1, _2);
		dynServer.setCallback(f);

		/* Initialize node state */
		this->next_time = ros::Time::now();
		this->previous_time = ros::Time::now();

		/*this->next_pos[] = 0.0;
		this->previous_pos[] = 0.0;
		double r = 0.07,lx = 0.2, ly = 0.169, T = 5, N = 42;  VECCHIA VERSIONE in alternativa a da riga 46 a 52*/

		for(int j=0;j<4;j++){
			this->vel_xyz[j]=0;
		};

		for(int j=0;j<4;j++){
			this->next_pos[j] = 0.0;
			this->previous_pos[j] = 0.0;
		}; /*NUOVA VERSIONE da riga 46 a 52*/

		ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
	};

	void RunPeriod(void){
		ROS_INFO("Node %s running.", ros::this_node::getName().c_str());

		ros::Rate loop_rate(10); /*anche questa new IN ALTERNATIVA A RIGA 66*/

		//Wait other nodes to start
		sleep (1.0);

		//initialize ticks
		ros::spinOnce();
		for(int j=0;j<4;j++)
			this->previous_pos[j] = this->next_pos[j];
		sleep(0.5);

		while (ros::ok()){
			ros::spinOnce();
			vel_computation(); /*NB PRIMA C'ERA vel:: anche questa new*/
			publish(); /*NB PRIMA C'ERA vel::*/
			loop_rate.sleep();
		};
	};
  	void Stop(void){
		ROS_INFO("Node %s is closing", ros::this_node::getName().c_str());
	};
};
      
int main (int argc, char **argv) {
	ros::init(argc,argv, "vel");
	vel vel_node;
	vel_node.Data();
	vel_node.RunPeriod();
	vel_node.Stop();

	return (0);
}