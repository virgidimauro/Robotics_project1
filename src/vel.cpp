#include "vel/vel.h"

void vel::Data(void){

	/*recover parameters from Ros parameter server*/
	std::string name;
	std::string shortname="OmnidirectionalRobot";

	name= ros::this_node::getName()+"/LoopRate";
	if (false==Handle.getParam(name,LoopRate))
		ROS_ERROR("Node %s couldn't recover parameter %s",ros::this_node::getName().c_str(),name.c_str());

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
	this->input_sub = this->Handle.subscribe("/wheels_msg", 1, &vel::inputMsg_Callback, this)
	this->output_pub = this->Handle.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 1); /*cambiato da versione prec dove c'era <nav_msgs::vel>("/vel", 1000); (gli 1 erano 1000)

	/* Initialize node state */
	this->current_time = ros::Time::now();
	this->past_time = ros::Time::now();

	/*this->current_pos[] = 0.0;
	this->past_pos[] = 0.0;
	double r = 0.07,lx = 0.2, ly = 0.169, T = 5, N = 42;  VECCHIA VERSIONE in alternativa a da riga 46 a 52*/

	for(int j=0;j<4;j++){
		this->vel[j]=0;
	};

	for(int j=0;j<4;j++){
		this->current_pos[j] = 0.0;
		this->past_pos[j] = 0.0;
	}; /*NUOVA VERSIONE da riga 46 a 52*/

	ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
};

void vel::RunPeriod(void){
	ROS_INFO("Node %s running.", ros::this_node::getName().c_str());

	ros::Rate LoopRate (this->LoopRate)/*anche questa new IN ALTERNATIVA A RIGA 66*/

	//Wait other nodes to start
	sleep (1.0);

	/*ros::Rate r(10);*/

	while (ros::ok()){
		ros::spinOnce();
		vel::vel_computation(); /*anche questa new*/
		vel::publish();
		LoopRate.sleep(); /*anche questa new Loop era r*/
	};
};

void vel::Stop(void){
	ROS_INFO("Node %s is closing", ros::this_node::getName().c_str());
};

/*void vel::inputMsg_Callback(const sensor_msgs::JointState::ConstPtr& wheels_msg) {
	this->current_pos[]=wheels_msg-> pos[];

	vel_computation();
} VECCHIA VERSIONE IN ALTERATIVA A 86-91*/

void vel::inputMsg_Callback(const sensor_msgs::JointState::ConstPtr& wheels_msg) {
	/*reads msg and stores info*/
	for(int j=0;j<4;j++){
		this->current_pos[j]=wheels_msg-> pos[j];
	};
};

void vel::vel_computation(void){
	this->current_time=ros::Time::now();
	double dt=(this->current_time-this->past_time).toSec();
	/*double tick[]=this->current_pos[]-this->past_pos[]; VECCHIA VERSIONE IN ALTERNATIVA A RIGHE 97-100*/
	double tick[4];
	for(int j=0;j<4;j++){
		tick[j]=this->current_pos[j]-this->past_pos[j];
	};
	double wheel_vel[4];

	/*wheel_vel[]=tick[]*2*3,14159/dt/N/T;
	this->vel[]=r/4*[1 1 1 1; -1 1 1 -1; -1/(l+w) 1/(l+w) -1/(l+w) 1/(l+w)]*wheel_vel[];

	this->past_time=this->current_time; /*questa ora è riga 120
	this->past_pos=this->current_pos; VECCHIA VERSIONE di 109-117 MA SCRITTURA MAT SBAGLIATA*/

	double matrix_fromwheelveltovel[3][4]={{r/4, r/4, r/4, r/4},{-r/4,r/4,r/4,-r/4},{(-r/4)/(l+w),(r/4)/(l+w),-(r/4)/(l+w),(r/4)/(l+w)}}; /*CHECK IF OK EVEN IF r/4 out of parentheses*/
	for(int i=0;i<3;i++)
		vel[j]=0;
	for(int=i=0;i<3;i++){
		for(int=j=0;j<4;j++){
			wheel_vel[j]=tick[j]*2*3,14159/dt/N/T;
			this->vel[i]=this->vel[i]+matrix_fromwheelveltovel[i][j]*wheel_vel[j];
		};
	};

	ROS_INFO("estimated velocity vx,vy,wz is [%f, %f, %f]", (double)this->vel[0], (double)this->vel[1], (double)this->vel[2]); /*anche questo è new*/
	this->past_time=this->current_time;

	for(int=j=0;j<4;j++){
		this->past_pos[j]=this->current_pos[j];
	};
};

void vel::publish(void){

	geometry_msgs:TwistStamped cmd_vel;

	cmd_vel.header.stamp=this->current_time;
	cmd_vel.header.frame_id="robot";
	cmd_vel.twist.linear.x=this->vel[0];
	cmd_vel.twist.linear.y=this->vel[1];
	cmd_vel.twist.linear.z=0.0;
	cmd_vel.twist.angular.x=0.0;
	cmd_vel.twist.angular.y=0.0;
	cmd_vel.twist.angular.z=this->vel[2];

	output_pub.publish(cmd_vel);
}; /*da riga 131 a 138 è tuto nuovo*/


/*NUOVOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO*/
#include "velocity/velocity.h"

void velocity::Prepare(void)
{
    /* Retrieve parameters from ROS parameter server */
    std::string FullParamName;
    std::string PartialName = "omnirobot";
    
    FullParamName = ros::this_node::getName()+"/LoopRate";
    if (false == Handle.getParam(FullParamName, LoopRate))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());
    
    FullParamName = PartialName+"/l";
    if (false == Handle.getParam(FullParamName, l))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());
    
    FullParamName = PartialName+"/w";
    if (false == Handle.getParam(FullParamName, w))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());
    
    FullParamName = PartialName+"/r";
    if (false == Handle.getParam(FullParamName, r))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());
    
    FullParamName = PartialName+"/T";
    if (false == Handle.getParam(FullParamName, T))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());
    
    FullParamName = PartialName+"/N";
    if (false == Handle.getParam(FullParamName, N))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());
    
    
    
    /* ROS topics */
    this->input_subscriber = this->Handle.subscribe("/wheel_states", 1, &velocity::input_MessageCallback, this);
    this->output_publisher = this->Handle.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 1);
    

    /* Initialize node state */
    this->current_time = ros::Time::now();
    this->past_time = ros::Time::now();
    
    for(int i = 0; i<4; i++){
        this->vel[i] = 0;
    };
    
    for(int i=0; i<4; i++){
        this->position_curr[i] = 0.0;
        this->position_past[i] = 0.0;
    };

    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
};

void velocity::RunPeriodically(void)
{
    ROS_INFO("Node %s running.", ros::this_node::getName().c_str());

    ros::Rate LoopRate(this->LoopRate);

    // Wait other nodes start
    sleep(1.0);

    while (ros::ok()){
        
        ros::spinOnce();
        velocity::compute_velocity();
        velocity::publish();
        LoopRate.sleep();
    };
};

void velocity::Shutdown(void)
{

    ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
};

void velocity::input_MessageCallback(const sensor_msgs::JointState::ConstPtr& wheel_state)
{
    /* Read message and store information */
    for(int i=0; i<4; i++){
        this->position_curr[i] = wheel_state->position[i];
    };
};

void velocity::compute_velocity(void){
    
    this-> current_time = ros::Time::now();
    double Ts = (this->current_time - this->past_time).toSec();
    double ticks[4];
    for (int i = 0; i < 4; i++){
        ticks[i] = this->position_curr[i] - this->position_past[i];
    };
    double wheel_vel[4];
    double mat[3][4] = {{r/4, r/4, r/4, r/4},
                     {-r/4, r/4, r/4, -r/4},
                     {-r/4/(l+w), r/4/(l+w), -r/4/(l+w), r/4/(l+w)}};

    
    for(int i=0; i<3; i++)
        vel[i] = 0;

    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 4; j++){
            wheel_vel[j] = ticks[j] / Ts / N / T * 2 * 3.14;
            this->vel[i] = this->vel[i] + mat[i][j] * wheel_vel[j];
        };
    };

    ROS_INFO("supposed velocity is [%f,%f,%f]", (double)this->vel[0], (double)this->vel[1], (double)this->vel[2]);


    this->past_time = this->current_time;

    for(int i = 0; i<4; i++){
        this->position_past[i] = this->position_curr[i];
    };
};
    
    
void velocity::publish(){
    geometry_msgs::TwistStamped cmd_vel;
    cmd_vel.header.stamp = this->current_time;
    cmd_vel.header.frame_id = "robot";
        
    cmd_vel.twist.linear.x = this->vel[0];
    cmd_vel.twist.linear.y = this->vel[1];
    cmd_vel.twist.linear.z = 0.0;
        
    cmd_vel.twist.angular.x = 0.0;
    cmd_vel.twist.angular.y = 0.0;
    cmd_vel.twist.angular.z = this->vel[2];
        
    output_publisher.publish(cmd_vel);
};