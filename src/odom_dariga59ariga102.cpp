%%%%%%%%%%%%%%%%%%%## io da riga 59 a riga 102%%%%%%%
void odom::Stop(void){
	ROS_INFO("Node %s is closing", ros::this_node::getName().c_str());
}

void odom::inputMsg_Callback(const sensor_msgs::JointState::ConstPtr& cmd_vel) {
	this->vel_x=cmd_vel-> twist.linear.x;
	this->vel_y=cmd_vel-> twist.linear.y;
	this->ome=cmd_vel-> twist.angular.z;

	integration();
}

bool odom::odomReset_Callback(Robotics_project1::Reset_Odometry::Request& request,Robotics_project1::Reset_Odometry::Response& response){
	response.x=this->x;
	response.y=this->y;
	response.theta=this->theta;

	this->x=request.x;
	this->y=request.y;
	this->theta=request.theta;

	ROS_INFO("Requested to reset the pose of the odometry to [%f %f %f]. Responded with the last pose [%f %f %f].", 
			(double)request.x, (double)request.y, (double)request.theta, (double)response.x, (double)response.y, (double)response.theta);
	return true;
}

void odom::reconfig_Callback(Robotics_project1::integration_methodsConfig &config,uint32_t level){
	ROS_INFO("Reconfigure request: %d - level %d", 
		integration_methodsConfig,level);
	this->integration_method=config.integration_method;
}


void odom::integration(void){
	this->current_time=ros::Time::now();
	double dt=(this->current_time-this->past_time).toSec();
	double delta_x, delta_y, delta_theta;




	double wheel_vel[4];

	wheel_vel[]=tick[]*2*3,14159/dt/N/T;
	this->vel[]=r/4*[1 1 1 1; -1 1 1 -1; -1/(l+w) 1/(l+w) -1/(l+w) 1/(l+w)]*wheel_vel[];

	this->past_time=this->current_time;
	this->past_pos=this->current_pos;
}